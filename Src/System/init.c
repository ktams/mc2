/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2020 Tams Elektronik GmbH and Andreas Kretzer
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <string.h>
#include <stdlib.h>
#include "rb2.h"
#include "lwip/ip.h"
#include "lwip/tcpip.h"
#include "lwip/apps/mdns.h"
#include "ethernet.h"
#include "nandflash.h"
#include "decoder.h"
#include "config.h"
#include "events.h"

static TaskHandle_t rebootHandler;

static void mount (void)
{
	Y_LOFF_T total, avail, used;
	int percent;
	int rc;

	struct yaffs_dev *dev = NULL;
	struct yaffs_param *param;
	struct yaffs_driver *drv;

	yaffsfs_OSInitialisation();

	if ((dev = calloc(1, sizeof(*dev))) == NULL) {
		fprintf(stderr, "%s(): NO RAM for device structure\n", __func__);
		return;
	}

	dev->param.name = "/";		// param name is used as mount point!
	drv = &dev->drv;
	drv->drv_write_chunk_fn = nand_write_chunk;
	drv->drv_read_chunk_fn = nand_read_chunk;
	drv->drv_erase_fn = nand_erase;
	drv->drv_mark_bad_fn = nand_mark_bad;
	drv->drv_check_bad_fn = nand_check_bad;
	drv->drv_initialise_fn = nand_initialise;
	drv->drv_deinitialise_fn = nand_deinitialise;

	/* Toshiba TC58CVG1S3HxAIx 256MB serial NAND-Flash */
	param = &dev->param;
	param->total_bytes_per_chunk = 2048;	// we have 2k blocks
	param->spare_bytes_per_chunk = 60;		// let's reserve 4 bytes for a bad block marker
	param->chunks_per_block = 64;			// we have 64 pages in a block
	param->start_block = 0;					// we use the whole flash as a file system
	param->end_block = 2047;				// we have 2048 blocks with 64 2k-pages each = 256MB + spare areas
	param->n_reserved_blocks = 5;			// follow the recommandation in the YafFs tuning document
	param->inband_tags = 0;					// we have enough spare area for tags
	param->use_nand_ecc = 1;				// even though tuning document tells us, this is only for Yaffs1, the samples set it to '1'
	param->no_tags_ecc = 0;					// even the spare part is covered by ECC
	param->is_yaffs2 = 1;					// we want to use the YAFFS2 format
	param->n_caches = 10;					// let's follow the minimum typical from the tuning document
	param->empty_lost_n_found = 1;			// we don't really make use of the 'lost+found' directory
	param->skip_checkpt_rd = 0;				// skipping checkpoint reads makes mount slower
	param->skip_checkpt_wr = 0;				// to read the checkpoints, we should write them on a sync()
	param->refresh_period = 5000;			// don't really know how that is counted, 1000 is typical, but at least one sample uses 10000!
	param->enable_xattr = 0;				// we don't really need extended attributes, so disable them

	yaffs_add_device(dev);

    rc = yaffs_mount("/");

	log_msg (LOG_INFO, "%s(): mount() = %d (errno = %d)\n", __func__, rc, errno);
	if (rc == 0) {
		total = yaffs_totalspace("/");
		avail = yaffs_freespace("/");
		used = total - avail;
		percent = (int) ((used * 10000) / total);
		log_msg (LOG_INFO, "%s(): Total %lldkb / Free %lldkb / Used %lldkb (%d.%02d%%)\n", __func__, (total + 512) / 1024, (avail + 512) / 1024,
				(used + 512) / 1024, percent / 100, percent % 100);
	}
}

/**
 * Doing a clean reboot by unmounting the YAFFS file system, giving the system some time
 * to drain it's output (i.e. debug messages, close HTTP-Connection, ...) and then trigger
 * a reboot thru the SCB->AIRCR register.
 *
 * \note	There is no special treatment or whatsoever to inform other tasks of this
 * 			shutdown. It is assumed, that this request may come from a Web-request and
 * 			due to the fact, that the reset is executed as a "background" task, this
 * 			connection has the chance to finish it's job gracefully.
 *
 * \param pvParameter	ignored thread parameter
 */
static void vRebootProc (void *pvParameter)
{
	int rc, retry;

	(void) pvParameter;

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	vTaskDelay(200);
	retry = 0;
	do {
		rc = yaffs_unmount("/");
		if (rc) {
			if (errno == -EINVAL) vTaskDelete(NULL);	// umount already done
			fprintf(stderr, "%s(): unmount() = %d (errno = %d)\n", __func__, rc, errno);
		} else {
			printf("%s(): unmount() OK\n", __func__);
		}
		vTaskDelay(100);
		retry++;
	} while (rc != 0 && retry < 3);

	NVIC_SystemReset();		// does not return!
}

#define RST_ALL_FLAGS		(RCC_RSR_LPWRRSTF | RCC_RSR_WWDG1RSTF | RCC_RSR_IWDG1RSTF | RCC_RSR_SFTRSTF | RCC_RSR_PORRSTF | \
							 RCC_RSR_PINRSTF | RCC_RSR_BORRSTF | RCC_RSR_D2RSTF | RCC_RSR_D1RSTF | RCC_RSR_CPURSTF)
#define RST_PWR_ON			(RCC_RSR_PORRSTF | RCC_RSR_PINRSTF | RCC_RSR_BORRSTF | RCC_RSR_D2RSTF | RCC_RSR_D1RSTF | RCC_RSR_CPURSTF)
#define RST_NRST_PIN		(RCC_RSR_PINRSTF | RCC_RSR_CPURSTF)
#define RST_BROWNOUT		(RCC_RSR_PINRSTF | RCC_RSR_BORRSTF | RCC_RSR_CPURSTF)
#define RST_SOFTRESET		(RCC_RSR_SFTRSTF | RCC_RSR_PINRSTF | RCC_RSR_CPURSTF)
#define RST_CPU_RESET		(RCC_RSR_CPURSTF)
#define RST_WWDG1			(RCC_RSR_WWDG1RSTF | RCC_RSR_PINRSTF | RCC_RSR_CPURSTF)
#define RST_IWDG1			(RCC_RSR_IWDG1RSTF | RCC_RSR_PINRSTF | RCC_RSR_CPURSTF)
#define RST_D1_EXIT_STDBY	(RCC_RSR_D1RSTF)
#define RST_D2_EXIT_STDBY	(RCC_RSR_D2RSTF)
#define RST_ERROR_STDBY		(RCC_RSR_LPWRRSTF | RCC_RSR_PINRSTF | RCC_RSR_CPURSTF)

static void reset_reason (void)
{
	switch (RCC->RSR & RST_ALL_FLAGS) {
		case RST_PWR_ON: 		log_msg (LOG_INFO, "%s(): Power-ON\n", __func__); break;
		case RST_NRST_PIN:		log_msg (LOG_INFO, "%s(): RESET-Pin (NRST)\n", __func__); break;
		case RST_BROWNOUT:		log_msg (LOG_INFO, "%s(): BROWNOUT\n", __func__); break;
		case RST_SOFTRESET:		log_msg (LOG_INFO, "%s(): SOFTRESET by CPU\n", __func__); break;
		case RST_CPU_RESET:		log_msg (LOG_INFO, "%s(): CPU reset (CPURST)\n", __func__); break;
		case RST_WWDG1:			log_msg (LOG_INFO, "%s(): WWDG1 fired\n", __func__); break;
		case RST_IWDG1:			log_msg (LOG_INFO, "%s(): IWDG1 fired\n", __func__); break;
		case RST_D1_EXIT_STDBY:	log_msg (LOG_INFO, "%s(): D1 exits STANDBY\n", __func__); break;
		case RST_D2_EXIT_STDBY:	log_msg (LOG_INFO, "%s(): D2 exits STANDBY\n", __func__); break;
		case RST_ERROR_STDBY:	log_msg (LOG_INFO, "%s(): D1 or CPU erroneously enter STANDBY/CSTOP\n", __func__); break;
		default: log_msg (LOG_WARNING, "%s(): unknown RESET reason (RCC_RSR=0x%08lx)\n", __func__, RCC->RSR); break;
	}

	// now clear all flags
	RCC->RSR = RCC_RSR_RMVF;
	RCC->RSR = 0;
}

void vInit(void *pvParameters)
{
	struct yaffs_stat stat;
	struct sysconf *cfg;
//    ip4_addr_t ip_addr;
//    ip4_addr_t ip_mask;
//    ip4_addr_t ip_gw;

	(void) pvParameters;
    xTaskCreate(rgb_handler, "RGBleds", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

//	rt.status = SS_STOP;
	ts_init();
	sig_setMode(TM_STOP);

	log_msg (LOG_INFO, "====================================================================\n");
	log_msg (LOG_INFO, "Tams mc2 startup %s (HW %x.%x)\n", SOFT_VERSION, hwinfo->HW >> 4, hwinfo->HW & 0xF);
	log_msg (LOG_INFO, "CORE revision r%dp%d\n", cpu.r, cpu.p);
	log_msg (LOG_INFO, "DEVICE ID 0x%04lX Rev. 0x%04lX (%c)\n", cpu.idcode & 0xFFF, cpu.idcode >> 16, cpu.revcode);
	log_msg (LOG_INFO, "%s() %dK bytes RAM free\n", __func__, xPortGetFreeHeapSize() / 1024);
    reset_reason();

    xTaskCreate(rgb_handler, "RGBleds", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    seg_registerEvents();
    mount();

    if (KEY1_PRESSED()) {		// GO on boot means: drop configuration and start with factory defauls
    	seg_factoryReset();
    	while (KEY1_PRESSED()) vTaskDelay(20);
    	yaffs_unlink(CONFIG_SYSTEM);
    	yaffs_unlink(CONFIG_LOCO);
    	seg_display(0, 0);
    }
    cfg = cnf_readConfig();

    tcpip_init(NULL, NULL);
	if ((rt.en = malloc(sizeof(*rt.en))) != NULL) {
		memset(rt.en, 0, sizeof(*rt.en));
#if 1
		if (cfg->ipm == IPMETHOD_MANUAL && cfg->ip_addr.addr && cfg->ip_mask.addr) {
			netifapi_netif_add(rt.en, &cfg->ip_addr, &cfg->ip_mask, &cfg->ip_gw, NULL, stmenet_init, tcpip_input);
		} else {
			netifapi_netif_add(rt.en, NULL,     NULL,     NULL,   NULL, stmenet_init, tcpip_input);
		}
#else
		IP4_ADDR(&ip_addr, 192, 168, 1, 42);
		IP4_ADDR(&ip_mask, 255, 255, 255, 0);
		IP4_ADDR(&ip_gw, 0, 0, 0, 0);
		netifapi_netif_add(rt.en, &ip_addr, &ip_mask, &ip_gw, NULL, stmenet_init, tcpip_input);
//		netifapi_netif_add(rt.en, NULL,     NULL,     NULL,   NULL, stmenet_init, tcpip_input);
#endif
		dbg_init();
		netifapi_netif_set_link_down(rt.en);
		netifapi_netif_set_up(rt.en);
		netifapi_netif_set_default(rt.en);
		ip4_set_default_multicast_netif(rt.en);
		if (cfg->ipm == IPMETHOD_DHCP) netifapi_dhcp_start(rt.en);
	}
	mdns_resp_init();
	mdns_resp_add_netif(rt.en, "mc2", 120);

//	xTaskCreate(idlefunc, "IDLE", 256, NULL, tskIDLE_PRIORITY, NULL);		// a timeburner task at idle priority (DEBUG)
	key_init();
    rc_init();	// ... instead of the now useless thread function

    xTaskCreate(vSigGeneration, "SIGNAL", 1024, NULL, 4, NULL);							// a quite high priority task!
    xTaskCreate(vAnalog, "Analog", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vS88Bus, "s88", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);

#ifdef HW_REV07
	// around 18V at booster output
	DAC1->DHR12R1 = 3000;
#else
	DAC1->DHR12R1 = 2250;	// 8 Volt
#endif
    xTaskCreate(vKeyHandler, "KeyHandler", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
//    xTaskCreate(player, "Audioplayer", 1024, NULL, 1, NULL);
//    xTaskCreate(esp_testthread, "ESP-01", 2048, NULL, 1, NULL);
    xTaskCreate(vAudioTest, "AUDIOtest", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    if (yaffs_access(FLASH_FILE, 0) == 0) {
    	yaffs_stat(FLASH_FILE, &stat);
    	if (stat.st_size > 0) {
			printf ("%s(): Updatefile will be truncated\n", __func__);
			yaffs_truncate(FLASH_FILE, 0);
			yaffs_sync("/");
    	}
    }
    yaffs_mkdir(CONFIG_DIR, S_IREAD | S_IWRITE | S_IEXEC);
    yaffs_mkdir(FIRMWARE_DIR, S_IREAD | S_IWRITE | S_IEXEC);
    yaffs_mkdir(MANUALS_DIR, S_IREAD | S_IWRITE | S_IEXEC);
    webup_manuals();

    if (yaffs_access(CONFIG_DIR "company.js", 0) != 0) {
    	char *tmp;
    	int fd;

    	log_msg (LOG_INFO, "%s() generating /config/company.js\n", __func__);
    	if ((fd = yaffs_open(CONFIG_DIR "company.js", O_CREAT | O_RDWR, 0666)) >= 0) {
    		tmp = tmpbuf(60);
    		sprintf (tmp,"var company = %d; // 1=Tams, 2=KM-1\n", (hwinfo->manufacturer == DCC_MANUFACTURER_TAMS) ? 1 : 2);
    		yaffs_write(fd, tmp, strlen(tmp));
    		yaffs_close(fd);
    	} else {
        	log_error("%s() cannot create /config/company.js\n", __func__);
    	}
    }

    ftpd_start();
    httpd_start();

    db_init();
	// all external interfaces should be started AFTER the loco DB is initialised
	xTaskCreate(vXpressNet, "XpressNet", 1024, NULL, 1, NULL);
	xTaskCreate(vLocoNet, "LocoNet", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
	xTaskCreate(vMCanHandler, "CANHandler", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(sniffer, "SNIFFER", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(trnt_service, "TRNT-SVC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(dccA_service, "DCC-A", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
	xTaskCreate(reply_callbackHandler, "reply-CB", 2048, NULL, 3, NULL);

	p50x_start(cfg->p50_port);
	bidib_start();

//    sntp_init();
//    sntp_getoperatingmode();

	mt_init();

//	event_fire(EVENT_SYS_STATUS, SYSEVENT_STOP, NULL);	// fire an event
	log_msg (LOG_INFO, "%s() ready ... %dK bytes RAM free after mount()\n", __func__, xPortGetFreeHeapSize() / 1024);

    vTaskDelay(1000);
	xTaskCreate(easynet, "EasyNet", 1024, NULL, 1, NULL);
    xTaskCreate(z21_service, "Z21-Service", configMINIMAL_STACK_SIZE * 2, (void *) 21105, 1, NULL);

    // prepare the Reboot-Task, so it won't be necessary to create it in an emergency situation
    xTaskCreate(vRebootProc, "REBOOT", configMINIMAL_STACK_SIZE, NULL, 4, &rebootHandler);

	if(cfg->sysflags & SYSFLAG_STARTSTATE) sig_setMode(TM_GO); else  sig_setMode(TM_STOP);

	vTaskDelete(NULL);
}

/**
 * Reboots the System by creating a new Thread (\ref vRebootProc()), that unmounts
 * the filesystem then triggers a system reset via the SCB->AIRCR using the CMSIS
 * function NVIC_SystemReset().
 */
void reboot (void)
{
	printf("%s() restart requested\n", __func__);
	xTaskNotifyGive(rebootHandler);
}

void pwrfail (void)
{
	int rc;

	MAINBST_OFF();
	seg_powerfail();
	rgb_off();											// turn off RGB-LEDs
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);	// get highest priority
	rc = yaffs_unmount2("/", 1);
	fprintf(stderr, "%s() forced unmount (rc = %d)\n", __func__, rc);
	MKLNBST_OFF();
	vTaskPrioritySet(NULL, 1);	// get normal (low) priority
	vTaskDelay(200);			// give output time to drain (this will probably never return because of the supply dropping too fast)
	NVIC_SystemReset();			// does not return!
}
