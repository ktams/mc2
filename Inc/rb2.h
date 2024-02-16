/*
 * File:   rbboot.h
 * Author: Andi
 *
 * Created on 14.08.2019
 */

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

#ifndef __RB2_H__
#define	__RB2_H__

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>
#include "stm32h7xx.h"
#include "myalloc.h"
#include "FreeRTOS.h"
#include "lwip/sys.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "logging.h"

#define USE_CACHE
#define SYSCLK_FREQ				400000000uL		///< CPU frequency (in Hz)
#define HCLK_FREQ				200000000uL		///< HCLK frequency (in Hz)

#define SDRAM_BASE				0x60000000
#define SDRAM_SIZE				(8 * 1024 * 1024)
#define INITIAL_STACK_SIZE		(2 * 1024)		///< 2k stack for startup (and maybe ISR?)

// definition of RAM block sizes (not available through system includes (CMSIS))
#define D1_DTCMRAM_SIZE			(128 * 1024)
#define D1_AXISRAM_SIZE			(512 * 1024)
#define D2_AXISRAM_SIZE			(288 * 1024)
#define D3_SRAM_SIZE			(64 * 1024)

#define MAC_EEPROM				0x50	///< the seven bit address of the I2C MAC address EEPROM on I2C4 bus

#define str(s)					#s
#define xstr(s)					str(s)
#define SOFT_VERSION_MAJOR		1
#define SOFT_VERSION_MINOR		7
#define SOFT_VERSION_SUB		3
#define SOFT_VERSION_BETA		"test"
#define SOFT_VERSION			"V" xstr(SOFT_VERSION_MAJOR) "." xstr(SOFT_VERSION_MINOR) "." xstr(SOFT_VERSION_SUB) SOFT_VERSION_BETA

#define DCC_MANUFACTURER_TAMS	0x3E	///< assigned manufacturer ID for Tams
#define DCC_MANUFACTURER_PD		0x0D	///< Public Domain & Do-It-Yourself Decoders

struct hwinfo {
	char		proddate[17];	///< production date string "YYYY-MM-DD_HH:MM\0"
	uint8_t		reserved1[3];	///< fill up for alignment
	uint32_t	resereved2;		///< fill up unused space
	int			serial;			///< the serial number (will be manipulated in HEX file to have different numbers)
	uint8_t		HW;				///< a BCD variable showing the PCB revision (i.e. 0x12 for HW 1.2)
	uint8_t		manufacturer;	///< the manufacture code for DCC (Tams = 62)
	uint8_t		reserved3;		///< fill up for alignment
	uint8_t		reserved4;		///< fill up for alignment
};
extern const struct hwinfo *hwinfo;

#ifndef HW_REV07
#define EASYNET_USE_SPI1		///< in HW 0.7 we used SPI6 - from HW 1.0 onwards, SPI1 is used
#endif

/*
 * The hardware versions are coded using BCD (i.e. hexadecimal).
 * To not get in trouble when errneously using decimal version numbers
 * we define some handy macros here.
 */
#define HW07		0x07
#define HW10		0x10
#define HW11		0x11
#define HW12		0x12
#define HW14		0x14
#define HW15		0x15
#define HW16		0x16

// Einstellungen für Booster, s88

#define BOOSTER_BUILTIN		1
#define BOOSTER_MM			2
#define BOOSTER_CDE			4
#define BOOSTER_BIDIB		8
#define BOOSTER_ALL			(BOOSTER_BUILTIN | BOOSTER_MM | BOOSTER_CDE | BOOSTER_BIDIB)

#define MAX_FEEDBACKS		(64 * 1024)	///< maximum number of managed feedback bits
// the following defines are old (but currently still in use) and will be removed in a future version
#define MAX_S88MODULES		64			///< the maximum number of modules that we will scan
#define MAX_CANMODULES		64			///< the maximum number of modules that we support
#define FB_MCAN_OFFSET		(192 * 16)	///< all feedbacks will report by this offset (i.e. MCAN #0 -> FB #3072)
#define MAX_LNETMODULES		64			///< the maximum number of modules that we support
#define FB_LNET_OFFSET		(64 * 16)	///< all feedbacks will report by this offset (i.e. LNET #0 -> FB #1024)
#define MAX_FBMODULES		(MAX_S88MODULES + MAX_CANMODULES + MAX_LNETMODULES)

#define CENTRAL_FEEDBACK				///< we use the new feedback design in Interfaces/feedback.c

struct s88_status {
	int				 modcnt;		///< count of currently scanned modules
	uint16_t		 sum[MAX_FBMODULES];			///< summation of s88 stati (each feedback module contains 16 bits)
	uint32_t		 evFlag[MAX_FBMODULES / 32];	///< a flag for each changed feedback module
};

extern uint32_t volatile ui32DisplayFilter; /* Bit	display
											 * 0	loco 28
											 * 1	loco 128
											 * 2	loco SDF
											 * 3	spare
											 * 4	loco function f0 - f4
											 * 5	loco function f5 - f8
											 * 6	loco function f9 - f12
											 * 7	loco function f13 - f20
											 * 8	loco function f21 - f28
											 * 9	loco function f29 - f36
											 * 10	loco function f37 - f44
											 * 11	loco function f45 - f52
											 * 12	loco function f53 - f60
											 * 13	loco function f61 - f68
											 * 14	spare
											 * 15	RailCom
											 * 16	basic accessory
											 * 17	extended accessory
											 * 18	spare
											 * 19	spare
											 * 20	loco MM
											 * 21	accessory MM
											 * 22	... (TBD)
											 * 23	... (TBD)
											 */

#define LOCO28		(ui32DisplayFilter & 1)
#define LOCO128		(ui32DisplayFilter & 2)
#define LOCOSDF		(ui32DisplayFilter & 4)
#define LOCOfunc1	(ui32DisplayFilter & 0x10)
#define LOCOfunc2	(ui32DisplayFilter & 0x20)
#define LOCOfunc3	(ui32DisplayFilter & 0x40)
#define LOCOfunc4	(ui32DisplayFilter & 0x80)
#define LOCOfunc5	(ui32DisplayFilter & 0x100)
#define LOCOfunc6	(ui32DisplayFilter & 0x200)
#define LOCOfunc7	(ui32DisplayFilter & 0x400)
#define LOCOfunc8	(ui32DisplayFilter & 0x800)
#define LOCOfunc9	(ui32DisplayFilter & 0x1000)
#define LOCOfunc10	(ui32DisplayFilter & 0x2000)
#define _RC_		(ui32DisplayFilter & 0x8000)
#define ACC_B		(ui32DisplayFilter & 0x10000)
#define ACC_E		(ui32DisplayFilter & 0x20000)
#define LOCOMM		(ui32DisplayFilter & 0x100000)
#define ACCMM		(ui32DisplayFilter & 0x200000)

extern struct cpu_info {
	uint32_t	cpuid;					///< the content of SCB->CPUID
	uint32_t	idcode;					///< the content of DBGMCU->IDCODE
	uint8_t		r;						///< the 'n' in the RnPm notatipon of core revision, should be n=1 and m=1 => "r1p1"
	uint8_t		p;						///< the 'm' in the RnPm notatipon of core revision
	char		revcode;				///< one of the chip revision codes V, X, Y and Z (or '?' if none of the defined values)
} cpu;

/**
 * a flexible variable interpretable as different types without casting
 */
typedef union {
	int32_t			 i32;				///< value interpreted as integer
	uint32_t		 u32;				///< value interpreted as unsigned
	uint8_t			 ui8[4];			///< value interpreted as byte array (4 bytes, little endian on STM32)
	void			*p;					///< value interpreted as pointer
	struct {
		uint8_t		 bitpos:3;			///< a bit position for DCC CV bit handling functions
		uint8_t		 bitval:1;			///< the value (0 or 1) of the bit to verify/write
	};
} flexval;

extern const flexval fvNULL;

/**
 * the operational mode for the booster(s)
 */
enum trackmode {
	TM_STOP = 0,						///< power and signal delivery to track is stopped
	TM_SHORT,							///< power and signal delivery to track is stopped after an overcurrent condition
	TM_HALT,							///< power delivery to track is nominal but every speed information is replaced by the STOP code
	TM_GO,								///< power and signal delivery to track is nominal - locos are full operational
	TM_SIGON,							///< signal generation is switched on but boosters stay off (commanded from BiDiB)
	TM_DCCPROG,							///< relais is switched to programming track, packet contain no decoder addresses
	TM_TAMSPROG,						///< relais is switched to programming track, only MM1/2 packets are used to program Motorola decoders
	TM_TESTDRIVE,						///< relais is switched to programming track and normal operation with currentlimiter is done
	TM_POWERFAIL,						///< not a real trackmode but the system state when a powerfail is detected
	TM_RESET,							///< not a real trackmode but the system state when preparing a reset
	TM_OVERTTEMP,						///< system too hot - can only be cleared by TM_TEMPOK
	TM_TEMPOK,							///< a special state that brings us out of over temperature condition and goes to TM_STOP
};

/**
 * The interface that controls the operation of the whole layout.
 * This value is also used in the event EVENT_EXTCONTROL.
 */
#define	EXTCTRL_NONE	0				///< we are not controlled externally and so are fully responsible for every detail
#define EXTCTRL_LOCKED	1				///< web interface lock
#define	EXTCTRL_P50X	2				///< we are controlled by at least one P50X interface connection
#define	EXTCTRL_BIDIB	4				///< we are controlled by a BIDIB connection


/**
 * For device addresses on external busses, we should
 * know what interface the address belongs to.
 */
enum extbus {
	BUS_EASYNET,						///< Tams EasyNet
	BUS_XPRESSNET,						///< Lenz XPressNet
	BUS_LOCONET,						///< Digitrax LocoNet
	BUS_MCAN,							///< Märklin CAN bus
	BUS_BIDIBUS,						///< openDCC BiDiBus (RS485 implementation)
};

enum devtype {
	DEV_GENERIC,						///< everything that is somehow unknown
	DEV_CONTROL,						///< a control
	DEV_BLA,							///< some other type of connected device (TBD)
};

/**
 * A global structure that holds all kind of global status information and
 * variables we may need access to.
 */
extern struct runtime {
    struct netif    *en;				///< the ethernet networkinterface
    struct netif    *wlan;				///< the WLAN networkinterface
    volatile enum trackmode tm;			///< the track mode
    uint32_t		 ctrl;				///< the interface type which controls our functions
    uint32_t		 totalHeap;			///< the total heap size when starting the system
} rt;

/**
 * A construct to hold an arbitrary list of key=value pairs
 */
struct key_value {
	struct key_value	*next;			///< linked list of key=value fields
	int					 idx;			///< index for ini-file keys. In "icon(5) = 7", the index would be 5 and the key only "icon".
	char				*value;			///< the value for this key
	bool				 indexed;		///< if true, the index is output to an ini file
	char				 key[];			///< the key
};

struct ini_section {
	struct ini_section	*next;			///< linked list of init sections consisting of key/value pairs
	struct key_value	*kv;			///< the associated key/value pairs in this section
	char				 name[];		///< the name of the ini section without "[" + "]" (variable length)
};

struct railcom_answer {
	uint8_t				 data[8];		///< the bytes from windows #1 + #2
	int					 count;			///< number of received bytes from both windows
};

struct modeltime {
	int		year;						///< the modeled year (0 .. 4095)
	int		mon;						///< the modeled month (1 .. 12)
	int		mday;						///< the modeled day in month (0 .. 31)
	int		wday;						///< the modeled week day (0 = Monday .. 6 = Sunday)
	int		hour;						///< the virtual hour in 24-hours clock
	int		min;						///< the virtual minute (0 .. 59)
	int 	speedup;					///< a speedup factor (0 = stopped, 1 = real time, .. 63 = very fast)
};

/**
 * Idea of a structure to hold device information for all external
 * bus systems. This is used to report attachment and removal of
 * devices on EasyNet, XPressNet, LocoNet, MCan, ...
 */
struct extDevice {
	enum extbus		bus;				///< the bus (EasyNet, XPressNet, ...)
	enum devtype	tp;					///< the type of device connect to the bux
	int				id;					///< an address ID on the bus if it supports addresses
	uint32_t		serial;				///< a serial number if supported
	char			hwrev[8];			///< a hardware revision if available
	char			swrev[8];			///< a software revision if available
};

#define BIT(x)			(1uL << (x))
#define ON				true
#define OFF				false
#define HIGH			true
#define LOW				false

#define BITS_PER_WORD		32			///< number of bits in an uint32_t
#define INDEX_BITSHIFT		5			///< bit positions to shift to the right to divide an index by BITS_PER_WORD

// this fancy macro is taken from the linux kernel :-)
// ... and probably not used here :-P
#define container_of(ptr, type, member) ({ \
                const typeof( ((type *)0)->member ) *__mptr = (ptr); \
                (type *)( (char *)__mptr - offsetof(type,member) );})

#define DIM(x)		((int)(sizeof(x) / sizeof(x[0])))
#define BCD(x)		((((x) / 10) << 4) | ((x) % 10))	// format a byte to a BCD value

// use optimized builtin versions for of byte swapping for htonl() and cousins
#define htonl(x)		__builtin_bswap32 (x)
#define ntohl(x)		__builtin_bswap32 (x)
#define htons(x)		__builtin_bswap16 (x)
#define ntohs(x)		__builtin_bswap16 (x)

// check if a time has passed the current scheduler time taking wrap around into account
#define time_is_earlier(ref, chk)	(((chk) - (ref)) & (1 << 31))
#define time_check(now, chk)		(((chk) == (now)) || time_is_earlier(now, chk))

#define SEG_A1()		do { GPIOE->BSRR = GPIO_BSRR_BS2 | GPIO_BSRR_BR3; } while (0)
#define SEG_A2()		do { GPIOE->BSRR = GPIO_BSRR_BS3 | GPIO_BSRR_BR2; } while (0)
#define SEG_OFF()		do { GPIOE->BSRR = GPIO_BSRR_BR3 | GPIO_BSRR_BR2; } while (0)
#define ETHLED_ON()		do { GPIOB->BSRR = GPIO_BSRR_BS2; } while (0)
#define ETHLED_OFF()	do { GPIOB->BSRR = GPIO_BSRR_BR2; } while (0)

#define KEY1_PRESSED()	((GPIOH->IDR & GPIO_IDR_ID1) == 0)
#define KEY2_PRESSED()	((GPIOH->IDR & GPIO_IDR_ID0) == 0)
#define BIDIBUS_ACK()	((GPIOD->IDR & GPIO_IDR_ID6) == 0)
#define MB_ISSHORT()	((GPIOG->IDR & GPIO_IDR_ID3) != 0)
#ifdef HW_REV07
#define DCC_ISSHORT()	((GPIOD->IDR & GPIO_IDR_ID12) == 0)		/* actually this was already changed on our boards! */
#define MAINBST_ON()	do { GPIOB->BSRR = GPIO_BSRR_BS1 | GPIO_BSRR_BS0; } while (0)		/* switch main booster ON */
#define MAINBST_OFF()	do { GPIOB->BSRR = GPIO_BSRR_BR1 | GPIO_BSRR_BR0; } while (0)		/* switch main booster OFF */
#define MAINBST_ISON()	((GPIOB->ODR & GPIO_ODR_OD1) != 0)									/* check if main booster and booster supply is ON */
#define MKLNBST_ON()	do { GPIOC->BSRR = GPIO_BSRR_BS7; } while (0)						/* switch märklin booster ON */
#define MKLNBST_OFF()	do { GPIOC->BSRR = GPIO_BSRR_BR7; } while (0)						/* switch märklin booster OFF */
#define MKLNBST_ISON()	((GPIOC->ODR & GPIO_ODR_OD7) != 0)									/* check if märklin booster is ON */
#else
#define DCC_ISSHORT()	((GPIOB->IDR & GPIO_IDR_ID1) == 0)
#define MAINBST_ON()	do { GPIOC->BSRR = GPIO_BSRR_BS7; } while (0)						/* switch main booster and booster supply ON */
#define MAINBST_OFF()	do { GPIOC->BSRR = GPIO_BSRR_BR7; } while (0)						/* switch main booster and booster supply OFF */
#define MAINBST_ISON()	((GPIOC->ODR & GPIO_ODR_OD7) != 0)									/* check if main booster and booster supply is ON */
#define MKLNBST_ON()	do { GPIOB->BSRR = GPIO_BSRR_BS10; } while (0)						/* switch märklin booster ON */
#define MKLNBST_OFF()	do { GPIOB->BSRR = GPIO_BSRR_BR10; } while (0)						/* switch märklin booster OFF */
#define MKLNBST_ISON()	((GPIOB->ODR & GPIO_ODR_OD10) != 0)									/* check if märklin booster is ON */
#endif
#define PRGRELAIS_ON()	do { GPIOG->BSRR = GPIO_BSRR_BS7; } while (0)						/* switch on the Programming track relais */
#define PRGRELAIS_OFF()	do { GPIOG->BSRR = GPIO_BSRR_BR7; } while (0)						/* switch off the Programming track relais */
#define PRGRELAIS_ISON()	((GPIOG->ODR & GPIO_ODR_OD7) != 0)								/* check if the Programming track relais is engaged */
#define BIDIB_ACKON()	do { GPIOD->BSRR = GPIO_BSRR_BS7; } while (0)						/* Send an ACK on BiDiB */
#define BIDIB_ACKOFF()	do { GPIOD->BSRR = GPIO_BSRR_BR7; } while (0)						/* disarm ACK on BiDiB */
#define ESP_RST_ON()	do { GPIOE->BSRR = GPIO_BSRR_BR4; } while (0)						/* aktivate RST of ESP-01 (pull LOW) */
#define ESP_RST_OFF()	do { GPIOE->BSRR = GPIO_BSRR_BS4; } while (0)						/* de-aktivate RST of ESP-01 (pull HIGH) */
#define ESP_GP0_HIGH()	do { GPIOG->BSRR = GPIO_BSRR_BS10; } while (0)						/* let GP0 be pulled HIGH (normal startup) */
#define ESP_GP0_LOW()	do { GPIOG->BSRR = GPIO_BSRR_BR10; } while (0)						/* force GP0 LOW (bootloader startup) */

#define NOKEY			0		///< no key event registered
#define KEY_STOP		1		///< the STOP key was pressed (or released)
#define KEY_GO			2		///< the GO key was pressed (or released)
#define KEY_BIDIB_ACK	3		///< an LOW on BiDiBus ACK for more than 0ms
#define NUMBER_OF_KEYS	3		///< numer of physical keys that need debouncing

// more virtual keys for certain events:
#define MB_SHORT		0x10
#define DCC_SHORT		0x11

#define MAKE(k)			(k)
#define BREAK(k)		((k) | 0x80)
#define ISMAKE(k)		(((k) & 0x80) == 0)
#define ISBREAK(k)		(((k) & 0x80) != 0)
#define KEY(k)			((k) & 0x7F)
typedef uint8_t			keyevent;

// some useful binary definitions
#define ON				true
#define OFF				false
#define HIGH			true
#define LOW				false
#define THROWN			true
#define STRAIGHT		false

/*
 * Prototypes Decoder/reply.c
 */
void reply_callbackHandler (void *pvParameter);

/*
 * Prototypes HW/adc.c
 */
int an_getSupply (void);
int an_getTemperature (void);
int an_getTrackCurrent (void);
int an_getProgCurrent (int samples);
void an_temperaturTest (int newoffs);
void vAnalog (void *pvParameter);
void adc_CCmonitor (int current);
void adc_ACKcurrent (int current, void (*cb)(int));
int adc_getFastCurrent (void);

/*
 * Prototypes HW/audio.c
 */
void vAudioTest (void *pvParameter);

/*
 * Prototypes HW/cache.c
 */
void cache_flushall (void);
void cache_flush (uint32_t adr, int size);
void cache_invalidate (uint32_t adr, int size);

/*
 * Prototypes HW/can.c
 */
void can_handler (void *pvParameter);
/*
 * Prototypes HW/esp01.c
 */
void esp_testthread (void *pvParameter);
void esp_triggerUpdate (void);

/*
 * Prototypes HW/i2c.c
 */
void i2c_init (I2C_TypeDef *i2c);
int i2c_read (I2C_TypeDef *i2c, uint8_t devadr, uint32_t regadr, int reglen, uint8_t *data, int datalen);
int i2c_write (I2C_TypeDef *i2c, uint8_t devadr, uint32_t regadr, int reglen, uint8_t *data, int datalen);

/*
 * Prototypes HW/keys.c
 */
void key_scan (void);
void key_init (void);
keyevent key_getEvent (TickType_t waittime);
void key_resetShort (void);
bool key_isActive (int key);

/*
 * Prototypes HW/nand.c
 */
void nand_init (void *pvParameters);

/*
 * Prototypes HW/rgb.c
 */
void rgb_handler (void *pvParameter);
void rgb_color (int r, int g, int b);
void rgb_swell (void);
void rgb_stop (void);
void rgb_go (void);
void rgb_identify (bool on);
void rgb_overtemp (void);
void rgb_short (void);
void rgb_off (void);

/*
 * Prototypes HW/sdram.c
 */
int sdram_init (void);

/*
 * Prototypes HW/setup.c
 */
void hw_setup (void);

/*
 * Prototypes HW/seven_segment.c
 */
void seg_timer (void);
void seg_decimal (int n, bool dp);
void seg_display (uint8_t left, uint8_t right);
uint8_t seg_getHexDigit (int v);
void seg_stop (void);
void seg_pause (void);
void seg_short (void);
void seg_go (void);
void seg_reboot (void);
void seg_progmode (void);
void seg_testdrive (void);
void seg_factoryReset (void);
void seg_powerfail (void);
void seg_overtemp (void);
void seg_pairing (bool on);
void seg_registerEvents (void);

/*
 * Prototypes HW/spi.c
 */
void tim13_updateIRQ (void);

/*
 * Prototypes HW/tracksupply.c
 */
char *ts_getRanges (void);
int ts_setVoltage (int volt);
void ts_setPtVoltage (int volt);
int ts_getVoltage (void);
int ts_getPtVoltage (void);
int ts_setCurrentMilliAmpere (int current);
int ts_setCurrent (int current);
void ts_setCCMode (int limit);
int ts_getCurrent (void);
int ts_getCurrentMilliAmpere (void);
void ts_setSensitivity (int ms);
int ts_getSensitivity (void);
void ts_setInrush (int ms);
int ts_getInrush (void);
bool ts_voltageLevelReached (void);
void ts_boosteron (bool pt);
void ts_boosteroff (void);
void ts_init (void);
void ts_handler (void);

/*
 * Prototypes Interfaces/BiDiB/bidib.c
 */
void bidib_start (void);

/*
 * Prototypes Interfaces/easynet.c
 */
void easynet (void *pvParameter);

/*
 * Prototypes Interfaces/feedback.c
 */
uint16_t fb_getModuleState (int mod);
uint8_t fb_getHalfModuleState (int hmod);
uint8_t fb_msb2lsb8 (uint8_t b);
uint16_t fb_msb2lsb16 (uint16_t w);
void fb_s88input (int modules, uint16_t *data);
void fb_BitInput (int adr, bool occupy);
void fb_rangeInput (int offset, int len, uint8_t *data);

/*
 * Prototypes Interfaces/loconet.c
 */
void vLocoNet (void *pvParameter);
int ln_dispatchLoco( int adr );
void lnet_setModules (int count);

/*
 * Prototypes Interfaces/mcan.c
 */
void vMCanHandler (void *pvParameter);
void can_setModules (int count);

/*
 * Prototypes Interfaces/p50x.c
 */
int p50x_start (uint16_t port);

/*
 * Prototypes Interfaces/rfid.c
 */
void rfid (void *pvParameter);

/*
 * Prototypes Interfaces/s88.c
 */
void vS88Bus (void *pvParameter);
void s88_setModules (int count);
int s88_getModules(void);
void s88_setFrequency (int hz);
int s88_getFrequency(void);
#ifndef CENTRAL_FEEDBACK
void s88_triggerUpdate (void);
int s88_getCanModules(void);
void s88_genBiDiBnodes (void);
uint16_t s88_getInput(int m);
volatile uint16_t *s88_getInputs();
#endif

/*
 * Prototypes Interfaces/xpressnet.c
 */
void vXpressNet (void *pvParameter);
struct xpn_node *get_nodes(void);

/*
 * Prototypes Sound/player.c
 */
void player (void *pvParameter);
void player_play (const char *fname);
void player_stop (void);
void player_volume (int newvolume);

/*
 * Prototypes System/debug.c
 */
void dbg_send_buffered_messages (void);
void dbg_init(void);
void irqdbg_printf (const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void dbg_puts (const char *str);
void dbg_write (const char *s, int len);
void dbg_putc (const char c);
void dbg_link_cb (struct netif *netif);
void dbg_status_cb (struct netif *netif);

/*
 * Prototypes System/idle.c
 */
void idlefunc (void *pvParameter);

/*
 * Prototypes System/init.c
 */
void vInit(void *pvParameters);
void reboot (void);
void pwrfail (void);

/*
 * Prototypes System/keyhandler.c
 */
void vKeyHandler (void *pvParameter);
bool key_pairing (void);

/*
 * Prototypes System/modeltime.c
 */
void mt_init (void);
void mt_speedup (int factor);
void mt_setdatetime (int year, int mon, int mday, int hour, int min);
void mt_report (void);

/*
 * Prototypes Track/railcom.c
 *
 * (only the thread routine is declared here, for others refer to decoder.h)
 */
void rc_init (void);

/*
 * Prototypes Track/sniffer_m3.c
 *
 * (only the thread routines are declared here, for others refer to decoder.h)
 */
void sniffer (void *pvParameter);

/*
 * Prototypes Utilities/bitset.c
 */
void bs_set (uint32_t *bitset, int bit);
void bs_clear (uint32_t *bitset, int bit);
bool bs_isset (uint32_t *bitset, int bit);
bool bs_isempty (uint32_t *bitset, int bits);
int bc_byte (uint8_t b);
int bc_short (uint16_t s);
int bc_long (uint32_t l);

/*
 * Prototypes Utilities/calendar.c
 */
char *timestamp (TickType_t t);
bool isLeapYear(int year);
int daysInMonth(int year, int mon);
int calc_weekday (int year, int mon, int mday);
const char *weekday (int wday);

/*
 * Prototypes Utilities/cpio.c
 */
int cpio_copyIn (const char *fname, const char *path);

/*
 * Prototypes Utilities/hexdump.c
 */
void hexdump (void *addr, int len);

/*
 * Prototypes Utilities/ini.c
 */
struct ini_section *ini_addEx (struct ini_section *ini, const char *name, int name_len);
struct ini_section *ini_add (struct ini_section *ini, const char *name);
struct ini_section *ini_addSection (struct ini_section **root, const char *name);
struct key_value *ini_addItem (struct ini_section *ini, const char *name, const char *value);
struct key_value *ini_addIntItem (struct ini_section *ini, const char *name, int value);
struct key_value *ini_addBoolItem (struct ini_section *ini, const char *name, bool value);
void ini_free (struct ini_section *ini);
struct ini_section *ini_readFile (const char *fname);
int ini_writeFile (const char *fname, struct ini_section *ini);

/*
 * Prototypes Utilities/keyvalue.c
 */
struct key_value *kv_addEx (struct key_value *kv, const char *key, int key_len, const char *value, int val_len);
struct key_value *kv_add (struct key_value *kv, const char *key, const char *value);
struct key_value *kv_addIndexed (struct key_value *kv, const char *key, int idx, const char *value);
void kv_free (struct key_value *kv);
struct key_value *kv_lookup (struct key_value *kv, char *key);
char *kv_strcpy (struct key_value *kv, char *dest, size_t maxlen);

/*
 * Prototypes Utilities/lists.c
 */
void list_append (void *lst, void *entry);
int list_len (void *lst);
void *list_getIndexed (void *lst, int idx);

/*
 * Prototypes Utilities/minmax.c
 */
int min (int a, int b);
int max (int a, int b);

/*
 * Prototypes Utilities/mutex.c
 */
bool mutex_lock(SemaphoreHandle_t * volatile mutex, TickType_t tout, const char *fn);
void mutex_unlock(SemaphoreHandle_t *mutex);
void mutex_destroy (SemaphoreHandle_t *mutex);

/*
 * Prototypes Utilities/pathutils.c
 */
char *canonical_path(char *buf, const char *cwd, const char *fname);
int ensure_path (const char *fname);

/*
 * Prototypes Utilities/socket.c
 */
int socket_senddata (int sock, const void *data, int len);
int socket_sendstring (int sock, const char *str);
int socket_printf (int sock, const char *fmt, ...);

/*
 * Prototypes Utilities/tcp_server.c
 */
bool tcp_checkSocket (int s);
int tcp_listenSocket (int port, int timeout);
int tcpsrv_startserver (uint16_t port, void (*accept_func)(void *), int stacksize, int prio);

/*
 * Prototypes Utilities/timing.c
 */
TickType_t tim_timeout (int ms);
bool tim_isover (TickType_t to);
bool tim_isoverUnset (TickType_t to);

/*
 * Prototypes Utilities/tmpstring.c
 */
void *tmpbuf (size_t siz);
char *tmp64 (void);
char *tmp256 (void);
char *tmp1k (void);

/*
 * Prototypes Utilities/udpcomm.c
 */
void z21_service (void *pvParameter);

/*
 * Prototypes Utilities/unicode.c
 */
int utf8_bytelen (uint32_t codepoint);
uint32_t utf8_codepoint (const void *s);
int utf8_writeBuffer (uint32_t codepoint, void *buf);
char *utf8_advance (char *s);
char *utf8_strncpy (char *dst, size_t maxbytes, const char *src, size_t len);
char *utf8_iso2utf (char *s);

/*
 * Prototypes WEB/cgi.c
 */
int informBusses(int loco, int interface);

/*
 * Prototypes WEB/ftpd.c
 */
int ftpd_start (void);

/*
 * Prototypes WEB/httpd.c
 */
int httpd_start (void);

/*
 * Prototypes WEB/webupdate.c
 */
int webup_manuals (void);
int webup_update (const char *cpio);

#endif	/* __RB2_H__ */

