/**
 * @file player.c
 *
 * @author Andi
 * @date   01.03.2020
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

#if 0
#include <stdio.h>
#include "rb2.h"
#include "yaffsfs.h"
#include "ogg/ogg.h"
#include "opus.h"

typedef int16_t			sample;	///< samples are 16-bit unsigned per channel

#define OGG_READ_SIZE		4096	///< size of the buffer and the read requests
#define SAMPLE_FREQUENCY	48000	///< 48kHz standard sampling frequency
#define MAX_FRAME_TIME		120		///< maximum frame size of decoded audio is 120ms
#define MAX_CHANNELS		2		///< a maximum of 2 channels can be used (L/R, stereo)
#define PCM_BUFFER_SIZE		(SAMPLE_FREQUENCY * MAX_FRAME_TIME * MAX_CHANNELS / 1000)

enum status {
	AUDIO_IDLE = 0,				///< the player is idle - no sound played
	AUDIO_STOP,					///< immediately stop audio output and close current file
	AUDIO_START,				///< start playing a file
};

enum audiocmd {
	CMD_PLAY,					///< play a file (if currently playing, stop current output and then start over)
	CMD_STOP,					///< stop playing
	CMD_VOLUME,					///< change volume of playback
};

struct playercmd {
	enum audiocmd	cmd;		///< the command
	const char		*file;		///< for CMD_PLAY: the file name to play
	int				volume;		///< for CMD_VOLUME: the new volume
};

static struct oggstream {
	struct oggstream	*next;	///< linked list of streams
	ogg_stream_state	state;	///< the state of this stream
} *streams;

static QueueHandle_t control;
static int volume;
static sample pcm[PCM_BUFFER_SIZE];

static int player_open (const char *fname, ogg_sync_state *oy)
{
	int fd;

	ogg_sync_clear(oy);
	if ((fd = yaffs_open(fname, O_RDONLY, 0)) < 0) {
		fprintf (stderr, "%s(): cannot open file '%s'\n", __func__, fname);
	}

	return fd;
}

static int player_close (int fd, ogg_sync_state *oy)
{
	struct oggstream *tmp;

	if (fd >= 0) yaffs_close(fd);
	ogg_sync_clear(oy);
	while ((tmp = streams) != NULL) {
		streams = streams->next;
		ogg_stream_clear(&tmp->state);
		free (tmp);
	}
	return -1;		// always -1, as this will be the new file descriptor, which means: file is closed
}

static struct oggstream *player_findStream (long serial)
{
	struct oggstream **pp, *os;
	int rc;

	pp = &streams;
	while ((os = *pp) != NULL && os->state.serialno != serial) pp = &os->next;
	if (os) {
//		printf ("%s() found stream #%lx\n", __func__, serial);
		return os;
	}

	if ((os = calloc (1, sizeof(*os))) == NULL) {
		return NULL;
	}
	printf ("%s() new stream #%lx\n", __func__, serial);
	rc = ogg_stream_init(&os->state, serial);
	if (rc) {
		fprintf (stderr, "%s(): ogg_stream_init() failed\n", __func__);
		free (os);
		return NULL;
	}
	*pp = os;
	return os;
}

struct wavheader {
	uint32_t ChunkID;       /* 0 */
	uint32_t FileSize;      /* 4 */
	uint32_t FileFormat;    /* 8 */
	uint32_t SubChunk1ID;   /* 12 */
	uint32_t SubChunk1Size; /* 16 */
	uint16_t AudioFormat;   /* 20 */
	uint16_t NbrChannels;   /* 22 */
	uint32_t SampleRate;    /* 24 */

	uint32_t ByteRate;      /* 28 */
	uint16_t BlockAlign;    /* 32 */
	uint16_t BitPerSample;  /* 34 */
	uint32_t SubChunk2ID;   /* 36 */
	uint32_t SubChunk2Size; /* 40 */
};

static int player_openWav (const char *fname, int channels)
{
	int fd;
	struct wavheader w;
	char *p;

	if ((fd = yaffs_open(fname, O_RDWR | O_CREAT | O_TRUNC, 0666)) < 0) {
		log_error ("%s(); cannot create '%s'\n", __func__, fname);
		return -1;
	}

	p = (char *) &w.ChunkID;
	*p++ = 'R';
	*p++ = 'I';
	*p++ = 'F';
	*p++ = 'F';
	w.FileSize = 0;		// currently not known
	p = (char *) &w.FileFormat;
	*p++ = 'W';
	*p++ = 'A';
	*p++ = 'V';
	*p++ = 'E';
	p = (char *) &w.SubChunk1ID;
	*p++ = 'f';
	*p++ = 'm';
	*p++ = 't';
	*p++ = ' ';
	w.SubChunk1Size = 16;
	w.AudioFormat = 1;		// WAVE_FORMAT_PCM
	w.NbrChannels = channels;
	w.SampleRate = SAMPLE_FREQUENCY;
	w.ByteRate = SAMPLE_FREQUENCY * channels * sizeof(sample);
	w.BlockAlign = channels * sizeof(sample);
	w.BitPerSample = sizeof(sample) * 8;
	p = (char *) &w.SubChunk2ID;
	*p++ = 'd';
	*p++ = 'a';
	*p++ = 't';
	*p++ = 'a';
	w.SubChunk2Size = 0;	// currently not known
	yaffs_write(fd, &w, sizeof(w));

	return fd;
}

static int player_finalizeWav (int fd)
{
	struct wavheader w;
	uint32_t offset;

	if (fd < 0) return -1;

	offset = yaffs_lseek(fd, 0, SEEK_CUR) - 36;
	yaffs_lseek(fd, 0, SEEK_SET);
	yaffs_read(fd, &w, sizeof(w));
	w.FileSize = 36 + offset;
	w.SubChunk2Size = offset;
	yaffs_lseek(fd, 0, SEEK_SET);
	yaffs_write(fd, &w, sizeof(w));
	yaffs_close(fd);

	return -1;		// to simply use fd=player_finalizeWav(fd, n);
}

static void dump (const char *head, uint8_t *buf, int len)
{
	int i;

	printf ("%s ----------------------\n", head);
	for (i = 0; i < len; i += 16) {
		printf ("  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				buf[i + 0], buf[i + 1], buf[i + 2], buf[i + 3], buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7],
				buf[i + 8], buf[i + 9], buf[i + 10], buf[i + 11], buf[i + 12], buf[i + 13], buf[i + 14], buf[i + 15]);
	}
}

void player (void *pvParameter)
{
	BaseType_t rc;
	struct playercmd pc;
	struct oggstream *os;
	ogg_sync_state state;
	ogg_page page;
	ogg_packet packet;
	OpusDecoder *dec = NULL;
	int fd = -1, fout = -1;
	int serial, channels, samples;
	uint16_t preskip;
	uint32_t samplerate;
	long rdsz, fsize;

	(void) pvParameter;

	if ((control = xQueueCreate(8, sizeof(struct playercmd))) == NULL) {
		fprintf (stderr, "%s(): cannot create queue - give up", __func__);
		vTaskDelete(NULL);
	}

	volume = 100;		// $$$$ let's see where we get here
	ogg_sync_init(&state);
	fsize = 0;
	preskip = 0;
	channels = 2;

	for (;;) {
		if (fd < 0) {
			if ((rc = xQueueReceive(control, &pc, portMAX_DELAY)) == pdTRUE) {
				switch (pc.cmd) {
					case CMD_PLAY:		// start playing a new file
						fd = player_open(pc.file, &state);
						fsize = 0;
						break;
					case CMD_STOP:
						// nothing to do - we are not playing a file
						break;
					case CMD_VOLUME:
						volume = pc.volume;
						break;
				}
			}
		} else {
			if ((rc = xQueueReceive(control, &pc, 0)) == pdTRUE) {
				switch (pc.cmd) {
					case CMD_PLAY:		// stop current playback and start a new file
						fd = player_close(fd, &state);
						fout = player_finalizeWav(fout);
						fd = player_open(pc.file, &state);
						fsize = 0;
						break;
					case CMD_STOP:
						fd = player_close(fd, &state);
						fout = player_finalizeWav(fout);
						if (dec) free(dec);
						dec = NULL;
						continue;		// jump to beginning of loop
					case CMD_VOLUME:
						volume = pc.volume;
						break;
				}
			}

			/* read from file and post it to the sync buffer of ogg library */
			while (ogg_sync_pageout(&state, &page) != 1) {
				char *buf = ogg_sync_buffer(&state, OGG_READ_SIZE);
				if (!buf) {
					fprintf(stderr, "%s() Error getting sync buffer\n", __func__);
					fd = player_close(fd, &state);
					fout = player_finalizeWav(fout);
					if (dec) free(dec);
					dec = NULL;
					break;
				}
				rdsz = yaffs_read(fd, buf, OGG_READ_SIZE);
				if (rdsz <= 0) {
					fd = player_close(fd, &state);
					fout = player_finalizeWav(fout);
					if (dec) free(dec);
					dec = NULL;
					break;
				}
				ogg_sync_wrote(&state, rdsz);
			}
			if (fd < 0) continue;		// restart loop
			serial = ogg_page_serialno(&page);
			os = player_findStream(serial);
			if (os) {
				ogg_stream_pagein(&os->state, &page);
				while (ogg_stream_packetout(&os->state, &packet) == 1) {
					if (packet.packetno == 0 && packet.bytes >= 19) {		// this is the packet with ID header
						channels = packet.packet[9];
						log_msg (LOG_INFO, "%s(): %8.8s V%d %d channels\n", __func__, packet.packet, packet.packet[8], channels);
						preskip = packet.packet[10] | (packet.packet[11] << 8);
						samplerate = packet.packet[12] | (packet.packet[13] << 8) | (packet.packet[14] << 16) | (packet.packet[15] << 24);
						log_msg (LOG_INFO, "%s(): pre-skip %u, sample rate %luHz\n", __func__, preskip, samplerate);
						dec = opus_decoder_create(SAMPLE_FREQUENCY, packet.packet[9], NULL);
						fout = player_openWav("/sound.wav", channels);
						opus_decoder_ctl(dec, OPUS_SET_GAIN(10));
					}
					if (packet.packetno >= 2) {
						samples = opus_decode(dec, packet.packet, packet.bytes, pcm, PCM_BUFFER_SIZE / channels, 1);
//						log_msg (LOG_INFO, "%s(): decoded %d samples\n", __func__, samples);
						if (packet.packetno >= 2 && packet.packetno <= 6) {
							dump ("OpusRAW", packet.packet, packet.bytes);
							dump ("PCM", (uint8_t *) pcm, samples * channels * sizeof(sample));
						}
//						if (fout >= 0 && fsize < 20000) yaffs_write(fout, pcm, samples * channels * sizeof(sample));
					}
					fsize += packet.bytes;
//					printf ("%s(): #%lx Packet %ld with %ld bytes\n", __func__, os->state.serialno, (unsigned long) packet.packetno, packet.bytes);
					/* forward packet to decoder */
				}
//				printf ("%s(): total up to here: %ld\n", __func__, fsize);
			}
		}
	}
}

void player_play (const char *fname)
{
	struct playercmd pc;
	char *tmp;

	if (control == NULL) return;	// queue not (yet) created - ignore request

	pc.cmd = CMD_PLAY;
	tmp = tmp256();
	strcpy (tmp, fname);
	pc.file = tmp;
	pc.volume = 0;		// ignored

	log_msg(LOG_INFO, "%s() playing %s\n", __func__, fname);
	xQueueSendToBack(control, &pc, 200);
}

void player_stop (void)
{
	struct playercmd pc;

	if (control == NULL) return;	// queue not (yet) created - ignore request

	pc.cmd = CMD_STOP;
	pc.file = NULL;		// ignored
	pc.volume = 0;		// ignored

	log_msg(LOG_INFO, "%s() stop playing\n", __func__);
	xQueueSendToBack(control, &pc, 200);
}

void player_volume (int newvolume)
{
	struct playercmd pc;

	if (control == NULL) return;	// queue not (yet) created - ignore request

	pc.cmd = CMD_VOLUME;
	pc.file = NULL;		// ignored
	pc.volume = newvolume;

	log_msg(LOG_INFO, "%s() volume %d\n", __func__, newvolume);
	xQueueSendToBack(control, &pc, 200);
}

#endif
