/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2023-2025 Shenzhen Sipeed Technology Co., Ltd.
 * (深圳市矽速科技有限公司) <support@sipeed.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_SIPEED_SLOGIC_ANALYZER_PROTOCOL_H
#define LIBSIGROK_HARDWARE_SIPEED_SLOGIC_ANALYZER_PROTOCOL_H

#include <glib.h>
#include <libusb.h>
#include <stdint.h>

#include <libsigrok/libsigrok.h>

#include "libsigrok-internal.h"

#define LOG_PREFIX "sipeed-slogic-analyzer"

#define USB_VID_SIPEED UINT16_C(0x359f)
#define NUM_MAX_TRANSFERS 16
#define TRANSFERS_DURATION_TOLERANCE 0.3f

enum {
	PATTERN_MODE_NORMAL,
	PATTERN_MODE_TEST_HARDWARE_USB_MAX_SPEED,
	PATTERN_MODE_TEST_HARDWARE_EMU_DATA,
};

struct slogic_model {
	const char *name;
	const uint16_t pid;
	const uint8_t ep_in;
	const uint64_t max_bandwidth; // limit by hardware
	const int32_t *samplechannel_table;
	const uint64_t samplechannel_table_size;
	const uint64_t *limit_samplerate_table;
	const uint64_t *samplerate_table;
	const uint64_t samplerate_table_size;
	const struct {
		int (*remote_reset)(const struct sr_dev_inst *sdi);
		int (*remote_run)(const struct sr_dev_inst *sdi);
		int (*remote_stop)(const struct sr_dev_inst *sdi);
		/* NULL on models without the built-in test patterns. */
		int (*remote_test_mode)(const struct sr_dev_inst *sdi,
					uint32_t mode);
	} operation;
	void (*submit_raw_data)(void *data, size_t len,
				const struct sr_dev_inst *sdi);
};

struct dev_context {
	const struct slogic_model *model;

	struct sr_channel_group *digital_group;

	struct {
		uint64_t limit_samplerate;
		int32_t limit_samplechannel;
	};

	struct {
		uint64_t cur_limit_samples;
		/*
		 * req_samplerate is what the frontend last asked for;
		 * cur_samplerate is that clamped to the ceiling for the channel
		 * count currently selected. Both are needed because the two
		 * settings arrive in an order the driver does not control:
		 * sigrok-cli applies --config before it enables channels, so
		 * clamping the request and forgetting it pinned the rate to the
		 * 16-channel ceiling even for a capture that ends up 4-channel.
		 * Keeping the request lets the ceiling be re-applied whenever
		 * the channel count moves.
		 */
		uint64_t req_samplerate;
		uint64_t cur_samplerate;
		int32_t cur_samplechannel;
		int64_t cur_pattern_mode_idx;
		uint32_t expected_rate_MBps;
	}; // configuration

	/*
	 * Acquisition runs across two threads: receive_transfer() is called
	 * on the libusb event thread, handle_events() and the soft trigger
	 * helper it calls run on the session thread. Fields below are marked
	 * with the thread that owns them. Only acq_aborted, trigger_fired and
	 * num_transfers_used are shared, and those are accessed through
	 * g_atomic_int_*(). Everything else must stay single-owner.
	 */
	struct {
		GThread *libusb_event_thread;
		gint libusb_event_thread_run; /* Shared, atomic. */

		enum libusb_speed speed;

		/* Set up before the threads start, read-only afterwards. */
		uint64_t samples_need_nbytes;
		uint64_t per_transfer_duration; /* unit: ms */
		uint64_t per_transfer_nbytes;
		size_t timeout_count_limit;

		/*
		 * Bytes accepted off the wire. Drives transfer flow control
		 * only. Owned by the libusb event thread.
		 */
		uint64_t samples_got_nbytes;

		/*
		 * Bytes actually passed on to the session, which is what the
		 * sample limit is enforced against. Owned by the session
		 * thread.
		 */
		uint64_t samples_sent_nbytes;

		/* libusb event thread. */
		size_t num_transfers_completed;
		uint64_t transfers_reached_nbytes; /* real received bytes in all */
		uint64_t transfers_reached_nbytes_latest; /* real received bytes this transfer */
		int64_t transfers_reached_time_start;
		int64_t transfers_reached_time_latest;
		uint64_t timeout_count;
		/* Whether the hardware's 4 junk head bytes were dropped. */
		gboolean head_dropped;

		/* Shared, atomic. */
		gint num_transfers_used;

		/*
		 * A transfer is freed by its own callback, on the libusb
		 * event thread, while handle_events() cancels transfers from
		 * the session thread. Both reach them through this array, so
		 * it needs a lock: without one the session thread can cancel
		 * a transfer that was just freed.
		 */
		GMutex transfers_mutex;
		struct libusb_transfer *transfers[NUM_MAX_TRANSFERS];

		GAsyncQueue *raw_data_queue; /* Thread safe by itself. */
	}; // usb

	/* Session thread. */
	gboolean remote_stopped;

	/* Shared, atomic. */
	gint acq_aborted;
	gint trigger_fired;

	/* Triggers. Session thread. */
	uint64_t capture_ratio;
	struct soft_trigger_logic *stl;

	double voltage_threshold[2];
};

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int sipeed_slogic_acquisition_stop(struct sr_dev_inst *sdi);
/* Returns the number of bytes sent to the session, or -1 if not triggered. */
SR_PRIV int slogic_soft_trigger_raw_data(void *data, size_t len,
					 const struct sr_dev_inst *sdi);

#endif
