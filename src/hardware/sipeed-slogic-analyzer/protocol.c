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

#include <config.h>

#include "protocol.h"

/*
 * Retire a transfer that will not be resubmitted. Called only from the
 * transfer's own callback, which libusb explicitly permits: it snapshots
 * transfer->flags before invoking the callback for exactly this reason.
 * Clearing the slot under the lock first means handle_events() can no
 * longer reach the transfer, so the free itself needs no lock.
 */
/* Slot of a transfer in devc->transfers[], or -1. For logging only. */
static int transfer_idx(struct dev_context *devc,
			const struct libusb_transfer *transfer)
{
	int idx = -1;

	g_mutex_lock(&devc->transfers_mutex);
	for (int i = 0; i < NUM_MAX_TRANSFERS; i++) {
		if (devc->transfers[i] == transfer) {
			idx = i;
			break;
		}
	}
	g_mutex_unlock(&devc->transfers_mutex);

	return idx;
}

static void release_transfer(struct dev_context *devc,
			     struct libusb_transfer *transfer)
{
	g_mutex_lock(&devc->transfers_mutex);
	for (size_t i = 0; i < NUM_MAX_TRANSFERS; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}
	g_atomic_int_dec_and_test(&devc->num_transfers_used);
	g_mutex_unlock(&devc->transfers_mutex);

	libusb_free_transfer(transfer);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	int ret;
	const struct sr_dev_inst *sdi;
	struct dev_context *devc;

	sdi = transfer->user_data;
	if (!sdi)
		return;
	devc = sdi->priv;

	int64_t transfers_reached_time_now = g_get_monotonic_time();
	int64_t transfers_reached_duration =
		transfers_reached_time_now -
		devc->transfers_reached_time_latest;
	int64_t transfers_all_duration =
		transfers_reached_time_now - devc->transfers_reached_time_start;

	double expected_rate = devc->expected_rate_MBps;
	double expected_transfer_duration =
		(double)devc->per_transfer_nbytes / expected_rate;

	/*
	 * Snapshot the trigger state once so every decision below in this
	 * callback is made against the same value, even if the session
	 * thread flips it midway.
	 */
	gboolean trigger_fired = g_atomic_int_get(&devc->trigger_fired);

	/*
	 * The transfer stays counted until it is actually freed below, so
	 * the count can never read zero while a transfer is still in use.
	 */
	gboolean resubmitted = FALSE;

	devc->num_transfers_completed += 1;
	sr_spew("[%zu] Transfer #%d status: %d(%s).",
		devc->num_transfers_completed, transfer_idx(devc, transfer),
		transfer->status, libusb_error_name(transfer->status));
	switch (transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED: /* normal case */
	case LIBUSB_TRANSFER_TIMED_OUT: /* may have received some data */
	{
		devc->transfers_reached_time_latest =
			transfers_reached_time_now;

		devc->transfers_reached_nbytes_latest = transfer->actual_length;
		devc->transfers_reached_nbytes +=
			devc->transfers_reached_nbytes_latest;

		// remove 32bit for hardware bug workaround
		if (!devc->head_dropped) {
			const int drop_bytes = 4;
			if (transfer->actual_length >= drop_bytes) {
				transfer->actual_length -= drop_bytes;
				memmove(transfer->buffer, transfer->buffer + drop_bytes,
					transfer->actual_length);
				devc->head_dropped = TRUE;
			}
			/* Too short: retry on the next transfer. */
		}

		/*
		 * samples_need_nbytes is 0 when running continuously. There
		 * is no limit to clamp against then, and every byte is kept.
		 */
		if (trigger_fired && devc->samples_need_nbytes) {
			uint64_t remaining =
				devc->samples_need_nbytes >
						devc->samples_got_nbytes ?
					devc->samples_need_nbytes -
						devc->samples_got_nbytes :
					0;
			if ((uint64_t)transfer->actual_length > remaining)
				transfer->actual_length = remaining;
		}
		devc->samples_got_nbytes += transfer->actual_length;

		/* No meaningful percentage without a limit to divide by. */
		double progress_pct = devc->samples_need_nbytes ?
					      100. * devc->samples_got_nbytes /
						      devc->samples_need_nbytes :
					      0.;

		sr_dbg("[%zu] Got %" PRIu64 "/%" PRIu64
		       "(%.2f%%) => speed: %.2fMBps, %.2fMBps(avg), %.0fMBps(exp) => "
		       "+%.3f=%.3fms.",
		       devc->num_transfers_completed, devc->samples_got_nbytes,
		       devc->samples_need_nbytes, progress_pct,
		       (double)devc->transfers_reached_nbytes_latest /
			       transfers_reached_duration,
		       (double)devc->transfers_reached_nbytes /
			       transfers_all_duration,
			   expected_rate,
		       (double)transfers_reached_duration / SR_KHZ(1),
		       (double)transfers_all_duration / SR_KHZ(1));

		/* TODO: move out submit to ensure continuous transfers */
		if (devc->raw_data_queue) {
			uint8_t *d = transfer->buffer;
			size_t len = transfer->actual_length;
			// sr_dbg("HEAD: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x
			// %02x %02x %02x %02x %02x", 	d[0], d[1], d[2], d[3], d[4], d[5], d[6],
			// d[7], d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15]);
			// devc->model->submit_raw_data(d, len, sdi);

			uint8_t *ptr = malloc(devc->per_transfer_nbytes);
			if (!ptr) {
				sr_err("Failed to allocate memory: %" PRIu64
				       " bytes!",
				       devc->per_transfer_nbytes);
				g_atomic_int_set(&devc->acq_aborted, 1);
				break;
			}
			transfer->buffer = ptr;
			GByteArray *array = g_byte_array_new_take(d, len);
			g_async_queue_push(devc->raw_data_queue, array);
		}

		/*
		 * Until the trigger fires nothing counts towards the limit,
		 * so keep the budget full and the transfers flowing.
		 */
		if (!trigger_fired)
			devc->samples_got_nbytes = 0;

		uint64_t others_pending =
			(uint64_t)g_atomic_int_get(&devc->num_transfers_used) - 1;
		/*
		 * Continuous mode has no budget to run down, so the transfer
		 * is always put back in flight.
		 */
		if (!devc->samples_need_nbytes ||
		    devc->samples_got_nbytes +
				    others_pending * devc->per_transfer_nbytes <
			    devc->samples_need_nbytes) {
			transfer->actual_length = 0;
			transfer->timeout = (TRANSFERS_DURATION_TOLERANCE + 1) *
					    devc->per_transfer_duration *
					    (others_pending + 3);
			ret = libusb_submit_transfer(transfer);
			if (ret) {
				sr_dbg("Failed to submit transfer: %s",
				       libusb_error_name(ret));
			} else {
				sr_spew("Resubmit transfer: %p", transfer);
				resubmitted = TRUE;
			}
		}
	} break;

	case LIBUSB_TRANSFER_OVERFLOW:
	case LIBUSB_TRANSFER_STALL:
	case LIBUSB_TRANSFER_NO_DEVICE:
	default:
		g_atomic_int_set(&devc->acq_aborted, 1);
		break;
	}

	double actual_rate = (double)devc->transfers_reached_nbytes_latest / transfers_reached_duration;
	double average_rate = (double)devc->transfers_reached_nbytes / transfers_all_duration;
	if (devc->num_transfers_completed > 1 &&
	    ((double)transfers_reached_duration >
		    (TRANSFERS_DURATION_TOLERANCE + 1) * expected_transfer_duration ||
		actual_rate < expected_rate * (1.0 - TRANSFERS_DURATION_TOLERANCE) ||
		average_rate < expected_rate * 0.95
	    )
	) {
		devc->timeout_count += 1;
	} else {
		devc->timeout_count = 0;
	}

	if (devc->timeout_count >= devc->timeout_count_limit) {
		sr_err("Transfer timeout: duration %.3fms (limit %.3fms), "
			"rate %.2fMBps (minimum %.2fMBps), average %.2fMBps "
			"(minimum %.2fMBps), %" G_GUINT64_FORMAT
			" consecutive slow transfers.",
				(double)transfers_reached_duration / SR_KHZ(1),
				(TRANSFERS_DURATION_TOLERANCE + 1) *
					expected_transfer_duration / SR_KHZ(1),
				actual_rate,
				expected_rate * (1.0 - TRANSFERS_DURATION_TOLERANCE),
				average_rate, expected_rate * 0.95,
				devc->timeout_count);
		g_atomic_int_set(&devc->acq_aborted, 1);
	}

	/* Last use of transfer: it is dangling past this point. */
	if (!resubmitted)
		release_transfer(devc, transfer);

	if (g_atomic_int_get(&devc->num_transfers_used) == 0)
		g_atomic_int_set(&devc->acq_aborted, 1);
};

static int handle_events(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;

	// sr_spew("handle_events enter");

	if (g_atomic_int_get(&devc->acq_aborted)) {
		if (g_atomic_int_get(&devc->num_transfers_used)) {
			/*
			 * Only cancel here. Each transfer is freed by its own
			 * callback once it completes, so the pointers stay
			 * valid for as long as the lock is held.
			 */
			g_mutex_lock(&devc->transfers_mutex);
			for (size_t i = 0; i < NUM_MAX_TRANSFERS; ++i) {
				struct libusb_transfer *transfer =
					devc->transfers[i];
				if (transfer) {
					libusb_cancel_transfer(transfer);
				}
			}
			g_mutex_unlock(&devc->transfers_mutex);
		} else {
			/* Every transfer has been freed by its callback. */
			if (!devc->remote_stopped) {
				if ((devc->model->operation.remote_stop(sdi)) < 0) {
					sr_err("Unhandled `CMD_STOP`");
				}
				devc->remote_stopped = TRUE;
			}
			{
				if (!g_async_queue_length(
					   devc->raw_data_queue)) {
					sr_dbg("Freed all transfers.");
					g_async_queue_unref(devc->raw_data_queue);
					devc->raw_data_queue = NULL;

					if (devc->stl) {
						soft_trigger_logic_free(devc->stl);
						devc->stl = NULL;
						g_atomic_int_set(&devc->trigger_fired,
								 FALSE);
					}
				}
			}
		}
	}

	if (!devc->raw_data_queue) {
		sr_info("Bulk in %" PRIu64 "/%" PRIu64
			" bytes with %zu transfers.",
			devc->samples_got_nbytes, devc->samples_need_nbytes,
			devc->num_transfers_completed);
		std_session_send_df_end(sdi);
		sr_session_source_remove(sdi->session,
					 -1 * (size_t)drvc->sr_ctx->libusb_ctx);
	} else if (g_async_queue_length(devc->raw_data_queue)) {
		GByteArray *array = g_async_queue_try_pop(devc->raw_data_queue);
		if (array != NULL) {
			if (g_atomic_int_get(&devc->trigger_fired)) {
				size_t len = array->len;

				/*
				 * Transfers that were queued before the
				 * trigger fired never passed the length clamp
				 * in receive_transfer(), which only applies
				 * once trigger_fired is set. Enforce the
				 * sample limit here as well, or that backlog
				 * gets sent to the session in full.
				 * samples_need_nbytes is 0 when running
				 * continuously, i.e. without a limit.
				 */
				if (devc->samples_need_nbytes) {
					uint64_t remaining =
						devc->samples_need_nbytes >
								devc->samples_sent_nbytes ?
							devc->samples_need_nbytes -
								devc->samples_sent_nbytes :
							0;
					if (len > remaining)
						len = remaining;
				}

				if (len) {
					devc->model->submit_raw_data(
						array->data, len, sdi);
					devc->samples_sent_nbytes += len;
				}
			} else if (devc->stl) {
				/* Returns bytes sent, or -1 if not triggered yet. */
				if (slogic_soft_trigger_raw_data(array->data,
								 array->len,
								 sdi) >= 0)
					g_atomic_int_set(&devc->trigger_fired,
							 TRUE);
			}
			g_byte_array_unref(array);

			if (devc->samples_need_nbytes &&
			    devc->samples_sent_nbytes >=
				    devc->samples_need_nbytes)
				g_atomic_int_set(&devc->acq_aborted, 1);
		}
	}

	return TRUE;
}

// to find out the maixmum size of ONE transfer
static int train_bulk_in_transfer(struct dev_context *devc,
				  libusb_device_handle *dev_handle)
{
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		sr_err("Failed to allocate libusb transfer!");
		return SR_ERR_IO;
	}

	uint64_t sr = devc->cur_samplerate;
	uint64_t ch = devc->cur_samplechannel;
	uint64_t bps = sr * ch;
	uint64_t Bps = bps / 8;
	uint64_t BpMs = Bps / SR_KHZ(1);

	devc->expected_rate_MBps = BpMs/SR_KHZ(1);

	uint64_t cur_transfer_duration = 250 /* ms */;
	uint64_t try_transfer_nbytes = cur_transfer_duration * BpMs /* bytes */;

	const uint64_t ALIGN_SIZE = 32 * 1024; /* 32kiB */
	do {
		// Align up
		try_transfer_nbytes = (try_transfer_nbytes + (ALIGN_SIZE - 1)) &
				      ~(ALIGN_SIZE - 1);

		uint8_t *transfer_buffer = malloc(try_transfer_nbytes);
		if (!transfer_buffer) {
			sr_dbg("Failed to allocate memory: %" PRIu64
			       " bytes! Half it.",
			       try_transfer_nbytes);
			try_transfer_nbytes >>= 1;
			continue;
		}

		cur_transfer_duration = try_transfer_nbytes / BpMs;
		sr_dbg("Train: receive %" PRIu64 " bytes per %" PRIu64 "ms...",
		       try_transfer_nbytes, cur_transfer_duration);

		libusb_fill_bulk_transfer(transfer, dev_handle,
					  devc->model->ep_in, transfer_buffer,
					  try_transfer_nbytes, NULL, NULL, 0);
		transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
		transfer->flags |= LIBUSB_TRANSFER_FREE_TRANSFER;
		int ret = libusb_submit_transfer(transfer);
		if (ret) {
			sr_dbg("Failed to submit transfer: %s!",
			       libusb_error_name(ret));
			if (ret == LIBUSB_ERROR_NO_MEM) {
				free(transfer->buffer);
				sr_dbg("Half it and try again.");
				try_transfer_nbytes >>= 1;
				continue;
			} else {
				libusb_free_transfer(transfer);
				return SR_ERR_IO;
			}
		}

		ret = libusb_cancel_transfer(transfer);
		if (ret) {
			sr_dbg("Failed to cancel transfer: %s!",
			       libusb_error_name(ret));
		}

		try_transfer_nbytes >>=
			2; // At least 4 transfets can be pending.
		break;
	} while (try_transfer_nbytes >
		 ALIGN_SIZE); // 32kiB > 125ms * 1MHZ * 2ch

	cur_transfer_duration = try_transfer_nbytes / BpMs;
	sr_dbg("Choose: receive %" PRIu64 " bytes per %" PRIu64 "ms :)",
	       try_transfer_nbytes, cur_transfer_duration);

	// Assign
	devc->per_transfer_duration = cur_transfer_duration;
	devc->per_transfer_nbytes = try_transfer_nbytes;

	return SR_OK;
}

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;

	int ret;

	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;
	usb = sdi->conn;

	if ((ret = devc->model->operation.remote_stop(sdi)) < 0) {
		sr_err("Unhandled `CMD_STOP`");
		return ret;
	}

	devc->samples_got_nbytes = 0;
	devc->samples_sent_nbytes = 0;
	devc->samples_need_nbytes =
		devc->cur_limit_samples * devc->cur_samplechannel / 8;
	sr_info("Need %" PRIu64 "x %dch@%" PRIu64 "MHz in %" PRIu64 "ms.",
		devc->cur_limit_samples, devc->cur_samplechannel,
		devc->cur_samplerate / SR_MHZ(1),
		1000 * devc->cur_limit_samples / devc->cur_samplerate);

	if ((ret = train_bulk_in_transfer(devc, usb->devhdl)) != SR_OK) {
		sr_err("Failed to train bulk_in_transfer!`");
		return ret;
	}

	/*
	 * Resetting the state below is only safe because no callback from a
	 * previous acquisition can still be in flight: handle_events() sends
	 * SR_DF_END and removes the session source only once
	 * num_transfers_used has reached 0, i.e. after every transfer has
	 * been freed by its own callback. The session will not start another
	 * acquisition before SR_DF_END.
	 */
	g_atomic_int_set(&devc->acq_aborted, 0);
	g_atomic_int_set(&devc->num_transfers_used, 0);
	devc->num_transfers_completed = 0;
	memset(devc->transfers, 0, sizeof(devc->transfers));
	devc->transfers_reached_nbytes = 0;
	devc->timeout_count = 0;
	devc->head_dropped = FALSE;
	devc->remote_stopped = FALSE;
	/*
	 * Establish the baseline before anything is submitted. Setting it
	 * after remote_run() left callbacks reading whatever the previous
	 * acquisition happened to leave behind.
	 */
	devc->transfers_reached_time_start = g_get_monotonic_time();
	devc->transfers_reached_time_latest = devc->transfers_reached_time_start;
	devc->raw_data_queue = g_async_queue_new();

	if (!devc->raw_data_queue) {
		sr_err("New g_async_queue failed, can't handle data anymore!");
		return SR_ERR_MALLOC;
	}

	/*
	 * samples_need_nbytes is 0 in continuous mode. Queue the full set of
	 * transfers then: without this the loop below submits nothing and the
	 * acquisition fails with SR_ERR_IO before a single byte is read.
	 */
	while (TRUE) {
		/*
		 * Read the count once per iteration: a transfer that fails
		 * early can retire itself from the libusb event thread while
		 * this loop is still submitting the others.
		 */
		int used = g_atomic_int_get(&devc->num_transfers_used);

		if (used >= NUM_MAX_TRANSFERS)
			break;
		if (devc->samples_need_nbytes &&
		    devc->samples_got_nbytes +
				    (uint64_t)used * devc->per_transfer_nbytes >=
			    devc->samples_need_nbytes)
			break;

		uint8_t *dev_buf = malloc(devc->per_transfer_nbytes);
		if (!dev_buf) {
			sr_dbg("Failed to allocate memory[%d]", used);
			break;
		}

		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			sr_dbg("Failed to allocate transfer[%d]", used);
			free(dev_buf);
			break;
		}

		libusb_fill_bulk_transfer(
			transfer, usb->devhdl, devc->model->ep_in, dev_buf,
			devc->per_transfer_nbytes, receive_transfer,
			(void *)sdi,
			(TRANSFERS_DURATION_TOLERANCE + 1) *
				devc->per_transfer_duration * (used + 2));
		transfer->actual_length = 0;

		transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;

		/*
		 * Publish the slot before submitting. The callback can run as
		 * soon as libusb_submit_transfer() is called and has to find
		 * the transfer here in order to retire it. The lock matters
		 * even though no transfer occupies this slot yet: the libusb
		 * event thread scans the whole array in release_transfer(),
		 * so an unlocked store here would race with that scan.
		 */
		g_mutex_lock(&devc->transfers_mutex);
		devc->transfers[used] = transfer;
		g_atomic_int_inc(&devc->num_transfers_used);
		g_mutex_unlock(&devc->transfers_mutex);

		ret = libusb_submit_transfer(transfer);
		if (ret) {
			sr_dbg("Failed to submit transfer[%d]: %s.", used,
			       libusb_error_name(ret));
			/* Never submitted, so no callback can be pending. */
			g_mutex_lock(&devc->transfers_mutex);
			devc->transfers[used] = NULL;
			g_atomic_int_dec_and_test(&devc->num_transfers_used);
			g_mutex_unlock(&devc->transfers_mutex);
			libusb_free_transfer(transfer);
			break;
		}
	}

	int num_submitted = g_atomic_int_get(&devc->num_transfers_used);
	devc->timeout_count_limit = num_submitted;
	sr_dbg("Submited %d transfers", num_submitted);

	if (!num_submitted) {
		return SR_ERR_IO;
	}

	std_session_send_df_header(sdi);
	std_session_send_df_frame_begin(sdi);

	sr_session_source_add(sdi->session,
			      -1 * (size_t)drvc->sr_ctx->libusb_ctx, 0,
			      (devc->per_transfer_duration / 2) ?: 1,
			      handle_events, (void *)sdi);

	g_atomic_int_set(&devc->trigger_fired, TRUE);

	struct sr_trigger *trigger = NULL;
	/* Setup triggers */
	if ((trigger = sr_session_trigger_get(sdi->session))) {
		int pre_trigger_samples = 0;
		if (devc->cur_limit_samples > 0)
			pre_trigger_samples = (devc->capture_ratio * devc->cur_limit_samples) / 100;
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		g_atomic_int_set(&devc->trigger_fired, FALSE);
	}

	if ((ret = devc->model->operation.remote_run(sdi)) < 0) {
		sr_err("Unhandled `CMD_RUN`");
		/* Signature is fixed by the dev_acquisition_stop callback. */
		sipeed_slogic_acquisition_stop((struct sr_dev_inst *)sdi);
		return ret;
	}

	return SR_OK;
}

SR_PRIV int sipeed_slogic_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	g_atomic_int_set(&devc->acq_aborted, 1);

	return SR_OK;
}
