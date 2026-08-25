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

static int slogic16U3_remote_test_mode(const struct sr_dev_inst *sdi, uint32_t mode);

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_LOGIC_CHANNELS | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST
};

/* Same, minus SR_CONF_PATTERN_MODE, for models without test patterns. */
static const uint32_t devopts_no_patterns[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_LOGIC_CHANNELS | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST
};

static const uint64_t samplerates_slogiccombo8[] = {
	/**
	 * SLogic Combo 8 (USBHS 480Mbps bw: 40MB/s)
	 *  160M = 2^5*5^1  M
	*/

	SR_MHZ(1),
	SR_MHZ(2),
	SR_MHZ(4),
	SR_MHZ(5),
	SR_MHZ(8),
	SR_MHZ(10),
	SR_MHZ(16),
	SR_MHZ(20),
	SR_MHZ(32),
	/* x 8ch */
	SR_MHZ(40),
	/* x 4ch */
	SR_MHZ(80),
	/* x 2ch */
	SR_MHZ(160),
};

static const int32_t samplechannels_slogiccombo8[] = { 2, 4, 8 };
static const uint64_t limit_samplerates_slogiccombo8[] = { SR_MHZ(160), SR_MHZ(80), SR_MHZ(40) };

static const uint64_t samplerates_slogic16u3[] = {
	/**
	 * SLogic 16U3 (USBSS 5Gbps bw: 400MB/s)
	 *  800M = 2^5*5^2  M
	 * --1200M = 2^4*3^1*5^2  M
	 * --1500M = 2^2*3^1*5^3  M
	 * --1600M = 2^6    *5^2  M
	*/

	// SR_MHZ(1),
	// SR_MHZ(2),
	// SR_MHZ(4),
	SR_MHZ(5),
	SR_MHZ(8),
	SR_MHZ(10),
	// SR_MHZ(15),
	SR_MHZ(16),
	SR_MHZ(20),
	// SR_MHZ(24),
	SR_MHZ(25),
	// SR_MHZ(30),
	SR_MHZ(32),
	SR_MHZ(40),
	// SR_MHZ(48),
	SR_MHZ(50),
	// SR_MHZ(60),
	SR_MHZ(80),
	SR_MHZ(100),
	// SR_MHZ(125),
	// SR_MHZ(150),
	SR_MHZ(160),
	/* x 16ch */
	SR_MHZ(200),
	/* x 8ch */
	// SR_MHZ(300),
	SR_MHZ(400),
	/* x 4ch */
	// SR_MHZ(500),
	// SR_MHZ(600),
	// SR_MHZ(750),
	SR_MHZ(800),
	/* x 2ch */
	// SR_MHZ(1200),
	// SR_MHZ(1500),
};

static const int32_t samplechannels_slogic16u3[] = { /*2, */4, 8, 16 };
static const uint64_t limit_samplerates_slogic16u3[] =
#ifdef _WIN32
	{ /*SR_MHZ(1500), */SR_MHZ(400), SR_MHZ(200), SR_MHZ(100) };
#else
	{ /*SR_MHZ(1500), */SR_MHZ(800), SR_MHZ(400), SR_MHZ(200) };
#endif

static const uint64_t samplerates_slogic32u3[] = {
	SR_MHZ(5),
	SR_MHZ(8),
	SR_MHZ(10),
	SR_MHZ(16),
	SR_MHZ(20),
	SR_MHZ(25),
	SR_MHZ(32),
	SR_MHZ(40),
	SR_MHZ(50),
	SR_MHZ(80),
	SR_MHZ(100),
	SR_MHZ(160),
	SR_MHZ(200),
	SR_MHZ(400),
	SR_MHZ(800),
	SR_MHZ(1600),
};
static const int32_t samplechannels_slogic32u3[] = { /*2, */4, 8, 16 , 32 };
static const uint64_t limit_samplerates_slogic32u3[] =
	{ SR_MHZ(1600), SR_MHZ(800), SR_MHZ(400), SR_MHZ(200) };

static const char *patterns[] = {
	[PATTERN_MODE_NORMAL] = "Normal",
	[PATTERN_MODE_TEST_HARDWARE_USB_MAX_SPEED] = "USB connection test",
	[PATTERN_MODE_TEST_HARDWARE_EMU_DATA] = "Emulation",
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,    SR_TRIGGER_ONE,  SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING, SR_TRIGGER_EDGE,
};

static struct sr_dev_driver sipeed_slogic_analyzer_driver_info;

static const struct slogic_model *const support_models_ptr;

static gpointer libusb_event_thread_func(gpointer user_data)
{
	struct sr_dev_inst *sdi;
	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;

	sdi = user_data;
	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;

	while (g_atomic_int_get(&devc->libusb_event_thread_run)) {
		libusb_handle_events_timeout_completed(
			drvc->sr_ctx->libusb_ctx, &(struct timeval){ 1, 0 },
			NULL);
	}

	return NULL;
}

/*
 * std_u64_idx() and std_i32_idx() do not take ownership of the GVariant
 * they are given, so handing them a freshly created one leaks it. Search
 * the tables directly instead.
 */
static int u64_idx(uint64_t value, const uint64_t *arr, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (arr[i] == value)
			return i;
	}

	return -1;
}

static int i32_idx(int32_t value, const int32_t *arr, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (arr[i] == value)
			return i;
	}

	return -1;
}

/*
 * Read a USB string descriptor into buf and return a copy of it, or NULL
 * if the descriptor is absent or cannot be read.
 */
static char *read_string_descriptor(libusb_device_handle *devhdl, uint8_t idx,
				    char *buf, size_t buflen)
{
	int ret;

	if (!idx)
		return NULL;

	ret = libusb_get_string_descriptor_ascii(devhdl, idx,
						 (unsigned char *)buf, buflen);
	if (ret < 0) {
		sr_dbg("Failed to read string descriptor %u: %s.", idx,
		       libusb_error_name(ret));
		return NULL;
	}

	return g_strdup(buf);
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	int ret;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct drv_context *drvc;
	struct dev_context *devc;

	const struct slogic_model *model;
	struct sr_config *option;
	struct libusb_device_descriptor des;
	GSList *devices;
	GSList *l, *conn_devices;
	const char *conn_opt;
	char *conn;
	char cbuf[128];
	char *iManufacturer, *iProduct, *iSerialNumber, *iPortPath;

	struct sr_channel *ch;
	int32_t i;
	gchar *channel_name;

	(void)options;

	conn = NULL;

	devices = NULL;
	drvc = di->context;
	// drvc->instances = NULL;

	/* scan for devices, either based on a SR_CONF_CONN option
   * or on a USB scan. */
	for (l = options; l; l = l->next) {
		option = l->data;
		switch (option->key) {
		case SR_CONF_CONN:
			conn_opt = g_variant_get_string(option->data, NULL);
			sr_info("Use conn: %s", conn_opt);
			sr_err("Not supported now!");
			return NULL;
		default:
			sr_warn("Unhandled option key: %u", option->key);
		}
	}

	for (model = support_models_ptr; model->name; model++) {
		conn = g_strdup_printf("%04x.%04x", USB_VID_SIPEED, model->pid);
		/* Find all slogic compatible devices. */
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
		for (l = conn_devices; l; l = l->next) {
			usb = l->data;
			ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
			if (SR_OK != ret) {
				sr_usb_dev_inst_free(usb);
				l->data = NULL;
				continue;
			}
			libusb_get_device_descriptor(
				libusb_get_device(usb->devhdl), &des);
			/*
			 * cbuf is left untouched when a descriptor read
			 * fails, so check every result. Otherwise the
			 * previous string, or uninitialised stack on the
			 * first read, ends up in the device instance.
			 */
			iManufacturer = read_string_descriptor(
				usb->devhdl, des.iManufacturer, cbuf,
				sizeof(cbuf));
			iProduct = read_string_descriptor(
				usb->devhdl, des.iProduct, cbuf, sizeof(cbuf));
			iSerialNumber = read_string_descriptor(
				usb->devhdl, des.iSerialNumber, cbuf,
				sizeof(cbuf));
			usb_get_port_path(libusb_get_device(usb->devhdl), cbuf,
					  sizeof(cbuf));
			iPortPath = g_strdup(cbuf);

			sdi = sr_dev_inst_user_new(iManufacturer, iProduct,
						   NULL);
			sdi->serial_num = iSerialNumber;
			sdi->connection_id = iPortPath;
			sdi->status = SR_ST_INACTIVE;
			sdi->conn = usb;
			sdi->inst_type = SR_INST_USB;

			devc = g_malloc0(sizeof(struct dev_context));
			g_mutex_init(&devc->transfers_mutex);
			sdi->priv = devc;

			{
				devc->model = model;

				devc->limit_samplechannel = devc->model->samplechannel_table[
					devc->model->samplechannel_table_size - 1];
				devc->limit_samplerate = devc->model->limit_samplerate_table[
					i32_idx(devc->limit_samplechannel,
						devc->model->samplechannel_table, devc->model->samplechannel_table_size)
				];

				devc->cur_samplechannel =
					devc->limit_samplechannel;
				devc->cur_samplerate = devc->limit_samplerate;
				devc->cur_pattern_mode_idx = PATTERN_MODE_NORMAL;
				devc->capture_ratio = 10;
				devc->voltage_threshold[0] =
					devc->voltage_threshold[1] = 1.7;

				devc->digital_group =
					sr_channel_group_new(sdi, "LA", NULL);
				for (i = 0; i < devc->limit_samplechannel;
				     i++) {
					channel_name =
						g_strdup_printf("D%d", i);
					ch = sr_channel_new(sdi, i,
							    SR_CHANNEL_LOGIC,
							    TRUE, channel_name);
					g_free(channel_name);
					devc->digital_group
						->channels = g_slist_append(
						devc->digital_group->channels,
						ch);
				}

				devc->speed = libusb_get_device_speed(
					libusb_get_device(usb->devhdl));
			}

			sr_usb_close(usb);
			devices = g_slist_append(devices, sdi);
		}
		/*
		 * The sr_usb_dev_inst of a device that was kept is now owned
		 * by its sdi, so only the list itself may be freed here.
		 */
		g_slist_free(conn_devices);
		g_free(conn);
	}

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_dev_driver *di;
	struct drv_context *drvc;

	usb = sdi->conn;
	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;

	ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
	if (SR_OK != ret)
		return ret;

	ret = libusb_claim_interface(usb->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		switch (ret) {
		case LIBUSB_ERROR_BUSY:
			sr_err("Unable to claim USB interface. Another "
			       "program or driver has already claimed it.");
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			break;
		default:
			sr_err("Unable to claim interface: %s.",
			       libusb_error_name(ret));
			break;
		}
		return SR_ERR;
	}

	g_atomic_int_set(&devc->libusb_event_thread_run, 1);
	devc->libusb_event_thread = g_thread_new("libusb_event_thread",
						 libusb_event_thread_func, sdi);
	if (!devc->libusb_event_thread) {
		g_atomic_int_set(&devc->libusb_event_thread_run, 0);
		sr_err("Unable to new libusb_event_thread!");
		return SR_ERR_MALLOC;
	}

	if (devc->model->operation.remote_reset)
		devc->model->operation.remote_reset(sdi);

	/*
	 * Set the default threshold directly. Calling sr_config_set() here
	 * cannot work: sr_dev_open() only marks the instance SR_ST_ACTIVE
	 * after this function returns, so the call is rejected and merely
	 * logs an error. config_set() would not touch the hardware either,
	 * the threshold is programmed at acquisition start.
	 */
	devc->voltage_threshold[0] = devc->voltage_threshold[1] = 1.7;

	return std_dummy_dev_open(sdi);
}

static int dev_close(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;

	usb = sdi->conn;
	devc = sdi->priv;

	ret = libusb_release_interface(usb->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		switch (ret) {
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			// return SR_ERR_DEV_CLOSED;
			break;
		default:
			sr_err("Unable to release Interface for %s.",
			       libusb_error_name(ret));
			break;
		}
	}

	g_atomic_int_set(&devc->libusb_event_thread_run, 0);
	sr_usb_close(usb);
	if (devc->libusb_event_thread) {
		g_thread_join(devc->libusb_event_thread);
		devc->libusb_event_thread = NULL;
	}

	return std_dummy_dev_close(sdi);
}

static int config_get(uint32_t key, GVariant **data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	int ret;
	struct dev_context *devc;

	(void)cg;

	devc = sdi->priv;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
		*data = g_variant_new_int32(devc->cur_samplechannel);
		break;
	case SR_CONF_PATTERN_MODE:
		/* Not listed on models without the built-in test patterns. */
		if (!devc->model->operation.remote_test_mode)
			return SR_ERR_NA;
		*data = g_variant_new_string(
			patterns[devc->cur_pattern_mode_idx]);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->cur_limit_samples);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		*data = std_gvar_tuple_double(devc->voltage_threshold[0],
					      devc->voltage_threshold[1]);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t key, GVariant *data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	int ret, idx;
	uint64_t capture_ratio;
	struct dev_context *devc;

	(void)cg;

	devc = sdi->priv;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		if (g_variant_get_uint64(data) > devc->limit_samplerate ||
		    std_u64_idx(data, devc->model->samplerate_table, devc->model->samplerate_table_size) < 0) {
			devc->cur_samplerate = devc->limit_samplerate;
			sr_warn("Reach limit or not supported, wrap to %" PRIu64 "MHz.",
				devc->limit_samplerate / SR_MHZ(1));
		} else {
			devc->cur_samplerate = g_variant_get_uint64(data);

			if (devc->cur_samplerate > devc->limit_samplerate)
				devc->cur_samplerate = devc->limit_samplerate;
		}

		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
		if (std_i32_idx(data, devc->model->samplechannel_table, devc->model->samplechannel_table_size) < 0) {
			devc->cur_samplechannel = devc->limit_samplechannel;
			sr_warn("Reach limit or not supported, wrap to %uch.",
				devc->limit_samplechannel);
		} else {
			devc->cur_samplechannel = g_variant_get_int32(data);

			devc->limit_samplerate = devc->model->limit_samplerate_table[
				i32_idx(devc->cur_samplechannel,
					devc->model->samplechannel_table, devc->model->samplechannel_table_size)
			];

			if (devc->cur_samplerate > devc->limit_samplerate)
				devc->cur_samplerate = devc->limit_samplerate;
		}
		// [en|dis]able channels and dbg
		{
			for (GSList *l = devc->digital_group->channels; l;
			     l = l->next) {
				struct sr_channel *ch = l->data;
				if (ch->type ==
					SR_CHANNEL_LOGIC) { /* Might as well do this now, these
                                               are static. */
					ch->enabled = ch->index >= devc->cur_samplechannel ? FALSE : TRUE;
				} else {
					sr_warn("devc->digital_group->channels[%u] is not Logic?",
						ch->index);
				}
				sr_dbg("\tch[%2u] %-3s:%d %sabled priv:%p.",
				       ch->index, ch->name, ch->type,
				       ch->enabled ? "en" : "dis", ch->priv);
			}
		}
		break;
	case SR_CONF_PATTERN_MODE:
		/*
		 * Report the option as unavailable instead of accepting a
		 * value that would then be silently ignored.
		 */
		if (!devc->model->operation.remote_test_mode) {
			sr_dbg("%s has no test patterns.", devc->model->name);
			return SR_ERR_NA;
		}
		idx = std_str_idx(data, ARRAY_AND_SIZE(patterns));
		if (idx < 0)
			return SR_ERR_ARG;
		devc->cur_pattern_mode_idx = idx;
		if (devc->cur_pattern_mode_idx == PATTERN_MODE_NORMAL) {
			if (devc->model->operation.remote_reset)
				devc->model->operation.remote_reset(sdi);
			devc->model->operation.remote_test_mode(sdi, 0x0);
			sr_dbg("reset model: %s success.", devc->model->name);
		} else if (devc->cur_pattern_mode_idx == PATTERN_MODE_TEST_HARDWARE_USB_MAX_SPEED) {
			devc->model->operation.remote_test_mode(sdi, 0x1);
		} else if (devc->cur_pattern_mode_idx == PATTERN_MODE_TEST_HARDWARE_EMU_DATA) {
			devc->model->operation.remote_test_mode(sdi, 0x2);
		}
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->cur_limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		capture_ratio = g_variant_get_uint64(data);
		if (capture_ratio > 100)
			return SR_ERR_ARG;
		devc->capture_ratio = capture_ratio;
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		g_variant_get(data, "(dd)", &devc->voltage_threshold[0],
			      &devc->voltage_threshold[1]);
		break;
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static int config_channel_set(const struct sr_dev_inst *sdi,
		struct sr_channel *ch, unsigned int changes)
{
	(void)ch;

	struct dev_context *devc = sdi ? (sdi->priv) : NULL;
	if(!devc || !devc->model || !devc->model->samplechannel_table || !devc->model->limit_samplerate_table){
		return SR_ERR;
	}

	if(changes != SR_CHANNEL_SET_ENABLED){
		return SR_OK;
	}

	int32_t new_samplechannel = devc->model->samplechannel_table[0];
	for (GSList *l = devc->digital_group->channels; l;l = l->next) {
		struct sr_channel *ch = l->data;
		if(!ch->enabled || ch->index < new_samplechannel){
			continue;
		}
		for(unsigned int i = 0; i < devc->model->samplechannel_table_size; i++){
			if(devc->model->samplechannel_table[i] > ch->index){
				new_samplechannel = devc->model->samplechannel_table[i];
				break;
			}
		}
	}

	if(new_samplechannel > devc->cur_samplechannel){
		devc->cur_samplechannel = new_samplechannel;
		devc->limit_samplerate = devc->model->limit_samplerate_table[
				i32_idx(devc->cur_samplechannel,
					devc->model->samplechannel_table, devc->model->samplechannel_table_size)
			];
		if (devc->cur_samplerate > devc->limit_samplerate)
			devc->cur_samplerate = devc->limit_samplerate;
	}
	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
		       const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	int ret;
	struct dev_context *devc;

	(void)cg;

	devc = sdi ? (sdi->priv) : NULL;

	/*
	 * Only the option lists can be queried without a device. Every other
	 * key below reads the model, so reject those early rather than after
	 * having already dereferenced it.
	 */
	if (key != SR_CONF_SCAN_OPTIONS && key != SR_CONF_DEVICE_OPTIONS &&
	    (!devc || !devc->model))
		return SR_ERR_ARG;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		if (devc && !devc->model->operation.remote_test_mode)
			ret = std_opts_config_list(
				key, data, sdi, cg, ARRAY_AND_SIZE(scanopts),
				ARRAY_AND_SIZE(drvopts),
				ARRAY_AND_SIZE(devopts_no_patterns));
		else
			ret = STD_CONFIG_LIST(key, data, sdi, cg, scanopts,
					      drvopts, devopts);
		break;
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(
			devc->model->samplerate_table,
			1 + u64_idx(devc->limit_samplerate,
					devc->model->samplerate_table, devc->model->samplerate_table_size));
		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
		*data = std_gvar_array_i32(devc->model->samplechannel_table, devc->model->samplechannel_table_size);
		break;
	case SR_CONF_PATTERN_MODE:
		if (!devc->model->operation.remote_test_mode)
			return SR_ERR_NA;
		*data = g_variant_new_strv(ARRAY_AND_SIZE(patterns));
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		*data = std_gvar_min_max_step_thresholds(0, 6, 0.1);
		break;
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static void clear_helper(struct dev_context *devc)
{
	g_mutex_clear(&devc->transfers_mutex);
}

static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear_with_callback(di,
					   (std_dev_clear_callback)clear_helper);
}

static struct sr_dev_driver sipeed_slogic_analyzer_driver_info = {
	.name = "sipeed-slogic-analyzer",
	.longname = "Sipeed SLogic Analyzer",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_channel_set = config_channel_set,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = sipeed_slogic_acquisition_start,
	.dev_acquisition_stop = sipeed_slogic_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(sipeed_slogic_analyzer_driver_info);

static int slogic_usb_control_write(const struct sr_dev_inst *sdi,
				    uint8_t request, uint16_t value,
				    uint16_t index, const uint8_t *data,
				    size_t len, int timeout)
{
	int ret;
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;

	sr_spew("%s: req:%u value:%u index:%u %p:%zu in %dms.", __func__,
		request, value, index, (const void *)data, len, timeout);
	if (!data && len) {
		sr_warn("%s: Nothing to write although len(%zu)>0!", __func__,
			len);
		len = 0;
	} else if (len & 0x3) {
		size_t len_aligndup = (len + 0x3) & (~0x3);
		sr_warn("%s: Align up to %zu(from %zu)!", __func__,
			len_aligndup, len);
		len = len_aligndup;
	}

	ret = 0;
	for (size_t i = 0; i < len; i += 4) {
		/*
		 * Test the chunk on its own. Accumulating first would let a
		 * later error be cancelled out by the bytes already
		 * transferred, e.g. 4 + LIBUSB_ERROR_NO_DEVICE == 0.
		 */
		int chunk = libusb_control_transfer(
			usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
			request, value + i, index,
			(unsigned char *)(data + i), 4, timeout);
		if (chunk < 0) {
			sr_err("%s: failed(libusb: %s)!", __func__,
			       libusb_error_name(chunk));
			return SR_ERR_NA;
		}
		ret += chunk;
	}

	return ret;
}

static int slogic_usb_control_read(const struct sr_dev_inst *sdi,
				   uint8_t request, uint16_t value,
				   uint16_t index, uint8_t *data, size_t len,
				   int timeout)
{
	int ret;
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;

	sr_spew("%s: req:%u value:%u index:%u %p:%zu in %dms.", __func__,
		request, value, index, (void *)data, len, timeout);
	if (!data && len) {
		sr_err("%s: Can't read to NULL while len(%zu)>0!", __func__,
		       len);
		return SR_ERR_ARG;
	} else if (len & 0x3) {
		size_t len_aligndup = (len + 0x3) & (~0x3);
		sr_warn("%s: Align up to %zu(from %zu)!", __func__,
			len_aligndup, len);
		len = len_aligndup;
	}

	ret = 0;
	for (size_t i = 0; i < len; i += 4) {
		/* Test the chunk on its own, see the write path above. */
		int chunk = libusb_control_transfer(
			usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
			request, value + i, index, (unsigned char *)data + i, 4,
			timeout);
		if (chunk < 0) {
			sr_err("%s: failed(libusb: %s)!", __func__,
			       libusb_error_name(chunk));
			return SR_ERR_NA;
		}
		ret += chunk;
	}

	return ret;
}

static void slogic_submit_raw_data(void *data, size_t len,
				   const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	uint8_t *ptr = data;
	uint64_t nCh = devc->cur_samplechannel;

	if (nCh < 8) {
		size_t nsp_in_bytes = 8 / nCh; // NOW must be 2 and 4
		ptr = malloc(len * nsp_in_bytes);
		for (size_t i = 0; i < len; i += nCh) {
			for (size_t j = 0; j < 8; j++) {
				ptr[i * nsp_in_bytes + j] =
					(((uint8_t *)
						  data)[i + j / nsp_in_bytes] >>
					 (j % nsp_in_bytes * nCh)) &
					((1 << nCh) - 1);
			}
		}
		len *= nsp_in_bytes; // need reshape
	}

	sr_session_send(sdi, &(struct sr_datafeed_packet){
				     .type = SR_DF_LOGIC,
				     .payload = &(struct sr_datafeed_logic){
					     .length = len,
					     .unitsize = (nCh + 7) / 8,
					     .data = ptr,
				     } });

	if (nCh < 8)
		free(ptr);
}

SR_PRIV int slogic_soft_trigger_raw_data(void *data, size_t len,
					 const struct sr_dev_inst *sdi)
{
	int ret = 0;
	struct dev_context *devc = sdi->priv;

	uint8_t *ptr = data;
	uint64_t nCh = devc->cur_samplechannel;
	uint8_t uintsize = (nCh + 7) / 8;

	if (nCh < 8) {
		size_t nsp_in_bytes = 8 / nCh; // NOW must be 2 or 4
		ptr = malloc(len * nsp_in_bytes);
		for (size_t i = 0; i < len; i += nCh) {
			for (size_t j = 0; j < 8; j++) {
				ptr[i * nsp_in_bytes + j] =
					(((uint8_t *)
						  data)[i + j / nsp_in_bytes] >>
					 (j % nsp_in_bytes * nCh)) &
					((1 << nCh) - 1);
			}
		}
		len *= nsp_in_bytes; // need reshape
	}

	// // debug raw data
	// sr_session_send(sdi, &(struct sr_datafeed_packet){
	// 				.type = SR_DF_LOGIC,
	// 				.payload = &(struct sr_datafeed_logic){
	// 					.length = len,
	// 					.unitsize = (nCh + 7) / 8,
	// 					.data = ptr,
	// 				} });

	/*
	 * From here on everything is counted in *session* bytes: one byte per
	 * sample, after the sub-byte channel packing above has been expanded.
	 *
	 * The shared counters are in raw wire bytes - samples_need_nbytes is
	 * cur_limit_samples * cur_samplechannel / 8, and the untriggered path
	 * in handle_events() adds the pre-demux transfer length. The two units
	 * coincide at 8 and 16 channels but differ by `expand` at 4, where the
	 * device packs two samples per byte. Mixing them made acquisition stop
	 * at half the requested sample count, which in turn aborted the run
	 * before the trigger could fire.
	 */
	const uint64_t expand = (nCh < 8) ? 8 / nCh : 1;

	/* stl->unitsize and its pre-trigger ring are set up in the caller. */
	int pre_trigger_samples;
	int64_t trigger_offset = soft_trigger_logic_check(devc->stl, ptr, len, &pre_trigger_samples);
	if (trigger_offset > -1) {
		/* soft_trigger_logic_check() already sent the pre-trigger data. */
		ret += pre_trigger_samples * uintsize;

		uint64_t remain = len - trigger_offset * uintsize;

		/*
		 * A zero samples_need_nbytes means acquisition runs without a
		 * sample limit, so everything from the trigger on is wanted.
		 */
		if (devc->samples_need_nbytes) {
			uint64_t need_session = devc->samples_need_nbytes * expand;
			uint64_t sent_session =
				devc->samples_sent_nbytes * expand + ret;
			uint64_t need = need_session > sent_session ?
						need_session - sent_session :
						0;
			if (remain > need)
				remain = need;
		}

		if (remain) {
			sr_session_send(sdi, &(struct sr_datafeed_packet){
						.type = SR_DF_LOGIC,
						.payload = &(struct sr_datafeed_logic){
							.length = remain,
							.unitsize = uintsize,
							.data = ptr + trigger_offset * uintsize,
						} });

			ret += remain;
		}

		/*
		 * Only account against the session-side counter here.
		 * samples_got_nbytes belongs to the libusb event thread and
		 * merely gates transfer submission; letting it run on from
		 * zero over-submits by at most one transfer, which the
		 * limit check in handle_events() then discards.
		 *
		 * Convert back to raw wire bytes, rounding up so a partial
		 * byte is never counted twice. Over-counting by at most one
		 * byte per transfer ends acquisition marginally early, which
		 * is safe; under-counting would overrun the sample limit.
		 */
		devc->samples_sent_nbytes += ((uint64_t)ret + expand - 1) / expand;
	} else {
		/*
		 * Report "not triggered" distinctly. Returning the byte count
		 * alone cannot express this: a trigger that matches with no
		 * pre-trigger data and nothing left to send is a valid hit
		 * that yields zero bytes.
		 */
		ret = -1;
	}

	if (nCh < 8)
		free(ptr);

	return ret;
}

// #define __USE_MISC 1
// #include <endian.h>
static inline uint16_t htole16(uint16_t value)
{
	const union {
		uint16_t val;
		uint8_t bytes[2];
	} u = { .val = 0x1234 };
	if (u.bytes[0] == 0x34) { // __LITTLE_ENDIAN
		return value;
	} else {
		return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
	}
}

static inline void clear_ep(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t ep = devc->model->ep_in;

	size_t tmp_size = 4 * 1024 * 1024;
	uint8_t *tmp = malloc(tmp_size);
	int actual_length = 0;
	int64_t deadline;

	if (!tmp) {
		sr_err("Failed to allocate %zu bytes to drain EP 0x%02x.",
		       tmp_size, ep);
		return;
	}

	/*
	 * Bound the drain. This runs on the session thread during teardown,
	 * before SR_DF_END is sent, and the Combo 8 stop path never tells
	 * the device to stop streaming, so an unbounded loop can hang the
	 * session outright.
	 */
	deadline = g_get_monotonic_time() + G_TIME_SPAN_SECOND;
	do {
		if (libusb_bulk_transfer(usb->devhdl, ep, tmp, tmp_size,
					 &actual_length, 100) < 0)
			break;
		if (g_get_monotonic_time() > deadline) {
			sr_warn("Gave up draining EP 0x%02x, still streaming.",
				ep);
			break;
		}
	} while (actual_length);

	free(tmp);
	sr_dbg("Cleared EP: 0x%02x", ep);
}

/* SLogic Combo 8 start */
#pragma pack(push, 1)
struct cmd_start_acquisition {
	union {
		struct {
			uint8_t sample_rate_l;
			uint8_t sample_rate_h;
		};
		uint16_t sample_rate;
	};
	uint8_t sample_channel;
	/*
	 * slogic_usb_control_write() rounds the transfer length up to a
	 * multiple of four. Without this byte the command is three bytes
	 * long and the fourth one handed to the device is read from past
	 * the end of the object. Pad explicitly so it is defined.
	 */
	uint8_t reserved;
};
#pragma pack(pop)

#define CMD_START 0xb1
#define CMD_STOP 0xb3

static int slogic_combo8_remote_run(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	const struct cmd_start_acquisition cmd_run = {
		.sample_rate = htole16(devc->cur_samplerate /
				       SR_MHZ(1)), // force little endian
		.sample_channel = devc->cur_samplechannel,
	};
	return slogic_usb_control_write(sdi, CMD_START, 0x0000, 0x0000,
					(uint8_t *)&cmd_run, sizeof(cmd_run),
					500);
}

static int slogic_combo8_remote_stop(const struct sr_dev_inst *sdi)
{
	clear_ep(sdi);
	return SR_OK;
	/* not stable, but can be ignored */
	// int ret = slogic_usb_control_write(sdi, CMD_STOP, 0x0000, 0x0000, NULL, 0,
	// 500); clear_ep(sdi); return ret;
}
/* SLogic Combo 8 end */

/* SLogic16U3 start */
#define SLOGIC16U3_CONTROL_IN_REQ_REG_READ 0x00
#define SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE 0x01

#define SLOGIC16U3_R32_CTRL 0x0004
#define SLOGIC16U3_R32_FLAG 0x0008
#define SLOGIC16U3_R32_AUX 0x000c

/*
 * Aux command buffer. The device's registers are addressed as halfwords
 * and words, but the control helpers take a byte pointer. Casting a
 * uint8_t array to uint16_t/uint32_t is undefined behaviour whatever the
 * target's alignment rules, so give those accesses a real type: a union
 * makes the punning well defined in C and carries the alignment the
 * wider members need.
 */
union aux_buf {
	uint8_t u8[64];
	uint16_t u16[32];
	uint32_t u32[16];
};

/*
 * Length of the aux payload, as reported by the device in the top seven
 * bits of the first halfword. The payload is read into u8 + 4, so only
 * sizeof(u8) - 4 bytes are available, while the field can hold up to
 * 127. Clamp it: an unclamped length lets the device overflow the
 * caller's stack buffer. The limit is kept a multiple of four because
 * both control helpers round the transfer length up to that.
 */
static size_t aux_payload_len(const union aux_buf *aux)
{
	size_t len, max;

	len = aux->u16[0] >> 9;
	max = (sizeof(aux->u8) - 4) & ~(size_t)3;

	if (len > max) {
		sr_warn("Device reported aux length %zu, clamping to %zu.",
			len, max);
		len = max;
	}

	return len;
}

static int slogic16U3_remote_test_mode(const struct sr_dev_inst *sdi, uint32_t mode) {
	union aux_buf aux = { 0 }; // configure aux

	{
		size_t retry = 0;
		memset(&aux, 0, sizeof(aux));
		aux.u32[0] = 0x00000005;
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4,
					 500);
		do {
			slogic_usb_control_read(
				sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
				SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4, 500);
			sr_dbg("[%zu]read aux testmode: %08x.", retry,
			       aux.u32[0]);
			retry += 1;
			if (retry > 5)
				return SR_ERR_TIMEOUT;
		} while (!(aux.u8[2] & 0x01));

		sr_dbg("test_mode length: %zu.", aux_payload_len(&aux));
		slogic_usb_control_read(sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4,
					aux_payload_len(&aux), 500);

		sr_dbg("aux rd: %08x %08x.", aux.u32[0], aux.u32[1]);

		aux.u32[1] = mode;

		sr_dbg("aux wr: %08x %08x.", aux.u32[0], aux.u32[1]);
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX + 4, 0x0000,
					 aux.u8 + 4,
					 aux_payload_len(&aux), 500);

		slogic_usb_control_read(sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4,
					aux_payload_len(&aux), 500);
		sr_dbg("aux rd: %08x %08x.", aux.u32[0], aux.u32[1]);

		if (mode != aux.u32[1]) {
			sr_dbg("Failed to configure test_mode.");
		} else {
			sr_dbg("Succeed to configure test_mode.");
		}
	}

	return SR_OK;
}

static int slogic16U3_remote_reset(const struct sr_dev_inst *sdi) {
	const uint8_t cmd_rst[] = { 0x02, 0x00, 0x00, 0x00 };
	const uint8_t cmd_derst[] = { 0x00, 0x00, 0x00, 0x00 };

	slogic_usb_control_write(sdi, SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
		SLOGIC16U3_R32_CTRL, 0x0000,
		ARRAY_AND_SIZE(cmd_rst), 500);

	return slogic_usb_control_write(sdi, SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
				 SLOGIC16U3_R32_CTRL, 0x0000,
				 ARRAY_AND_SIZE(cmd_derst), 500);
}

static int slogic16U3_remote_run(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	const uint8_t cmd_run[] = { 0x01, 0x00, 0x00, 0x00 };
	union aux_buf aux = { 0 }; // configure aux

	{
		size_t retry = 0;
		memset(&aux, 0, sizeof(aux));
		aux.u32[0] = 0x00000001;
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4,
					 500);
		do {
			slogic_usb_control_read(
				sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
				SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4, 500);
			sr_dbg("[%zu]read aux channel: %08x.", retry,
			       aux.u32[0]);
			retry += 1;
			if (retry > 5)
				return SR_ERR_TIMEOUT;
		} while (!(aux.u8[2] & 0x01));
		sr_dbg("channel length: %zu.", aux_payload_len(&aux));
		slogic_usb_control_read(sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4,
					aux_payload_len(&aux), 500);

		sr_dbg("aux rd: %08x %08x.", aux.u32[0], aux.u32[1]);

		aux.u32[1] = (1ull << devc->cur_samplechannel) - 1;

		sr_dbg("aux wr: %08x %08x.", aux.u32[0], aux.u32[1]);
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX + 4, 0x0000,
					 aux.u8 + 4,
					 aux_payload_len(&aux), 500);

		slogic_usb_control_read(sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4,
					aux_payload_len(&aux), 500);
		sr_dbg("aux rd: %08x %08x.", aux.u32[0], aux.u32[1]);

		if ((1ull << devc->cur_samplechannel) - 1 !=
		    aux.u32[1]) {
			sr_dbg("Failed to configure sample channel.");
		} else {
			sr_dbg("Succeed to configure sample channel.");
		}
	}

	{
		size_t retry = 0;
		memset(&aux, 0, sizeof(aux));
		aux.u32[0] = 0x00000002;
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4,
					 500);
		do {
			slogic_usb_control_read(
				sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
				SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4, 500);
			sr_dbg("[%zu]read aux samplerate: %08x.", retry,
			       aux.u32[0]);
			retry += 1;
			if (retry > 5)
				return SR_ERR_TIMEOUT;
		} while (!(aux.u8[2] & 0x01));
		sr_dbg("samplerate length: %zu.", aux_payload_len(&aux));

		/*
		 * The loop variable is re-read from the device on every pass,
		 * so the increment below only sticks if the device agrees.
		 * Cap the iterations: without one, firmware that keeps
		 * reporting the same base index spins here forever, on the
		 * session thread, in the middle of starting an acquisition.
		 */
		size_t base_retry = 0;
		while (aux.u16[2] <= 1) {
			if (base_retry++ > 5) {
				sr_err("Giving up configuring samplerate: "
				       "device keeps reporting base index %u.",
				       aux.u16[2]);
				return SR_ERR_TIMEOUT;
			}
			if (slogic_usb_control_read(
				    sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
				    SLOGIC16U3_R32_AUX + 4, 0x0000, aux.u8 + 4,
				    aux_payload_len(&aux),
				    500) < 0) {
				sr_err("Failed to read samplerate base.");
				return SR_ERR_IO;
			}

			sr_dbg("aux rd: %08x %x %u %u.", aux.u32[0],
			       aux.u16[2],
			       aux.u16[3],
			       aux.u32[2]);

			uint64_t base =
				SR_MHZ(1) * aux.u16[3];
			if (base % devc->cur_samplerate) {
				sr_dbg("Failed to configure samplerate from base[%u] %" PRIu64 ".",
				       aux.u16[2], base);
				aux.u16[2] += 1;
				slogic_usb_control_write(
					sdi,
					SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4, 4, 500);
				continue;
			}
			uint32_t div = base / devc->cur_samplerate;
			aux.u32[2] = div-1;

			sr_dbg("aux wr: %08x %x %u %u.", aux.u32[0],
			       aux.u16[2],
			       aux.u16[3],
			       aux.u32[2]);
			slogic_usb_control_write(
				sdi, SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
				SLOGIC16U3_R32_AUX + 4, 0x0000, aux.u8 + 4,
				aux_payload_len(&aux), 500);

			slogic_usb_control_read(
				sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
				SLOGIC16U3_R32_AUX + 4, 0x0000, aux.u8 + 4,
				aux_payload_len(&aux), 500);
			sr_dbg("aux rd: %08x %x %u %u.", aux.u32[0],
			       aux.u16[2],
			       aux.u16[3],
			       aux.u32[2]);
			break;
		}

		if (aux.u16[2] <= 1) {
			sr_dbg("Succeed to configure samplerate.");
		} else {
			sr_dbg("Failed to configure samplerate.");
		}
	}

	{
		size_t retry = 0;
		memset(&aux, 0, sizeof(aux));
		aux.u32[0] = 0x00000003;
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4,
					 500);
		do {
			slogic_usb_control_read(
				sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
				SLOGIC16U3_R32_AUX, 0x0000, aux.u8, 4, 500);
			sr_dbg("[%zu]read vref(/1024x1v6): %08x.", retry,
			       aux.u32[0]);
			retry += 1;
			if (retry > 5)
				return SR_ERR_TIMEOUT;
		} while (!(aux.u8[2] & 0x01));
		// *(uint16_t*)aux.u8 &= ~0xfe00;
		// *(uint16_t*)aux.u8 |= 0x800;
		sr_dbg("vref length: %zu.", aux_payload_len(&aux));
		slogic_usb_control_read(sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4,
					aux_payload_len(&aux), 500);

		sr_dbg("aux rd: %08x %08x.", aux.u32[0], aux.u32[1]);

		/*
		 * Map the requested threshold to a vref DAC code.
		 *
		 * The original expression here was code = V / 6.66 * 1024,
		 * i.e. a 6.66V full scale passing through the origin. Measured
		 * against a lab supply on an SLogic16 U3 (S/N 202512261505),
		 * the real transfer function has both a smaller slope and a
		 * non-zero intercept:
		 *
		 *     threshold = 0.005166 * code + 0.4318   volts
		 *
		 * Three calibration points (1.005V, 2.000V and 3.303V applied,
		 * crossovers found by bisecting the requested threshold) fit
		 * that line to within 13mV, and it independently predicted a
		 * 3.32V logic high crossing at 3.638V against 3.643V measured.
		 * Under the old formula a 3.3V input needed a 3.64V request,
		 * a 10% error that pushed a 3.3V logic high near the top of
		 * the usable range.
		 *
		 * Linearity degrades above roughly 4V: a 4.003V input crossed
		 * 116mV away from the fit, and a 5V input never crossed at all
		 * before the DAC ran out of range. Thresholds beyond ~4V are
		 * therefore approximate. That is well clear of the 1.2-3.3V
		 * logic families this hardware targets.
		 *
		 * NOTE: these constants come from one unit. Part-to-part
		 * spread is unmeasured, so this is not necessarily right for
		 * every SLogic16 U3 - but the *form* of the old expression was
		 * wrong for all of them, since it has no intercept term.
		 */
		const double vref_slope = 0.005166;	/* volts per LSB */
		const double vref_offset = 0.4318;	/* volts at code 0 */
		const double vref_want = (devc->voltage_threshold[0] +
					  devc->voltage_threshold[1]) / 2;
		double vref_raw = (vref_want - vref_offset) / vref_slope;

		if (vref_raw < 0)
			vref_raw = 0;
		else if (vref_raw > 1023)
			vref_raw = 1023;

		uint32_t vref_code = (uint32_t)(vref_raw + 0.5);
		aux.u32[1] = vref_code;
		sr_dbg("vref: want %.3fV -> code %u (%.3fV achieved).",
		       vref_want, vref_code,
		       vref_slope * vref_code + vref_offset);

		sr_dbg("aux wr: %08x %08x.", aux.u32[0], aux.u32[1]);
		slogic_usb_control_write(sdi,
					 SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					 SLOGIC16U3_R32_AUX + 4, 0x0000,
					 aux.u8 + 4,
					 aux_payload_len(&aux), 500);

		slogic_usb_control_read(sdi, SLOGIC16U3_CONTROL_IN_REQ_REG_READ,
					SLOGIC16U3_R32_AUX + 4, 0x0000,
					aux.u8 + 4,
					aux_payload_len(&aux), 500);
		sr_dbg("aux rd: %08x %08x.", aux.u32[0], aux.u32[1]);

		/*
		 * Compare against what was written, not against a fixed 1024.
		 * A readback of 1024 corresponds to a 6.66V threshold, which
		 * is outside the advertised 0-6V range, so the fixed
		 * comparison reported failure for every valid setting.
		 */
		if (vref_code != aux.u32[1]) {
			sr_dbg("Failed to configure vref: wrote %u, read back %u.",
			       vref_code, aux.u32[1]);
		} else {
			sr_dbg("Succeed to configure vref.");
		}
	}

	return slogic_usb_control_write(sdi,
					SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					SLOGIC16U3_R32_CTRL, 0x0000,
					ARRAY_AND_SIZE(cmd_run), 500);
}

static int slogic16U3_remote_stop(const struct sr_dev_inst *sdi)
{
	const uint8_t cmd_stop[] = { 0x00, 0x00, 0x00, 0x00 };
	return slogic_usb_control_write(sdi,
					SLOGIC16U3_CONTROL_OUT_REQ_REG_WRITE,
					SLOGIC16U3_R32_CTRL, 0x0000,
					ARRAY_AND_SIZE(cmd_stop), 500);
}
/* SLogic16U3 end */

static const struct slogic_model support_models[] = {
    {
        .name = "SLogic Combo 8",
        .pid = 0x0300,
        .ep_in = 0x01 | LIBUSB_ENDPOINT_IN,
        .max_bandwidth = SR_MHZ(320),
		.samplerate_table = samplerates_slogiccombo8,
		.samplerate_table_size = ARRAY_SIZE(samplerates_slogiccombo8),
		.samplechannel_table = samplechannels_slogiccombo8,
		.samplechannel_table_size = ARRAY_SIZE(samplechannels_slogiccombo8),
		.limit_samplerate_table = limit_samplerates_slogiccombo8,
        .operation =
            {
                .remote_reset = NULL,
                .remote_run = slogic_combo8_remote_run,
                .remote_stop = slogic_combo8_remote_stop,
                /* No built-in test patterns on this model. */
                .remote_test_mode = NULL,
            },
        .submit_raw_data = slogic_submit_raw_data,
    },
    {
        .name = "SLogic16U3",
        .pid = 0x3031,
        .ep_in = 0x02 | LIBUSB_ENDPOINT_IN,
        .max_bandwidth = SR_MHZ(3200),
		.samplerate_table = samplerates_slogic16u3,
		.samplerate_table_size = ARRAY_SIZE(samplerates_slogic16u3),
		.samplechannel_table = samplechannels_slogic16u3,
		.samplechannel_table_size = ARRAY_SIZE(samplechannels_slogic16u3),
		.limit_samplerate_table = limit_samplerates_slogic16u3,
        .operation =
            {
                .remote_reset = slogic16U3_remote_reset,
                .remote_run = slogic16U3_remote_run,
                .remote_stop = slogic16U3_remote_stop,
                .remote_test_mode = slogic16U3_remote_test_mode,
            },
        .submit_raw_data = slogic_submit_raw_data,
    },
    {
        .name = "SLogic32U3",
        .pid = 0x3032,
        .ep_in = 0x02 | LIBUSB_ENDPOINT_IN,
        .max_bandwidth = SR_MHZ(6400),
		.samplerate_table = samplerates_slogic32u3,
		.samplerate_table_size = ARRAY_SIZE(samplerates_slogic32u3),
		.samplechannel_table = samplechannels_slogic32u3,
		.samplechannel_table_size = ARRAY_SIZE(samplechannels_slogic32u3),
		.limit_samplerate_table = limit_samplerates_slogic32u3,
        .operation =
            {
                .remote_reset = slogic16U3_remote_reset,
                .remote_run = slogic16U3_remote_run,
                .remote_stop = slogic16U3_remote_stop,
                .remote_test_mode = slogic16U3_remote_test_mode,
            },
        .submit_raw_data = slogic_submit_raw_data,
    },
    {
        .name = NULL,
        .pid = 0x0000,
    }};

static const struct slogic_model *const support_models_ptr = &support_models[0];
