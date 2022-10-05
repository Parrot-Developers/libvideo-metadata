/**
 * Copyright (c) 2016 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vmeta_priv.h"

/* clang-format off */
#define CHECK(_x) do { if ((res = (_x)) < 0) goto out; } while (0)
/* clang-format on */


int vmeta_frame_v1_streaming_basic_write(
	struct vmeta_buffer *buf,
	const struct vmeta_frame_v1_streaming_basic *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.yaw, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.pitch, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.roll, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->camera_pan, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->camera_tilt, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.w, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.x, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.y, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.z, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->exposure_time, 8));
	CHECK(vmeta_write_u16(buf, meta->gain));
	CHECK(vmeta_write_i8(buf, meta->wifi_rssi));
	CHECK(vmeta_write_u8(buf, meta->battery_percentage));

	/* Check expected size */
	if (buf->pos - start != VMETA_FRAME_V1_STREAMING_BASIC_SIZE) {
		res = -EPROTO;
		ULOGW("vmeta_frame_v1: bad length: %zu (%u)",
		      buf->pos - start,
		      VMETA_FRAME_V1_STREAMING_BASIC_SIZE);
		goto out;
	}

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_V1_STREAMING_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_v1_streaming_basic_read(
	struct vmeta_buffer *buf,
	struct vmeta_frame_v1_streaming_basic *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_V1_STREAMING_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_V1_STREAMING_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.yaw, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.pitch, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.roll, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->camera_pan, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->camera_tilt, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.w, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.x, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.y, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.z, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->exposure_time, 8));
	CHECK(vmeta_read_u16(buf, &meta->gain));
	CHECK(vmeta_read_i8(buf, &meta->wifi_rssi));
	CHECK(vmeta_read_u8(buf, &meta->battery_percentage));

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_v1_streaming_basic_to_json(
	const struct vmeta_frame_v1_streaming_basic *meta,
	struct json_object *jobj)
{
	vmeta_json_add_euler(jobj, "drone_attitude", &meta->drone_attitude);

	vmeta_json_add_quaternion(jobj, "frame_quat", &meta->frame_quat);
	vmeta_json_add_double(jobj, "camera_pan", meta->camera_pan);
	vmeta_json_add_double(jobj, "camera_tilt", meta->camera_tilt);
	vmeta_json_add_double(jobj, "exposure_time", meta->exposure_time);
	vmeta_json_add_int(jobj, "gain", meta->gain);

	vmeta_json_add_int(jobj, "wifi_rssi", meta->wifi_rssi);
	vmeta_json_add_int(
		jobj, "battery_percentage", meta->battery_percentage);

	return 0;
}


size_t vmeta_frame_v1_streaming_basic_to_csv(
	const struct vmeta_frame_v1_streaming_basic *meta,
	char *str,
	size_t maxlen)
{
	size_t len = 0;

	len += vmeta_csv_add_euler(
		str + len, maxlen - len, &meta->drone_attitude);
	VMETA_STR_SPACE(str + len, len, maxlen - len);
	struct vmeta_location loc;
	memset(&loc, 0, sizeof(loc));
	len += vmeta_csv_add_location(str + len, maxlen - len, &loc);
	VMETA_STR_PRINT(str + len, len, maxlen - len, " %.2f %.2f ", 0., 0.);
	struct vmeta_xyz speed;
	memset(&speed, 0, sizeof(speed));
	len += vmeta_csv_add_xyz(str + len, maxlen - len, &speed);

	VMETA_STR_SPACE(str + len, len, maxlen - len);
	len += vmeta_csv_add_quaternion(
		str + len, maxlen - len, &meta->frame_quat);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.4f %.4f %.4f %d",
			meta->camera_pan,
			meta->camera_tilt,
			meta->exposure_time,
			meta->gain);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d",
			meta->wifi_rssi,
			meta->battery_percentage);

	VMETA_STR_PRINT(
		str + len, len, maxlen - len, " %d %d %d %d", 0, 0, 0, 0);

	return len;
}


size_t vmeta_frame_v1_streaming_basic_csv_header(char *str, size_t maxlen)
{
	return vmeta_frame_v1_streaming_extended_csv_header(str, maxlen);
}


int vmeta_frame_v1_streaming_extended_write(
	struct vmeta_buffer *buf,
	const struct vmeta_frame_v1_streaming_extended *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	struct vmeta_location location;
	int32_t gps_altitude = 0;
	int32_t gps_altitude_and_sv_count = 0;
	uint8_t state = 0, mode = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Adjust location if needed */
	vmeta_location_adjust_write(&meta->location, &location);

	/* Pack some fields manually */
	gps_altitude = (int32_t)(location.altitude_egm96amsl * (1 << 8));
	gps_altitude_and_sv_count = ((gps_altitude << 8) & 0xffffff00) |
				    (meta->location.sv_count & 0xff);
	state = ((meta->binning << 7) & 0x80) | (meta->state & 0x7f);
	mode = ((meta->animation << 7) & 0x80) | (meta->mode & 0x7f);

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.yaw, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.pitch, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.roll, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->camera_pan, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->camera_tilt, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.w, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.x, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.y, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.z, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->exposure_time, 8));
	CHECK(vmeta_write_u16(buf, meta->gain));
	CHECK(vmeta_write_i8(buf, meta->wifi_rssi));
	CHECK(vmeta_write_u8(buf, meta->battery_percentage));
	CHECK(vmeta_write_f64_i32(buf, location.latitude, 20));
	CHECK(vmeta_write_f64_i32(buf, location.longitude, 20));
	CHECK(vmeta_write_i32(buf, gps_altitude_and_sv_count));
	CHECK(vmeta_write_f64_i32(buf, meta->altitude, 16));
	CHECK(vmeta_write_f64_i32(buf, meta->distance_from_home, 16));
	CHECK(vmeta_write_f32_i16(buf, meta->speed.x, 8));
	CHECK(vmeta_write_f32_i16(buf, meta->speed.y, 8));
	CHECK(vmeta_write_f32_i16(buf, meta->speed.z, 8));
	CHECK(vmeta_write_u8(buf, state));
	CHECK(vmeta_write_u8(buf, mode));

	/* Check expected size */
	if (buf->pos - start != VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE) {
		res = -EPROTO;
		ULOGW("vmeta_frame_v1: bad length: %zu (%u)",
		      buf->pos - start,
		      VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE);
		goto out;
	}

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_V1_STREAMING_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_v1_streaming_extended_read(
	struct vmeta_buffer *buf,
	struct vmeta_frame_v1_streaming_extended *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	struct vmeta_location location;
	int32_t gps_altitude = 0;
	int32_t gps_altitude_and_sv_count = 0;
	uint8_t state = 0, mode = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_V1_STREAMING_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_V1_STREAMING_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	memset(&location, 0, sizeof(location));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.yaw, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.pitch, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.roll, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->camera_pan, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->camera_tilt, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.w, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.x, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.y, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.z, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->exposure_time, 8));
	CHECK(vmeta_read_u16(buf, &meta->gain));
	CHECK(vmeta_read_i8(buf, &meta->wifi_rssi));
	CHECK(vmeta_read_u8(buf, &meta->battery_percentage));
	CHECK(vmeta_read_f64_i32(buf, &location.latitude, 20));
	CHECK(vmeta_read_f64_i32(buf, &location.longitude, 20));
	CHECK(vmeta_read_i32(buf, &gps_altitude_and_sv_count));
	CHECK(vmeta_read_f64_i32(buf, &meta->altitude, 16));
	CHECK(vmeta_read_f64_i32(buf, &meta->distance_from_home, 16));
	CHECK(vmeta_read_f32_i16(buf, &meta->speed.x, 8));
	CHECK(vmeta_read_f32_i16(buf, &meta->speed.y, 8));
	CHECK(vmeta_read_f32_i16(buf, &meta->speed.z, 8));
	CHECK(vmeta_read_u8(buf, &state));
	CHECK(vmeta_read_u8(buf, &mode));

	/* Unpack some fields manually */
	gps_altitude = (gps_altitude_and_sv_count & 0xffffff00) >> 8;
	location.altitude_wgs84ellipsoid = NAN;
	location.altitude_egm96amsl = (double)gps_altitude / (1 << 8);
	location.sv_count = gps_altitude_and_sv_count & 0xff;
	meta->binning = (state & 0x80) >> 7;
	meta->state = state & 0x7f;
	meta->animation = (mode & 0x80) >> 7;
	meta->mode = mode & 0x7f;

	/* Adjust location if needed */
	vmeta_location_adjust_read(&location, &meta->location);

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v1: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_v1_streaming_extended_to_json(
	const struct vmeta_frame_v1_streaming_extended *meta,
	struct json_object *jobj)
{
	vmeta_json_add_euler(jobj, "drone_attitude", &meta->drone_attitude);
	vmeta_json_add_location(jobj, "location", &meta->location);
	vmeta_json_add_double(jobj, "altitude", meta->altitude);
	vmeta_json_add_double(
		jobj, "distance_from_home", meta->distance_from_home);
	vmeta_json_add_xyz(jobj, "speed", &meta->speed);

	vmeta_json_add_quaternion(jobj, "frame_quat", &meta->frame_quat);
	vmeta_json_add_double(jobj, "camera_pan", meta->camera_pan);
	vmeta_json_add_double(jobj, "camera_tilt", meta->camera_tilt);
	vmeta_json_add_double(jobj, "exposure_time", meta->exposure_time);
	vmeta_json_add_int(jobj, "gain", meta->gain);

	vmeta_json_add_int(jobj, "wifi_rssi", meta->wifi_rssi);
	vmeta_json_add_int(
		jobj, "battery_percentage", meta->battery_percentage);

	vmeta_json_add_int(jobj, "binning", meta->binning);
	vmeta_json_add_int(jobj, "animation", meta->animation);
	vmeta_json_add_str(jobj, "state", vmeta_flying_state_str(meta->state));
	vmeta_json_add_str(jobj, "mode", vmeta_piloting_mode_str(meta->mode));

	return 0;
}


size_t vmeta_frame_v1_streaming_extended_to_csv(
	const struct vmeta_frame_v1_streaming_extended *meta,
	char *str,
	size_t maxlen)
{
	size_t len = 0;

	len += vmeta_csv_add_euler(
		str + len, maxlen - len, &meta->drone_attitude);
	VMETA_STR_SPACE(str + len, len, maxlen - len);
	len += vmeta_csv_add_location(str + len, maxlen - len, &meta->location);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.2f %.2f ",
			meta->altitude,
			meta->distance_from_home);
	len += vmeta_csv_add_xyz(str + len, maxlen - len, &meta->speed);

	VMETA_STR_SPACE(str + len, len, maxlen - len);
	len += vmeta_csv_add_quaternion(
		str + len, maxlen - len, &meta->frame_quat);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.4f %.4f %.4f %d",
			meta->camera_pan,
			meta->camera_tilt,
			meta->exposure_time,
			meta->gain);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d",
			meta->wifi_rssi,
			meta->battery_percentage);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d %d %d",
			meta->binning,
			meta->animation,
			meta->state,
			meta->mode);

	return len;
}


size_t vmeta_frame_v1_streaming_extended_csv_header(char *str, size_t maxlen)
{
	size_t len = 0;

	VMETA_STR_PRINT(
		str + len,
		len,
		maxlen - len,
		"drone_attitude_yaw drone_attitude_pitch drone_attitude_roll "
		"location_valid location_latitude location_longitude "
		"location_altitude_wgs84ellipsoid location_altitude_egm96amsl "
		"location_horizontal_accuracy location_vertical_accuracy "
		"location_sv_count altitude distance_from_home "
		"speed_x speed_y speed_z "
		"frame_quat_w frame_quat_x frame_quat_y frame_quat_z "
		"camera_pan camera_tilt exposure_time gain "
		"wifi_rssi battery_percentage "
		"binning animation state mode");

	return len;
}


int vmeta_frame_v1_recording_write(struct vmeta_buffer *buf,
				   const struct vmeta_frame_v1_recording *meta)
{
	int res = 0;
	struct vmeta_location location;
	int32_t gps_altitude = 0;
	int32_t gps_altitude_and_sv_count = 0;
	uint8_t state = 0, mode = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Adjust location if needed */
	vmeta_location_adjust_write(&meta->location, &location);

	/* Pack some fields manually */
	gps_altitude = (int32_t)(location.altitude_egm96amsl * (1 << 8));
	gps_altitude_and_sv_count = ((gps_altitude << 8) & 0xffffff00) |
				    (meta->location.sv_count & 0xff);
	state = ((meta->binning << 7) & 0x80) | (meta->state & 0x7f);
	mode = ((meta->animation << 7) & 0x80) | (meta->mode & 0x7f);

	/* Write fields */
	CHECK(vmeta_write_u64(buf, meta->frame_timestamp));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.yaw, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.pitch, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->drone_attitude.roll, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->camera_pan, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->camera_tilt, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.w, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.x, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.y, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->frame_quat.z, 12));
	CHECK(vmeta_write_f32_i16(buf, meta->exposure_time, 8));
	CHECK(vmeta_write_u16(buf, meta->gain));
	CHECK(vmeta_write_i8(buf, meta->wifi_rssi));
	CHECK(vmeta_write_u8(buf, meta->battery_percentage));
	CHECK(vmeta_write_f64_i32(buf, location.latitude, 20));
	CHECK(vmeta_write_f64_i32(buf, location.longitude, 20));
	CHECK(vmeta_write_i32(buf, gps_altitude_and_sv_count));
	CHECK(vmeta_write_f64_i32(buf, meta->altitude, 16));
	CHECK(vmeta_write_f64_i32(buf, meta->distance_from_home, 16));
	CHECK(vmeta_write_f32_i16(buf, meta->speed.x, 8));
	CHECK(vmeta_write_f32_i16(buf, meta->speed.y, 8));
	CHECK(vmeta_write_f32_i16(buf, meta->speed.z, 8));
	CHECK(vmeta_write_u8(buf, state));
	CHECK(vmeta_write_u8(buf, mode));

out:
	return res;
}


int vmeta_frame_v1_recording_read(struct vmeta_buffer *buf,
				  struct vmeta_frame_v1_recording *meta)
{
	int res = 0;
	struct vmeta_location location;
	int32_t gps_altitude = 0;
	int32_t gps_altitude_and_sv_count = 0;
	uint8_t state = 0, mode = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	memset(&location, 0, sizeof(location));
	CHECK(vmeta_read_u64(buf, &meta->frame_timestamp));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.yaw, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.pitch, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->drone_attitude.roll, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->camera_pan, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->camera_tilt, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.w, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.x, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.y, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->frame_quat.z, 12));
	CHECK(vmeta_read_f32_i16(buf, &meta->exposure_time, 8));
	CHECK(vmeta_read_u16(buf, &meta->gain));
	CHECK(vmeta_read_i8(buf, &meta->wifi_rssi));
	CHECK(vmeta_read_u8(buf, &meta->battery_percentage));
	CHECK(vmeta_read_f64_i32(buf, &location.latitude, 20));
	CHECK(vmeta_read_f64_i32(buf, &location.longitude, 20));
	CHECK(vmeta_read_i32(buf, &gps_altitude_and_sv_count));
	CHECK(vmeta_read_f64_i32(buf, &meta->altitude, 16));
	CHECK(vmeta_read_f64_i32(buf, &meta->distance_from_home, 16));
	CHECK(vmeta_read_f32_i16(buf, &meta->speed.x, 8));
	CHECK(vmeta_read_f32_i16(buf, &meta->speed.y, 8));
	CHECK(vmeta_read_f32_i16(buf, &meta->speed.z, 8));
	CHECK(vmeta_read_u8(buf, &state));
	CHECK(vmeta_read_u8(buf, &mode));

	/* Unpack some fields manually */
	gps_altitude = (gps_altitude_and_sv_count & 0xffffff00) >> 8;
	location.altitude_wgs84ellipsoid = NAN;
	location.altitude_egm96amsl = (double)gps_altitude / (1 << 8);
	location.sv_count = gps_altitude_and_sv_count & 0xff;
	meta->binning = (state & 0x80) >> 7;
	meta->state = state & 0x7f;
	meta->animation = (mode & 0x80) >> 7;
	meta->mode = mode & 0x7f;

	/* Adjust location if needed */
	vmeta_location_adjust_read(&location, &meta->location);

out:
	return res;
}


int vmeta_frame_v1_recording_to_json(
	const struct vmeta_frame_v1_recording *meta,
	struct json_object *jobj)
{
	vmeta_json_add_euler(jobj, "drone_attitude", &meta->drone_attitude);
	vmeta_json_add_location(jobj, "location", &meta->location);
	vmeta_json_add_double(jobj, "altitude", meta->altitude);
	vmeta_json_add_double(
		jobj, "distance_from_home", meta->distance_from_home);
	vmeta_json_add_xyz(jobj, "speed", &meta->speed);

	vmeta_json_add_int64(jobj, "frame_timestamp", meta->frame_timestamp);
	vmeta_json_add_quaternion(jobj, "frame_quat", &meta->frame_quat);
	vmeta_json_add_double(jobj, "camera_pan", meta->camera_pan);
	vmeta_json_add_double(jobj, "camera_tilt", meta->camera_tilt);
	vmeta_json_add_double(jobj, "exposure_time", meta->exposure_time);
	vmeta_json_add_int(jobj, "gain", meta->gain);

	vmeta_json_add_int(jobj, "wifi_rssi", meta->wifi_rssi);
	vmeta_json_add_int(
		jobj, "battery_percentage", meta->battery_percentage);

	vmeta_json_add_int(jobj, "binning", meta->binning);
	vmeta_json_add_int(jobj, "animation", meta->animation);
	vmeta_json_add_str(jobj, "state", vmeta_flying_state_str(meta->state));
	vmeta_json_add_str(jobj, "mode", vmeta_piloting_mode_str(meta->mode));

	return 0;
}


size_t
vmeta_frame_v1_recording_to_csv(const struct vmeta_frame_v1_recording *meta,
				char *str,
				size_t maxlen)
{
	size_t len = 0;

	len += vmeta_csv_add_euler(
		str + len, maxlen - len, &meta->drone_attitude);
	VMETA_STR_SPACE(str + len, len, maxlen - len);
	len += vmeta_csv_add_location(str + len, maxlen - len, &meta->location);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.2f %.2f ",
			meta->altitude,
			meta->distance_from_home);
	len += vmeta_csv_add_xyz(str + len, maxlen - len, &meta->speed);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %" PRIu64 " ",
			meta->frame_timestamp);
	len += vmeta_csv_add_quaternion(
		str + len, maxlen - len, &meta->frame_quat);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.4f %.4f %.4f %d",
			meta->camera_pan,
			meta->camera_tilt,
			meta->exposure_time,
			meta->gain);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d",
			meta->wifi_rssi,
			meta->battery_percentage);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d %d %d",
			meta->binning,
			meta->animation,
			meta->state,
			meta->mode);

	return len;
}


size_t vmeta_frame_v1_recording_csv_header(char *str, size_t maxlen)
{
	size_t len = 0;

	VMETA_STR_PRINT(
		str + len,
		len,
		maxlen - len,
		"drone_attitude_yaw drone_attitude_pitch drone_attitude_roll "
		"location_valid location_latitude location_longitude "
		"location_altitude_wgs84ellipsoid location_altitude_egm96amsl "
		"location_horizontal_accuracy location_vertical_accuracy "
		"location_sv_count altitude distance_from_home "
		"speed_x speed_y speed_z "
		"frame_timestamp "
		"frame_quat_w frame_quat_x frame_quat_y frame_quat_z "
		"camera_pan camera_tilt exposure_time gain "
		"wifi_rssi battery_percentage "
		"binning animation state mode");

	return len;
}
