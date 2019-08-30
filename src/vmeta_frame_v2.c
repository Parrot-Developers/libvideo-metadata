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


int vmeta_frame_v2_write(struct vmeta_buffer *buf,
			 const struct vmeta_frame_v2 *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	const struct vmeta_frame_v2_base *base = NULL;
	struct vmeta_location location;
	int32_t gps_altitude = 0;
	int32_t gps_altitude_and_sv_count = 0;
	uint8_t state = 0, mode = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	base = &meta->base;

	/* Remember start position */
	start = buf->pos;

	/* Adjust location if needed */
	vmeta_location_adjust_write(&base->location, &location);

	/* Pack some fields manually */
	gps_altitude = (int32_t)(location.altitude * (1 << 8));
	gps_altitude_and_sv_count = ((gps_altitude << 8) & 0xffffff00) |
				    (base->location.sv_count & 0xff);
	state = ((base->binning << 7) & 0x80) | (base->state & 0x7f);
	mode = ((base->animation << 7) & 0x80) | (base->mode & 0x7f);

	/* Write base fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f64_i32(buf, base->ground_distance, 16));
	CHECK(vmeta_write_f64_i32(buf, location.latitude, 22));
	CHECK(vmeta_write_f64_i32(buf, location.longitude, 22));
	CHECK(vmeta_write_i32(buf, gps_altitude_and_sv_count));
	CHECK(vmeta_write_f32_i16(buf, base->speed.north, 8));
	CHECK(vmeta_write_f32_i16(buf, base->speed.east, 8));
	CHECK(vmeta_write_f32_i16(buf, base->speed.down, 8));
	CHECK(vmeta_write_f32_i16(buf, base->air_speed, 8));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.w, 14));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.x, 14));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.y, 14));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.z, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.w, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.x, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.y, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.z, 14));
	CHECK(vmeta_write_f32_i16(buf, base->camera_pan, 12));
	CHECK(vmeta_write_f32_i16(buf, base->camera_tilt, 12));
	CHECK(vmeta_write_f32_i16(buf, base->exposure_time, 8));
	CHECK(vmeta_write_u16(buf, base->gain));
	CHECK(vmeta_write_u8(buf, state));
	CHECK(vmeta_write_u8(buf, mode));
	CHECK(vmeta_write_i8(buf, base->wifi_rssi));
	CHECK(vmeta_write_u8(buf, base->battery_percentage));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v2: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write extensions */
	if (meta->has_timestamp)
		CHECK(vmeta_frame_ext_timestamp_write(buf, &meta->timestamp));
	if (meta->has_followme)
		CHECK(vmeta_frame_ext_followme_write(buf, &meta->followme));

	/* Check again for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v2: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_V2_BASE_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_v2_read(struct vmeta_buffer *buf, struct vmeta_frame_v2 *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	struct vmeta_location location;
	int32_t gps_altitude = 0;
	int32_t gps_altitude_and_sv_count = 0;
	uint8_t state = 0, mode = 0;
	struct vmeta_frame_v2_base *base = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	memset(meta, 0, sizeof(*meta));
	memset(&location, 0, sizeof(location));
	base = &meta->base;

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_V2_BASE_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v2: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_V2_BASE_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v2: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read base fields */
	CHECK(vmeta_read_f64_i32(buf, &base->ground_distance, 16));
	CHECK(vmeta_read_f64_i32(buf, &location.latitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &location.longitude, 22));
	CHECK(vmeta_read_i32(buf, &gps_altitude_and_sv_count));
	CHECK(vmeta_read_f32_i16(buf, &base->speed.north, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->speed.east, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->speed.down, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->air_speed, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.w, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.x, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.y, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.z, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.w, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.x, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.y, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.z, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->camera_pan, 12));
	CHECK(vmeta_read_f32_i16(buf, &base->camera_tilt, 12));
	CHECK(vmeta_read_f32_i16(buf, &base->exposure_time, 8));
	CHECK(vmeta_read_u16(buf, &base->gain));
	CHECK(vmeta_read_u8(buf, &state));
	CHECK(vmeta_read_u8(buf, &mode));
	CHECK(vmeta_read_i8(buf, &base->wifi_rssi));
	CHECK(vmeta_read_u8(buf, &base->battery_percentage));

	/* Unpack some fields manually */
	gps_altitude = (gps_altitude_and_sv_count & 0xffffff00) >> 8;
	location.altitude = (double)gps_altitude / (1 << 8);
	location.sv_count = gps_altitude_and_sv_count & 0xff;
	base->binning = (state & 0x80) >> 7;
	base->state = state & 0x7f;
	base->animation = (mode & 0x80) >> 7;
	base->mode = mode & 0x7f;

	/* Adjust location if needed */
	vmeta_location_adjust_read(&location, &meta->base.location);

	/* Read extensions */
	while (buf->pos + 4 < buf->len) {
		/* Read Id and length and rewind */
		CHECK(vmeta_read_u16(buf, &id));
		CHECK(vmeta_read_u16(buf, &len));
		buf->pos -= 4;
		if (buf->len - buf->pos < (size_t)len * 4 + 4) {
			res = -EPROTO;
			ULOGE("vmeta_frame_v2: bad length: %zu (%u)",
			      buf->len - buf->pos,
			      len * 4 + 4);
			goto out;
		}

		/* Read extension */
		start = buf->pos;
		switch (id) {
		case VMETA_FRAME_EXT_TIMESTAMP_ID:
			res = vmeta_frame_ext_timestamp_read(buf,
							     &meta->timestamp);
			if (res == 0)
				meta->has_timestamp = 1;
			break;

		case VMETA_FRAME_EXT_FOLLOWME_ID:
			res = vmeta_frame_ext_followme_read(buf,
							    &meta->followme);
			if (res == 0)
				meta->has_followme = 1;
			break;

		default:
			ULOGW("vmeta_frame_v2: unknown extension id: 0x%04x",
			      id);
			break;
		}

		/* In any case continue after the extension */
		buf->pos = start + len * 4 + 4;
	}

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v2: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_v2_to_json(const struct vmeta_frame_v2 *meta,
			   struct json_object *jobj)
{
	vmeta_json_add_quaternion(jobj, "drone_quat", &meta->base.drone_quat);
	vmeta_json_add_location(jobj, "location", &meta->base.location);
	vmeta_json_add_double(
		jobj, "ground_distance", meta->base.ground_distance);
	vmeta_json_add_ned(jobj, "speed", &meta->base.speed);
	vmeta_json_add_double(jobj, "air_speed", meta->base.air_speed);

	vmeta_json_add_quaternion(jobj, "frame_quat", &meta->base.frame_quat);
	vmeta_json_add_double(jobj, "camera_pan", meta->base.camera_pan);
	vmeta_json_add_double(jobj, "camera_tilt", meta->base.camera_tilt);
	vmeta_json_add_double(jobj, "exposure_time", meta->base.exposure_time);
	vmeta_json_add_int(jobj, "gain", meta->base.gain);

	vmeta_json_add_int(jobj, "wifi_rssi", meta->base.wifi_rssi);
	vmeta_json_add_int(
		jobj, "battery_percentage", meta->base.battery_percentage);

	vmeta_json_add_int(jobj, "binning", meta->base.binning);
	vmeta_json_add_int(jobj, "animation", meta->base.animation);
	vmeta_json_add_str(
		jobj, "state", vmeta_flying_state_str(meta->base.state));
	vmeta_json_add_str(
		jobj, "mode", vmeta_piloting_mode_str(meta->base.mode));

	if (meta->has_timestamp) {
		vmeta_json_add_int64(jobj,
				     "frame_timestamp",
				     meta->timestamp.frame_timestamp);
	}

	if (meta->has_followme) {
		struct json_object *jobj_followme = json_object_new_object();
		vmeta_json_add_location(
			jobj_followme, "target", &meta->followme.target);
		vmeta_json_add_int(
			jobj_followme, "enabled", meta->followme.enabled);
		vmeta_json_add_int(jobj_followme, "mode", meta->followme.mode);
		vmeta_json_add_int(jobj_followme,
				   "angle_locked",
				   meta->followme.angle_locked);
		vmeta_json_add_str(
			jobj_followme,
			"animation",
			vmeta_followme_anim_str(meta->followme.animation));
		json_object_object_add(jobj, "followme", jobj_followme);
	}

	return 0;
}


size_t vmeta_frame_v2_to_csv(const struct vmeta_frame_v2 *meta,
			     char *str,
			     size_t maxlen)
{
	size_t len = 0;

	len += vmeta_csv_add_quaternion(
		str + len, maxlen - len, &meta->base.drone_quat);
	VMETA_STR_SPACE(str + len, len, maxlen - len);
	len += vmeta_csv_add_location(
		str + len, maxlen - len, &meta->base.location);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.2f ",
			meta->base.ground_distance);
	len += vmeta_csv_add_ned(str + len, maxlen - len, &meta->base.speed);
	VMETA_STR_PRINT(
		str + len, len, maxlen - len, " %.3f ", meta->base.air_speed);

	len += vmeta_csv_add_quaternion(
		str + len, maxlen - len, &meta->base.frame_quat);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.4f %.4f %.4f %d",
			meta->base.camera_pan,
			meta->base.camera_tilt,
			meta->base.exposure_time,
			meta->base.gain);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d",
			meta->base.wifi_rssi,
			meta->base.battery_percentage);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d %d %d",
			meta->base.binning,
			meta->base.animation,
			meta->base.state,
			meta->base.mode);

	if (meta->has_timestamp) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %" PRIu64,
				meta->timestamp.frame_timestamp);
	} else {
		VMETA_STR_PRINT(str + len, len, maxlen - len, " 0");
	}

	if (meta->has_followme) {
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_location(
			str + len, maxlen - len, &meta->followme.target);
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %d %d %d %d",
				meta->followme.enabled,
				meta->followme.mode,
				meta->followme.angle_locked,
				meta->followme.animation);
	} else {
		struct vmeta_location loc;
		memset(&loc, 0, sizeof(loc));
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_location(str + len, maxlen - len, &loc);
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %d %d %d %d",
				0,
				0,
				0,
				0);
	}

	return len;
}


size_t vmeta_frame_v2_csv_header(char *str, size_t maxlen)
{
	size_t len = 0;

	VMETA_STR_PRINT(
		str + len,
		len,
		maxlen - len,
		"drone_quat_w drone_quat_x drone_quat_y drone_quat_z "
		"location_valid location_latitude location_longitude "
		"location_altitude location_horizontal_accuracy "
		"location_vertical_accuracy location_sv_count "
		"ground_distance speed_north speed_east speed_down air_speed "
		"frame_quat_w frame_quat_x frame_quat_y frame_quat_z "
		"camera_pan camera_tilt exposure_time gain "
		"wifi_rssi battery_percentage "
		"binning animation state mode "
		"frame_timestamp "
		"followme_target_valid followme_target_latitude "
		"followme_target_longitude followme_target_altitude "
		"followme_target_sv_count "
		"followme_enabled followme_mode "
		"followme_angle_locked followme_animation");

	return len;
}
