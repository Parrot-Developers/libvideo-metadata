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


int vmeta_frame_v3_write(struct vmeta_buffer *buf,
			 const struct vmeta_frame_v3 *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	const struct vmeta_frame_v3_base *base = NULL;
	struct vmeta_location location;
	int32_t gpsAltitude = 0;
	int32_t gpsAltitudeAndSvCount = 0;
	uint32_t link_quality = 0;
	uint8_t mode = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	base = &meta->base;

	/* Remember start position */
	start = buf->pos;

	/* Adjust location if needed */
	vmeta_location_adjust_write(&base->location, &location);

	/* Pack some fields manually */
	gpsAltitude = (int32_t)(location.altitude_egm96amsl * (1 << 8));
	gpsAltitudeAndSvCount = ((gpsAltitude << 8) & 0xffffff00) |
				(base->location.sv_count & 0xff);
	link_quality = ((base->link_goodput << 8) & 0xffffff00) |
		       (base->link_quality & 0xff);
	mode = ((base->animation << 7) & 0x80) | (base->mode & 0x7f);

	/* Write base fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f64_i32(buf, base->ground_distance, 16));
	CHECK(vmeta_write_f64_i32(buf, location.latitude, 22));
	CHECK(vmeta_write_f64_i32(buf, location.longitude, 22));
	CHECK(vmeta_write_i32(buf, gpsAltitudeAndSvCount));
	CHECK(vmeta_write_f32_i16(buf, base->speed.north, 8));
	CHECK(vmeta_write_f32_i16(buf, base->speed.east, 8));
	CHECK(vmeta_write_f32_i16(buf, base->speed.down, 8));
	CHECK(vmeta_write_f32_i16(buf, base->air_speed, 8));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.w, 14));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.x, 14));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.y, 14));
	CHECK(vmeta_write_f32_i16(buf, base->drone_quat.z, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_base_quat.w, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_base_quat.x, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_base_quat.y, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_base_quat.z, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.w, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.x, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.y, 14));
	CHECK(vmeta_write_f32_i16(buf, base->frame_quat.z, 14));
	CHECK(vmeta_write_f32_u16(buf, base->exposure_time, 8));
	CHECK(vmeta_write_u16(buf, base->gain));
	CHECK(vmeta_write_f32_u16(buf, base->awb_r_gain, 14));
	CHECK(vmeta_write_f32_u16(buf, base->awb_b_gain, 14));
	CHECK(vmeta_write_f32_u16(buf, base->picture_hfov, 8));
	CHECK(vmeta_write_f32_u16(buf, base->picture_vfov, 8));
	CHECK(vmeta_write_u32(buf, link_quality));
	CHECK(vmeta_write_i8(buf, base->wifi_rssi));
	CHECK(vmeta_write_u8(buf, base->battery_percentage));
	CHECK(vmeta_write_u8(buf, base->state));
	CHECK(vmeta_write_u8(buf, mode));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v3: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write extensions */
	if (meta->has_timestamp)
		CHECK(vmeta_frame_ext_timestamp_write(buf, &meta->timestamp));
	if (meta->has_automation)
		CHECK(vmeta_frame_ext_automation_write(buf, &meta->automation));
	if (meta->has_thermal)
		CHECK(vmeta_frame_ext_thermal_write(buf, &meta->thermal));
	if (meta->has_lfic)
		CHECK(vmeta_frame_ext_lfic_write(buf, &meta->lfic));

	/* Check again for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v3: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_V3_BASE_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_v3_read(struct vmeta_buffer *buf, struct vmeta_frame_v3 *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	struct vmeta_location location;
	int32_t gpsAltitude = 0;
	int32_t gpsAltitudeAndSvCount = 0;
	uint32_t link_quality = 0;
	uint8_t state = 0, mode = 0;
	struct vmeta_frame_v3_base *base = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	memset(meta, 0, sizeof(*meta));
	memset(&location, 0, sizeof(location));
	base = &meta->base;

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_V3_BASE_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v3: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_V3_BASE_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v3: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read base fields */
	CHECK(vmeta_read_f64_i32(buf, &base->ground_distance, 16));
	CHECK(vmeta_read_f64_i32(buf, &location.latitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &location.longitude, 22));
	CHECK(vmeta_read_i32(buf, &gpsAltitudeAndSvCount));
	CHECK(vmeta_read_f32_i16(buf, &base->speed.north, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->speed.east, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->speed.down, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->air_speed, 8));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.w, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.x, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.y, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->drone_quat.z, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_base_quat.w, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_base_quat.x, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_base_quat.y, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_base_quat.z, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.w, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.x, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.y, 14));
	CHECK(vmeta_read_f32_i16(buf, &base->frame_quat.z, 14));
	CHECK(vmeta_read_f32_u16(buf, &base->exposure_time, 8));
	CHECK(vmeta_read_u16(buf, &base->gain));
	CHECK(vmeta_read_f32_u16(buf, &base->awb_r_gain, 14));
	CHECK(vmeta_read_f32_u16(buf, &base->awb_b_gain, 14));
	CHECK(vmeta_read_f32_u16(buf, &base->picture_hfov, 8));
	CHECK(vmeta_read_f32_u16(buf, &base->picture_vfov, 8));
	CHECK(vmeta_read_u32(buf, &link_quality));
	CHECK(vmeta_read_i8(buf, &base->wifi_rssi));
	CHECK(vmeta_read_u8(buf, &base->battery_percentage));
	CHECK(vmeta_read_u8(buf, &state));
	CHECK(vmeta_read_u8(buf, &mode));

	/* Unpack some fields manually */
	gpsAltitude = (gpsAltitudeAndSvCount & 0xffffff00) >> 8;
	location.altitude_wgs84ellipsoid = NAN;
	location.altitude_egm96amsl = (double)gpsAltitude / (1 << 8);
	location.sv_count = gpsAltitudeAndSvCount & 0xff;
	base->link_goodput = (link_quality & 0xffffff00) >> 8;
	base->link_quality = link_quality & 0xff;
	base->state = state;
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
			ULOGE("vmeta_frame_v3: bad length: %zu (%u)",
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

		case VMETA_FRAME_EXT_AUTOMATION_ID:
			res = vmeta_frame_ext_automation_read(
				buf, &meta->automation);
			if (res == 0)
				meta->has_automation = 1;
			break;

		case VMETA_FRAME_EXT_THERMAL_ID:
			res = vmeta_frame_ext_thermal_read(buf, &meta->thermal);
			if (res == 0)
				meta->has_thermal = 1;
			break;

		case VMETA_FRAME_EXT_LFIC_ID:
			res = vmeta_frame_ext_lfic_read(buf, &meta->lfic);
			if (res == 0)
				meta->has_lfic = 1;
			break;

		default:
			ULOGW("vmeta_frame_v3: unknown extension id: 0x%04x",
			      id);
			break;
		}

		/* In any case continue after the extension */
		buf->pos = start + len * 4 + 4;
	}

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_v3: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_v3_to_json(const struct vmeta_frame_v3 *meta,
			   struct json_object *jobj)
{
	vmeta_json_add_quaternion(jobj, "drone_quat", &meta->base.drone_quat);
	vmeta_json_add_location(jobj, "location", &meta->base.location);
	vmeta_json_add_double(
		jobj, "ground_distance", meta->base.ground_distance);
	vmeta_json_add_ned(jobj, "speed", &meta->base.speed);
	vmeta_json_add_double(jobj, "air_speed", meta->base.air_speed);

	vmeta_json_add_quaternion(
		jobj, "frame_base_quat", &meta->base.frame_base_quat);
	vmeta_json_add_quaternion(jobj, "frame_quat", &meta->base.frame_quat);
	vmeta_json_add_double(jobj, "exposure_time", meta->base.exposure_time);
	vmeta_json_add_int(jobj, "gain", meta->base.gain);
	vmeta_json_add_double(jobj, "awb_r_gain", meta->base.awb_r_gain);
	vmeta_json_add_double(jobj, "awb_b_gain", meta->base.awb_b_gain);
	vmeta_json_add_double(jobj, "picture_hfov", meta->base.picture_hfov);
	vmeta_json_add_double(jobj, "picture_vfov", meta->base.picture_vfov);

	vmeta_json_add_int(jobj, "link_goodput", meta->base.link_goodput);
	vmeta_json_add_int(jobj, "link_quality", meta->base.link_quality);
	vmeta_json_add_int(jobj, "wifi_rssi", meta->base.wifi_rssi);
	vmeta_json_add_int(
		jobj, "battery_percentage", meta->base.battery_percentage);

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

	if (meta->has_automation) {
		struct json_object *jobj_automation = json_object_new_object();
		vmeta_json_add_location(jobj_automation,
					"framing_target",
					&meta->automation.framing_target);
		vmeta_json_add_location(jobj_automation,
					"flight_destination",
					&meta->automation.flight_destination);
		vmeta_json_add_int(jobj_automation,
				   "followme_enabled",
				   meta->automation.followme_enabled);
		vmeta_json_add_int(jobj_automation,
				   "lookatme_enabled",
				   meta->automation.lookatme_enabled);
		vmeta_json_add_int(jobj_automation,
				   "angle_locked",
				   meta->automation.angle_locked);
		vmeta_json_add_str(
			jobj_automation,
			"animation",
			vmeta_automation_anim_str(meta->automation.animation));
		json_object_object_add(jobj, "automation", jobj_automation);
	}

	if (meta->has_thermal) {
		struct json_object *jobj_thermal = json_object_new_object();
		vmeta_json_add_str(jobj_thermal,
				   "calib_state",
				   vmeta_thermal_calib_state_str(
					   meta->thermal.calib_state));
		vmeta_json_add_thermal_spot(
			jobj_thermal, "min", &meta->thermal.min);
		vmeta_json_add_thermal_spot(
			jobj_thermal, "max", &meta->thermal.max);
		vmeta_json_add_thermal_spot(
			jobj_thermal, "probe", &meta->thermal.probe);
		json_object_object_add(jobj, "thermal", jobj_thermal);
	}

	if (meta->has_lfic) {
		struct json_object *jobj_lfic = json_object_new_object();
		vmeta_json_add_double(
			jobj_lfic, "target_x", meta->lfic.target_x);
		vmeta_json_add_double(
			jobj_lfic, "target_y", meta->lfic.target_y);
		vmeta_json_add_location(jobj_lfic,
					"target_location",
					&meta->lfic.target_location);
		vmeta_json_add_double(jobj_lfic,
				      "estimated_precision",
				      meta->lfic.estimated_precision);
		vmeta_json_add_double(
			jobj_lfic, "grid_precision", meta->lfic.grid_precision);
		json_object_object_add(jobj, "lfic", jobj_lfic);
	}

	return 0;
}


size_t vmeta_frame_v3_to_csv(const struct vmeta_frame_v3 *meta,
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
		str + len, maxlen - len, &meta->base.frame_base_quat);
	VMETA_STR_SPACE(str + len, len, maxlen - len);
	len += vmeta_csv_add_quaternion(
		str + len, maxlen - len, &meta->base.frame_quat);
	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.4f %d",
			meta->base.exposure_time,
			meta->base.gain);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %.5f %.5f %.4f %.4f",
			meta->base.awb_r_gain,
			meta->base.awb_b_gain,
			meta->base.picture_hfov,
			meta->base.picture_vfov);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d %d %d",
			meta->base.link_goodput,
			meta->base.link_quality,
			meta->base.wifi_rssi,
			meta->base.battery_percentage);

	VMETA_STR_PRINT(str + len,
			len,
			maxlen - len,
			" %d %d %d",
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

	if (meta->has_automation) {
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_location(str + len,
					      maxlen - len,
					      &meta->automation.framing_target);
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_location(
			str + len,
			maxlen - len,
			&meta->automation.flight_destination);
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %d %d %d %d",
				meta->automation.followme_enabled,
				meta->automation.lookatme_enabled,
				meta->automation.angle_locked,
				meta->automation.animation);
	} else {
		struct vmeta_location loc;
		memset(&loc, 0, sizeof(loc));
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_location(str + len, maxlen - len, &loc);
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_location(str + len, maxlen - len, &loc);
		VMETA_STR_PRINT(str + len, len, maxlen - len, " 0 0 0 0");
	}

	if (meta->has_thermal) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %d ",
				meta->thermal.calib_state);
		len += vmeta_csv_add_thermal_spot(
			str + len, maxlen - len, &meta->thermal.min);
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_thermal_spot(
			str + len, maxlen - len, &meta->thermal.max);
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_thermal_spot(
			str + len, maxlen - len, &meta->thermal.probe);
	} else {
		struct vmeta_thermal_spot spot = {0};
		VMETA_STR_PRINT(str + len, len, maxlen - len, " 0 ");
		len += vmeta_csv_add_thermal_spot(
			str + len, maxlen - len, &spot);
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_thermal_spot(
			str + len, maxlen - len, &spot);
		VMETA_STR_SPACE(str + len, len, maxlen - len);
		len += vmeta_csv_add_thermal_spot(
			str + len, maxlen - len, &spot);
	}

	if (meta->has_lfic) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %.3f %.3f ",
				meta->lfic.target_x,
				meta->lfic.target_y);
		len += vmeta_csv_add_location(
			str + len, maxlen - len, &meta->lfic.target_location);
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				" %.2lf %.2lf",
				meta->lfic.estimated_precision,
				meta->lfic.grid_precision);
	} else {
		struct vmeta_location loc;
		memset(&loc, 0, sizeof(loc));
		VMETA_STR_PRINT(str + len, len, maxlen - len, " 0.000 0.000");
		len += vmeta_csv_add_location(str + len, maxlen - len, &loc);
		VMETA_STR_PRINT(str + len, len, maxlen - len, " 0.00 0.00");
	}

	return len;
}


size_t vmeta_frame_v3_csv_header(char *str, size_t maxlen)
{
	size_t len = 0;

	VMETA_STR_PRINT(
		str + len,
		len,
		maxlen - len,
		"drone_quat_w drone_quat_x drone_quat_y drone_quat_z "
		"location_valid location_latitude location_longitude "
		"location_altitude_wgs84ellipsoid location_altitude_egm96amsl "
		"location_horizontal_accuracy location_vertical_accuracy "
		"location_sv_count ground_distance speed_north speed_east "
		"speed_down air_speed frame_base_quat_w frame_base_quat_x "
		"frame_base_quat_y frame_base_quat_z frame_quat_w frame_quat_x "
		"frame_quat_y frame_quat_z exposure_time gain "
		"awb_r_gain awb_b_gain picture_hfov picture_vfov "
		"link_goodput link_quality wifi_rssi battery_percentage "
		"animation state mode "
		"frame_timestamp "
		"automation_framing_target_valid "
		"automation_framing_target_latitude "
		"automation_framing_target_longitude "
		"automation_framing_target_altitude_wgs84ellipsoid "
		"automation_framing_target_altitude_egm96amsl "
		"automation_framing_target_sv_count "
		"automation_flight_destination_valid "
		"automation_flight_destination_latitude "
		"automation_flight_destination_longitude "
		"automation_flight_destination_altitude_wgs84ellipsoid "
		"automation_flight_destination_altitude_egm96amsl "
		"automation_flight_destination_sv_count "
		"automation_followme_enabled automation_lookatme_enabled "
		"automation_angle_locked automation_animation "
		"thermal_cablib_state "
		"thermal_min_valid "
		"thermal_min_x thermal_min_y thermal_min_temp "
		"thermal_max_valid "
		"thermal_max_x thermal_max_y thermal_max_temp "
		"thermal_probe_valid "
		"thermal_probe_x thermal_probe_y thermal_probe_temp "
		"lfic_target_x lfic_target_y lfic_target_location_valid "
		"lfic_target_location_latitude lfic_target_location_longitude "
		"lfic_target_location_altitude_wgs84ellipsoid "
		"lfic_target_location_altitude_egm96amsl "
		"lfic_target_location_sv_count "
		"lfic_estimated_precision lfic_grid_precision");

	return len;
}
