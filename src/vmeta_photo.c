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
#include <video-metadata/vmeta_photo.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Shorthand macros for metadata indices */
#define EXIF_(_name) PMETA_DEFS_EXIF_IDX_##_name
#define XMP_(_name) PMETA_DEFS_XMP_IDX_##_name


static inline void write_exif(vmeta_photo_write_cb_t cb,
			      enum pmeta_defs_exif_idx idx,
			      const char *val,
			      void *user)
{
	const struct pmeta_defs_exif_def *def =
		pmeta_defs_get_exif_tag_by_idx(idx);
	if (!def) {
		ULOGE("%s: invalid index: %d", __func__, idx);
		return;
	}

	cb(PMETA_DEFS_DEST_EXIF, def, NULL, val, user);
}


static inline void write_xmp(vmeta_photo_write_cb_t cb,
			     enum pmeta_defs_xmp_idx idx,
			     const char *val,
			     void *user)
{
	const struct pmeta_defs_xmp_def *def =
		pmeta_defs_get_xmp_tag_by_idx(idx);
	if (!def) {
		ULOGE("%s: invalid index: %d", __func__, idx);
		return;
	}

	cb(PMETA_DEFS_DEST_XMP, NULL, def, val, user);
}


static int format_exif_date(uint64_t utc_ts, char *out_date, size_t max_len)
{
	int res;
	/* EXIF date format is fixed at 19 characters: "YYYY:MM:DD HH:MM:SS" */
	const size_t exif_date_len = 19;

	if (out_date == NULL || max_len <= exif_date_len)
		return -EINVAL;

	/* Get local time in ISO 8601 long format: "YYYY-MM-DDTHH:MM:SS+HH:MM"
	 */
	res = time_local_format(
		utc_ts / 1000000, 0, TIME_FMT_LONG, out_date, max_len);
	if (res < 0)
		return res;

	/* Ensure the generated string is long enough before manipulation */
	if (strnlen(out_date, max_len) < exif_date_len)
		return -EPROTO;

	/* Patch characters to comply with EXIF standard (TIFF Rev. 6.0) */
	/* Replace 1st dash (year) */
	out_date[4] = ':';
	/* Replace 2nd dash (month) */
	out_date[7] = ':';
	/* Replace 'T' separator with space */
	out_date[10] = ' ';

	/* Truncate after seconds to remove ISO timezone offset */
	out_date[exif_date_len] = '\0';

	return 0;
}


static void write_session_maker_model(const struct vmeta_session *meta,
				      vmeta_photo_write_cb_t cb,
				      void *userdata)
{
	if (meta->maker[0] != '\0') {
		write_exif(cb, EXIF_(MAKE), meta->maker, userdata);
		write_xmp(cb, XMP_(TIFF_MAKE), meta->maker, userdata);
	}

	if (meta->model[0] != '\0') {
		write_exif(cb, EXIF_(MODEL), meta->model, userdata);
		write_xmp(cb, XMP_(TIFF_MODEL), meta->model, userdata);
	}

	if (meta->maker[0] != '\0' && meta->model[0] != '\0') {
		char unique_model[128];
		snprintf(unique_model,
			 sizeof(unique_model),
			 "%s %s",
			 meta->maker,
			 meta->model);
		write_exif(
			cb, EXIF_(UNIQUE_CAMERA_MODEL), unique_model, userdata);
	}
}


static void write_session_versions(const struct vmeta_session *meta,
				   vmeta_photo_write_cb_t cb,
				   void *userdata)
{
	if (meta->software_version[0] != '\0') {
		write_exif(
			cb, EXIF_(SOFTWARE), meta->software_version, userdata);
		write_xmp(cb,
			  XMP_(TIFF_SOFTWARE),
			  meta->software_version,
			  userdata);
		write_xmp(cb,
			  XMP_(SOFTWARE_VERSION),
			  meta->software_version,
			  userdata);
	}

	if (meta->serial_number[0] != '\0') {
		write_exif(cb,
			   EXIF_(BODY_SERIAL_NUMBER),
			   meta->serial_number,
			   userdata);
		write_xmp(
			cb, XMP_(SERIAL_NUMBER), meta->serial_number, userdata);
	}

	if (meta->model_id[0] != '\0')
		write_xmp(cb, XMP_(MODEL_ID), meta->model_id, userdata);

	if (meta->build_id[0] != '\0')
		write_xmp(
			cb, XMP_(SOFTWARE_BUILD_ID), meta->build_id, userdata);
}


static void write_session_dates(const struct vmeta_session *meta,
				vmeta_photo_write_cb_t cb,
				void *userdata)
{
	if (meta->boot_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->boot_date,
						       meta->boot_date_gmtoff);
		if (ret > 0)
			write_xmp(cb, XMP_(BOOT_DATE), date, userdata);
	}

	if (meta->flight_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret =
			vmeta_session_date_write(date,
						 sizeof(date),
						 meta->flight_date,
						 meta->flight_date_gmtoff);
		if (ret > 0)
			write_xmp(cb, XMP_(FLIGHT_DATE), date, userdata);
	}

	/* Standard Date/Time tags */
	uint64_t date_ts = 0;
	int32_t date_gmtoff = 0;

	if (meta->media_date != 0) {
		date_ts = meta->media_date;
		date_gmtoff = meta->media_date_gmtoff;
	} else if (meta->flight_date != 0) {
		date_ts = meta->flight_date;
		date_gmtoff = meta->flight_date_gmtoff;
	}

	if (date_ts != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		char tz[10];
		int ret;

		/* Exif Date/Time (YYYY:MM:DD HH:MM:SS) */
		ret = time_local_format(date_ts,
					date_gmtoff,
					TIME_FMT_LONG,
					date,
					sizeof(date));
		if (ret >= 0) {
			if (strlen(date) >= 10) {
				date[4] = ':';
				date[7] = ':';
			}
			write_exif(cb, EXIF_(DATETIME), date, userdata);
			write_exif(
				cb, EXIF_(DATETIME_ORIGINAL), date, userdata);
		}

		/* Exif Offset Time (+/-HH:MM) */
		int h = abs(date_gmtoff) / 3600;
		int m = (abs(date_gmtoff) % 3600) / 60;
		snprintf(tz,
			 sizeof(tz),
			 "%s%02d:%02d",
			 (date_gmtoff >= 0) ? "+" : "-",
			 h,
			 m);

		write_exif(cb, EXIF_(OFFSET_TIME), tz, userdata);
		write_exif(cb, EXIF_(OFFSET_TIME_ORIGINAL), tz, userdata);

		/* XMP Dates (ISO 8601) */
		ret = time_local_format(date_ts,
					date_gmtoff,
					TIME_FMT_ISO8601_LONG,
					date,
					sizeof(date));
		if (ret >= 0) {
			write_xmp(cb, XMP_(CREATE_DATE), date, userdata);
			write_xmp(cb, XMP_(MODIFY_DATE), date, userdata);
		}
	}
}


static void write_session_ids_and_misc(const struct vmeta_session *meta,
				       vmeta_photo_write_cb_t cb,
				       void *userdata)
{
	if (meta->camera_spectrum != VMETA_CAMERA_SPECTRUM_UNKNOWN) {
		write_xmp(cb,
			  XMP_(CAMERA_SPECTRUM),
			  vmeta_camera_spectrum_to_str(meta->camera_spectrum),
			  userdata);
	}

	if (meta->boot_id[0] != '\0')
		write_xmp(cb, XMP_(BOOT_ID), meta->boot_id, userdata);

	if (meta->flight_id[0] != '\0') {
		write_xmp(cb, XMP_(FLIGHT_ID), meta->flight_id, userdata);
		write_xmp(cb,
			  XMP_(CAMERA_FLIGHT_UUID),
			  meta->flight_id,
			  userdata);
	}

	if (meta->custom_id[0] != '\0')
		write_xmp(cb, XMP_(CUSTOM_ID), meta->custom_id, userdata);

	if (meta->photo_mode != VMETA_PHOTO_MODE_UNKNOWN) {
		write_xmp(cb,
			  XMP_(PHOTO_MODE),
			  vmeta_photo_mode_to_str(meta->photo_mode),
			  userdata);
	}

	if (meta->panorama_type != VMETA_PANORAMA_TYPE_UNKNOWN) {
		write_xmp(cb,
			  XMP_(PANORAMA_TYPE),
			  vmeta_panorama_type_to_str(meta->panorama_type),
			  userdata);
	}

	if (meta->photo_count != 0) {
		char count[11];
		snprintf(count, sizeof(count), "%" PRIu32, meta->photo_count);
		write_xmp(cb, XMP_(PHOTO_COUNT), count, userdata);
	}

	if (meta->secure_cn[0] != '\0')
		write_xmp(cb, XMP_(SECURE_CN), meta->secure_cn, userdata);

	if (meta->media_id != 0) {
		char val[11];
		snprintf(val, sizeof(val), "%" PRIu32, meta->media_id);
		write_xmp(cb, XMP_(MEDIA_ID), val, userdata);
	}

	if (meta->resource_index != 0) {
		char val[11];
		snprintf(val, sizeof(val), "%" PRIu32, meta->resource_index);
		write_xmp(cb, XMP_(RESOURCE_INDEX), val, userdata);
	}
}


static void write_session_camera_model(const struct vmeta_session *meta,
				       vmeta_photo_write_cb_t cb,
				       void *userdata)
{
	switch (meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
		char dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
		write_xmp(
			cb,
			XMP_(CAMERA_MODEL_TYPE),
			vmeta_camera_model_type_to_str(meta->camera_model.type),
			userdata);

		ssize_t ret = vmeta_session_perspective_distortion_write(
			dist,
			sizeof(dist),
			meta->camera_model.perspective.distortion.r1,
			meta->camera_model.perspective.distortion.r2,
			meta->camera_model.perspective.distortion.r3,
			meta->camera_model.perspective.distortion.t1,
			meta->camera_model.perspective.distortion.t2);
		if (ret > 0) {
			write_xmp(cb,
				  XMP_(PERSPECTIVE_DISTORTION),
				  dist,
				  userdata);
		}
		break;
	}
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
		char matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
		char coef[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];

		write_xmp(
			cb,
			XMP_(CAMERA_MODEL_TYPE),
			vmeta_camera_model_type_to_str(meta->camera_model.type),
			userdata);

		ssize_t ret = vmeta_session_fisheye_affine_matrix_write(
			matrix,
			sizeof(matrix),
			meta->camera_model.fisheye.affine_matrix.c,
			meta->camera_model.fisheye.affine_matrix.d,
			meta->camera_model.fisheye.affine_matrix.e,
			meta->camera_model.fisheye.affine_matrix.f);
		if (ret > 0) {
			write_xmp(cb,
				  XMP_(FISHEYE_AFFINE_MATRIX),
				  matrix,
				  userdata);

			if (meta->camera_model.fisheye.affine_matrix
				    .symmetric_valid) {
				char val[11];
				snprintf(val,
					 sizeof(val),
					 "%u",
					 meta->camera_model.fisheye
						 .affine_matrix.symmetric);
				write_xmp(cb,
					  XMP_(FISHEYE_AFFINE_SYMMETRIC),
					  val,
					  userdata);
			}
		}
		ret = vmeta_session_fisheye_polynomial_write(
			coef,
			sizeof(coef),
			meta->camera_model.fisheye.polynomial.p2,
			meta->camera_model.fisheye.polynomial.p3,
			meta->camera_model.fisheye.polynomial.p4);
		if (ret > 0)
			write_xmp(cb, XMP_(FISHEYE_POLYNOMIAL), coef, userdata);
		break;
	}
	default:
		break;
	}
}


int vmeta_session_photo_write(const struct vmeta_session *meta,
			      vmeta_photo_write_cb_t cb,
			      void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	if (meta->title[0] != '\0')
		write_exif(cb, EXIF_(IMAGE_DESCRIPTION), meta->title, userdata);

	write_session_maker_model(meta, cb, userdata);
	write_session_versions(meta, cb, userdata);
	write_session_ids_and_misc(meta, cb, userdata);
	write_session_dates(meta, cb, userdata);
	write_session_camera_model(meta, cb, userdata);

	return 0;
}


static void write_frame_timestamps(const struct vmeta_frame *meta,
				   vmeta_photo_write_cb_t cb,
				   void *userdata)
{
	uint64_t ts = 0;
	if (vmeta_frame_get_frame_timestamp((struct vmeta_frame *)meta, &ts) ==
	    0) {
		char val[21];
		snprintf(val, sizeof(val), "%" PRIu64, ts);
		write_xmp(cb, XMP_(CAPTURE_TS_US), val, userdata);
	}

	uint64_t utc_ts = 0;
	if (vmeta_frame_get_frame_utc_timestamp((struct vmeta_frame *)meta,
						&utc_ts) == 0 &&
	    utc_ts != 0) {
		/* Format: YYYY:MM:DD HH:MM:SS for Exif */
		char date[VMETA_SESSION_DATE_MAX_LEN];
		format_exif_date(utc_ts, date, sizeof(date));

		write_exif(cb, EXIF_(DATETIME_ORIGINAL), date, userdata);
		write_exif(cb, EXIF_(DATETIME), date, userdata);
		write_exif(cb, EXIF_(DATETIME_DIGITIZED), date, userdata);

		/* Timezone offsets (assuming UTC) */
		const char *tz = "+00:00";
		write_exif(cb, EXIF_(OFFSET_TIME), tz, userdata);
		write_exif(cb, EXIF_(OFFSET_TIME_ORIGINAL), tz, userdata);
		write_exif(cb, EXIF_(OFFSET_TIME_DIGITIZED), tz, userdata);

		/* Format: ISO 8601 for XMP */
		char iso_date[30];
		time_local_format(utc_ts / 1000000,
				  0,
				  TIME_FMT_ISO8601_LONG,
				  iso_date,
				  sizeof(iso_date));

		write_xmp(cb, XMP_(CREATE_DATE), iso_date, userdata);
		write_xmp(cb, XMP_(MODIFY_DATE), iso_date, userdata);
		write_xmp(cb, XMP_(DC_DATE), iso_date, userdata);

		/* SubSecTime (milliseconds) */
		char subsec[4];
		snprintf(subsec,
			 sizeof(subsec),
			 "%03" PRIu64,
			 (utc_ts / 1000) % 1000);

		write_exif(cb, EXIF_(SUBSEC_TIME), subsec, userdata);
		write_exif(cb, EXIF_(SUBSEC_TIME_ORIGINAL), subsec, userdata);
		write_exif(cb, EXIF_(SUBSEC_TIME_DIGITIZED), subsec, userdata);
	}
}


static void write_frame_location(const struct vmeta_frame *meta,
				 vmeta_photo_write_cb_t cb,
				 void *userdata)
{
	struct vmeta_location loc;
	if (vmeta_frame_get_location((struct vmeta_frame *)meta, &loc) == 0 &&
	    loc.valid) {
		char val[32];

		/* Latitude */
		snprintf(val, sizeof(val), "%.8f", fabs(loc.latitude));
		write_exif(cb, EXIF_(GPS_LATITUDE), val, userdata);
		write_exif(cb,
			   EXIF_(GPS_LATITUDE_REF),
			   loc.latitude >= 0 ? "N" : "S",
			   userdata);

		snprintf(val, sizeof(val), "%.8f", loc.latitude);
		write_xmp(cb, XMP_(DRONE_LATITUDE), val, userdata);

		/* Longitude */
		snprintf(val, sizeof(val), "%.8f", fabs(loc.longitude));
		write_exif(cb, EXIF_(GPS_LONGITUDE), val, userdata);
		write_exif(cb,
			   EXIF_(GPS_LONGITUDE_REF),
			   loc.longitude >= 0 ? "E" : "W",
			   userdata);

		snprintf(val, sizeof(val), "%.8f", loc.longitude);
		write_xmp(cb, XMP_(DRONE_LONGITUDE), val, userdata);

		/* Altitude */
		if (!isnan(loc.altitude_egm96amsl)) {
			snprintf(val,
				 sizeof(val),
				 "%.2f",
				 fabs(loc.altitude_egm96amsl));
			write_exif(cb, EXIF_(GPS_ALTITUDE), val, userdata);
			write_exif(cb,
				   EXIF_(GPS_ALTITUDE_REF),
				   loc.altitude_egm96amsl >= 0 ? "0" : "1",
				   userdata);

			snprintf(val,
				 sizeof(val),
				 "%.2f",
				 loc.altitude_egm96amsl);
			write_xmp(cb, XMP_(DRONE_ALTITUDE), val, userdata);
		}

		if (!isnan(loc.altitude_wgs84ellipsoid)) {
			snprintf(val,
				 sizeof(val),
				 "%.2f",
				 loc.altitude_wgs84ellipsoid);
			write_xmp(
				cb, XMP_(DRONE_ALTITUDE_WGS84), val, userdata);
		}

		if (loc.horizontal_accuracy != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.2f",
				 loc.horizontal_accuracy);
			write_xmp(
				cb, XMP_(PIX4D_GPS_XY_ACCURACY), val, userdata);
		}

		if (loc.vertical_accuracy != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.2f",
				 loc.vertical_accuracy);
			write_xmp(
				cb, XMP_(PIX4D_GPS_Z_ACCURACY), val, userdata);
		}

		if (loc.sv_count != VMETA_LOCATION_INVALID_SV_COUNT) {
			snprintf(val, sizeof(val), "%u", loc.sv_count);
			write_exif(cb, EXIF_(GPS_SATELLITES), val, userdata);
		}
	}
}


static void write_frame_orientation(const struct vmeta_frame *meta,
				    vmeta_photo_write_cb_t cb,
				    void *userdata)
{
	struct vmeta_euler euler;
	if (vmeta_frame_get_frame_euler((struct vmeta_frame *)meta, &euler) ==
	    0) {
		char val[16];
		/* Convert radians to degrees */
		snprintf(val, sizeof(val), "%.6f", euler.roll * 180.0 / M_PI);
		write_xmp(cb, XMP_(CAMERA_ROLL), val, userdata);
		write_xmp(cb, XMP_(PIX4D_CAMERA_ROLL), val, userdata);

		snprintf(val, sizeof(val), "%.6f", euler.pitch * 180.0 / M_PI);
		write_xmp(cb, XMP_(CAMERA_PITCH), val, userdata);
		write_xmp(cb, XMP_(PIX4D_CAMERA_PITCH), val, userdata);

		snprintf(val, sizeof(val), "%.6f", euler.yaw * 180.0 / M_PI);
		write_xmp(cb, XMP_(CAMERA_YAW), val, userdata);
		write_xmp(cb, XMP_(PIX4D_CAMERA_YAW), val, userdata);
	}

	struct vmeta_quaternion base_quat;
	if (vmeta_frame_get_frame_base_quat((struct vmeta_frame *)meta,
					    &base_quat) == 0) {
		char val[64];
		snprintf(val,
			 sizeof(val),
			 "%.5f,%.5f,%.5f,%.5f",
			 base_quat.w,
			 base_quat.x,
			 base_quat.y,
			 base_quat.z);
		write_xmp(cb, XMP_(DRONE_CAMERA_NED_START_QUAT), val, userdata);
	}
}


static void write_frame_exposure(const struct vmeta_frame *meta,
				 vmeta_photo_write_cb_t cb,
				 void *userdata)
{
	float exposure_time = 0;
	if (vmeta_frame_get_exposure_time((struct vmeta_frame *)meta,
					  &exposure_time) == 0 &&
	    exposure_time > 0) {
		char val[16];
		/* Exposure time is in ms, convert to seconds */
		snprintf(val, sizeof(val), "%.6f", exposure_time / 1000.0);
		write_exif(cb, EXIF_(EXPOSURE_TIME), val, userdata);

		/* ShutterSpeedValue (APEX) = -log2(exposure_time_sec) */
		snprintf(val,
			 sizeof(val),
			 "%.4f",
			 -log2(exposure_time / 1000.0));
		write_exif(cb, EXIF_(SHUTTER_SPEED_VALUE), val, userdata);
	}

	uint32_t iso_speed = 0;
	if (vmeta_frame_get_iso_speed((struct vmeta_frame *)meta, &iso_speed) ==
		    0 &&
	    iso_speed > 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", iso_speed);
		write_exif(cb, EXIF_(ISO_SPEED_RATINGS), val, userdata);
		write_exif(cb, EXIF_(ISO_SPEED), val, userdata);
	}
}


static void write_frame_levels(const struct vmeta_frame *meta,
			       vmeta_photo_write_cb_t cb,
			       void *userdata)
{
	uint16_t black_level = 0;
	if (vmeta_frame_get_black_level((struct vmeta_frame *)meta,
					&black_level) == 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", black_level);
		write_exif(cb, EXIF_(BLACK_LEVEL), val, userdata);
	}

	uint16_t white_level = 0;
	if (vmeta_frame_get_white_level((struct vmeta_frame *)meta,
					&white_level) == 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", white_level);
		write_exif(cb, EXIF_(WHITE_LEVEL), val, userdata);
	}
}


static void write_frame_optics(const struct vmeta_frame *meta,
			       vmeta_photo_write_cb_t cb,
			       void *userdata)
{
	double ground_distance = 0;
	if (vmeta_frame_get_ground_distance((struct vmeta_frame *)meta,
					    &ground_distance) == 0) {
		char val[32];
		snprintf(val, sizeof(val), "%.2f/1", ground_distance);
		write_xmp(
			cb, XMP_(CAMERA_ABOVE_GROUND_ALTITUDE), val, userdata);
	}

	struct vmeta_xy pp;
	if (vmeta_frame_get_camera_principal_point((struct vmeta_frame *)meta,
						   &pp) == 0) {
		char val[64];
		snprintf(val, sizeof(val), "%.8f,%.8f", pp.x, pp.y);
		write_xmp(cb, XMP_(PRINCIPAL_POINT), val, userdata);
	}

	uint16_t calibration_illuminant_1 = 0;
	if (vmeta_frame_get_calibration_illuminant_1(
		    (struct vmeta_frame *)meta, &calibration_illuminant_1) ==
	    0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", calibration_illuminant_1);
		write_exif(cb, EXIF_(CALIBRATION_ILLUMINANT_1), val, userdata);
	}

	float awb_r_gain = 0.0f;
	float awb_b_gain = 0.0f;
	if (vmeta_frame_get_awb_r_gain((struct vmeta_frame *)meta,
				       &awb_r_gain) == 0 &&
	    vmeta_frame_get_awb_b_gain((struct vmeta_frame *)meta,
				       &awb_b_gain) == 0) {

		if (awb_r_gain > 0.0f && awb_b_gain > 0.0f) {
			char val[64];
			snprintf(val,
				 sizeof(val),
				 "%.6f,%.6f,%.6f",
				 1.0 / awb_r_gain,
				 1.0,
				 1.0 / awb_b_gain);
			write_exif(cb, EXIF_(AS_SHOT_NEUTRAL), val, userdata);
		}
	}
}


static void write_frame_color_matrix(const struct vmeta_frame *meta,
				     vmeta_photo_write_cb_t cb,
				     void *userdata)
{
	size_t cm_count = 0;
	double *cm = NULL;
	char *val = NULL;
	int res;
	size_t len = 0;
	size_t offset = 0;

	res = vmeta_frame_get_color_matrix(
		(struct vmeta_frame *)meta, NULL, &cm_count);
	if (res != 0 || cm_count == 0)
		return;

	cm = calloc(cm_count, sizeof(double));
	if (cm == NULL)
		return;

	res = vmeta_frame_get_color_matrix(
		(struct vmeta_frame *)meta, cm, &cm_count);
	if (res != 0)
		goto out;

	len = cm_count * 24;
	val = calloc(len, sizeof(char));
	if (val == NULL)
		goto out;

	for (size_t i = 0; i < cm_count; i++) {
		int ret = snprintf(val + offset,
				   len - offset,
				   "%.8f%s",
				   cm[i],
				   (i < cm_count - 1) ? "," : "");
		if (ret > 0)
			offset += (size_t)ret;
	}

	write_xmp(cb, XMP_(COLOR_MATRIX), val, userdata);
	write_exif(cb, EXIF_(COLOR_MATRIX_1), val, userdata);

out:
	free(val);
	free(cm);
}


static void write_frame_proto(const struct vmeta_frame *meta,
			      vmeta_photo_write_cb_t cb,
			      void *userdata)
{
	const Vmeta__TimedMetadata *tm = NULL;
	int res =
		vmeta_frame_proto_get_unpacked((struct vmeta_frame *)meta, &tm);
	if (res != 0 || tm == NULL)
		return;

	if (tm->photo) {
		char val[64];
		if (tm->photo->exposure_program != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->photo->exposure_program);
			write_exif(cb, EXIF_(EXPOSURE_PROGRAM), val, userdata);
		}

		snprintf(val,
			 sizeof(val),
			 "%.4f",
			 tm->photo->exposure_bias_value);
		write_exif(cb, EXIF_(EXPOSURE_BIAS), val, userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->metering_mode);
		write_exif(cb, EXIF_(METERING_MODE), val, userdata);
		snprintf(val, sizeof(val), "%u", tm->photo->light_source);
		write_exif(cb, EXIF_(LIGHT_SOURCE), val, userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->exposure_mode);
		write_exif(cb, EXIF_(EXPOSURE_MODE), val, userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->white_balance);
		write_exif(cb, EXIF_(WHITE_BALANCE), val, userdata);

		if (tm->photo->focal_length != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.2f",
				 tm->photo->focal_length);
			write_exif(cb, EXIF_(FOCAL_LENGTH), val, userdata);
			write_xmp(cb,
				  XMP_(PERSPECTIVE_FOCAL_LENGTH),
				  val,
				  userdata);
			write_xmp(cb,
				  XMP_(PERSPECTIVE_FOCAL_LENGTH_UNITS),
				  "mm",
				  userdata);
		}
		if (tm->photo->focal_length_in_35mm_film != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.0f",
				 tm->photo->focal_length_in_35mm_film);
			write_exif(cb, EXIF_(FOCAL_LENGTH_35MM), val, userdata);
		}
		if (tm->photo->f_number != 0.) {
			snprintf(val, sizeof(val), "%.2f", tm->photo->f_number);
			write_exif(cb, EXIF_(FNUMBER), val, userdata);
			write_exif(cb, EXIF_(APERTURE_VALUE), val, userdata);
		}
		snprintf(val, sizeof(val), "%u", tm->photo->contrast);
		write_exif(cb, EXIF_(CONTRAST), val, userdata);
		snprintf(val, sizeof(val), "%u", tm->photo->saturation);
		write_exif(cb, EXIF_(SATURATION), val, userdata);
		snprintf(val, sizeof(val), "%u", tm->photo->sharpness);
		write_exif(cb, EXIF_(SHARPNESS), val, userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->sequence_number);
		write_xmp(cb, XMP_(SEQUENCE_NUMBER), val, userdata);

		if (tm->photo->pixel_x_dimension != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->photo->pixel_x_dimension);
			write_exif(cb, EXIF_(PIXEL_X_DIMENSION), val, userdata);
		}
		if (tm->photo->pixel_y_dimension != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->photo->pixel_y_dimension);
			write_exif(cb, EXIF_(PIXEL_Y_DIMENSION), val, userdata);
		}
		if (tm->photo->focal_plane_x_resolution != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.4f",
				 tm->photo->focal_plane_x_resolution);
			write_exif(cb, EXIF_(FOCAL_PLANE_X_RES), val, userdata);
			write_exif(cb,
				   EXIF_(FOCAL_PLANE_RES_UNIT),
				   "3",
				   userdata); /* cm */
		}
		if (tm->photo->focal_plane_y_resolution != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.4f",
				 tm->photo->focal_plane_y_resolution);
			write_exif(cb, EXIF_(FOCAL_PLANE_Y_RES), val, userdata);
		}
		if (tm->photo->media_id != 0) {
			char val[11];
			snprintf(val,
				 sizeof(val),
				 "%" PRIu32,
				 tm->photo->media_id);
			write_xmp(cb, XMP_(MEDIA_ID), val, userdata);
		}
		if (tm->photo->resource_index != 0) {
			char val[11];
			snprintf(val,
				 sizeof(val),
				 "%" PRIu32,
				 tm->photo->resource_index);
			write_xmp(cb, XMP_(RESOURCE_INDEX), val, userdata);
		}
		if (tm->photo->sequence_number != 0) {
			char val[11];
			snprintf(val,
				 sizeof(val),
				 "%" PRIu32,
				 tm->photo->sequence_number);
			write_xmp(cb, XMP_(SEQUENCE_NUMBER), val, userdata);
		}
	}

	if (tm->camera) {
		char val[64];
		if (tm->camera->utc_timestamp_accuracy != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->camera->utc_timestamp_accuracy);
			write_xmp(cb, XMP_(UTC_TS_ACCURACY), val, userdata);
		}
		if (tm->camera->spectrum !=
		    VMETA__CAMERA_SPECTRUM__CS_UNKNOWN) {
			write_xmp(
				cb,
				XMP_(CAMERA_SPECTRUM),
				vmeta_camera_spectrum_to_str(
					/* codecheck_ignore[LONG_LINE] */
					vmeta_frame_camera_spectrum_proto_to_vmeta(
						tm->camera->spectrum)),
				userdata);
		}
		if (tm->camera->serial_number[0] != '\0') {
			write_xmp(cb,
				  XMP_(CAMERA_SERIAL_NUMBER),
				  tm->camera->serial_number,
				  userdata);
			write_exif(cb,
				   EXIF_(CAMERA_SERIAL_NUMBER),
				   tm->camera->serial_number,
				   userdata);
		}
	}

	if (tm->thermal) {
		char val[128];
		if (tm->thermal->min) {
			snprintf(val,
				 sizeof(val),
				 "%.8f,%.8f,%.2f",
				 tm->thermal->min->x,
				 tm->thermal->min->y,
				 tm->thermal->min->temp);
			write_xmp(cb, XMP_(THERMAL_SPOT_MIN), val, userdata);
		}
		if (tm->thermal->max) {
			snprintf(val,
				 sizeof(val),
				 "%.8f,%.8f,%.2f",
				 tm->thermal->max->x,
				 tm->thermal->max->y,
				 tm->thermal->max->temp);
			write_xmp(cb, XMP_(THERMAL_SPOT_MAX), val, userdata);
		}
		if (tm->thermal->mask) {
			snprintf(val,
				 sizeof(val),
				 "%.8f,%.8f,%.8f,%.8f",
				 tm->thermal->mask->x,
				 tm->thermal->mask->y,
				 tm->thermal->mask->width,
				 tm->thermal->mask->height);
			write_xmp(cb, XMP_(THERMAL_MASK), val, userdata);
		}
	}

	vmeta_frame_proto_release_unpacked((struct vmeta_frame *)meta, tm);
}


static void write_frame_constants(vmeta_photo_write_cb_t cb, void *userdata)
{
	/* Constant values for Exif tags */
	write_exif(cb, EXIF_(FLASH), "0", userdata);
	write_exif(cb, EXIF_(DIGITAL_ZOOM_RATIO), "0", userdata);
	write_exif(cb, EXIF_(SCENE_CAPTURE_TYPE), "0", userdata);
	write_exif(cb, EXIF_(ORIENTATION), "1", userdata);
	write_exif(cb, EXIF_(X_RESOLUTION), "72", userdata);
	write_exif(cb, EXIF_(Y_RESOLUTION), "72", userdata);
	write_exif(cb, EXIF_(RESOLUTION_UNIT), "2", userdata);
	write_exif(cb,
		   EXIF_(YCBCR_POSITIONING),
		   "2",
		   userdata); /* Centered for JPEG */
	write_exif(cb, EXIF_(EXIF_VERSION), "0231", userdata);
	write_exif(cb, EXIF_(COMPONENTS_CONFIG), "1230", userdata);
	write_exif(cb, EXIF_(FLASHPIX_VERSION), "0100", userdata);
	write_exif(cb, EXIF_(COLOR_SPACE), "1", userdata); /* sRGB */
	write_exif(cb, EXIF_(FILE_SOURCE), "3", userdata); /* DSC */
	write_exif(cb,
		   EXIF_(SCENE_TYPE),
		   "1",
		   userdata); /* Directly photographed */
	write_exif(cb, EXIF_(GPS_MAP_DATUM), "WGS-84", userdata);
	write_exif(cb, EXIF_(SENSITIVITY_TYPE), "3", userdata);

	/* Constant values for Xmp tags */
	write_xmp(cb, XMP_(CAMERA_HORIZ_CS), "EPSG:4326", userdata);
	write_xmp(cb, XMP_(CAMERA_VERT_CS), "ellipsoidal", userdata);
}


int vmeta_frame_photo_write(const struct vmeta_frame *meta,
			    vmeta_photo_write_cb_t cb,
			    void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	write_frame_timestamps(meta, cb, userdata);
	write_frame_location(meta, cb, userdata);
	write_frame_orientation(meta, cb, userdata);
	write_frame_exposure(meta, cb, userdata);
	write_frame_levels(meta, cb, userdata);
	write_frame_optics(meta, cb, userdata);
	write_frame_color_matrix(meta, cb, userdata);

	/* Handle PROTO specific fields */
	if (meta->type == VMETA_FRAME_TYPE_PROTO)
		write_frame_proto(meta, cb, userdata);

	write_frame_constants(cb, userdata);

	return 0;
}


#undef EXIF_
#undef XMP_
