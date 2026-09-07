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

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Shorthand macros for metadata indices */
#define EXIF_(_name) PMETA_DEFS_EXIF_IDX_##_name
#define XMP_(_name) PMETA_DEFS_XMP_IDX_##_name


struct vmeta_photo_write_ctx {
	const struct vmeta_session *session;
	struct vmeta_frame *frame;
	vmeta_photo_write_cb_t cb;
	void *userdata;
};


static enum vmeta_camera_model_type
get_resolved_camera_model_type(const struct vmeta_photo_write_ctx *ctx)
{
	int res;
	const Vmeta__TimedMetadata *tm = NULL;
	enum vmeta_camera_model_type type = VMETA_CAMERA_MODEL_TYPE_UNKNOWN;

	if (!ctx->frame || ctx->frame->type != VMETA_FRAME_TYPE_PROTO)
		goto fallback;

	res = vmeta_frame_proto_get_unpacked(ctx->frame, &tm);
	if (res != 0 || tm == NULL)
		goto fallback;

	if (tm->photo && tm->photo->camera_model) {
		const Vmeta__CameraModel *model = tm->photo->camera_model;
		if (model->id_case == VMETA__CAMERA_MODEL__ID_PERSPECTIVE)
			type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
		else if (model->id_case == VMETA__CAMERA_MODEL__ID_FISHEYE)
			type = VMETA_CAMERA_MODEL_TYPE_FISHEYE;
	}
	vmeta_frame_proto_release_unpacked(ctx->frame, tm);

	if (type != VMETA_CAMERA_MODEL_TYPE_UNKNOWN)
		return type;

fallback:
	if (ctx->session)
		return ctx->session->camera_model.type;

	return VMETA_CAMERA_MODEL_TYPE_UNKNOWN;
}


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


static int format_exif_date(uint64_t utc_ts,
			    int32_t date_gmtoff,
			    char *out_date,
			    size_t max_len)
{
	int res;
	/* EXIF date format is fixed at 19 characters: "YYYY:MM:DD HH:MM:SS" */
	const size_t exif_date_len = 19;

	if (out_date == NULL || max_len <= exif_date_len)
		return -EINVAL;

	/* Get local time in ISO 8601 long format: "YYYY-MM-DDTHH:MM:SS+HH:MM"
	 */
	res = time_local_format(utc_ts / 1000000,
				date_gmtoff,
				TIME_FMT_LONG,
				out_date,
				max_len);
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

	if (meta->build_id[0] != '\0') {
		write_exif(cb, EXIF_(SOFTWARE), meta->build_id, userdata);
		write_xmp(cb, XMP_(TIFF_SOFTWARE), meta->build_id, userdata);
		write_xmp(
			cb, XMP_(SOFTWARE_BUILD_ID), meta->build_id, userdata);
	}
}


static void write_dates_internal(uint64_t date_ts_us,
				 int32_t date_gmtoff,
				 vmeta_photo_write_cb_t cb,
				 void *userdata)
{
	int ret;
	char date[VMETA_SESSION_DATE_MAX_LEN];
	char tz[10];

	format_exif_date(date_ts_us, date_gmtoff, date, sizeof(date));
	write_exif(cb, EXIF_(DATETIME), date, userdata);
	write_exif(cb, EXIF_(DATETIME_ORIGINAL), date, userdata);
	write_exif(cb, EXIF_(DATETIME_DIGITIZED), date, userdata);

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
	write_exif(cb, EXIF_(OFFSET_TIME_DIGITIZED), tz, userdata);

	/* XMP Dates (ISO 8601) */
	ret = time_local_format(date_ts_us / 1000000,
				date_gmtoff,
				TIME_FMT_ISO8601_LONG,
				date,
				sizeof(date));
	if (ret >= 0) {
		write_xmp(cb, XMP_(DATETIME_ORIGINAL), date, userdata);
		write_xmp(cb, XMP_(CREATE_DATE), date, userdata);
		write_xmp(cb, XMP_(MODIFY_DATE), date, userdata);
		write_xmp(cb, XMP_(DC_DATE), date, userdata);
	}

	char subsec[4];
	snprintf(subsec,
		 sizeof(subsec),
		 "%03" PRIu64,
		 (date_ts_us / 1000) % 1000);

	write_exif(cb, EXIF_(SUBSEC_TIME), subsec, userdata);
	write_exif(cb, EXIF_(SUBSEC_TIME_ORIGINAL), subsec, userdata);
	write_exif(cb, EXIF_(SUBSEC_TIME_DIGITIZED), subsec, userdata);
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
	uint64_t date_ts_us = 0;
	int32_t date_gmtoff = 0;

	if (meta->media_date != 0) {
		date_ts_us = meta->media_date * 1000000ULL;
		date_gmtoff = meta->media_date_gmtoff;
		write_dates_internal(date_ts_us, date_gmtoff, cb, userdata);
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


static void write_frame_camera_model(const Vmeta__PhotoMetadata *photo_meta,
				     vmeta_photo_write_cb_t cb,
				     void *userdata)
{
	if (!photo_meta || !photo_meta->camera_model)
		return;

	const Vmeta__CameraModel *model = photo_meta->camera_model;

	/* Clear previous model specific tags */
	write_xmp(cb, XMP_(PERSPECTIVE_DISTORTION), NULL, userdata);
	write_xmp(cb, XMP_(FISHEYE_AFFINE_MATRIX), NULL, userdata);
	write_xmp(cb, XMP_(FISHEYE_AFFINE_SYMMETRIC), NULL, userdata);
	write_xmp(cb, XMP_(FISHEYE_POLYNOMIAL), NULL, userdata);

	if (model->id_case == VMETA__CAMERA_MODEL__ID_PERSPECTIVE &&
	    model->perspective) {
		char dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
		write_xmp(cb,
			  XMP_(CAMERA_MODEL_TYPE),
			  vmeta_camera_model_type_to_str(
				  VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE),
			  userdata);

		if (model->perspective->distorsion) {
			ssize_t ret =
				vmeta_session_perspective_distortion_write(
					dist,
					sizeof(dist),
					model->perspective->distorsion->r1,
					model->perspective->distorsion->r2,
					model->perspective->distorsion->r3,
					model->perspective->distorsion->t1,
					model->perspective->distorsion->t2);
			if (ret > 0) {
				write_xmp(cb,
					  XMP_(PERSPECTIVE_DISTORTION),
					  dist,
					  userdata);
			}
		}
	} else if (model->id_case == VMETA__CAMERA_MODEL__ID_FISHEYE &&
		   model->fisheye) {
		char matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
		char coef[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
		write_xmp(cb,
			  XMP_(CAMERA_MODEL_TYPE),
			  vmeta_camera_model_type_to_str(
				  VMETA_CAMERA_MODEL_TYPE_FISHEYE),
			  userdata);

		if (model->fisheye->affine_matrix) {
			ssize_t ret = vmeta_session_fisheye_affine_matrix_write(
				matrix,
				sizeof(matrix),
				model->fisheye->affine_matrix->c,
				model->fisheye->affine_matrix->d,
				model->fisheye->affine_matrix->e,
				model->fisheye->affine_matrix->f);
			if (ret > 0) {
				write_xmp(cb,
					  XMP_(FISHEYE_AFFINE_MATRIX),
					  matrix,
					  userdata);

				if (model->fisheye->affine_matrix->symmetric) {
					char val[11];
					snprintf(val,
						 sizeof(val),
						 "%u",
						 model->fisheye->affine_matrix
							 ->symmetric->value);
					write_xmp(
						cb,
						XMP_(FISHEYE_AFFINE_SYMMETRIC),
						val,
						userdata);
				}
			}
		}

		if (model->fisheye->polynomial) {
			ssize_t ret = vmeta_session_fisheye_polynomial_write(
				coef,
				sizeof(coef),
				model->fisheye->polynomial->p2,
				model->fisheye->polynomial->p3,
				model->fisheye->polynomial->p4);
			if (ret > 0)
				write_xmp(cb,
					  XMP_(FISHEYE_POLYNOMIAL),
					  coef,
					  userdata);
		}
	}
}


static void write_frame_timestamps(struct vmeta_frame *meta,
				   vmeta_photo_write_cb_t cb,
				   void *userdata)
{
	uint64_t ts = 0;
	if (vmeta_frame_get_frame_timestamp(meta, &ts) == 0) {
		char val[21];
		snprintf(val, sizeof(val), "%" PRIu64, ts);
		write_xmp(cb, XMP_(CAPTURE_TS_US), val, userdata);
	}
}


#define FRAC_MAX_TERMS 30
#define FRAC_MIN_DIVISOR 1e-10
#define FRAC_MAX_ERROR 1e-8


static int get_gcd(int a, int b)
{
	while (b != 0) {
		int temp = b;
		b = a % b;
		a = temp;
	}
	return a;
}


static void double_to_fraction(double src, int *dest_n, int *dest_d)
{
	double V;
	double F;
	int N = 1;
	int D = 1;
	int A;
	int64_t N1 = 1;
	int64_t D1 = 0;
	int64_t N2 = 0;
	int64_t D2 = 1;
	int gcd_val;
	int negative = 0;

	F = src;
	if (F < 0.0) {
		F = -F;
		negative = 1;
	}
	V = F;

	for (int i = 0; i < FRAC_MAX_TERMS; i++) {
		A = (int)F;
		F = F - A;

		N2 = N1 * A + N2;
		D2 = D1 * A + D2;

		/* Overflow protection */
		if (N2 > INT_MAX || D2 > INT_MAX)
			break;

		N = (int)N2;
		D = (int)D2;

		N2 = N1;
		D2 = D1;
		N1 = N;
		D1 = D;

		if (F < FRAC_MIN_DIVISOR ||
		    fabs(V - ((double)N) / D) < FRAC_MAX_ERROR) {
			break;
		}
		F = 1.0 / F;
	}

	if (D == 0) {
		N = INT_MAX;
		D = 1;
	}

	if (negative)
		N = -N;

	gcd_val = get_gcd(abs(N), abs(D));
	if (gcd_val > 0) {
		N /= gcd_val;
		D /= gcd_val;
	}

	*dest_n = N;
	*dest_d = D;
}


static void format_xmp_gps_coordinate(char *buf,
				      size_t buf_size,
				      double value,
				      char pos_ref,
				      char neg_ref)
{
	char direction;

	if (value < 0) {
		direction = neg_ref;
		value = fabs(value);
	} else {
		direction = pos_ref;
	}

	int degrees = (int)value;
	double minutes = (value - degrees) * 60.0;

	snprintf(buf, buf_size, "%d,%.8f%c", degrees, minutes, direction);
}


static inline void
format_xmp_gps_latitude(char *buf, size_t buf_size, double lat)
{
	format_xmp_gps_coordinate(buf, buf_size, lat, 'N', 'S');
}


static inline void
format_xmp_gps_longitude(char *buf, size_t buf_size, double lon)
{
	format_xmp_gps_coordinate(buf, buf_size, lon, 'E', 'W');
}


static inline void
format_fraction_string(char *buf, size_t buf_size, double value)
{
	int num;
	int den;
	double_to_fraction(value, &num, &den);
	snprintf(buf, buf_size, "%d/%d", num, den);
}


static inline void
format_exif_gps_coordinate_dms(char *buf, size_t buf_size, double value)
{
	value = fabs(value);
	int degrees = (int)value;
	double minutes_full = (value - degrees) * 60.0;
	int minutes = (int)minutes_full;
	double seconds = (minutes_full - minutes) * 60.0;

	const long sec_den = 100000;
	long sec_num = (long)(seconds * (double)sec_den + 0.5);

	snprintf(buf,
		 buf_size,
		 "%d/1,%d/1,%ld/%ld",
		 degrees,
		 minutes,
		 sec_num,
		 sec_den);
}


static void write_frame_location(struct vmeta_frame *meta,
				 vmeta_photo_write_cb_t cb,
				 void *userdata)
{
	/* Camera location */
	struct vmeta_location loc;
	if (vmeta_frame_get_camera_location(meta, &loc) == 0 && loc.valid) {
		char val[64];

		/* Latitude */
		format_exif_gps_coordinate_dms(val, sizeof(val), loc.latitude);
		write_exif(cb, EXIF_(GPS_LATITUDE), val, userdata);
		write_exif(cb,
			   EXIF_(GPS_LATITUDE_REF),
			   loc.latitude >= 0 ? "N" : "S",
			   userdata);

		format_xmp_gps_latitude(val, sizeof(val), loc.latitude);
		write_xmp(cb, XMP_(GPS_LATITUDE), val, userdata);

		/* Longitude */
		format_exif_gps_coordinate_dms(val, sizeof(val), loc.longitude);
		write_exif(cb, EXIF_(GPS_LONGITUDE), val, userdata);
		write_exif(cb,
			   EXIF_(GPS_LONGITUDE_REF),
			   loc.longitude >= 0 ? "E" : "W",
			   userdata);

		format_xmp_gps_longitude(val, sizeof(val), loc.longitude);
		write_xmp(cb, XMP_(GPS_LONGITUDE), val, userdata);

		/* Altitude above takeoff */
		double altitude_ato = 0;
		if (vmeta_frame_get_altitude_above_takeoff(
			    meta, &altitude_ato) == 0) {
			format_fraction_string(val, sizeof(val), altitude_ato);
			write_xmp(cb,
				  XMP_(CAMERA_ABOVE_GROUND_ALTITUDE),
				  val,
				  userdata);
		}

		/* Altitude (EGM96 AMSL) */
		if (!isnan(loc.altitude_egm96amsl)) {
			snprintf(val,
				 sizeof(val),
				 "%.4f",
				 fabs(loc.altitude_egm96amsl));
			write_xmp(cb, XMP_(ALTITUDE_AMSL), val, userdata);
		}

		/* Altitude (WGS84 Ellipsoid) */
		if (!isnan(loc.altitude_wgs84ellipsoid)) {
			format_fraction_string(
				val,
				sizeof(val),
				fabs(loc.altitude_wgs84ellipsoid));
			write_exif(cb, EXIF_(GPS_ALTITUDE), val, userdata);
			write_xmp(cb, XMP_(GPS_ALTITUDE), val, userdata);
			write_exif(cb,
				   EXIF_(GPS_ALTITUDE_REF),
				   loc.altitude_wgs84ellipsoid >= 0 ? "0" : "1",
				   userdata);
			write_xmp(cb,
				  XMP_(GPS_ALTITUDE_REF),
				  loc.altitude_wgs84ellipsoid >= 0 ? "0" : "1",
				  userdata);
		}

		if (loc.horizontal_accuracy != 0.) {
			format_fraction_string(
				val, sizeof(val), loc.horizontal_accuracy);
			write_xmp(
				cb, XMP_(PIX4D_GPS_XY_ACCURACY), val, userdata);
		}

		if (loc.vertical_accuracy != 0.) {
			format_fraction_string(
				val, sizeof(val), loc.vertical_accuracy);
			write_xmp(
				cb, XMP_(PIX4D_GPS_Z_ACCURACY), val, userdata);
		}

		if (loc.sv_count != VMETA_LOCATION_INVALID_SV_COUNT) {
			snprintf(val, sizeof(val), "%u", loc.sv_count);
			write_exif(cb, EXIF_(GPS_SATELLITES), val, userdata);
		}

		/* Constants */
		write_exif(cb, EXIF_(GPS_VERSION_ID), "2300", userdata);
		write_exif(cb, EXIF_(GPS_MAP_DATUM), "WGS-84", userdata);
		write_xmp(cb, XMP_(CAMERA_HORIZ_CS), "EPSG:4326", userdata);
		write_xmp(cb, XMP_(CAMERA_VERT_CS), "ellipsoidal", userdata);
	}

	/* Drone location */
	if (vmeta_frame_get_location(meta, &loc) == 0 && loc.valid) {
		char val[32];

		/* Latitude */
		snprintf(val, sizeof(val), "%.8f", loc.latitude);
		write_xmp(cb, XMP_(DRONE_LATITUDE), val, userdata);

		/* Longitude */
		snprintf(val, sizeof(val), "%.8f", loc.longitude);
		write_xmp(cb, XMP_(DRONE_LONGITUDE), val, userdata);

		/* Altitude (EGM96 AMSL) */
		if (!isnan(loc.altitude_egm96amsl)) {
			snprintf(val,
				 sizeof(val),
				 "%.4f",
				 loc.altitude_egm96amsl);
			write_xmp(cb, XMP_(DRONE_ALTITUDE_AMSL), val, userdata);
		}

		/* Altitude (WGS84 Ellipsoid) */
		if (!isnan(loc.altitude_wgs84ellipsoid)) {
			snprintf(val,
				 sizeof(val),
				 "%.4f",
				 loc.altitude_wgs84ellipsoid);
			write_xmp(cb,
				  XMP_(DRONE_ALTITUDE_ELLIPSOID),
				  val,
				  userdata);
		}
	}
}


static int quaternion_mul(struct vmeta_quaternion *q_res,
			  const struct vmeta_quaternion *q1,
			  const struct vmeta_quaternion *q2)
{
	struct vmeta_quaternion tq; /* in case res aliases q1 or q2 */

	if (!q_res || !q1 || !q2)
		return -1;

	tq.x = q1->x * q2->w + q1->y * q2->z - q1->z * q2->y + q1->w * q2->x;
	tq.y = -q1->x * q2->z + q1->y * q2->w + q1->z * q2->x + q1->w * q2->y;
	tq.z = q1->x * q2->y - q1->y * q2->x + q1->z * q2->w + q1->w * q2->z;
	tq.w = -q1->x * q2->x - q1->y * q2->y - q1->z * q2->z + q1->w * q2->w;

	*q_res = tq;

	return 0;
}


static void write_frame_pix4d_orientation(struct vmeta_frame *meta,
					  vmeta_photo_write_cb_t cb,
					  void *userdata)
{
	char val[16];
	struct vmeta_euler euler;
	const struct vmeta_quaternion q_90y = {
		0.70710678f, 0.f, 0.70710678f, 0.f};
	struct vmeta_quaternion quat;
	struct vmeta_quaternion pix4d_quat;

	if (vmeta_frame_get_frame_quat(meta, &quat) != 0)
		return;

	quaternion_mul(&pix4d_quat, &quat, &q_90y);

	vmeta_quat_to_euler_zyx(&pix4d_quat, &euler);

	/* Convert radians to degrees */
	snprintf(val, sizeof(val), "%.6f", euler.yaw * 180.0 / M_PI);
	write_xmp(cb, XMP_(PIX4D_CAMERA_YAW), val, userdata);

	snprintf(val, sizeof(val), "%.6f", euler.roll * 180.0 / M_PI);
	write_xmp(cb, XMP_(PIX4D_CAMERA_ROLL), val, userdata);

	snprintf(val, sizeof(val), "%.6f", euler.pitch * 180.0 / M_PI);
	write_xmp(cb, XMP_(PIX4D_CAMERA_PITCH), val, userdata);
}


static void write_frame_orientation(struct vmeta_frame *meta,
				    vmeta_photo_write_cb_t cb,
				    void *userdata)
{
	struct vmeta_euler euler;
	if (vmeta_frame_get_frame_euler(meta, &euler) == 0) {
		char val[16];
		/* Convert radians to degrees */
		snprintf(val, sizeof(val), "%.6f", euler.roll * 180.0 / M_PI);
		write_xmp(cb, XMP_(CAMERA_ROLL), val, userdata);

		snprintf(val, sizeof(val), "%.6f", euler.pitch * 180.0 / M_PI);
		write_xmp(cb, XMP_(CAMERA_PITCH), val, userdata);

		snprintf(val, sizeof(val), "%.6f", euler.yaw * 180.0 / M_PI);
		write_xmp(cb, XMP_(CAMERA_YAW), val, userdata);
	}

	struct vmeta_quaternion quat;
	if (vmeta_frame_get_frame_local_quat(meta, &quat) == 0) {
		char val[64];
		snprintf(val,
			 sizeof(val),
			 "%.8f,%.8f,%.8f,%.8f",
			 quat.w,
			 quat.x,
			 quat.y,
			 quat.z);
		write_xmp(cb, XMP_(DRONE_CAMERA_NED_START_QUAT), val, userdata);
	}

	write_frame_pix4d_orientation(meta, cb, userdata);
}


static void write_frame_exposure(struct vmeta_frame *meta,
				 vmeta_photo_write_cb_t cb,
				 void *userdata)
{
	float exposure_time = 0;
	if (vmeta_frame_get_exposure_time(meta, &exposure_time) == 0 &&
	    exposure_time > 0) {
		char val[32];
		/* Exposure time is in ms, convert to seconds */
		format_fraction_string(
			val, sizeof(val), exposure_time / 1000.0);
		write_exif(cb, EXIF_(EXPOSURE_TIME), val, userdata);
		write_xmp(cb, XMP_(EXPOSURE_TIME), val, userdata);

		/* ShutterSpeedValue (APEX) = -log2(exposure_time_sec) */
		snprintf(val,
			 sizeof(val),
			 "%.9f",
			 -log2(exposure_time / 1000.0));
		write_exif(cb, EXIF_(SHUTTER_SPEED_VALUE), val, userdata);
	}

	uint32_t iso_speed = 0;
	if (vmeta_frame_get_iso_speed(meta, &iso_speed) == 0 && iso_speed > 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", iso_speed);
		write_exif(cb, EXIF_(ISO_SPEED_RATINGS), val, userdata);
		write_xmp(cb, XMP_(ISO_SPEED_RATINGS), val, userdata);
		write_exif(cb, EXIF_(ISO_SPEED), val, userdata);
	}
}


static void write_frame_levels(struct vmeta_frame *meta,
			       vmeta_photo_write_cb_t cb,
			       void *userdata)
{
	uint16_t black_level = 0;
	if (vmeta_frame_get_black_level(meta, &black_level) == 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", black_level);
		write_exif(cb, EXIF_(BLACK_LEVEL), val, userdata);
	}

	uint16_t white_level = 0;
	if (vmeta_frame_get_white_level(meta, &white_level) == 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", white_level);
		write_exif(cb, EXIF_(WHITE_LEVEL), val, userdata);
	}
}


static void write_frame_optics(struct vmeta_frame *meta,
			       vmeta_photo_write_cb_t cb,
			       void *userdata)
{
	struct vmeta_xy pp;
	if (vmeta_frame_get_camera_principal_point(meta, &pp) == 0) {
		char val[64];
		snprintf(val, sizeof(val), "%.8f,%.8f", pp.x, pp.y);
		write_xmp(cb, XMP_(PRINCIPAL_POINT), val, userdata);
	}

	uint16_t calibration_illuminant_1 = 0;
	if (vmeta_frame_get_calibration_illuminant_1(
		    meta, &calibration_illuminant_1) == 0) {
		char val[16];
		snprintf(val, sizeof(val), "%u", calibration_illuminant_1);
		write_exif(cb, EXIF_(CALIBRATION_ILLUMINANT_1), val, userdata);
	}

	float awb_r_gain = 0.0f;
	float awb_b_gain = 0.0f;
	if ((vmeta_frame_get_awb_r_gain(meta, &awb_r_gain) == 0) &&
	    (vmeta_frame_get_awb_b_gain(meta, &awb_b_gain) == 0) &&
	    (awb_r_gain > 0.0f) && (awb_b_gain > 0.0f)) {
		char val[128];
		snprintf(val,
			 sizeof(val),
			 "%.9f,%.9f,%.9f",
			 1.0 / awb_r_gain,
			 1.0,
			 1.0 / awb_b_gain);
		write_exif(cb, EXIF_(AS_SHOT_NEUTRAL), val, userdata);
	}
}


static void write_frame_color_matrix(struct vmeta_frame *meta,
				     vmeta_photo_write_cb_t cb,
				     void *userdata)
{
	size_t cm_count = 0;
	double *cm = NULL;
	char *val = NULL;
	int res;
	size_t len = 0;
	size_t offset = 0;

	res = vmeta_frame_get_color_matrix(meta, NULL, &cm_count);
	if (res != 0 || cm_count == 0)
		return;

	cm = calloc(cm_count, sizeof(double));
	if (cm == NULL)
		return;

	res = vmeta_frame_get_color_matrix(meta, cm, &cm_count);
	if (res != 0)
		goto out;

	len = cm_count * 24;
	val = calloc(len, sizeof(char));
	if (val == NULL)
		goto out;

	for (size_t i = 0; i < cm_count; i++) {
		int ret = snprintf(val + offset,
				   len - offset,
				   "%.9f%s",
				   cm[i],
				   (i < cm_count - 1) ? "," : "");
		if (ret > 0)
			offset += (size_t)ret;
	}

	write_exif(cb, EXIF_(COLOR_MATRIX_1), val, userdata);

out:
	free(val);
	free(cm);
}


static void write_frame_proto(const struct vmeta_photo_write_ctx *ctx)
{
	const Vmeta__TimedMetadata *tm = NULL;
	int res = vmeta_frame_proto_get_unpacked(ctx->frame, &tm);
	if (res != 0 || tm == NULL)
		return;

	if (tm->photo) {
		char val[64];
		if (tm->photo->exposure_program != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->photo->exposure_program);
			write_exif(ctx->cb,
				   EXIF_(EXPOSURE_PROGRAM),
				   val,
				   ctx->userdata);
		}

		if (tm->photo->photo_date != 0) {
			write_dates_internal(tm->photo->photo_date,
					     tm->photo->photo_date_gmtoff,
					     ctx->cb,
					     ctx->userdata);
		}

		format_fraction_string(
			val, sizeof(val), tm->photo->exposure_bias_value);
		write_exif(ctx->cb, EXIF_(EXPOSURE_BIAS), val, ctx->userdata);
		write_xmp(ctx->cb, XMP_(EXPOSURE_BIAS), val, ctx->userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->metering_mode);
		write_exif(ctx->cb, EXIF_(METERING_MODE), val, ctx->userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->light_source);
		write_exif(ctx->cb, EXIF_(LIGHT_SOURCE), val, ctx->userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->exposure_mode);
		write_exif(ctx->cb, EXIF_(EXPOSURE_MODE), val, ctx->userdata);

		snprintf(val, sizeof(val), "%u", tm->photo->white_balance);
		write_exif(ctx->cb, EXIF_(WHITE_BALANCE), val, ctx->userdata);

		if (tm->photo->focal_length != 0.) {
			format_fraction_string(
				val, sizeof(val), tm->photo->focal_length);
			write_exif(ctx->cb,
				   EXIF_(FOCAL_LENGTH),
				   val,
				   ctx->userdata);

			enum vmeta_camera_model_type model_type =
				get_resolved_camera_model_type(ctx);
			if (model_type == VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE) {
				write_xmp(ctx->cb,
					  XMP_(PERSPECTIVE_FOCAL_LENGTH),
					  val,
					  ctx->userdata);
				write_xmp(ctx->cb,
					  XMP_(PERSPECTIVE_FOCAL_LENGTH_UNITS),
					  "mm",
					  ctx->userdata);
			}
		}
		if (tm->photo->focal_length_in_35mm_film != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.0f",
				 tm->photo->focal_length_in_35mm_film);
			write_exif(ctx->cb,
				   EXIF_(FOCAL_LENGTH_35MM),
				   val,
				   ctx->userdata);
		}
		if (tm->photo->f_number != 0.) {
			snprintf(val, sizeof(val), "%.9f", tm->photo->f_number);
			write_exif(ctx->cb, EXIF_(FNUMBER), val, ctx->userdata);
			snprintf(val,
				 sizeof(val),
				 "%.9f",
				 2.0 * log2(tm->photo->f_number));
			write_exif(ctx->cb,
				   EXIF_(APERTURE_VALUE),
				   val,
				   ctx->userdata);
		}
		snprintf(val, sizeof(val), "%u", tm->photo->contrast);
		write_exif(ctx->cb, EXIF_(CONTRAST), val, ctx->userdata);
		snprintf(val, sizeof(val), "%u", tm->photo->saturation);
		write_exif(ctx->cb, EXIF_(SATURATION), val, ctx->userdata);
		snprintf(val, sizeof(val), "%u", tm->photo->sharpness);
		write_exif(ctx->cb, EXIF_(SHARPNESS), val, ctx->userdata);

		if (tm->photo->pixel_x_dimension != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->photo->pixel_x_dimension);
			write_exif(ctx->cb,
				   EXIF_(PIXEL_X_DIMENSION),
				   val,
				   ctx->userdata);
		}
		if (tm->photo->pixel_y_dimension != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->photo->pixel_y_dimension);
			write_exif(ctx->cb,
				   EXIF_(PIXEL_Y_DIMENSION),
				   val,
				   ctx->userdata);
		}
		if (tm->photo->focal_plane_x_resolution != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.6f",
				 tm->photo->focal_plane_x_resolution);
			write_exif(ctx->cb,
				   EXIF_(FOCAL_PLANE_X_RES),
				   val,
				   ctx->userdata);
			write_exif(ctx->cb,
				   EXIF_(FOCAL_PLANE_RES_UNIT),
				   "3",
				   ctx->userdata); /* cm */
		}
		if (tm->photo->focal_plane_y_resolution != 0.) {
			snprintf(val,
				 sizeof(val),
				 "%.6f",
				 tm->photo->focal_plane_y_resolution);
			write_exif(ctx->cb,
				   EXIF_(FOCAL_PLANE_Y_RES),
				   val,
				   ctx->userdata);
		}
		if (tm->photo->media_id != 0) {
			snprintf(val,
				 sizeof(val),
				 "%" PRIu32,
				 tm->photo->media_id);
			write_xmp(ctx->cb, XMP_(MEDIA_ID), val, ctx->userdata);
		}
		if (tm->photo->resource_index != 0) {
			snprintf(val,
				 sizeof(val),
				 "%" PRIu32,
				 tm->photo->resource_index);
			write_xmp(ctx->cb,
				  XMP_(RESOURCE_INDEX),
				  val,
				  ctx->userdata);
		}

		if (ctx->session &&
		    ctx->session->photo_mode == VMETA_PHOTO_MODE_PANORAMA) {
			snprintf(val,
				 sizeof(val),
				 "%" PRIu32,
				 tm->photo->sequence_number);
			write_xmp(ctx->cb,
				  XMP_(SEQUENCE_NUMBER),
				  val,
				  ctx->userdata);
		}

		if (tm->photo->camera_model)
			write_frame_camera_model(
				tm->photo, ctx->cb, ctx->userdata);
	}

	if (tm->camera) {
		char val[64];
		if (tm->camera->utc_timestamp_accuracy != 0) {
			snprintf(val,
				 sizeof(val),
				 "%u",
				 tm->camera->utc_timestamp_accuracy);
			write_xmp(ctx->cb,
				  XMP_(UTC_TS_ACCURACY),
				  val,
				  ctx->userdata);
		}
		if (tm->camera->spectrum !=
		    VMETA__CAMERA_SPECTRUM__CS_UNKNOWN) {
			write_xmp(
				ctx->cb,
				XMP_(CAMERA_SPECTRUM),
				vmeta_camera_spectrum_to_str(
					/* codecheck_ignore[LONG_LINE] */
					vmeta_frame_camera_spectrum_proto_to_vmeta(
						tm->camera->spectrum)),
				ctx->userdata);
		}
		if (tm->camera->serial_number[0] != '\0') {
			write_xmp(ctx->cb,
				  XMP_(CAMERA_SERIAL_NUMBER),
				  tm->camera->serial_number,
				  ctx->userdata);
			write_exif(ctx->cb,
				   EXIF_(CAMERA_SERIAL_NUMBER),
				   tm->camera->serial_number,
				   ctx->userdata);
		}
	}

	if (tm->thermal) {
		char val[128];
		if (tm->thermal->min) {
			snprintf(val,
				 sizeof(val),
				 "%.8f,%.8f,%u",
				 tm->thermal->min->x,
				 tm->thermal->min->y,
				 tm->thermal->min->value);
			write_xmp(ctx->cb,
				  XMP_(THERMAL_SPOT_MIN),
				  val,
				  ctx->userdata);
		}
		if (tm->thermal->max) {
			snprintf(val,
				 sizeof(val),
				 "%.8f,%.8f,%u",
				 tm->thermal->max->x,
				 tm->thermal->max->y,
				 tm->thermal->max->value);
			write_xmp(ctx->cb,
				  XMP_(THERMAL_SPOT_MAX),
				  val,
				  ctx->userdata);
		}
		if (tm->thermal->mask) {
			snprintf(val,
				 sizeof(val),
				 "%.8f,%.8f,%.8f,%.8f",
				 tm->thermal->mask->x,
				 tm->thermal->mask->y,
				 tm->thermal->mask->width,
				 tm->thermal->mask->height);
			write_xmp(ctx->cb,
				  XMP_(THERMAL_MASK),
				  val,
				  ctx->userdata);
		}
	}

	vmeta_frame_proto_release_unpacked(ctx->frame, tm);
}


static void write_frame_constants(vmeta_photo_write_cb_t cb, void *userdata)
{
	/* Constant values for Exif tags */
	write_exif(cb, EXIF_(FLASH), "0", userdata);
	write_exif(cb, EXIF_(SCENE_CAPTURE_TYPE), "0", userdata);
	write_exif(cb, EXIF_(ORIENTATION), "1", userdata);
	write_exif(cb, EXIF_(X_RESOLUTION), "72", userdata);
	write_exif(cb, EXIF_(Y_RESOLUTION), "72", userdata);
	write_exif(cb, EXIF_(RESOLUTION_UNIT), "2", userdata);
	write_exif(cb,
		   EXIF_(YCBCR_POSITIONING),
		   "1",
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
	write_exif(cb, EXIF_(SENSITIVITY_TYPE), "3", userdata); /* ISO Speed */

	/* Constant values for Xmp tags */
	write_xmp(cb, XMP_(ORIENTATION), "1", userdata);
	write_xmp(cb,
		  XMP_(YCBCR_POSITIONING),
		  "1",
		  userdata); /* Centered for JPEG */
}


int vmeta_photo_write(const struct vmeta_session *session,
		      struct vmeta_frame *frame,
		      vmeta_photo_write_cb_t cb,
		      void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	struct vmeta_photo_write_ctx ctx = {
		.session = session,
		.frame = frame,
		.cb = cb,
		.userdata = userdata,
	};

	if (session != NULL) {
		if (session->title[0] != '\0') {
			write_exif(cb,
				   EXIF_(IMAGE_DESCRIPTION),
				   session->title,
				   userdata);
			write_xmp(cb,
				  XMP_(DC_DESCRIPTION),
				  session->title,
				  userdata);
		}

		if (session->copyright[0] != '\0') {
			write_exif(cb,
				   EXIF_(COPYRIGHT),
				   session->copyright,
				   userdata);
			write_xmp(cb,
				  XMP_(TIFF_COPYRIGHT),
				  session->copyright,
				  userdata);
			write_xmp(cb,
				  XMP_(DC_RIGHTS),
				  session->copyright,
				  userdata);
		}

		write_session_maker_model(session, cb, userdata);
		write_session_versions(session, cb, userdata);
		write_session_ids_and_misc(session, cb, userdata);
		write_session_dates(session, cb, userdata);
		write_session_camera_model(session, cb, userdata);
	}

	if (frame != NULL) {
		write_frame_timestamps(frame, cb, userdata);
		write_frame_location(frame, cb, userdata);
		write_frame_orientation(frame, cb, userdata);
		write_frame_exposure(frame, cb, userdata);
		write_frame_levels(frame, cb, userdata);
		write_frame_optics(frame, cb, userdata);
		write_frame_color_matrix(frame, cb, userdata);

		/* Handle PROTO specific fields */
		if (frame->type == VMETA_FRAME_TYPE_PROTO)
			write_frame_proto(&ctx);

		write_frame_constants(cb, userdata);
	}

	return 0;
}


#undef EXIF_
#undef XMP_
