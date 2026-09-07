/**
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "vmeta_test.h"

#include <video-metadata/vmeta_photo.h>

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


/* Shorthand macros mirroring the ones used internally by vmeta_photo.c, to
 * keep the test assertions readable */
#define EXIF_(_name) PMETA_DEFS_EXIF_IDX_##_name
#define XMP_(_name) PMETA_DEFS_XMP_IDX_##_name


#define MAX_CAPTURED_ITEMS 256


/* One entry captured by the test's vmeta_photo_write_cb_t callback.
 * exif_def/xmp_def store the pointer returned by the library's own
 * pmeta_defs_get_{exif,xmp}_tag_by_idx() lookups; since those functions
 * return pointers into static const tables, comparing pointers (rather than
 * re-deriving/parsing key strings) is a robust way to identify which tag a
 * given callback invocation refers to. */
struct captured_item {
	enum pmeta_defs_dest dest;
	const struct pmeta_defs_exif_def *exif_def;
	const struct pmeta_defs_xmp_def *xmp_def;
	int value_is_null;
	char value[256];
};


struct capture_ctx {
	struct captured_item items[MAX_CAPTURED_ITEMS];
	size_t count;
};


static void test_write_cb(enum pmeta_defs_dest dest,
			  const struct pmeta_defs_exif_def *exif_def,
			  const struct pmeta_defs_xmp_def *xmp_def,
			  const char *value,
			  void *userdata)
{
	struct capture_ctx *ctx = userdata;
	struct captured_item *item;

	/* Per the vmeta_photo_write_cb_t contract, exactly one of exif_def/
	 * xmp_def must be set, depending on dest */
	if (dest == PMETA_DEFS_DEST_EXIF) {
		CU_ASSERT_PTR_NOT_NULL(exif_def);
		CU_ASSERT_PTR_NULL(xmp_def);
	} else if (dest == PMETA_DEFS_DEST_XMP) {
		CU_ASSERT_PTR_NOT_NULL(xmp_def);
		CU_ASSERT_PTR_NULL(exif_def);
	} else {
		CU_FAIL("unexpected pmeta_defs_dest value");
	}

	if (ctx->count >= MAX_CAPTURED_ITEMS) {
		CU_FAIL("too many captured items, increase "
			"MAX_CAPTURED_ITEMS");
		return;
	}

	item = &ctx->items[ctx->count++];
	item->dest = dest;
	item->exif_def = exif_def;
	item->xmp_def = xmp_def;
	if (value != NULL) {
		item->value_is_null = 0;
		snprintf(item->value, sizeof(item->value), "%s", value);
	} else {
		/* write_frame_camera_model() clears some XMP tags by
		 * calling the callback with a NULL value */
		item->value_is_null = 1;
		item->value[0] = '\0';
	}
}


static const struct captured_item *find_exif_nth(const struct capture_ctx *ctx,
						 enum pmeta_defs_exif_idx idx,
						 size_t n)
{
	const struct pmeta_defs_exif_def *expect =
		pmeta_defs_get_exif_tag_by_idx(idx);
	size_t seen = 0;
	size_t i;

	CU_ASSERT_PTR_NOT_NULL(expect);

	for (i = 0; i < ctx->count; i++) {
		if (ctx->items[i].dest == PMETA_DEFS_DEST_EXIF &&
		    ctx->items[i].exif_def == expect) {
			if (seen == n)
				return &ctx->items[i];
			seen++;
		}
	}
	return NULL;
}


static const struct captured_item *find_xmp_nth(const struct capture_ctx *ctx,
						enum pmeta_defs_xmp_idx idx,
						size_t n)
{
	const struct pmeta_defs_xmp_def *expect =
		pmeta_defs_get_xmp_tag_by_idx(idx);
	size_t seen = 0;
	size_t i;

	CU_ASSERT_PTR_NOT_NULL(expect);

	for (i = 0; i < ctx->count; i++) {
		if (ctx->items[i].dest == PMETA_DEFS_DEST_XMP &&
		    ctx->items[i].xmp_def == expect) {
			if (seen == n)
				return &ctx->items[i];
			seen++;
		}
	}
	return NULL;
}


static const struct captured_item *find_exif(const struct capture_ctx *ctx,
					     enum pmeta_defs_exif_idx idx)
{
	return find_exif_nth(ctx, idx, 0);
}


static const struct captured_item *find_xmp(const struct capture_ctx *ctx,
					    enum pmeta_defs_xmp_idx idx)
{
	return find_xmp_nth(ctx, idx, 0);
}


static size_t count_xmp(const struct capture_ctx *ctx,
			enum pmeta_defs_xmp_idx idx)
{
	const struct pmeta_defs_xmp_def *expect =
		pmeta_defs_get_xmp_tag_by_idx(idx);
	size_t count = 0;
	size_t i;

	for (i = 0; i < ctx->count; i++) {
		if (ctx->items[i].dest == PMETA_DEFS_DEST_XMP &&
		    ctx->items[i].xmp_def == expect)
			count++;
	}
	return count;
}


#define ASSERT_EXIF_VALUE(_ctx, _idx, _expected)                               \
	do {                                                                   \
		const struct captured_item *_it =                              \
			find_exif((_ctx), EXIF_(_idx));                        \
		CU_ASSERT_PTR_NOT_NULL(_it);                                   \
		if (_it != NULL) {                                             \
			CU_ASSERT_FALSE(_it->value_is_null);                   \
			CU_ASSERT_STRING_EQUAL(_it->value, (_expected));       \
		}                                                              \
	} while (0)

#define ASSERT_XMP_VALUE(_ctx, _idx, _expected)                                \
	do {                                                                   \
		const struct captured_item *_it =                              \
			find_xmp((_ctx), XMP_(_idx));                          \
		CU_ASSERT_PTR_NOT_NULL(_it);                                   \
		if (_it != NULL) {                                             \
			CU_ASSERT_FALSE(_it->value_is_null);                   \
			CU_ASSERT_STRING_EQUAL(_it->value, (_expected));       \
		}                                                              \
	} while (0)

#define ASSERT_EXIF_PRESENT(_ctx, _idx)                                        \
	CU_ASSERT_PTR_NOT_NULL(find_exif((_ctx), EXIF_(_idx)))

#define ASSERT_XMP_PRESENT(_ctx, _idx)                                         \
	CU_ASSERT_PTR_NOT_NULL(find_xmp((_ctx), XMP_(_idx)))

#define ASSERT_EXIF_ABSENT(_ctx, _idx)                                         \
	CU_ASSERT_PTR_NULL(find_exif((_ctx), EXIF_(_idx)))

#define ASSERT_XMP_ABSENT(_ctx, _idx)                                          \
	CU_ASSERT_PTR_NULL(find_xmp((_ctx), XMP_(_idx)))

/* Like ASSERT_XMP_VALUE, but for a tag that write_frame_camera_model() writes
 * more than once for the same call: it first unconditionally emits every
 * model-specific tag with a NULL value ("clear previous model specific
 * tags"), then emits the real value for whichever tag(s) actually apply to
 * the frame's own camera model type. _n selects which of the (possibly
 * several) captured occurrences to check -- the real value is always the
 * *last* one written for tags that get a NULL clear first. */
#define ASSERT_XMP_VALUE_NTH(_ctx, _idx, _n, _expected)                        \
	do {                                                                   \
		const struct captured_item *_it =                              \
			find_xmp_nth((_ctx), XMP_(_idx), (_n));                \
		CU_ASSERT_PTR_NOT_NULL(_it);                                   \
		if (_it != NULL) {                                             \
			CU_ASSERT_FALSE(_it->value_is_null);                   \
			CU_ASSERT_STRING_EQUAL(_it->value, (_expected));       \
		}                                                              \
	} while (0)

/* For a model-specific XMP tag that belongs to the *other* camera model type
 * than the one the frame actually has: write_frame_camera_model() still
 * writes its unconditional NULL "clear" call for it, but never a real value
 * afterwards, so exactly one occurrence is captured, with a NULL value. */
#define ASSERT_XMP_CLEARED_ONLY(_ctx, _idx)                                    \
	do {                                                                   \
		CU_ASSERT_EQUAL(count_xmp((_ctx), XMP_(_idx)), (size_t)1);     \
		const struct captured_item *_it =                              \
			find_xmp((_ctx), XMP_(_idx));                          \
		CU_ASSERT_PTR_NOT_NULL(_it);                                   \
		if (_it != NULL)                                               \
			CU_ASSERT_TRUE(_it->value_is_null);                    \
	} while (0)


/**
 * Test 1: session fully populated, frame is NULL.
 * Picks a handful of concrete, hand-verifiable fields (maker/model, title,
 * copyright, dates computed from a fixed UTC epoch/gmtoff so the formatting
 * is deterministic, camera spectrum/photo mode enum strings, and the
 * perspective distortion string obtained from the same public formatting
 * helper the source uses) and checks the exact value vmeta_photo_write()
 * hands to the callback.
 */
static void test_photo_write_session_only(void)
{
	struct vmeta_session session;
	struct capture_ctx cap = {0};
	int res;
	char expected_distortion[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
	ssize_t dist_ret;

	memset(&session, 0, sizeof(session));
	snprintf(session.maker, sizeof(session.maker), "Parrot");
	snprintf(session.model, sizeof(session.model), "Anafi");
	snprintf(session.title, sizeof(session.title), "My Video");
	snprintf(session.copyright,
		 sizeof(session.copyright),
		 "Copyright 2026 Parrot Drones SAS");
	snprintf(session.build_id, sizeof(session.build_id), "0001.0002.0003");
	snprintf(session.serial_number,
		 sizeof(session.serial_number),
		 "PI040400AA0A123456");
	snprintf(session.software_version,
		 sizeof(session.software_version),
		 "GroundSDK 1.2.3");

	/* 2025-01-01T00:00:00Z, chosen so local-time formatting (gmtoff=0)
	 * is trivially deterministic regardless of the host timezone */
	session.media_date = 1735689600ULL;
	session.media_date_gmtoff = 0;

	session.camera_spectrum = VMETA_CAMERA_SPECTRUM_VISIBLE;
	session.photo_mode = VMETA_PHOTO_MODE_SINGLE;
	session.photo_count = 1;

	session.camera_model.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	session.camera_model.perspective.distortion.r1 = -0.152345f;
	session.camera_model.perspective.distortion.r2 = 0.022134f;
	session.camera_model.perspective.distortion.r3 = -0.003451f;
	session.camera_model.perspective.distortion.t1 = 0.000123f;
	session.camera_model.perspective.distortion.t2 = 0.000098f;

	/* Compute the expected distortion string using the exact same public
	 * helper vmeta_photo.c itself calls, rather than hand-typing the
	 * expected float formatting */
	dist_ret = vmeta_session_perspective_distortion_write(
		expected_distortion,
		sizeof(expected_distortion),
		session.camera_model.perspective.distortion.r1,
		session.camera_model.perspective.distortion.r2,
		session.camera_model.perspective.distortion.r3,
		session.camera_model.perspective.distortion.t1,
		session.camera_model.perspective.distortion.t2);
	CU_ASSERT(dist_ret > 0);

	res = vmeta_photo_write(&session, NULL, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	/* Maker / model */
	ASSERT_EXIF_VALUE(&cap, MAKE, "Parrot");
	ASSERT_EXIF_VALUE(&cap, MODEL, "Anafi");
	ASSERT_EXIF_VALUE(&cap, UNIQUE_CAMERA_MODEL, "Parrot Anafi");
	ASSERT_XMP_VALUE(&cap, TIFF_MAKE, "Parrot");
	ASSERT_XMP_VALUE(&cap, TIFF_MODEL, "Anafi");

	/* Versions / ids */
	ASSERT_EXIF_VALUE(&cap, SOFTWARE, "0001.0002.0003");
	ASSERT_XMP_VALUE(&cap, TIFF_SOFTWARE, "0001.0002.0003");
	ASSERT_XMP_VALUE(&cap, SOFTWARE_BUILD_ID, "0001.0002.0003");
	ASSERT_EXIF_VALUE(&cap, BODY_SERIAL_NUMBER, "PI040400AA0A123456");
	ASSERT_XMP_VALUE(&cap, SERIAL_NUMBER, "PI040400AA0A123456");
	ASSERT_XMP_VALUE(&cap, SOFTWARE_VERSION, "GroundSDK 1.2.3");

	/* Title / copyright */
	ASSERT_EXIF_VALUE(&cap, IMAGE_DESCRIPTION, "My Video");
	ASSERT_XMP_VALUE(&cap, DC_DESCRIPTION, "My Video");
	ASSERT_EXIF_VALUE(&cap, COPYRIGHT, "Copyright 2026 Parrot Drones SAS");
	ASSERT_XMP_VALUE(
		&cap, TIFF_COPYRIGHT, "Copyright 2026 Parrot Drones SAS");
	ASSERT_XMP_VALUE(&cap, DC_RIGHTS, "Copyright 2026 Parrot Drones SAS");

	/* Dates: EXIF uses "YYYY:MM:DD HH:MM:SS" (colons, no timezone),
	 * XMP uses ISO 8601 long form */
	ASSERT_EXIF_VALUE(&cap, DATETIME, "2025:01:01 00:00:00");
	ASSERT_EXIF_VALUE(&cap, DATETIME_ORIGINAL, "2025:01:01 00:00:00");
	ASSERT_EXIF_VALUE(&cap, DATETIME_DIGITIZED, "2025:01:01 00:00:00");
	ASSERT_EXIF_VALUE(&cap, OFFSET_TIME, "+00:00");
	ASSERT_EXIF_VALUE(&cap, OFFSET_TIME_ORIGINAL, "+00:00");
	ASSERT_EXIF_VALUE(&cap, OFFSET_TIME_DIGITIZED, "+00:00");
	ASSERT_EXIF_VALUE(&cap, SUBSEC_TIME, "000");
	ASSERT_EXIF_VALUE(&cap, SUBSEC_TIME_ORIGINAL, "000");
	ASSERT_EXIF_VALUE(&cap, SUBSEC_TIME_DIGITIZED, "000");
	ASSERT_XMP_VALUE(&cap, DATETIME_ORIGINAL, "2025-01-01T00:00:00+00:00");
	ASSERT_XMP_VALUE(&cap, CREATE_DATE, "2025-01-01T00:00:00+00:00");
	ASSERT_XMP_VALUE(&cap, MODIFY_DATE, "2025-01-01T00:00:00+00:00");
	ASSERT_XMP_VALUE(&cap, DC_DATE, "2025-01-01T00:00:00+00:00");

	/* Enum-valued fields: compare against the library's own to_str(),
	 * not a hand-typed string constant */
	ASSERT_XMP_VALUE(
		&cap,
		CAMERA_SPECTRUM,
		vmeta_camera_spectrum_to_str(VMETA_CAMERA_SPECTRUM_VISIBLE));
	ASSERT_XMP_VALUE(&cap,
			 PHOTO_MODE,
			 vmeta_photo_mode_to_str(VMETA_PHOTO_MODE_SINGLE));
	ASSERT_XMP_VALUE(&cap, PHOTO_COUNT, "1");

	/* Camera model (perspective) */
	ASSERT_XMP_VALUE(&cap,
			 CAMERA_MODEL_TYPE,
			 vmeta_camera_model_type_to_str(
				 VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE));
	ASSERT_XMP_VALUE(&cap, PERSPECTIVE_DISTORTION, expected_distortion);

	/* Frame-only / constants tags must be absent since frame == NULL */
	ASSERT_XMP_ABSENT(&cap, CAPTURE_TS_US);
	ASSERT_EXIF_ABSENT(&cap, GPS_LATITUDE);
	ASSERT_EXIF_ABSENT(&cap, FLASH);
	ASSERT_XMP_ABSENT(&cap, ORIENTATION);
}


/**
 * Build a VMETA_FRAME_TYPE_PROTO frame with a representative set of camera/
 * photo/thermal/drone fields filled in, following the same
 * vmeta_frame_new()/vmeta_frame_proto_get_unpacked_rw()/
 * vmeta_frame_proto_release_unpacked_rw() pattern used by
 * vmeta_test_proto.c's own unpacked_meta() helper. All numeric values are
 * deliberately chosen to be exactly representable (multiples of 0.5, powers
 * of two, whole numbers) so that the formatted output strings can be
 * computed by hand.
 */
static struct vmeta_frame *build_proto_frame(void)
{
	struct vmeta_frame *frame = NULL;
	Vmeta__TimedMetadata *meta = NULL;
	Vmeta__CameraMetadata *camera;
	Vmeta__PhotoMetadata *photo;
	Vmeta__ThermalMetadata *thermal;
	Vmeta__DroneMetadata *drone;
	Vmeta__Location *loc;
	Vmeta__Quaternion *quat;
	Vmeta__ThermalSpot *spot;
	Vmeta__Rectf *mask;
	double *cm;
	size_t i;
	int res;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(frame);

	res = vmeta_frame_proto_get_unpacked_rw(frame, &meta);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(meta);

	camera = vmeta_frame_proto_get_camera(meta);
	CU_ASSERT_PTR_NOT_NULL(camera);
	camera->timestamp = 123456789ULL;
	camera->exposure_time = 500.f; /* ms -> 0.5s -> fraction 1/2 */
	camera->awb_r_gain = 2.f;
	camera->awb_b_gain = 4.f;
	camera->spectrum = VMETA__CAMERA_SPECTRUM__CS_THERMAL;
	camera->serial_number = strdup("PI000000A00A000001");
	camera->utc_timestamp_accuracy = 5000;

	quat = vmeta_frame_proto_get_camera_quat(camera);
	CU_ASSERT_PTR_NOT_NULL(quat);
	quat->w = 1.f;
	quat->x = 0.f;
	quat->y = 0.f;
	quat->z = 0.f;

	loc = vmeta_frame_proto_get_camera_location(camera);
	CU_ASSERT_PTR_NOT_NULL(loc);
	loc->latitude = 48.5;
	loc->longitude = -2.5;
	loc->altitude_wgs84ellipsoid = 100.0;
	loc->altitude_egm96amsl = 95.0;
	loc->horizontal_accuracy = 1.5f;
	loc->vertical_accuracy = 2.5f;
	loc->sv_count = 12;

	drone = vmeta_frame_proto_get_drone(meta);
	CU_ASSERT_PTR_NOT_NULL(drone);
	drone->altitude_ato = 12.5;

	photo = vmeta_frame_proto_get_photo(meta);
	CU_ASSERT_PTR_NOT_NULL(photo);
	photo->iso_speed = 400;
	photo->focal_length = 4.0; /* exact -> fraction 4/1 */
	photo->focal_length_in_35mm_film = 35.0;
	photo->f_number = 2.0f; /* exact -> aperture value log2(2)=1 exact */
	photo->pixel_x_dimension = 5344;
	photo->pixel_y_dimension = 4016;
	photo->focal_plane_x_resolution = 300.0;
	photo->focal_plane_y_resolution = 300.0;
	photo->media_id = 222;
	photo->resource_index = 2;
	photo->raw_black_level = 64;
	photo->raw_white_level = 1023;
	photo->calibration_illuminant_1 = 1;
	for (i = 0; i < 9; i++) {
		cm = vmeta_frame_proto_get_color_matrix_1_by_index(photo, i);
		CU_ASSERT_PTR_NOT_NULL(cm);
		*cm = (i == 0 || i == 4 || i == 8) ? 1.0 : 0.0;
	}

	thermal = vmeta_frame_proto_get_thermal(meta);
	CU_ASSERT_PTR_NOT_NULL(thermal);
	spot = vmeta_frame_proto_get_thermal_min(thermal);
	CU_ASSERT_PTR_NOT_NULL(spot);
	spot->x = 0.25f;
	spot->y = 0.5f;
	spot->value = 1000;
	spot = vmeta_frame_proto_get_thermal_max(thermal);
	CU_ASSERT_PTR_NOT_NULL(spot);
	spot->x = 0.75f;
	spot->y = 1.0f;
	spot->value = 5000;
	mask = vmeta_frame_proto_get_thermal_mask(thermal);
	CU_ASSERT_PTR_NOT_NULL(mask);
	mask->x = 0.f;
	mask->y = 0.f;
	mask->width = 1.f;
	mask->height = 1.f;

	res = vmeta_frame_proto_release_unpacked_rw(frame, meta);
	CU_ASSERT_EQUAL(res, 0);

	return frame;
}


/**
 * Test 2: session is NULL, frame fully populated (proto type).
 * Verifies frame-derived EXIF/XMP entries appear with the expected values,
 * derived by hand from the source's formatting formulas (continued-fraction
 * rational approximation, DMS conversion, etc.) using deliberately "nice"
 * input values (multiples of 0.5, exact powers of two) so the output is
 * exactly predictable.
 */
static void test_photo_write_frame_only(void)
{
	struct vmeta_frame *frame = build_proto_frame();
	struct capture_ctx cap = {0};
	int res;

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	/* Frame timestamp */
	ASSERT_XMP_VALUE(&cap, CAPTURE_TS_US, "123456789");

	/* Camera location (GPS) */
	ASSERT_EXIF_VALUE(&cap, GPS_LATITUDE, "48/1,30/1,0/100000");
	ASSERT_EXIF_VALUE(&cap, GPS_LATITUDE_REF, "N");
	ASSERT_XMP_VALUE(&cap, GPS_LATITUDE, "48,30.00000000N");
	ASSERT_EXIF_VALUE(&cap, GPS_LONGITUDE, "2/1,30/1,0/100000");
	ASSERT_EXIF_VALUE(&cap, GPS_LONGITUDE_REF, "W");
	ASSERT_XMP_VALUE(&cap, GPS_LONGITUDE, "2,30.00000000W");
	ASSERT_XMP_VALUE(&cap, CAMERA_ABOVE_GROUND_ALTITUDE, "25/2");
	ASSERT_XMP_VALUE(&cap, ALTITUDE_AMSL, "95.0000");
	ASSERT_EXIF_VALUE(&cap, GPS_ALTITUDE, "100/1");
	ASSERT_XMP_VALUE(&cap, GPS_ALTITUDE, "100/1");
	ASSERT_EXIF_VALUE(&cap, GPS_ALTITUDE_REF, "0");
	ASSERT_XMP_VALUE(&cap, GPS_ALTITUDE_REF, "0");
	ASSERT_XMP_VALUE(&cap, PIX4D_GPS_XY_ACCURACY, "3/2");
	ASSERT_XMP_VALUE(&cap, PIX4D_GPS_Z_ACCURACY, "5/2");
	ASSERT_EXIF_VALUE(&cap, GPS_SATELLITES, "12");
	ASSERT_EXIF_VALUE(&cap, GPS_VERSION_ID, "2300");
	ASSERT_EXIF_VALUE(&cap, GPS_MAP_DATUM, "WGS-84");
	ASSERT_XMP_VALUE(&cap, CAMERA_HORIZ_CS, "EPSG:4326");
	ASSERT_XMP_VALUE(&cap, CAMERA_VERT_CS, "ellipsoidal");

	/* Drone location was never set: the "drone location" XMP tags must
	 * not appear */
	ASSERT_XMP_ABSENT(&cap, DRONE_LATITUDE);
	ASSERT_XMP_ABSENT(&cap, DRONE_LONGITUDE);

	/* Orientation: identity quaternion -> zero Euler angles */
	ASSERT_XMP_VALUE(&cap, CAMERA_ROLL, "0.000000");
	ASSERT_XMP_VALUE(&cap, CAMERA_PITCH, "0.000000");
	ASSERT_XMP_VALUE(&cap, CAMERA_YAW, "0.000000");
	/* The pix4d-rotated orientation is still emitted (a 90 degree
	 * rotation is applied even to an identity input quaternion), but its
	 * exact value depends on a quaternion multiply that is only
	 * replicated by re-implementing internal, non-exported logic; only
	 * presence is checked here */
	ASSERT_XMP_PRESENT(&cap, PIX4D_CAMERA_ROLL);
	ASSERT_XMP_PRESENT(&cap, PIX4D_CAMERA_PITCH);
	ASSERT_XMP_PRESENT(&cap, PIX4D_CAMERA_YAW);
	/* No local_quat was set: DRONE_CAMERA_NED_START_QUAT absent */
	ASSERT_XMP_ABSENT(&cap, DRONE_CAMERA_NED_START_QUAT);

	/* Exposure / ISO */
	ASSERT_EXIF_VALUE(&cap, EXPOSURE_TIME, "1/2");
	ASSERT_XMP_VALUE(&cap, EXPOSURE_TIME, "1/2");
	ASSERT_EXIF_VALUE(&cap, SHUTTER_SPEED_VALUE, "1.000000000");
	ASSERT_EXIF_VALUE(&cap, ISO_SPEED_RATINGS, "400");
	ASSERT_XMP_VALUE(&cap, ISO_SPEED_RATINGS, "400");
	ASSERT_EXIF_VALUE(&cap, ISO_SPEED, "400");

	/* RAW levels */
	ASSERT_EXIF_VALUE(&cap, BLACK_LEVEL, "64");
	ASSERT_EXIF_VALUE(&cap, WHITE_LEVEL, "1023");

	/* Optics */
	ASSERT_EXIF_VALUE(&cap, CALIBRATION_ILLUMINANT_1, "1");
	ASSERT_EXIF_VALUE(
		&cap, AS_SHOT_NEUTRAL, "0.500000000,1.000000000,0.250000000");

	/* Color matrix (identity 3x3) */
	ASSERT_EXIF_VALUE(&cap,
			  COLOR_MATRIX_1,
			  "1.000000000,0.000000000,0.000000000,0.000000000,"
			  "1.000000000,0.000000000,0.000000000,0.000000000,"
			  "1.000000000");

	/* PROTO-only "photo" fields */
	ASSERT_EXIF_VALUE(&cap, EXPOSURE_BIAS, "0/1"); /* default value 0.0 */
	ASSERT_XMP_VALUE(&cap, EXPOSURE_BIAS, "0/1");
	ASSERT_EXIF_VALUE(&cap, METERING_MODE, "0");
	ASSERT_EXIF_VALUE(&cap, LIGHT_SOURCE, "0");
	ASSERT_EXIF_VALUE(&cap, EXPOSURE_MODE, "0");
	ASSERT_EXIF_VALUE(&cap, WHITE_BALANCE, "0");
	ASSERT_EXIF_VALUE(&cap, CONTRAST, "0");
	ASSERT_EXIF_VALUE(&cap, SATURATION, "0");
	ASSERT_EXIF_VALUE(&cap, SHARPNESS, "0");
	ASSERT_EXIF_VALUE(&cap, FOCAL_LENGTH, "4/1");
	ASSERT_EXIF_VALUE(&cap, FOCAL_LENGTH_35MM, "35");
	ASSERT_EXIF_VALUE(&cap, FNUMBER, "2.000000000");
	ASSERT_EXIF_VALUE(&cap, APERTURE_VALUE, "2.000000000");
	ASSERT_EXIF_VALUE(&cap, PIXEL_X_DIMENSION, "5344");
	ASSERT_EXIF_VALUE(&cap, PIXEL_Y_DIMENSION, "4016");
	ASSERT_EXIF_VALUE(&cap, FOCAL_PLANE_X_RES, "300.000000");
	ASSERT_EXIF_VALUE(&cap, FOCAL_PLANE_RES_UNIT, "3");
	ASSERT_EXIF_VALUE(&cap, FOCAL_PLANE_Y_RES, "300.000000");
	ASSERT_XMP_VALUE(&cap, MEDIA_ID, "222");
	ASSERT_XMP_VALUE(&cap, RESOURCE_INDEX, "2");

	/* Since session == NULL and the proto photo has no camera_model of
	 * its own, get_resolved_camera_model_type() falls all the way back
	 * to VMETA_CAMERA_MODEL_TYPE_UNKNOWN: the perspective-only focal
	 * length XMP tags must not appear even though EXIF FOCAL_LENGTH did
	 */
	ASSERT_XMP_ABSENT(&cap, PERSPECTIVE_FOCAL_LENGTH);
	ASSERT_XMP_ABSENT(&cap, PERSPECTIVE_FOCAL_LENGTH_UNITS);

	/* session is NULL, so the panorama-only SEQUENCE_NUMBER tag (guarded
	 * by "ctx->session && ctx->session->photo_mode == PANORAMA") must be
	 * absent even though tm->photo exists */
	ASSERT_XMP_ABSENT(&cap, SEQUENCE_NUMBER);

	/* Camera block */
	ASSERT_XMP_VALUE(&cap, UTC_TS_ACCURACY, "5000");
	ASSERT_XMP_VALUE(&cap,
			 CAMERA_SPECTRUM,
			 vmeta_camera_spectrum_to_str(
				 vmeta_frame_camera_spectrum_proto_to_vmeta(
					 VMETA__CAMERA_SPECTRUM__CS_THERMAL)));
	ASSERT_XMP_VALUE(&cap, CAMERA_SERIAL_NUMBER, "PI000000A00A000001");
	ASSERT_EXIF_VALUE(&cap, CAMERA_SERIAL_NUMBER, "PI000000A00A000001");

	/* Thermal spots / mask */
	ASSERT_XMP_VALUE(&cap, THERMAL_SPOT_MIN, "0.25000000,0.50000000,1000");
	ASSERT_XMP_VALUE(&cap, THERMAL_SPOT_MAX, "0.75000000,1.00000000,5000");
	ASSERT_XMP_VALUE(&cap,
			 THERMAL_MASK,
			 "0.00000000,0.00000000,1.00000000,1.00000000");

	/* Constants are written whenever frame != NULL */
	ASSERT_EXIF_VALUE(&cap, FLASH, "0");
	ASSERT_EXIF_VALUE(&cap, ORIENTATION, "1");
	ASSERT_XMP_VALUE(&cap, ORIENTATION, "1");

	/* Session-only tags must be absent since session == NULL */
	ASSERT_EXIF_ABSENT(&cap, MAKE);
	ASSERT_XMP_ABSENT(&cap, TIFF_MAKE);
	ASSERT_EXIF_ABSENT(&cap, IMAGE_DESCRIPTION);

	vmeta_frame_unref(frame);
}


/**
 * Test 3: both session and frame populated.
 * Confirms both sets of entries appear together, and specifically exercises
 * two interactions between the two data sources:
 *  - the camera model type fallback: get_resolved_camera_model_type()
 *    resolves to the *session*'s camera_model.type when the proto frame has
 *    a focal_length but no camera_model of its own, which changes whether
 *    PERSPECTIVE_FOCAL_LENGTH{,_UNITS} are emitted (contrast with test 2,
 *    where the same frame without a session yields no such tag);
 *  - no de-duplication: when both the session and the frame each supply a
 *    value for the same XMP tag (MEDIA_ID/RESOURCE_INDEX here), the callback
 *    is invoked twice, once per source, in call order (session first, since
 *    vmeta_photo_write() processes the session block before the frame
 *    block); nothing in vmeta_photo_write() collapses/overwrites these.
 */
static void test_photo_write_session_and_frame(void)
{
	struct vmeta_session session;
	struct vmeta_frame *frame = build_proto_frame();
	struct capture_ctx cap = {0};
	int res;
	const struct captured_item *first;
	const struct captured_item *second;

	memset(&session, 0, sizeof(session));
	snprintf(session.maker, sizeof(session.maker), "Parrot");
	snprintf(session.model, sizeof(session.model), "Anafi");
	session.camera_model.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	session.camera_model.perspective.distortion.r1 = 0.1f;
	session.camera_model.perspective.distortion.r2 = 0.1f;
	session.camera_model.perspective.distortion.r3 = 0.1f;
	session.camera_model.perspective.distortion.t1 = 0.1f;
	session.camera_model.perspective.distortion.t2 = 0.1f;
	session.media_id = 111;
	session.resource_index = 1;

	res = vmeta_photo_write(&session, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	/* Session-derived tag still present */
	ASSERT_EXIF_VALUE(&cap, MAKE, "Parrot");

	/* Frame-derived tag still present */
	ASSERT_XMP_VALUE(&cap, CAPTURE_TS_US, "123456789");

	/* Camera model fallback: with a session camera_model.type of
	 * PERSPECTIVE and the frame's own photo->camera_model unset, the
	 * frame's non-zero focal_length now also produces the
	 * perspective-specific XMP tags (unlike test 2, where session was
	 * NULL and these were absent) */
	ASSERT_XMP_VALUE(&cap, PERSPECTIVE_FOCAL_LENGTH, "4/1");
	ASSERT_XMP_VALUE(&cap, PERSPECTIVE_FOCAL_LENGTH_UNITS, "mm");

	/* MEDIA_ID / RESOURCE_INDEX are supplied by both the session block
	 * (111 / 1) and the frame's proto photo block (222 / 2); both calls
	 * happen, in that order, with no de-duplication */
	CU_ASSERT_EQUAL(count_xmp(&cap, XMP_(MEDIA_ID)), 2);
	first = find_xmp_nth(&cap, XMP_(MEDIA_ID), 0);
	second = find_xmp_nth(&cap, XMP_(MEDIA_ID), 1);
	CU_ASSERT_PTR_NOT_NULL(first);
	CU_ASSERT_PTR_NOT_NULL(second);
	if (first != NULL && second != NULL) {
		CU_ASSERT_STRING_EQUAL(first->value, "111");
		CU_ASSERT_STRING_EQUAL(second->value, "222");
	}

	CU_ASSERT_EQUAL(count_xmp(&cap, XMP_(RESOURCE_INDEX)), 2);
	first = find_xmp_nth(&cap, XMP_(RESOURCE_INDEX), 0);
	second = find_xmp_nth(&cap, XMP_(RESOURCE_INDEX), 1);
	CU_ASSERT_PTR_NOT_NULL(first);
	CU_ASSERT_PTR_NOT_NULL(second);
	if (first != NULL && second != NULL) {
		CU_ASSERT_STRING_EQUAL(first->value, "1");
		CU_ASSERT_STRING_EQUAL(second->value, "2");
	}

	vmeta_frame_unref(frame);
}


/**
 * Test 4: both session and frame are NULL.
 * vmeta_photo_write() still requires a non-NULL callback (EINVAL otherwise),
 * but with both metadata sources NULL, neither the "if (session != NULL)"
 * nor the "if (frame != NULL)" block in the source ever executes: the
 * callback must never be invoked, and the function still returns 0 (success,
 * simply nothing to write) rather than an error.
 */
static void test_photo_write_both_null(void)
{
	struct capture_ctx cap = {0};
	int res;

	res = vmeta_photo_write(NULL, NULL, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(cap.count, 0);
}


/**
 * cb == NULL is rejected with -EINVAL regardless of session/frame, per the
 * ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL) guard at the top of
 * vmeta_photo_write().
 */
static void test_photo_write_null_callback(void)
{
	int res = vmeta_photo_write(NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


/**
 * Test 5: a frame that is *not* VMETA_FRAME_TYPE_PROTO (here, V3), with
 * session == NULL.
 *
 * This exercises the "generic getter" code paths in vmeta_photo.c (the ones
 * that go through vmeta_frame_get_*() and work across several frame types),
 * as opposed to the PROTO-only write_frame_proto() block, which is only
 * ever invoked when frame->type == VMETA_FRAME_TYPE_PROTO.
 *
 * Note/open question: get_resolved_camera_model_type()'s own
 * "!ctx->frame || ctx->frame->type != VMETA_FRAME_TYPE_PROTO" fallback
 * check appears to be dead code reachable only in theory: its single call
 * site sits inside write_frame_proto(), which vmeta_photo_write() only
 * invokes after already checking "frame->type == VMETA_FRAME_TYPE_PROTO",
 * and frame is guaranteed non-NULL at that point too (it is inside the
 * "if (frame != NULL)" block). So a V3 frame such as the one built here
 * never actually reaches get_resolved_camera_model_type() at all; the
 * fallback-to-session behavior of that function is instead exercised by
 * test_photo_write_frame_only()/test_photo_write_session_and_frame() above,
 * via the *other* way the "fallback:" label can be reached (a PROTO frame
 * whose photo->camera_model is simply unset).
 */
static void test_photo_write_frame_v3_type(void)
{
	struct vmeta_frame *frame = NULL;
	struct capture_ctx cap = {0};
	int res;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(frame);

	frame->v3.base.exposure_time = 250.f; /* ms -> 0.25s -> 1/4 */
	frame->v3.base.awb_r_gain = 2.f;
	frame->v3.base.awb_b_gain = 2.f;
	frame->v3.base.frame_quat.w = 1.f; /* identity quaternion */
	frame->v3.has_timestamp = 1;
	frame->v3.timestamp.frame_timestamp = 999;

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	/* Generic getters supported for V3: timestamp, exposure, AWB gains,
	 * frame orientation */
	ASSERT_XMP_VALUE(&cap, CAPTURE_TS_US, "999");
	ASSERT_EXIF_VALUE(&cap, EXPOSURE_TIME, "1/4");
	ASSERT_XMP_VALUE(&cap, EXPOSURE_TIME, "1/4");
	ASSERT_EXIF_VALUE(
		&cap, AS_SHOT_NEUTRAL, "0.500000000,1.000000000,0.500000000");
	ASSERT_XMP_VALUE(&cap, CAMERA_ROLL, "0.000000");
	ASSERT_XMP_VALUE(&cap, CAMERA_PITCH, "0.000000");
	ASSERT_XMP_VALUE(&cap, CAMERA_YAW, "0.000000");

	/* ISO speed is PROTO-only (vmeta_frame_get_iso_speed() returns
	 * -ENOENT for every other frame type) */
	ASSERT_EXIF_ABSENT(&cap, ISO_SPEED_RATINGS);
	ASSERT_XMP_ABSENT(&cap, ISO_SPEED_RATINGS);
	ASSERT_EXIF_ABSENT(&cap, ISO_SPEED);

	/* write_frame_proto() (and everything it writes) never runs for a
	 * non-PROTO frame type */
	ASSERT_EXIF_ABSENT(&cap, EXPOSURE_PROGRAM);
	ASSERT_EXIF_ABSENT(&cap, METERING_MODE);
	ASSERT_EXIF_ABSENT(&cap, LIGHT_SOURCE);
	ASSERT_EXIF_ABSENT(&cap, EXPOSURE_MODE);
	ASSERT_EXIF_ABSENT(&cap, WHITE_BALANCE);
	ASSERT_EXIF_ABSENT(&cap, PIXEL_X_DIMENSION);
	ASSERT_EXIF_ABSENT(&cap, CAMERA_SERIAL_NUMBER);
	ASSERT_XMP_ABSENT(&cap, UTC_TS_ACCURACY);
	ASSERT_XMP_ABSENT(&cap, THERMAL_SPOT_MIN);
	ASSERT_XMP_ABSENT(&cap, PERSPECTIVE_FOCAL_LENGTH);

	/* Camera/GPS location, RAW levels and color matrix getters are also
	 * PROTO-only */
	ASSERT_EXIF_ABSENT(&cap, GPS_LATITUDE);
	ASSERT_XMP_ABSENT(&cap, DRONE_LATITUDE);
	ASSERT_EXIF_ABSENT(&cap, BLACK_LEVEL);
	ASSERT_EXIF_ABSENT(&cap, COLOR_MATRIX_1);

	/* Constants are still written since frame != NULL */
	ASSERT_EXIF_VALUE(&cap, FLASH, "0");

	vmeta_frame_unref(frame);
}


/**
 * Test 6: session-only, camera model type FISHEYE (test 1 only ever exercised
 * PERSPECTIVE). Covers the FISHEYE case of write_session_camera_model(),
 * including the "symmetric" affine matrix sub-branch.
 */
static void test_write_session_camera_model_fisheye(void)
{
	struct vmeta_session session;
	struct capture_ctx cap = {0};
	int res;
	char expected_matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
	char expected_poly[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
	ssize_t ret;

	memset(&session, 0, sizeof(session));
	session.camera_model.type = VMETA_CAMERA_MODEL_TYPE_FISHEYE;
	session.camera_model.fisheye.affine_matrix.c = 1.000456f;
	session.camera_model.fisheye.affine_matrix.d = 0.000012f;
	session.camera_model.fisheye.affine_matrix.e = 0.000015f;
	session.camera_model.fisheye.affine_matrix.f = 0.999876f;
	session.camera_model.fisheye.affine_matrix.symmetric = 1;
	session.camera_model.fisheye.affine_matrix.symmetric_valid = 1;
	session.camera_model.fisheye.polynomial.p2 = 0.1f;
	session.camera_model.fisheye.polynomial.p3 = 0.2f;
	session.camera_model.fisheye.polynomial.p4 = 0.3f;

	ret = vmeta_session_fisheye_affine_matrix_write(
		expected_matrix,
		sizeof(expected_matrix),
		session.camera_model.fisheye.affine_matrix.c,
		session.camera_model.fisheye.affine_matrix.d,
		session.camera_model.fisheye.affine_matrix.e,
		session.camera_model.fisheye.affine_matrix.f);
	CU_ASSERT(ret > 0);
	ret = vmeta_session_fisheye_polynomial_write(
		expected_poly,
		sizeof(expected_poly),
		session.camera_model.fisheye.polynomial.p2,
		session.camera_model.fisheye.polynomial.p3,
		session.camera_model.fisheye.polynomial.p4);
	CU_ASSERT(ret > 0);

	res = vmeta_photo_write(&session, NULL, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_XMP_VALUE(&cap,
			 CAMERA_MODEL_TYPE,
			 vmeta_camera_model_type_to_str(
				 VMETA_CAMERA_MODEL_TYPE_FISHEYE));
	ASSERT_XMP_VALUE(&cap, FISHEYE_AFFINE_MATRIX, expected_matrix);
	ASSERT_XMP_VALUE(&cap, FISHEYE_AFFINE_SYMMETRIC, "1");
	ASSERT_XMP_VALUE(&cap, FISHEYE_POLYNOMIAL, expected_poly);
	/* PERSPECTIVE-only tag must be absent */
	ASSERT_XMP_ABSENT(&cap, PERSPECTIVE_DISTORTION);
}


/**
 * Build a minimal VMETA_FRAME_TYPE_PROTO frame with only photo->camera_model
 * set (as a PERSPECTIVE or FISHEYE model depending on 'fisheye'), to exercise
 * write_frame_camera_model() (only ever invoked when the frame itself, not
 * the session, carries a camera_model) and the PERSPECTIVE/FISHEYE branches
 * of get_resolved_camera_model_type()'s "if (tm->photo && tm->photo->
 * camera_model)" block (session == NULL, so this is the only way that block
 * resolves to a non-UNKNOWN type).
 */
static struct vmeta_frame *build_proto_frame_with_camera_model(bool fisheye)
{
	struct vmeta_frame *frame = NULL;
	Vmeta__TimedMetadata *meta = NULL;
	Vmeta__PhotoMetadata *photo;
	Vmeta__CameraModel *model;
	int res;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(frame);

	res = vmeta_frame_proto_get_unpacked_rw(frame, &meta);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(meta);

	photo = vmeta_frame_proto_get_photo(meta);
	CU_ASSERT_PTR_NOT_NULL(photo);

	model = vmeta_frame_proto_get_camera_model(photo);
	CU_ASSERT_PTR_NOT_NULL(model);
	photo->camera_model = model;

	if (!fisheye) {
		Vmeta__CameraModel__PerspectiveCameraModel *persp =
			vmeta_session_proto_get_perspective_camera_model(model);
		Vmeta__CameraModel__PerspectiveCameraModel__Distorsion *dist;
		CU_ASSERT_PTR_NOT_NULL(persp);
		dist = vmeta_session_proto_get_perspective_camera_model_distorsion(
			persp);
		CU_ASSERT_PTR_NOT_NULL(dist);
		dist->r1 = -0.1f;
		dist->r2 = 0.2f;
		dist->r3 = -0.05f;
		dist->t1 = 0.01f;
		dist->t2 = -0.02f;
	} else {
		Vmeta__CameraModel__FisheyeCameraModel *fish =
			vmeta_session_proto_get_fisheye_camera_model(model);
		Vmeta__CameraModel__FisheyeCameraModel__AffineMatrix *affine;
		Vmeta__CameraModel__FisheyeCameraModel__Polynomial *poly;
		Google__Protobuf__BoolValue *symmetric;
		CU_ASSERT_PTR_NOT_NULL(fish);
		affine =
			vmeta_session_proto_get_fisheye_camera_model_affine_matrix(
				fish);
		CU_ASSERT_PTR_NOT_NULL(affine);
		affine->c = 1.f;
		affine->d = 0.f;
		affine->e = 0.f;
		affine->f = 1.f;
		symmetric =
			vmeta_session_proto_get_fisheye_camera_model_affine_matrix_symmetric(
				affine);
		CU_ASSERT_PTR_NOT_NULL(symmetric);
		symmetric->value = 1;
		poly = vmeta_session_proto_get_fisheye_camera_model_polynomial(
			fish);
		CU_ASSERT_PTR_NOT_NULL(poly);
		poly->p2 = 0.1f;
		poly->p3 = 0.2f;
		poly->p4 = 0.3f;
	}

	res = vmeta_frame_proto_release_unpacked_rw(frame, meta);
	CU_ASSERT_EQUAL(res, 0);

	return frame;
}


/**
 * Test 7: session == NULL, frame's own photo->camera_model set to
 * PERSPECTIVE. Covers get_resolved_camera_model_type()'s
 * "tm->photo->camera_model" / VMETA__CAMERA_MODEL__ID_PERSPECTIVE branch and
 * write_frame_camera_model()'s PERSPECTIVE branch.
 */
static void test_photo_write_frame_camera_model_perspective(void)
{
	struct vmeta_frame *frame =
		build_proto_frame_with_camera_model(false /* fisheye */);
	struct capture_ctx cap = {0};
	int res;
	char expected_distortion[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
	ssize_t ret;

	ret = vmeta_session_perspective_distortion_write(
		expected_distortion,
		sizeof(expected_distortion),
		-0.1f,
		0.2f,
		-0.05f,
		0.01f,
		-0.02f);
	CU_ASSERT(ret > 0);

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_XMP_VALUE(&cap,
			 CAMERA_MODEL_TYPE,
			 vmeta_camera_model_type_to_str(
				 VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE));
	/* write_frame_camera_model() clears PERSPECTIVE_DISTORTION (NULL,
	 * occurrence 0) before writing the real value (occurrence 1) */
	ASSERT_XMP_VALUE_NTH(
		&cap, PERSPECTIVE_DISTORTION, 1, expected_distortion);
	/* The frame's own camera_model resolves
	 * get_resolved_camera_model_type() to PERSPECTIVE even with session ==
	 * NULL, unlike test_photo_write_frame_only() where the frame had no
	 * camera_model of its own */
	ASSERT_XMP_CLEARED_ONLY(&cap, FISHEYE_AFFINE_MATRIX);
	ASSERT_XMP_CLEARED_ONLY(&cap, FISHEYE_POLYNOMIAL);

	vmeta_frame_unref(frame);
}


/**
 * Test 8: same as above but FISHEYE, also exercising the "symmetric" affine
 * matrix sub-branch of write_frame_camera_model().
 */
static void test_photo_write_frame_camera_model_fisheye(void)
{
	struct vmeta_frame *frame =
		build_proto_frame_with_camera_model(true /* fisheye */);
	struct capture_ctx cap = {0};
	int res;
	char expected_matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
	char expected_poly[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
	ssize_t ret;

	ret = vmeta_session_fisheye_affine_matrix_write(
		expected_matrix, sizeof(expected_matrix), 1.f, 0.f, 0.f, 1.f);
	CU_ASSERT(ret > 0);
	ret = vmeta_session_fisheye_polynomial_write(
		expected_poly, sizeof(expected_poly), 0.1f, 0.2f, 0.3f);
	CU_ASSERT(ret > 0);

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_XMP_VALUE(&cap,
			 CAMERA_MODEL_TYPE,
			 vmeta_camera_model_type_to_str(
				 VMETA_CAMERA_MODEL_TYPE_FISHEYE));
	/* Same "NULL clear then real value" shape as in the perspective test
	 * above, for all three fisheye-specific tags */
	ASSERT_XMP_VALUE_NTH(&cap, FISHEYE_AFFINE_MATRIX, 1, expected_matrix);
	ASSERT_XMP_VALUE_NTH(&cap, FISHEYE_AFFINE_SYMMETRIC, 1, "1");
	ASSERT_XMP_VALUE_NTH(&cap, FISHEYE_POLYNOMIAL, 1, expected_poly);
	ASSERT_XMP_CLEARED_ONLY(&cap, PERSPECTIVE_DISTORTION);

	vmeta_frame_unref(frame);
}


/**
 * Test 9: session == NULL, frame's drone->location set. Covers the "Drone
 * location" block in write_frame_gps(), gated on vmeta_frame_get_location()
 * (the *drone*'s location, distinct from the camera's own GPS location
 * already covered by test_photo_write_frame_only()).
 */
static void test_photo_write_drone_location(void)
{
	struct vmeta_frame *frame = NULL;
	Vmeta__TimedMetadata *meta = NULL;
	Vmeta__DroneMetadata *drone;
	Vmeta__Location *loc;
	struct capture_ctx cap = {0};
	int res;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vmeta_frame_proto_get_unpacked_rw(frame, &meta);
	CU_ASSERT_EQUAL(res, 0);

	drone = vmeta_frame_proto_get_drone(meta);
	CU_ASSERT_PTR_NOT_NULL(drone);
	loc = vmeta_frame_proto_get_drone_location(drone);
	CU_ASSERT_PTR_NOT_NULL(loc);
	loc->latitude = 45.5;
	loc->longitude = -1.25;
	loc->altitude_egm96amsl = 120.0;
	loc->altitude_wgs84ellipsoid = 130.0;

	res = vmeta_frame_proto_release_unpacked_rw(frame, meta);
	CU_ASSERT_EQUAL(res, 0);

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_XMP_VALUE(&cap, DRONE_LATITUDE, "45.50000000");
	ASSERT_XMP_VALUE(&cap, DRONE_LONGITUDE, "-1.25000000");
	ASSERT_XMP_VALUE(&cap, DRONE_ALTITUDE_AMSL, "120.0000");
	ASSERT_XMP_VALUE(&cap, DRONE_ALTITUDE_ELLIPSOID, "130.0000");

	vmeta_frame_unref(frame);
}


/**
 * Test 10: session == NULL, frame's camera->local_quat set. Covers the
 * vmeta_frame_get_frame_local_quat() branch in write_frame_orientation()
 * (DRONE_CAMERA_NED_START_QUAT), left unset (and asserted absent) by
 * test_photo_write_frame_only().
 */
static void test_photo_write_local_quat(void)
{
	struct vmeta_frame *frame = NULL;
	Vmeta__TimedMetadata *meta = NULL;
	Vmeta__CameraMetadata *camera;
	Vmeta__Quaternion *quat;
	struct capture_ctx cap = {0};
	int res;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vmeta_frame_proto_get_unpacked_rw(frame, &meta);
	CU_ASSERT_EQUAL(res, 0);

	camera = vmeta_frame_proto_get_camera(meta);
	CU_ASSERT_PTR_NOT_NULL(camera);
	quat = vmeta_frame_proto_get_camera_local_quat(camera);
	CU_ASSERT_PTR_NOT_NULL(quat);
	quat->w = 1.f;
	quat->x = 0.f;
	quat->y = 0.f;
	quat->z = 0.f;

	res = vmeta_frame_proto_release_unpacked_rw(frame, meta);
	CU_ASSERT_EQUAL(res, 0);

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_XMP_VALUE(&cap,
			 DRONE_CAMERA_NED_START_QUAT,
			 "1.00000000,0.00000000,0.00000000,0.00000000");

	vmeta_frame_unref(frame);
}


/**
 * Test 11: session == NULL, frame's photo->photo_date/photo_date_gmtoff set.
 * Covers the "tm->photo->photo_date != 0" branch in write_frame_proto(),
 * which delegates to the same write_dates_internal() helper already exercised
 * (with session-derived dates) by test_photo_write_session_only() -- the same
 * fixed 2025-01-01T00:00:00Z / gmtoff=0 epoch is reused here so the expected
 * strings are identical in shape.
 */
static void test_photo_write_frame_photo_date(void)
{
	struct vmeta_frame *frame = NULL;
	Vmeta__TimedMetadata *meta = NULL;
	Vmeta__PhotoMetadata *photo;
	struct capture_ctx cap = {0};
	int res;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vmeta_frame_proto_get_unpacked_rw(frame, &meta);
	CU_ASSERT_EQUAL(res, 0);

	photo = vmeta_frame_proto_get_photo(meta);
	CU_ASSERT_PTR_NOT_NULL(photo);
	/* Unlike struct vmeta_session's media_date (seconds, scaled by
	 * write_session_dates() before reaching write_dates_internal()),
	 * Vmeta__PhotoMetadata.photo_date is passed to write_dates_internal()
	 * directly and is therefore already expected in microseconds */
	photo->photo_date = 1735689600ULL * 1000000ULL;
	photo->photo_date_gmtoff = 0;

	res = vmeta_frame_proto_release_unpacked_rw(frame, meta);
	CU_ASSERT_EQUAL(res, 0);

	res = vmeta_photo_write(NULL, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_EXIF_VALUE(&cap, DATETIME, "2025:01:01 00:00:00");
	ASSERT_EXIF_VALUE(&cap, DATETIME_ORIGINAL, "2025:01:01 00:00:00");
	ASSERT_EXIF_VALUE(&cap, DATETIME_DIGITIZED, "2025:01:01 00:00:00");
	ASSERT_EXIF_VALUE(&cap, OFFSET_TIME, "+00:00");
	ASSERT_EXIF_VALUE(&cap, OFFSET_TIME_ORIGINAL, "+00:00");
	ASSERT_EXIF_VALUE(&cap, OFFSET_TIME_DIGITIZED, "+00:00");
	ASSERT_EXIF_VALUE(&cap, SUBSEC_TIME, "000");
	ASSERT_EXIF_VALUE(&cap, SUBSEC_TIME_ORIGINAL, "000");
	ASSERT_EXIF_VALUE(&cap, SUBSEC_TIME_DIGITIZED, "000");
	ASSERT_XMP_VALUE(&cap, DATETIME_ORIGINAL, "2025-01-01T00:00:00+00:00");
	ASSERT_XMP_VALUE(&cap, CREATE_DATE, "2025-01-01T00:00:00+00:00");
	ASSERT_XMP_VALUE(&cap, MODIFY_DATE, "2025-01-01T00:00:00+00:00");
	ASSERT_XMP_VALUE(&cap, DC_DATE, "2025-01-01T00:00:00+00:00");

	vmeta_frame_unref(frame);
}


/**
 * Test 12: session.photo_mode == VMETA_PHOTO_MODE_PANORAMA and frame's
 * photo->sequence_number set. Covers the "ctx->session && ctx->session->
 * photo_mode == VMETA_PHOTO_MODE_PANORAMA" branch in write_frame_proto()
 * (XMP SEQUENCE_NUMBER), left unreachable by every other test in this file
 * (test_photo_write_frame_only() has session == NULL;
 * test_photo_write_session_and_frame()'s session has the default photo_mode).
 */
static void test_photo_write_panorama_sequence_number(void)
{
	struct vmeta_session session;
	struct vmeta_frame *frame = NULL;
	Vmeta__TimedMetadata *meta = NULL;
	Vmeta__PhotoMetadata *photo;
	struct capture_ctx cap = {0};
	int res;

	memset(&session, 0, sizeof(session));
	session.photo_mode = VMETA_PHOTO_MODE_PANORAMA;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	res = vmeta_frame_proto_get_unpacked_rw(frame, &meta);
	CU_ASSERT_EQUAL(res, 0);

	photo = vmeta_frame_proto_get_photo(meta);
	CU_ASSERT_PTR_NOT_NULL(photo);
	photo->sequence_number = 7;

	res = vmeta_frame_proto_release_unpacked_rw(frame, meta);
	CU_ASSERT_EQUAL(res, 0);

	res = vmeta_photo_write(&session, frame, &test_write_cb, &cap);
	CU_ASSERT_EQUAL(res, 0);

	ASSERT_XMP_VALUE(&cap, SEQUENCE_NUMBER, "7");

	vmeta_frame_unref(frame);
}


CU_TestInfo s_photo_tests[] = {
	{(char *)"vmeta photo write session only",
	 &test_photo_write_session_only},
	{(char *)"vmeta photo write frame only", &test_photo_write_frame_only},
	{(char *)"vmeta photo write session and frame",
	 &test_photo_write_session_and_frame},
	{(char *)"vmeta photo write both null", &test_photo_write_both_null},
	{(char *)"vmeta photo write null callback",
	 &test_photo_write_null_callback},
	{(char *)"vmeta photo write frame v3 type",
	 &test_photo_write_frame_v3_type},
	{(char *)"vmeta photo write session camera model fisheye",
	 &test_write_session_camera_model_fisheye},
	{(char *)"vmeta photo write frame camera model perspective",
	 &test_photo_write_frame_camera_model_perspective},
	{(char *)"vmeta photo write frame camera model fisheye",
	 &test_photo_write_frame_camera_model_fisheye},
	{(char *)"vmeta photo write drone location",
	 &test_photo_write_drone_location},
	{(char *)"vmeta photo write local quat", &test_photo_write_local_quat},
	{(char *)"vmeta photo write frame photo date",
	 &test_photo_write_frame_photo_date},
	{(char *)"vmeta photo write panorama sequence number",
	 &test_photo_write_panorama_sequence_number},
	CU_TEST_INFO_NULL,
};


#undef EXIF_
#undef XMP_
