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

#include "vmeta_test.h"

#include <json-c/json.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


/* codecheck_ignore[NEW_TYPEDEFS] */
#define TEST_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


static struct vmeta_session_test_size {
	char friendly_name[40];
	char maker[40];
	char model[40];
	char model_id[5];
	char serial_number[32];
	char software_version[20];
	char build_id[80];
	char title[80];
	char comment[100];
	char copyright[80];
	uint64_t media_date;
	int32_t media_date_gmtoff;
	uint64_t run_date;
	int32_t run_date_gmtoff;
	char run_id[33];
	uint64_t boot_date;
	int32_t boot_date_gmtoff;
	char boot_id[33];
	uint64_t flight_date;
	int32_t flight_date_gmtoff;
	char flight_id[33];
	char custom_id[80];
	struct vmeta_location takeoff_loc;
	struct vmeta_location location;
	struct vmeta_fov picture_fov;
	struct vmeta_thermal thermal;
	/* clang-format off */
	uint32_t has_thermal:1;
	uint32_t default_media:1;
	/* clang-format on */
	enum vmeta_camera_type camera_type;
	enum vmeta_camera_subtype camera_subtype;
	enum vmeta_camera_spectrum camera_spectrum;
	char camera_serial_number[VMETA_SESSION_CAMERA_SERIAL_PATTERN_MAX_LEN];
	struct vmeta_camera_model camera_model;
	struct vmeta_overlay overlay;
	struct vmeta_principal_point principal_point;
	enum vmeta_video_mode video_mode;
	enum vmeta_video_stop_reason video_stop_reason;
	enum vmeta_photo_mode photo_mode;
	enum vmeta_panorama_type panorama_type;
	uint32_t photo_count;
	char secure_cn[VMETA_SESSION_SECURE_CN_MAX_LEN];
	enum vmeta_dynamic_range dynamic_range;
	enum vmeta_tone_mapping tone_mapping;
	uint64_t first_frame_capture_ts;
	uint64_t first_frame_sample_index;
	uint32_t media_id;
	uint32_t resource_index;
} s_vmeta_session_test_size;


/* codecheck_ignore[COMPLEX_MACRO] */
#define NULL_TERMINATE(f) (f)[sizeof(f) - 1] = '\0'

static void fill_vmeta_with(struct vmeta_session *meta, int val)
{
	memset(meta, val, sizeof(*meta));

	NULL_TERMINATE(meta->friendly_name);
	NULL_TERMINATE(meta->maker);
	NULL_TERMINATE(meta->model);
	NULL_TERMINATE(meta->model_id);
	NULL_TERMINATE(meta->serial_number);
	NULL_TERMINATE(meta->software_version);
	NULL_TERMINATE(meta->build_id);
	NULL_TERMINATE(meta->title);
	NULL_TERMINATE(meta->comment);
	NULL_TERMINATE(meta->copyright);
	NULL_TERMINATE(meta->run_id);
	NULL_TERMINATE(meta->boot_id);
	NULL_TERMINATE(meta->flight_id);
	NULL_TERMINATE(meta->custom_id);
	NULL_TERMINATE(meta->camera_serial_number);
	NULL_TERMINATE(meta->secure_cn);
}

#undef NULL_TERMINATE


/* Here only to fail if a vmeta_session field is added without modifying the
 * functions - update vmeta_session_test_size to fix this and the vmeta_session
 * functions of the api */
static void test_session_size()
{
	TEST_STATIC_ASSERT(sizeof(s_vmeta_session_test_size) ==
			   sizeof(struct vmeta_session));
}


static void test_session_cmp()
{
	struct vmeta_session meta_a = {0};
	struct vmeta_session meta_b = {0};
	struct vmeta_session meta_c = {0};

	CU_ASSERT_EQUAL(vmeta_session_cmp(NULL, NULL), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, NULL), false);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, &meta_b), true);

	fill_vmeta_with(&meta_c, 1);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, &meta_c), false);

	fill_vmeta_with(&meta_a, 1);
	fill_vmeta_with(&meta_b, 1);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, &meta_b), true);

	fill_vmeta_with(&meta_a, 1);
	fill_vmeta_with(&meta_b, 2);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, &meta_b), false);

	fill_vmeta_with(&meta_a, 1);
	fill_vmeta_with(&meta_b, 1);
	meta_a.takeoff_loc.altitude_egm96amsl = NAN;
	meta_b.takeoff_loc.altitude_egm96amsl = NAN;
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, &meta_b), true);

	fill_vmeta_with(&meta_a, 1);
	fill_vmeta_with(&meta_b, 1);
	meta_a.takeoff_loc.altitude_wgs84ellipsoid = NAN;
	CU_ASSERT_EQUAL(vmeta_session_cmp(&meta_a, &meta_b), false);
}


static void test_session_merge_metadata(void)
{
	int err = 0;
	struct vmeta_session **metas = calloc(2, sizeof(*metas));
	struct vmeta_session common = {0};
	struct vmeta_session meta_a = {0};
	struct vmeta_session meta_b = {0};
	struct vmeta_session meta_c = {0};

	metas[1] = calloc(1, sizeof(*metas[0]));
	metas[0] = calloc(1, sizeof(*metas[0]));

	/* Invalid use */
	err = vmeta_session_merge_metadata(NULL, 2, &common);
	CU_ASSERT_EQUAL(err, -EINVAL);
	err = vmeta_session_merge_metadata(NULL, 2, NULL);
	CU_ASSERT_EQUAL(err, -EINVAL);
	err = vmeta_session_merge_metadata(metas, 2, NULL);
	CU_ASSERT_EQUAL(err, -EINVAL);

	/* Only equal values */
	fill_vmeta_with(metas[0], 1);
	fill_vmeta_with(metas[1], 1);
	fill_vmeta_with(&meta_a, 1);
	fill_vmeta_with(&meta_b, 0);
	err = vmeta_session_merge_metadata(metas, 2, &common);
	CU_ASSERT_EQUAL(err, 0);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&common, &meta_a), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(metas[0], metas[1]), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(metas[0], &meta_b), true);

	/* Only different values */
	fill_vmeta_with(metas[0], 1);
	fill_vmeta_with(metas[1], 2);
	fill_vmeta_with(&meta_a, 0);
	fill_vmeta_with(&meta_b, 1);
	fill_vmeta_with(&meta_c, 2);
	err = vmeta_session_merge_metadata(metas, 2, &common);
	CU_ASSERT_EQUAL(err, 0);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&common, &meta_a), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(metas[0], &meta_b), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(metas[1], &meta_c), true);

	/* Some different values (maker and camera_type are identical) */
	fill_vmeta_with(metas[0], 1);
	fill_vmeta_with(metas[1], 2);
	metas[0]->maker[0] = 'P';
	metas[0]->maker[1] = '\0';
	metas[0]->camera_type = VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT;
	metas[1]->maker[0] = 'P';
	metas[1]->maker[1] = '\0';
	metas[1]->camera_type = VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT;
	fill_vmeta_with(&meta_a, 0);
	meta_a.maker[0] = 'P';
	meta_a.maker[1] = '\0';
	meta_a.camera_type = VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT;
	fill_vmeta_with(&meta_b, 1);
	meta_b.maker[0] = '\0';
	meta_b.camera_type = VMETA_CAMERA_TYPE_UNKNOWN;
	fill_vmeta_with(&meta_c, 2);
	meta_c.maker[0] = '\0';
	meta_c.camera_type = VMETA_CAMERA_TYPE_UNKNOWN;
	err = vmeta_session_merge_metadata(metas, 2, &common);
	CU_ASSERT_EQUAL(err, 0);
	CU_ASSERT_EQUAL(vmeta_session_cmp(&common, &meta_a), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(metas[0], &meta_b), true);
	CU_ASSERT_EQUAL(vmeta_session_cmp(metas[1], &meta_c), true);

	free(metas[0]);
	free(metas[1]);
	free(metas);
}


static void test_session_is_valid()
{
	int valid = 0;
	struct vmeta_session meta = {0};

	/* einval */
	valid = vmeta_session_is_valid(NULL);
	CU_ASSERT_EQUAL(valid, 0);

	/* missing maker, friendly name and model */
	valid = vmeta_session_is_valid(&meta);
	CU_ASSERT_EQUAL(valid, 0);

	/* missing maker and model */
	snprintf(meta.friendly_name, 5, "test");
	valid = vmeta_session_is_valid(&meta);
	CU_ASSERT_EQUAL(valid, 0);

	/* missing maker */
	snprintf(meta.model, 5, "test");
	valid = vmeta_session_is_valid(&meta);
	CU_ASSERT_EQUAL(valid, 0);

	/* invalid maker */
	snprintf(meta.maker, 5, "test");
	valid = vmeta_session_is_valid(&meta);
	CU_ASSERT_EQUAL(valid, 0);

	snprintf(meta.maker, 7, "Parrot");
	valid = vmeta_session_is_valid(&meta);
	CU_ASSERT_EQUAL(valid, 1);
}


static void compare_session_proto(const Vmeta__SessionMetadata *proto,
				  struct vmeta_session *meta)
{
	CU_ASSERT_PTR_NOT_NULL_FATAL(proto);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta);

	CU_ASSERT_STRING_EQUAL(proto->friendly_name, meta->friendly_name);
	CU_ASSERT_STRING_EQUAL(proto->maker, meta->maker);
	CU_ASSERT_STRING_EQUAL(proto->model, meta->model);
	CU_ASSERT_STRING_EQUAL(proto->model_id, meta->model_id);
	CU_ASSERT_STRING_EQUAL(proto->serial_number, meta->serial_number);
	CU_ASSERT_STRING_EQUAL(proto->software_version, meta->software_version);
	CU_ASSERT_STRING_EQUAL(proto->build_id, meta->build_id);
	CU_ASSERT_STRING_EQUAL(proto->title, meta->title);
	CU_ASSERT_STRING_EQUAL(proto->comment, meta->comment);
	CU_ASSERT_STRING_EQUAL(proto->copyright, meta->copyright);
	CU_ASSERT_EQUAL(proto->media_date, meta->media_date);
	CU_ASSERT_EQUAL(proto->media_date_gmtoff, meta->media_date_gmtoff);
	CU_ASSERT_EQUAL(proto->boot_date, meta->boot_date);
	CU_ASSERT_EQUAL(proto->boot_date_gmtoff, meta->boot_date_gmtoff);
	CU_ASSERT_STRING_EQUAL(proto->boot_id, meta->boot_id);
	CU_ASSERT_EQUAL(proto->flight_date, meta->flight_date);
	CU_ASSERT_EQUAL(proto->flight_date_gmtoff, meta->flight_date_gmtoff);
	CU_ASSERT_STRING_EQUAL(proto->flight_id, meta->flight_id);
	CU_ASSERT_STRING_EQUAL(proto->custom_id, meta->custom_id);
	/* takeoff loc */
	compare_vmeta_proto_location(
		&meta->takeoff_loc, proto->takeoff_location, true);
	/* picture_fov: vmeta_session_to_proto() converts deg -> rad
	 * (fov->x/y = meta->picture_fov.horz/vert * M_PI / 180.f), so the
	 * expected value here must apply the same conversion before
	 * comparing */
	if (meta->picture_fov.has_horz && meta->picture_fov.has_vert) {
		struct vmeta_xy picture_fov = {
			.x = meta->picture_fov.horz * (float)M_PI / 180.f,
			.y = meta->picture_fov.vert * (float)M_PI / 180.f,
		};
		compare_vmeta_proto_xy(&picture_fov, proto->picture_fov);
	} else {
		CU_ASSERT_PTR_NULL(proto->picture_fov);
	}
	if (meta->has_thermal)
		compare_vmeta_proto_thermal(&meta->thermal, proto->thermal);
	else
		CU_ASSERT_PTR_NULL(proto->thermal);

	CU_ASSERT_EQUAL(proto->default_media, meta->default_media);
	CU_ASSERT_EQUAL(
		proto->camera_type,
		vmeta_session_camera_type_vmeta_to_proto(meta->camera_type));
	/* camera_model */
	compare_vmeta_proto_camera_model(&meta->camera_model,
					 proto->camera_model);
	/* overlay */
	compare_vmeta_proto_overlay(&meta->overlay, proto->overlay);
	/* principal_point */
	if (meta->principal_point.valid) {
		compare_vmeta_proto_xy(&meta->principal_point.position,
				       proto->principal_point);
	} else {
		CU_ASSERT_PTR_NULL(proto->principal_point);
	}
	CU_ASSERT_EQUAL(
		proto->video_mode,
		vmeta_session_video_mode_vmeta_to_proto(meta->video_mode));
	CU_ASSERT_EQUAL(proto->video_stop_reason,
			vmeta_session_video_stop_reason_vmeta_to_proto(
				meta->video_stop_reason));
	CU_ASSERT_EQUAL(proto->dynamic_range,
			vmeta_session_dynamic_range_vmeta_to_proto(
				meta->dynamic_range));
	CU_ASSERT_EQUAL(
		proto->photo_mode,
		vmeta_session_photo_mode_vmeta_to_proto(meta->photo_mode));
	CU_ASSERT_EQUAL(proto->panorama_type,
			vmeta_session_panorama_type_vmeta_to_proto(
				meta->panorama_type));
	CU_ASSERT_EQUAL(proto->photo_count, meta->photo_count);
	CU_ASSERT_STRING_EQUAL(proto->secure_cn, meta->secure_cn);
	CU_ASSERT_EQUAL(
		proto->tone_mapping,
		vmeta_session_tone_mapping_vmeta_to_proto(meta->tone_mapping));
	CU_ASSERT_EQUAL(proto->first_frame_capture_ts,
			meta->first_frame_capture_ts);
	CU_ASSERT_EQUAL(proto->first_frame_sample_index,
			meta->first_frame_sample_index);
	CU_ASSERT_EQUAL(proto->media_id, meta->media_id);
	CU_ASSERT_EQUAL(proto->resource_index, meta->resource_index);
}


static void test_session_proto_api()
{
	int ret;
	ssize_t ret2;
	const uint8_t *data;
	size_t len;
	size_t packed_len;
	struct vmeta_session meta = {0};
	struct vmeta_session_proto *meta_proto = NULL;
	const Vmeta__SessionMetadata *proto_meta = NULL;
	Vmeta__SessionMetadata *proto_meta_rw = NULL;

	fill_vmeta_with(&meta, 1);
	meta.camera_model.type = VMETA_CAMERA_MODEL_TYPE_UNKNOWN;
	meta.overlay.type = VMETA_OVERLAY_TYPE_NONE;

	/* Bad args */
	ret = vmeta_session_to_proto(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_to_proto(&meta, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_to_proto(NULL, &meta_proto);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret2 = vmeta_session_proto_get_packed_size(NULL);
	CU_ASSERT_EQUAL(ret2, -EINVAL);

	ret = vmeta_session_proto_get_buffer(NULL, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_buffer(meta_proto, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_buffer(NULL, &data, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_buffer(NULL, NULL, &len);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_buffer(meta_proto, &data, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_buffer(NULL, &data, &len);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_buffer(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_buffer(meta_proto, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_buffer(NULL, data);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_unpacked(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_unpacked(NULL, &proto_meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_unpacked(meta_proto, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_unpacked(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_unpacked(NULL, proto_meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_unpacked(meta_proto, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_unpacked_rw(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_unpacked_rw(NULL, &proto_meta_rw);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_get_unpacked_rw(meta_proto, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_unpacked_rw(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_unpacked_rw(NULL, proto_meta_rw);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_release_unpacked_rw(meta_proto, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_proto_destroy(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* OK */
	ret = vmeta_session_to_proto(&meta, &meta_proto);
	CU_ASSERT_EQUAL(ret, 0);

	ret2 = vmeta_session_proto_get_packed_size(meta_proto);
	CU_ASSERT(ret2 > 0);
	packed_len = ret2;

	/* ro packed buffer */
	ret = vmeta_session_proto_get_buffer(meta_proto, &data, &len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(data);
	CU_ASSERT_EQUAL(len, packed_len);

	ret = vmeta_session_proto_release_buffer(meta_proto, data);
	CU_ASSERT_EQUAL(ret, 0);

	/* ro unpacked buffer */
	ret = vmeta_session_proto_get_unpacked(meta_proto, &proto_meta);
	CU_ASSERT_EQUAL(ret, 0);
	compare_session_proto(proto_meta, &meta);

	ret = vmeta_session_proto_release_unpacked(meta_proto, proto_meta);
	CU_ASSERT_EQUAL(ret, 0);

	/* rw unpacked buffer */
	ret = vmeta_session_proto_get_unpacked_rw(meta_proto, &proto_meta_rw);
	CU_ASSERT_EQUAL(ret, 0);
	compare_session_proto(proto_meta_rw, &meta);

	ret = vmeta_session_proto_release_unpacked_rw(meta_proto,
						      proto_meta_rw);
	CU_ASSERT_EQUAL(ret, 0);

	ret = vmeta_session_proto_destroy(meta_proto);
	CU_ASSERT_EQUAL(ret, 0);
}


static void test_session_to_json(void)
{
	int ret;
	struct json_object *jobj;
	struct json_object *sub;
	struct json_object *val;
	struct vmeta_session meta = {0};

	snprintf(meta.friendly_name,
		 sizeof(meta.friendly_name),
		 "my friendly name");
	snprintf(meta.maker, sizeof(meta.maker), "Parrot");
	meta.media_date = 1706367600;
	meta.media_date_gmtoff = 3600;
	meta.takeoff_loc.valid = 1;
	meta.takeoff_loc.latitude = 48.87197634;
	meta.takeoff_loc.longitude = 2.30852667;
	meta.takeoff_loc.altitude_wgs84ellipsoid = NAN;
	meta.takeoff_loc.altitude_egm96amsl = NAN;
	meta.location.valid = 1;
	meta.location.latitude = 45.18384;
	meta.location.longitude = 5.72083;
	meta.location.altitude_wgs84ellipsoid = NAN;
	meta.location.altitude_egm96amsl = NAN;
	meta.picture_fov.has_horz = 1;
	meta.picture_fov.has_vert = 1;
	meta.picture_fov.horz = 78.00f;
	meta.picture_fov.vert = 49.00f;
	/* Set even though it must never appear in the output, so the
	 * assertion below actually proves the field is omitted rather than
	 * just happening to be zero */
	meta.default_media = 1;

	/* Bad args */
	ret = vmeta_session_to_json(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	jobj = json_object_new_object();
	ret = vmeta_session_to_json(NULL, jobj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_to_json(&meta, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* OK */
	ret = vmeta_session_to_json(&meta, jobj);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "friendly_name", &val));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(val), meta.friendly_name);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "maker", &val));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(val), meta.maker);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "media_date", &val));
	char expected_date[VMETA_SESSION_DATE_MAX_LEN];
	ssize_t dret = vmeta_session_date_write(expected_date,
						sizeof(expected_date),
						meta.media_date,
						meta.media_date_gmtoff);
	CU_ASSERT(dret > 0);
	CU_ASSERT_STRING_EQUAL(json_object_get_string(val), expected_date);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "takeoff_loc", &sub));
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "latitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(val),
			       meta.takeoff_loc.latitude,
			       0.00001);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "longitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(val),
			       meta.takeoff_loc.longitude,
			       0.00001);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "location", &sub));
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "latitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.location.latitude, 0.00001);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "longitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.location.longitude, 0.00001);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "picture_fov", &sub));
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "horz", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.picture_fov.horz, 0.001);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "vert", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.picture_fov.vert, 0.001);

	/* default_media is deliberately never serialized */
	CU_ASSERT_FALSE(json_object_object_get_ex(jobj, "default_media", &val));

	json_object_put(jobj);
}


static void test_session_date_write(void)
{
	ssize_t ret;
	char date[VMETA_SESSION_DATE_MAX_LEN];
	uint64_t timestamp = 1706367600; /* 27 Jan 2024 */
	int32_t gmtoff = 3600; /* UTC+1 */

	ret = vmeta_session_date_write(NULL, sizeof(date), timestamp, gmtoff);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_date_write(date, 0, timestamp, gmtoff);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_date_write(date, 5, timestamp, gmtoff);
	CU_ASSERT_EQUAL(ret, -ENOBUFS);

	memset(date, 0, sizeof(date));
	ret = vmeta_session_date_write(date, sizeof(date), timestamp, gmtoff);

	CU_ASSERT_TRUE(ret > 0);
	CU_ASSERT_TRUE((size_t)ret < sizeof(date));
	/* Check that string looks like a date */
	CU_ASSERT_PTR_NOT_NULL(strchr(date, ':'));
	CU_ASSERT_EQUAL(ret, (ssize_t)strlen(date));
}


#define FIELD_POS(_name) offsetof(struct vmeta_session, _name)


#define MAKE_VMETA_TEST(_type, _name, _field)                                  \
	{                                                                      \
		_type, VMETA_VALUE_TYPE_STRING, _type##_KEY_##_name, #_field,  \
			FIELD_POS(_field), false, NULL                         \
	}


#define MAKE_VMETA_TEST_SPECIAL(_type, _name, _value, _field)                  \
	{                                                                      \
		_type, VMETA_VALUE_TYPE_STRING, _type##_KEY_##_name, _value,   \
			FIELD_POS(_field), false, NULL                         \
	}

#define MAKE_VMETA_TEST_SPECIAL_UINT32(_type, _name, _value, _field)           \
	{                                                                      \
		_type, VMETA_VALUE_TYPE_UINT32, _type##_KEY_##_name, _value,   \
			FIELD_POS(_field), false, NULL                         \
	}


#define MAKE_VMETA_TEST_SPECIAL_UINT64(_type, _name, _value, _field)           \
	{                                                                      \
		_type, VMETA_VALUE_TYPE_UINT64, _type##_KEY_##_name, _value,   \
			FIELD_POS(_field), false, NULL                         \
	}


#define MAKE_VMETA_TEST_ENUM(_type, _name, _value, _field, _to_str)            \
	{                                                                      \
		_type, VMETA_VALUE_TYPE_ENUM, _type##_KEY_##_name, _value,     \
			FIELD_POS(_field), false, _to_str                      \
	}


#define MAKE_TEST_META(_name, _field)                                          \
	MAKE_VMETA_TEST(VMETA_REC_META, _name, _field)


#define MAKE_TEST_UDTA(_name, _field)                                          \
	MAKE_VMETA_TEST(VMETA_REC_UDTA, _name, _field)


/* codecheck_ignore[COMPLEX_MACRO] */
#define MAKE_TEST_META_AND_UDTA(_name, _field)                                 \
	MAKE_TEST_META(_name, _field), MAKE_TEST_UDTA(_name, _field)


enum vmeta_value_type {
	VMETA_VALUE_TYPE_STRING = 0,
	VMETA_VALUE_TYPE_UINT32,
	VMETA_VALUE_TYPE_UINT64,
	VMETA_VALUE_TYPE_ENUM,
};


struct vmeta_test_case {
	enum vmeta_record_type type;
	enum vmeta_value_type value_type;
	const char *key;
	const char *value;
	size_t offset;
	bool found;
	const char *(*enum_to_str)(int val);
};


static const char *enum_to_str_camera_type(int val)
{
	return vmeta_camera_type_to_str(val);
}


static const char *enum_to_str_camera_subtype(int val)
{
	return vmeta_camera_subtype_to_str(val);
}


static const char *enum_to_str_camera_spectrum(int val)
{
	return vmeta_camera_spectrum_to_str(val);
}


static const char *enum_to_str_camera_model_type(int val)
{
	return vmeta_camera_model_type_to_str(val);
}


static const char *enum_to_str_video_mode(int val)
{
	return vmeta_video_mode_to_str(val);
}


static const char *enum_to_str_video_stop_reason(int val)
{
	return vmeta_video_stop_reason_to_str(val);
}


static const char *enum_to_str_dynamic_range(int val)
{
	return vmeta_dynamic_range_to_str(val);
}


static const char *enum_to_str_tone_mapping(int val)
{
	return vmeta_tone_mapping_to_str(val);
}


static struct vmeta_test_case vmeta_test_array[] = {
	MAKE_TEST_META_AND_UDTA(FRIENDLY_NAME, friendly_name),
	MAKE_TEST_META_AND_UDTA(COMMENT, comment),
	MAKE_TEST_META_AND_UDTA(TITLE, title),
	MAKE_TEST_META_AND_UDTA(COPYRIGHT, copyright),
	MAKE_TEST_META_AND_UDTA(MAKER, maker),
	MAKE_TEST_META_AND_UDTA(MODEL, model),
	MAKE_TEST_META_AND_UDTA(SOFTWARE_VERSION, software_version),
	MAKE_TEST_META_AND_UDTA(SERIAL_NUMBER, serial_number),
	MAKE_VMETA_TEST_SPECIAL(VMETA_REC_META, MODEL_ID, "test", model_id),
	MAKE_TEST_META(CUSTOM_ID, custom_id),
	MAKE_TEST_META(BOOT_ID, boot_id),
	MAKE_TEST_META(RUN_ID, run_id),
	MAKE_TEST_META(BUILD_ID, build_id),
	MAKE_TEST_META(FLIGHT_ID, flight_id),
	MAKE_VMETA_TEST_SPECIAL_UINT32(VMETA_REC_META, MEDIA_ID, "1", media_id),
	MAKE_VMETA_TEST_SPECIAL_UINT32(VMETA_REC_META,
				       RESOURCE_INDEX,
				       "1",
				       resource_index),
	MAKE_TEST_META(CAMERA_SERIAL_NUMBER, camera_serial_number),
	MAKE_VMETA_TEST_SPECIAL_UINT64(VMETA_REC_META,
				       FIRST_FRAME_CAPTURE_TS,
				       "123456789012",
				       first_frame_capture_ts),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     CAMERA_TYPE,
			     "front",
			     camera_type,
			     enum_to_str_camera_type),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     CAMERA_SUBTYPE,
			     "wide",
			     camera_subtype,
			     enum_to_str_camera_subtype),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     CAMERA_SPECTRUM,
			     "visible",
			     camera_spectrum,
			     enum_to_str_camera_spectrum),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     VIDEO_MODE,
			     "standard",
			     video_mode,
			     enum_to_str_video_mode),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     VIDEO_STOP_REASON,
			     "user",
			     video_stop_reason,
			     enum_to_str_video_stop_reason),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     DYNAMIC_RANGE,
			     "sdr",
			     dynamic_range,
			     enum_to_str_dynamic_range),
	MAKE_VMETA_TEST_ENUM(VMETA_REC_META,
			     TONE_MAPPING,
			     "standard",
			     tone_mapping,
			     enum_to_str_tone_mapping),
};


static struct vmeta_test_case vmeta_test_array_invalid[] = {
	/* Key is invalid */
	{
		.type = VMETA_REC_META,
		.value_type = VMETA_VALUE_TYPE_STRING,
		.key = "invalid_key",
		.value = "test",
		.offset = 0,
		.found = false,
	},
	/* Key is null */
	{
		.type = VMETA_REC_META,
		.value_type = VMETA_VALUE_TYPE_STRING,
		.key = NULL,
		.value = "test",
		.offset = 0,
		.found = false,
	},
	/* Value is null */
	{
		.type = VMETA_REC_META,
		.value_type = VMETA_VALUE_TYPE_STRING,
		.key = VMETA_REC_META_KEY_FRIENDLY_NAME,
		.value = NULL,
		.offset = 0,
		.found = false,
	},
};


static inline void *get_field_ptr(struct vmeta_session *struct_ptr,
				  size_t offset)
{
	return (void *)((char *)struct_ptr + offset);
}


#define TEST_SESSION_RECORDING_READ_STR(_key, _value, _meta, _offset)          \
	do {                                                                   \
		int _ret = vmeta_session_recording_read(_key, _value, &_meta); \
		CU_ASSERT_EQUAL(_ret, 0);                                      \
		const char *ptr = get_field_ptr(&_meta, _offset);              \
		CU_ASSERT_STRING_EQUAL(ptr, _value);                           \
	} while (0)


#define TEST_SESSION_RECORDING_READ_UINT32(_key, _value, _meta, _offset)       \
	do {                                                                   \
		int _ret = vmeta_session_recording_read(_key, _value, &_meta); \
		CU_ASSERT_EQUAL(_ret, 0);                                      \
		const uint32_t *ptr = get_field_ptr(&_meta, _offset);          \
		uint32_t _val = strtoul(_value, NULL, 0);                      \
		CU_ASSERT_EQUAL((*ptr), _val);                                 \
	} while (0)


#define TEST_SESSION_RECORDING_READ_UINT64(_key, _value, _meta, _offset)       \
	do {                                                                   \
		int _ret = vmeta_session_recording_read(_key, _value, &_meta); \
		CU_ASSERT_EQUAL(_ret, 0);                                      \
		const uint64_t *ptr = get_field_ptr(&_meta, _offset);          \
		uint64_t _val = strtoull(_value, NULL, 0);                     \
		CU_ASSERT_EQUAL((*ptr), _val);                                 \
	} while (0)


#define TEST_SESSION_RECORDING_READ_ENUM(                                      \
	_key, _value, _meta, _offset, _to_str)                                 \
	do {                                                                   \
		int _ret = vmeta_session_recording_read(_key, _value, &_meta); \
		CU_ASSERT_EQUAL(_ret, 0);                                      \
		const int *ptr = get_field_ptr(&_meta, _offset);               \
		CU_ASSERT_STRING_EQUAL((*_to_str)(*ptr), _value);              \
	} while (0)


static void test_session_recording_read(void)
{
	struct vmeta_session meta = {};

	for (size_t i = 0; i < SIZEOF_ARRAY(vmeta_test_array); i++) {
		switch (vmeta_test_array[i].value_type) {
		case VMETA_VALUE_TYPE_STRING:
			TEST_SESSION_RECORDING_READ_STR(
				vmeta_test_array[i].key,
				vmeta_test_array[i].value,
				meta,
				vmeta_test_array[i].offset);
			break;
		case VMETA_VALUE_TYPE_UINT32:
			TEST_SESSION_RECORDING_READ_UINT32(
				vmeta_test_array[i].key,
				vmeta_test_array[i].value,
				meta,
				vmeta_test_array[i].offset);
			break;
		case VMETA_VALUE_TYPE_UINT64:
			TEST_SESSION_RECORDING_READ_UINT64(
				vmeta_test_array[i].key,
				vmeta_test_array[i].value,
				meta,
				vmeta_test_array[i].offset);
			break;
		case VMETA_VALUE_TYPE_ENUM:
			TEST_SESSION_RECORDING_READ_ENUM(
				vmeta_test_array[i].key,
				vmeta_test_array[i].value,
				meta,
				vmeta_test_array[i].offset,
				vmeta_test_array[i].enum_to_str);
			break;
		default:
			CU_FAIL();
		}
	}
}


static void session_meta_write_cb(enum vmeta_record_type type,
				  const char *key,
				  const char *value,
				  void *userdata)
{
	unsigned int *meta_count = (unsigned int *)(userdata);
	bool found = false;

	for (size_t i = 0; i < SIZEOF_ARRAY(vmeta_test_array); i++) {
		if (strcmp(key, vmeta_test_array[i].key) == 0) {
			/* Make sure meta is send only once */
			CU_ASSERT_FALSE(vmeta_test_array[i].found);
			CU_ASSERT_EQUAL(type, vmeta_test_array[i].type);
			CU_ASSERT_STRING_EQUAL(value,
					       vmeta_test_array[i].value);
			vmeta_test_array[i].found = true;
			found = true;
			break;
		}
	}

	(*meta_count)++;
	CU_ASSERT_FATAL(found);
}


static void test_session_recording_write_table_driven(void)
{
	unsigned int meta_count = 0;
	struct vmeta_session meta = {};

	for (size_t i = 0; i < SIZEOF_ARRAY(vmeta_test_array); i++) {
		vmeta_session_recording_read(vmeta_test_array[i].key,
					     vmeta_test_array[i].value,
					     &meta);
	}

	for (size_t i = 0; i < SIZEOF_ARRAY(vmeta_test_array_invalid); i++) {
		vmeta_session_recording_read(vmeta_test_array_invalid[i].key,
					     vmeta_test_array_invalid[i].value,
					     &meta);
	}

	int res = vmeta_session_recording_write(
		&meta, &session_meta_write_cb, &meta_count);

	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(meta_count, SIZEOF_ARRAY(vmeta_test_array));

	for (size_t i = 0; i < SIZEOF_ARRAY(vmeta_test_array); i++) {
		/* Test cb was called for all field */
		CU_ASSERT_TRUE(vmeta_test_array[i].found);
	}

	for (size_t i = 0; i < SIZEOF_ARRAY(vmeta_test_array_invalid); i++) {
		/* Test cb is not called with invalid fields */
		CU_ASSERT_FALSE(vmeta_test_array_invalid[i].found);
	}
}


/* Captures the value written for a single given key, for use by the
 * composite-format tests below (dates, locations, thermal...) where the
 * generic table-driven vmeta_test_array/session_meta_write_cb pattern
 * doesn't fit (multi-field structs, type-gated sub-fields). */
struct vmeta_test_capture {
	const char *key;
	char value[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
	bool found;
};


static void capture_write_cb(enum vmeta_record_type type,
			     const char *key,
			     const char *value,
			     void *userdata)
{
	struct vmeta_test_capture *capture = userdata;

	(void)type;

	if (strcmp(key, capture->key) != 0)
		return;

	/* Make sure the key is only written once */
	CU_ASSERT_FALSE(capture->found);
	snprintf(capture->value, sizeof(capture->value), "%s", value);
	capture->found = true;
}


static void test_session_recording_date(void)
{
	struct {
		const char *key;
		uint64_t date;
		int32_t gmtoff;
		size_t date_offset;
		size_t gmtoff_offset;
	} cases[] = {
		{VMETA_REC_META_KEY_MEDIA_DATE,
		 1700000000,
		 3600,
		 FIELD_POS(media_date),
		 FIELD_POS(media_date_gmtoff)},
		{VMETA_REC_META_KEY_RUN_DATE,
		 1650000000,
		 -21600,
		 FIELD_POS(run_date),
		 FIELD_POS(run_date_gmtoff)},
		{VMETA_REC_META_KEY_BOOT_DATE,
		 1600000000,
		 0,
		 FIELD_POS(boot_date),
		 FIELD_POS(boot_date_gmtoff)},
		{VMETA_REC_META_KEY_FLIGHT_DATE,
		 1690000000,
		 7200,
		 FIELD_POS(flight_date),
		 FIELD_POS(flight_date_gmtoff)},
	};

	for (size_t i = 0; i < SIZEOF_ARRAY(cases); i++) {
		char expected[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t wret = vmeta_session_date_write(expected,
							sizeof(expected),
							cases[i].date,
							cases[i].gmtoff);
		CU_ASSERT(wret > 0);

		struct vmeta_session meta = {0};
		int ret = vmeta_session_recording_read(
			cases[i].key, expected, &meta);
		CU_ASSERT_EQUAL(ret, 0);

		const uint64_t *date =
			get_field_ptr(&meta, cases[i].date_offset);
		const int32_t *gmtoff =
			get_field_ptr(&meta, cases[i].gmtoff_offset);
		CU_ASSERT_EQUAL(*date, cases[i].date);
		CU_ASSERT_EQUAL(*gmtoff, cases[i].gmtoff);

		struct vmeta_test_capture capture = {.key = cases[i].key};
		ret = vmeta_session_recording_write(
			&meta, &capture_write_cb, &capture);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_TRUE(capture.found);
		CU_ASSERT_STRING_EQUAL(capture.value, expected);
	}
}


static void test_session_recording_location(void)
{
	struct vmeta_location loc = {
		.latitude = 48.87197634,
		.longitude = 2.30852667,
		.altitude_wgs84ellipsoid = NAN,
		.altitude_egm96amsl = 156.32,
		.valid = 1,
		.sv_count = VMETA_LOCATION_INVALID_SV_COUNT,
	};
	char expected[VMETA_SESSION_LOCATION_MAX_LEN];
	ssize_t wret =
		vmeta_session_location_write(expected,
					     sizeof(expected),
					     VMETA_SESSION_LOCATION_ISO6709,
					     &loc);
	CU_ASSERT(wret > 0);

	/* TAKEOFF_LOC */
	struct vmeta_session meta = {0};
	int ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_TAKEOFF_LOC, expected, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	compare_vmeta_location(&meta.takeoff_loc, &loc, false);

	struct vmeta_test_capture capture = {
		.key = VMETA_REC_META_KEY_TAKEOFF_LOC};
	ret = vmeta_session_recording_write(&meta, &capture_write_cb, &capture);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture.found);
	CU_ASSERT_STRING_EQUAL(capture.value, expected);

	/* LOCATION */
	struct vmeta_session meta2 = {0};
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_LOCATION, expected, &meta2);
	CU_ASSERT_EQUAL(ret, 0);
	compare_vmeta_location(&meta2.location, &loc, false);

	struct vmeta_test_capture capture2 = {
		.key = VMETA_REC_META_KEY_LOCATION};
	ret = vmeta_session_recording_write(
		&meta2, &capture_write_cb, &capture2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture2.found);
	CU_ASSERT_STRING_EQUAL(capture2.value, expected);
}


static void test_session_recording_fov(void)
{
	struct vmeta_fov fov = {
		.horz = 78.00f, .vert = 49.00f, .has_horz = 1, .has_vert = 1};
	char expected[VMETA_SESSION_FOV_MAX_LEN];
	ssize_t wret =
		vmeta_session_fov_write(expected, sizeof(expected), &fov);
	CU_ASSERT(wret > 0);

	/* Combined key */
	struct vmeta_session meta = {0};
	int ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_PICTURE_FOV, expected, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(meta.picture_fov.has_horz);
	CU_ASSERT_TRUE(meta.picture_fov.has_vert);
	CU_ASSERT_DOUBLE_EQUAL(meta.picture_fov.horz, fov.horz, 0.001);
	CU_ASSERT_DOUBLE_EQUAL(meta.picture_fov.vert, fov.vert, 0.001);

	struct vmeta_test_capture capture = {
		.key = VMETA_REC_META_KEY_PICTURE_FOV};
	ret = vmeta_session_recording_write(&meta, &capture_write_cb, &capture);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture.found);
	CU_ASSERT_STRING_EQUAL(capture.value, expected);

	/* Deprecated split (read-only) keys */
	struct vmeta_session meta2 = {0};
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_PICTURE_HORZ_FOV, "78.00", &meta2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(meta2.picture_fov.has_horz);
	CU_ASSERT_FALSE(meta2.picture_fov.has_vert);
	CU_ASSERT_DOUBLE_EQUAL(meta2.picture_fov.horz, 78.00, 0.001);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_PICTURE_VERT_FOV, "49.00", &meta2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(meta2.picture_fov.has_vert);
	CU_ASSERT_DOUBLE_EQUAL(meta2.picture_fov.vert, 49.00, 0.001);
}


static void test_session_recording_camera_model(void)
{
	/* Perspective */
	struct vmeta_session meta = {0};
	meta.camera_model.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	meta.camera_model.perspective.distortion.r1 = 0.01f;
	meta.camera_model.perspective.distortion.r2 = 0.02f;
	meta.camera_model.perspective.distortion.r3 = 0.03f;
	meta.camera_model.perspective.distortion.t1 = 0.04f;
	meta.camera_model.perspective.distortion.t2 = 0.05f;

	char expected_dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
	ssize_t wret = vmeta_session_perspective_distortion_write(
		expected_dist,
		sizeof(expected_dist),
		meta.camera_model.perspective.distortion.r1,
		meta.camera_model.perspective.distortion.r2,
		meta.camera_model.perspective.distortion.r3,
		meta.camera_model.perspective.distortion.t1,
		meta.camera_model.perspective.distortion.t2);
	CU_ASSERT(wret > 0);

	struct vmeta_session parsed = {0};
	int ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_CAMERA_MODEL_TYPE, "perspective", &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(
		enum_to_str_camera_model_type(parsed.camera_model.type),
		"perspective");

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_PERSPECTIVE_DISTORTION,
		expected_dist,
		&parsed);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(parsed.camera_model.perspective.distortion.r1,
			       meta.camera_model.perspective.distortion.r1,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed.camera_model.perspective.distortion.r2,
			       meta.camera_model.perspective.distortion.r2,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed.camera_model.perspective.distortion.r3,
			       meta.camera_model.perspective.distortion.r3,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed.camera_model.perspective.distortion.t1,
			       meta.camera_model.perspective.distortion.t1,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed.camera_model.perspective.distortion.t2,
			       meta.camera_model.perspective.distortion.t2,
			       0.000001);

	struct vmeta_test_capture capture_dist = {
		.key = VMETA_REC_META_KEY_PERSPECTIVE_DISTORTION};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_dist);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_dist.found);
	CU_ASSERT_STRING_EQUAL(capture_dist.value, expected_dist);

	/* Fisheye */
	struct vmeta_session meta_fe = {0};
	meta_fe.camera_model.type = VMETA_CAMERA_MODEL_TYPE_FISHEYE;
	meta_fe.camera_model.fisheye.affine_matrix.c = 1.0001f;
	meta_fe.camera_model.fisheye.affine_matrix.d = 0.0002f;
	meta_fe.camera_model.fisheye.affine_matrix.e = -0.0003f;
	meta_fe.camera_model.fisheye.affine_matrix.f = 0.9998f;
	meta_fe.camera_model.fisheye.polynomial.p2 = 0.1f;
	meta_fe.camera_model.fisheye.polynomial.p3 = 0.2f;
	meta_fe.camera_model.fisheye.polynomial.p4 = 0.3f;

	char expected_matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
	wret = vmeta_session_fisheye_affine_matrix_write(
		expected_matrix,
		sizeof(expected_matrix),
		meta_fe.camera_model.fisheye.affine_matrix.c,
		meta_fe.camera_model.fisheye.affine_matrix.d,
		meta_fe.camera_model.fisheye.affine_matrix.e,
		meta_fe.camera_model.fisheye.affine_matrix.f);
	CU_ASSERT(wret > 0);

	char expected_poly[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
	wret = vmeta_session_fisheye_polynomial_write(
		expected_poly,
		sizeof(expected_poly),
		meta_fe.camera_model.fisheye.polynomial.p2,
		meta_fe.camera_model.fisheye.polynomial.p3,
		meta_fe.camera_model.fisheye.polynomial.p4);
	CU_ASSERT(wret > 0);

	struct vmeta_session parsed_fe = {0};
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_CAMERA_MODEL_TYPE, "fisheye", &parsed_fe);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(
		enum_to_str_camera_model_type(parsed_fe.camera_model.type),
		"fisheye");

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_FISHEYE_AFFINE_MATRIX,
		expected_matrix,
		&parsed_fe);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.affine_matrix.c,
			       meta_fe.camera_model.fisheye.affine_matrix.c,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.affine_matrix.d,
			       meta_fe.camera_model.fisheye.affine_matrix.d,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.affine_matrix.e,
			       meta_fe.camera_model.fisheye.affine_matrix.e,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.affine_matrix.f,
			       meta_fe.camera_model.fisheye.affine_matrix.f,
			       0.000001);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_FISHEYE_POLYNOMIAL,
		expected_poly,
		&parsed_fe);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.polynomial.p2,
			       meta_fe.camera_model.fisheye.polynomial.p2,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.polynomial.p3,
			       meta_fe.camera_model.fisheye.polynomial.p3,
			       0.000001);
	CU_ASSERT_DOUBLE_EQUAL(parsed_fe.camera_model.fisheye.polynomial.p4,
			       meta_fe.camera_model.fisheye.polynomial.p4,
			       0.000001);

	struct vmeta_test_capture capture_matrix = {
		.key = VMETA_REC_META_KEY_FISHEYE_AFFINE_MATRIX};
	ret = vmeta_session_recording_write(
		&meta_fe, &capture_write_cb, &capture_matrix);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_matrix.found);
	CU_ASSERT_STRING_EQUAL(capture_matrix.value, expected_matrix);

	struct vmeta_test_capture capture_poly = {
		.key = VMETA_REC_META_KEY_FISHEYE_POLYNOMIAL};
	ret = vmeta_session_recording_write(
		&meta_fe, &capture_write_cb, &capture_poly);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_poly.found);
	CU_ASSERT_STRING_EQUAL(capture_poly.value, expected_poly);
}


static void test_session_recording_overlay(void)
{
	struct vmeta_session meta = {0};
	meta.overlay.type = VMETA_OVERLAY_TYPE_HEADER_FOOTER;
	meta.overlay.header_footer.header_height = 0.05f;
	meta.overlay.header_footer.footer_height = 0.08f;

	char expected[VMETA_SESSION_OVERLAY_HEADER_FOOTER_MAX_LEN];
	ssize_t wret = vmeta_session_overlay_header_footer_write(
		expected,
		sizeof(expected),
		meta.overlay.header_footer.header_height,
		meta.overlay.header_footer.footer_height);
	CU_ASSERT(wret > 0);

	struct vmeta_session parsed = {0};
	int ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_HEADER_FOOTER, expected, &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(parsed.overlay.type, VMETA_OVERLAY_TYPE_HEADER_FOOTER);
	CU_ASSERT_DOUBLE_EQUAL(parsed.overlay.header_footer.header_height,
			       meta.overlay.header_footer.header_height,
			       0.0001);
	CU_ASSERT_DOUBLE_EQUAL(parsed.overlay.header_footer.footer_height,
			       meta.overlay.header_footer.footer_height,
			       0.0001);

	struct vmeta_test_capture capture = {
		.key = VMETA_REC_META_KEY_HEADER_FOOTER};
	ret = vmeta_session_recording_write(&meta, &capture_write_cb, &capture);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture.found);
	CU_ASSERT_STRING_EQUAL(capture.value, expected);
}


static void test_session_recording_principal_point(void)
{
	struct vmeta_session meta = {0};
	meta.principal_point.valid = 1;
	meta.principal_point.position.x = 0.491170f;
	meta.principal_point.position.y = 0.395359f;

	char expected[VMETA_SESSION_PRINCIPAL_POINT_MAX_LEN];
	ssize_t wret = vmeta_session_principal_point_write(
		expected, sizeof(expected), &meta.principal_point);
	CU_ASSERT(wret > 0);

	struct vmeta_session parsed = {0};
	int ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_PRINCIPAL_POINT, expected, &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(parsed.principal_point.valid);
	CU_ASSERT_DOUBLE_EQUAL(parsed.principal_point.position.x,
			       meta.principal_point.position.x,
			       0.00001);
	CU_ASSERT_DOUBLE_EQUAL(parsed.principal_point.position.y,
			       meta.principal_point.position.y,
			       0.00001);

	struct vmeta_test_capture capture = {
		.key = VMETA_REC_META_KEY_PRINCIPAL_POINT};
	ret = vmeta_session_recording_write(&meta, &capture_write_cb, &capture);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture.found);
	CU_ASSERT_STRING_EQUAL(capture.value, expected);
}


static void test_session_recording_thermal(void)
{
	struct vmeta_session meta = {0};
	meta.thermal.metaversion = 2;
	snprintf(meta.thermal.camserial,
		 sizeof(meta.thermal.camserial),
		 "THERM-0001");
	meta.thermal.alignment.valid = 1;
	meta.thermal.alignment.rotation.yaw = -1.355f;
	meta.thermal.alignment.rotation.pitch = 0.609f;
	meta.thermal.alignment.rotation.roll = 89.730f;
	meta.thermal.conv_low.valid = 1;
	meta.thermal.conv_low.r = 1390082.947851f;
	meta.thermal.conv_low.b = 1449.5f;
	meta.thermal.conv_low.f = 1.0f;
	meta.thermal.conv_low.o = 1476.356f;
	meta.thermal.conv_low.tau_win = 0.8f;
	meta.thermal.conv_low.t_win = 25.0f;
	meta.thermal.conv_low.t_bg = 22.0f;
	meta.thermal.conv_low.emissivity = 0.98f;
	meta.thermal.conv_high.valid = 1;
	meta.thermal.conv_high.r = 200000.123456f;
	meta.thermal.conv_high.b = 1500.0f;
	meta.thermal.conv_high.f = 1.0f;
	meta.thermal.conv_high.o = 100.500f;
	meta.thermal.conv_high.tau_win = 0.9f;
	meta.thermal.conv_high.t_win = 20.0f;
	meta.thermal.conv_high.t_bg = 15.0f;
	meta.thermal.conv_high.emissivity = 0.95f;
	meta.thermal.scale_factor = 1.035156;
	/* has_thermal is normally derived by vmeta_session_recording_read();
	 * since this meta is built by hand for the write-side check, set it
	 * explicitly. */
	meta.has_thermal = 1;

	char expected_align[VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN];
	ssize_t wret =
		vmeta_session_thermal_alignment_write(expected_align,
						      sizeof(expected_align),
						      &meta.thermal.alignment);
	CU_ASSERT(wret > 0);

	char expected_conv_low[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
	wret = vmeta_session_thermal_conversion_write(expected_conv_low,
						      sizeof(expected_conv_low),
						      &meta.thermal.conv_low);
	CU_ASSERT(wret > 0);

	char expected_conv_high[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
	wret = vmeta_session_thermal_conversion_write(
		expected_conv_high,
		sizeof(expected_conv_high),
		&meta.thermal.conv_high);
	CU_ASSERT(wret > 0);

	char expected_scale[VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN];
	wret = vmeta_session_thermal_scale_factor_write(
		expected_scale,
		sizeof(expected_scale),
		meta.thermal.scale_factor);
	CU_ASSERT(wret > 0);

	/* Read side: accumulate all thermal keys into a single struct, like a
	 * real MP4 'meta' box parser would */
	struct vmeta_session parsed = {0};
	int ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_THERMAL_METAVERSION, "2", &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_THERMAL_CAMSERIAL, "THERM-0001", &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_THERMAL_ALIGNMENT, expected_align, &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	ret = vmeta_session_recording_read(VMETA_REC_META_KEY_THERMAL_CONV_LOW,
					   expected_conv_low,
					   &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	ret = vmeta_session_recording_read(VMETA_REC_META_KEY_THERMAL_CONV_HIGH,
					   expected_conv_high,
					   &parsed);
	CU_ASSERT_EQUAL(ret, 0);
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_THERMAL_SCALE_FACTOR,
		expected_scale,
		&parsed);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(parsed.thermal.metaversion, 2);
	CU_ASSERT_STRING_EQUAL(parsed.thermal.camserial, "THERM-0001");
	CU_ASSERT_TRUE(parsed.thermal.alignment.valid);
	CU_ASSERT_DOUBLE_EQUAL(parsed.thermal.alignment.rotation.yaw,
			       meta.thermal.alignment.rotation.yaw,
			       0.001);
	CU_ASSERT_TRUE(parsed.thermal.conv_low.valid);
	CU_ASSERT_DOUBLE_EQUAL(
		parsed.thermal.conv_low.r, meta.thermal.conv_low.r, 1.0);
	CU_ASSERT_TRUE(parsed.thermal.conv_high.valid);
	CU_ASSERT_DOUBLE_EQUAL(
		parsed.thermal.conv_high.r, meta.thermal.conv_high.r, 1.0);
	CU_ASSERT_DOUBLE_EQUAL(parsed.thermal.scale_factor,
			       meta.thermal.scale_factor,
			       0.000001);
	/* has_thermal is derived from metaversion/alignment.valid/camserial */
	CU_ASSERT_TRUE(parsed.has_thermal);

	/* Only conv_low/conv_high/scale_factor set: has_thermal must stay 0,
	 * and vmeta_session_recording_write() must then skip all thermal
	 * keys, even though conv_low/conv_high are individually valid */
	struct vmeta_session conv_only = {0};
	conv_only.thermal.conv_low = meta.thermal.conv_low;
	conv_only.thermal.conv_high = meta.thermal.conv_high;
	conv_only.thermal.scale_factor = meta.thermal.scale_factor;
	CU_ASSERT_FALSE(conv_only.has_thermal);

	struct vmeta_test_capture capture_conv_low_absent = {
		.key = VMETA_REC_META_KEY_THERMAL_CONV_LOW};
	ret = vmeta_session_recording_write(
		&conv_only, &capture_write_cb, &capture_conv_low_absent);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_FALSE(capture_conv_low_absent.found);

	/* Write side: with has_thermal set, every thermal key round-trips */
	struct vmeta_test_capture capture_metaversion = {
		.key = VMETA_REC_META_KEY_THERMAL_METAVERSION};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_metaversion);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_metaversion.found);
	CU_ASSERT_STRING_EQUAL(capture_metaversion.value, "2");

	struct vmeta_test_capture capture_camserial = {
		.key = VMETA_REC_META_KEY_THERMAL_CAMSERIAL};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_camserial);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_camserial.found);
	CU_ASSERT_STRING_EQUAL(capture_camserial.value, "THERM-0001");

	struct vmeta_test_capture capture_align = {
		.key = VMETA_REC_META_KEY_THERMAL_ALIGNMENT};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_align);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_align.found);
	CU_ASSERT_STRING_EQUAL(capture_align.value, expected_align);

	struct vmeta_test_capture capture_conv_low = {
		.key = VMETA_REC_META_KEY_THERMAL_CONV_LOW};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_conv_low);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_conv_low.found);
	CU_ASSERT_STRING_EQUAL(capture_conv_low.value, expected_conv_low);

	struct vmeta_test_capture capture_conv_high = {
		.key = VMETA_REC_META_KEY_THERMAL_CONV_HIGH};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_conv_high);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_conv_high.found);
	CU_ASSERT_STRING_EQUAL(capture_conv_high.value, expected_conv_high);

	struct vmeta_test_capture capture_scale = {
		.key = VMETA_REC_META_KEY_THERMAL_SCALE_FACTOR};
	ret = vmeta_session_recording_write(
		&meta, &capture_write_cb, &capture_scale);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(capture_scale.found);
	CU_ASSERT_STRING_EQUAL(capture_scale.value, expected_scale);
}

/* ---------------------------------------------------------------------- */
/* Helpers shared by the tests added below                                */
/* ---------------------------------------------------------------------- */

/* Build a fully populated, realistic session metadata structure.
 * If 'fisheye' is true, the camera model is a fisheye model, otherwise it
 * is a perspective model. Numeric values used for the various sub-string
 * formats are taken from the doc comments in vmeta_session.h whenever
 * possible, so that write->read round trips are exact. */
static void build_full_session_meta(struct vmeta_session *meta, int fisheye)
{
	memset(meta, 0, sizeof(*meta));

	snprintf(meta->friendly_name,
		 sizeof(meta->friendly_name),
		 "MyDrone-1234");
	snprintf(meta->maker, sizeof(meta->maker), "Parrot");
	snprintf(meta->model, sizeof(meta->model), "Anafi");
	snprintf(meta->model_id, sizeof(meta->model_id), "0918");
	snprintf(meta->serial_number,
		 sizeof(meta->serial_number),
		 "PI040420AA8A123456");
	snprintf(meta->software_version,
		 sizeof(meta->software_version),
		 "4.4.2");
	snprintf(meta->build_id, sizeof(meta->build_id), "build-1234");
	snprintf(meta->title, sizeof(meta->title), "My flight video");
	snprintf(meta->comment, sizeof(meta->comment), "A nice flight");
	snprintf(meta->copyright, sizeof(meta->copyright), "Copyright Parrot");

	meta->media_date = 1706367600;
	meta->media_date_gmtoff = 3600;
	meta->run_date = 1706367600;
	meta->run_date_gmtoff = 3600;
	snprintf(meta->run_id,
		 sizeof(meta->run_id),
		 "0123456789abcdef0123456789abcdef");
	meta->boot_date = 1706360000;
	meta->boot_date_gmtoff = 3600;
	snprintf(meta->boot_id,
		 sizeof(meta->boot_id),
		 "abcdef0123456789abcdef0123456789");
	meta->flight_date = 1706361000;
	meta->flight_date_gmtoff = 3600;
	snprintf(meta->flight_id,
		 sizeof(meta->flight_id),
		 "fedcba9876543210fedcba9876543210");
	snprintf(meta->custom_id, sizeof(meta->custom_id), "custom-id-42");

	meta->takeoff_loc.valid = 1;
	meta->takeoff_loc.latitude = 16.42850589;
	meta->takeoff_loc.longitude = -61.53569552;
	meta->takeoff_loc.altitude_wgs84ellipsoid = 10.0;
	meta->takeoff_loc.altitude_egm96amsl = 6.80;
	meta->takeoff_loc.sv_count = 12;

	meta->location.valid = 1;
	meta->location.latitude = 48.8566;
	meta->location.longitude = 2.3522;
	meta->location.altitude_wgs84ellipsoid = 105.0;
	meta->location.altitude_egm96amsl = 100.0;
	meta->location.sv_count = 10;

	meta->picture_fov.horz = 78.f;
	meta->picture_fov.vert = 49.f;
	meta->picture_fov.has_horz = 1;
	meta->picture_fov.has_vert = 1;

	meta->has_thermal = 1;
	meta->thermal.metaversion = 2;
	snprintf(meta->thermal.camserial,
		 sizeof(meta->thermal.camserial),
		 "THERM12345");
	meta->thermal.alignment.valid = 1;
	meta->thermal.alignment.rotation.yaw = -1.355f;
	meta->thermal.alignment.rotation.pitch = 0.609f;
	meta->thermal.alignment.rotation.roll = 89.730f;
	meta->thermal.conv_low.valid = 1;
	meta->thermal.conv_low.r = 1390082.947851f;
	meta->thermal.conv_low.b = 1449.5f;
	meta->thermal.conv_low.f = 1.0f;
	meta->thermal.conv_low.o = 1476.356f;
	meta->thermal.conv_low.tau_win = 0.8f;
	meta->thermal.conv_low.t_win = 25.0f;
	meta->thermal.conv_low.t_bg = 22.0f;
	meta->thermal.conv_low.emissivity = 0.98f;
	meta->thermal.conv_high = meta->thermal.conv_low;
	meta->thermal.scale_factor = 1.035156;

	meta->default_media = 1;

	meta->camera_type = VMETA_CAMERA_TYPE_FRONT;
	meta->camera_subtype = VMETA_CAMERA_SUBTYPE_WIDE;
	meta->camera_spectrum = VMETA_CAMERA_SPECTRUM_VISIBLE;
	snprintf(meta->camera_serial_number,
		 sizeof(meta->camera_serial_number),
		 "wide:PI123456789012345");

	if (fisheye) {
		meta->camera_model.type = VMETA_CAMERA_MODEL_TYPE_FISHEYE;
		meta->camera_model.fisheye.affine_matrix.c = 1.0f;
		meta->camera_model.fisheye.affine_matrix.d = 0.001f;
		meta->camera_model.fisheye.affine_matrix.e = -0.001f;
		meta->camera_model.fisheye.affine_matrix.f = 1.0f;
		meta->camera_model.fisheye.affine_matrix.symmetric = 1;
		meta->camera_model.fisheye.affine_matrix.symmetric_valid = 1;
		meta->camera_model.fisheye.polynomial.p2 = 0.0001f;
		meta->camera_model.fisheye.polynomial.p3 = -0.0002f;
		meta->camera_model.fisheye.polynomial.p4 = 0.0003f;
	} else {
		meta->camera_model.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
		meta->camera_model.perspective.distortion.r1 = 0.01f;
		meta->camera_model.perspective.distortion.r2 = -0.02f;
		meta->camera_model.perspective.distortion.r3 = 0.003f;
		meta->camera_model.perspective.distortion.t1 = 0.0004f;
		meta->camera_model.perspective.distortion.t2 = -0.0005f;
	}

	meta->overlay.type = VMETA_OVERLAY_TYPE_HEADER_FOOTER;
	meta->overlay.header_footer.header_height = 0.05f;
	meta->overlay.header_footer.footer_height = 0.08f;

	meta->principal_point.valid = 1;
	meta->principal_point.position.x = 0.491170f;
	meta->principal_point.position.y = 0.395359f;

	meta->video_mode = VMETA_VIDEO_MODE_STANDARD;
	meta->video_stop_reason = VMETA_VIDEO_STOP_REASON_USER;
	meta->photo_mode = VMETA_PHOTO_MODE_SINGLE;
	meta->panorama_type = VMETA_PANORAMA_TYPE_NONE;
	meta->photo_count = 3;
	snprintf(meta->secure_cn, sizeof(meta->secure_cn), "drone-cert-cn");
	meta->dynamic_range = VMETA_DYNAMIC_RANGE_HDR10;
	meta->tone_mapping = VMETA_TONE_MAPPING_P_LOG;
	meta->first_frame_capture_ts = 123456789012ULL;
	meta->first_frame_sample_index = 42;
	meta->media_id = 777;
	meta->resource_index = 2;
}


/* ---------------------------------------------------------------------- */
/* Group A: low-level sub-string write/read helpers                       */
/* ---------------------------------------------------------------------- */

static void test_session_date_read(void)
{
	int ret;
	char date[VMETA_SESSION_DATE_MAX_LEN];
	uint64_t timestamp = 1706367600; /* 27 Jan 2024 */
	int32_t gmtoff = 3600; /* UTC+1 */
	uint64_t rdate = 0;
	int32_t rgmtoff = 0;

	/* Invalid args */
	ret = vmeta_session_date_read(NULL, &rdate, &rgmtoff);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_date_read(
		"2024-01-27T12:00:00+01:00", NULL, &rgmtoff);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_date_read(
		"2024-01-27T12:00:00+01:00", &rdate, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip */
	memset(date, 0, sizeof(date));
	CU_ASSERT_TRUE(vmeta_session_date_write(
			       date, sizeof(date), timestamp, gmtoff) > 0);
	ret = vmeta_session_date_read(date, &rdate, &rgmtoff);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rdate, timestamp);
	CU_ASSERT_EQUAL(rgmtoff, gmtoff);

	/* Malformed date string */
	ret = vmeta_session_date_read("not a date", &rdate, &rgmtoff);
	CU_ASSERT_TRUE(ret < 0);
}


static void test_session_location_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_LOCATION_MAX_LEN];
	struct vmeta_location loc = {0};
	struct vmeta_location rloc = {0};
	char expected[VMETA_SESSION_LOCATION_MAX_LEN];

	/* Invalid args */
	wret = vmeta_session_location_write(
		NULL, sizeof(str), VMETA_SESSION_LOCATION_CSV, &loc);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	wret = vmeta_session_location_write(
		str, sizeof(str), VMETA_SESSION_LOCATION_CSV, NULL);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_location_read(NULL, &rloc);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_location_read("1,2,3", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid location: nothing written, returns 0 */
	loc.valid = 0;
	memset(str, 0xaa, sizeof(str));
	wret = vmeta_session_location_write(
		str, sizeof(str), VMETA_SESSION_LOCATION_CSV, &loc);
	CU_ASSERT_EQUAL(wret, 0);

	/* CSV round trip */
	loc.valid = 1;
	loc.latitude = 48.85661000;
	loc.longitude = 2.29450000;
	loc.altitude_egm96amsl = 95.5;
	wret = vmeta_session_location_write(
		str, sizeof(str), VMETA_SESSION_LOCATION_CSV, &loc);
	CU_ASSERT_TRUE(wret > 0);
	snprintf(expected,
		 sizeof(expected),
		 VMETA_SESSION_LOCATION_FORMAT_CSV,
		 loc.latitude,
		 loc.longitude,
		 loc.altitude_egm96amsl);
	CU_ASSERT_STRING_EQUAL(str, expected);
	ret = vmeta_session_location_read(str, &rloc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rloc.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(rloc.latitude, loc.latitude, 1e-6);
	CU_ASSERT_DOUBLE_EQUAL(rloc.longitude, loc.longitude, 1e-6);
	CU_ASSERT_DOUBLE_EQUAL(
		rloc.altitude_egm96amsl, loc.altitude_egm96amsl, 1e-3);

	/* ISO6709 round trip (values taken from the format's doc example) */
	memset(&loc, 0, sizeof(loc));
	loc.valid = 1;
	loc.latitude = 16.42850589;
	loc.longitude = -61.53569552;
	loc.altitude_egm96amsl = 6.80;
	wret = vmeta_session_location_write(
		str, sizeof(str), VMETA_SESSION_LOCATION_ISO6709, &loc);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "+16.42850589-061.53569552+6.80/");
	memset(&rloc, 0, sizeof(rloc));
	ret = vmeta_session_location_read(str, &rloc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rloc.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(rloc.latitude, loc.latitude, 1e-6);
	CU_ASSERT_DOUBLE_EQUAL(rloc.longitude, loc.longitude, 1e-6);
	CU_ASSERT_DOUBLE_EQUAL(
		rloc.altitude_egm96amsl, loc.altitude_egm96amsl, 1e-2);
	CU_ASSERT_TRUE(isnan(rloc.altitude_wgs84ellipsoid));

	/* XYZ round trip (values taken from the format's doc example) */
	memset(&loc, 0, sizeof(loc));
	loc.valid = 1;
	loc.latitude = 48.8566;
	loc.longitude = 2.3522;
	wret = vmeta_session_location_write(
		str, sizeof(str), VMETA_SESSION_LOCATION_XYZ, &loc);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "+48.8566+002.3522/");
	memset(&rloc, 0, sizeof(rloc));
	ret = vmeta_session_location_read(str, &rloc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rloc.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(rloc.latitude, loc.latitude, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(rloc.longitude, loc.longitude, 1e-4);

	/* Invalid/unknown format value falls back to CSV */
	memset(&loc, 0, sizeof(loc));
	loc.valid = 1;
	loc.latitude = 1.0;
	loc.longitude = 2.0;
	loc.altitude_egm96amsl = 3.0;
	wret = vmeta_session_location_write(
		str, sizeof(str), (enum vmeta_session_location_format)99, &loc);
	CU_ASSERT_TRUE(wret > 0);
	snprintf(expected,
		 sizeof(expected),
		 VMETA_SESSION_LOCATION_FORMAT_CSV,
		 loc.latitude,
		 loc.longitude,
		 loc.altitude_egm96amsl);
	CU_ASSERT_STRING_EQUAL(str, expected);

	/* Read with an invalid (non-numeric) CSV-like location string: the
	 * read still succeeds, but since sscanf() cannot match all 3 CSV
	 * fields, latitude/longitude are left at their internal 500.
	 * sentinel value, which vmeta_location_adjust_read() then detects
	 * to mark the location as invalid */
	memset(&rloc, 0, sizeof(rloc));
	ret = vmeta_session_location_read("abc,def,ghi", &rloc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rloc.valid, 0);
}


static void test_session_fov_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_FOV_MAX_LEN];
	struct vmeta_fov fov = {0};
	struct vmeta_fov rfov;

	/* Invalid args */
	wret = vmeta_session_fov_write(NULL, sizeof(str), &fov);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	wret = vmeta_session_fov_write(str, sizeof(str), NULL);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_fov_read(NULL, &rfov);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_fov_read("78.00,49.00", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Not fully specified: nothing written */
	fov.has_horz = 1;
	fov.has_vert = 0;
	memset(str, 0xaa, sizeof(str));
	wret = vmeta_session_fov_write(str, sizeof(str), &fov);
	CU_ASSERT_EQUAL(wret, 0);

	/* Round trip (values from the format's doc example) */
	fov.horz = 78.00f;
	fov.vert = 49.00f;
	fov.has_horz = 1;
	fov.has_vert = 1;
	wret = vmeta_session_fov_write(str, sizeof(str), &fov);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "78.00,49.00");

	memset(&rfov, 0, sizeof(rfov));
	ret = vmeta_session_fov_read(str, &rfov);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rfov.has_horz, 1);
	CU_ASSERT_EQUAL(rfov.has_vert, 1);
	CU_ASSERT_DOUBLE_EQUAL(rfov.horz, fov.horz, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(rfov.vert, fov.vert, 1e-4);

	/* Malformed string: has_horz/has_vert left at 0, no error */
	memset(&rfov, 0, sizeof(rfov));
	rfov.has_horz = 1;
	rfov.has_vert = 1;
	ret = vmeta_session_fov_read("garbage", &rfov);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rfov.has_horz, 0);
	CU_ASSERT_EQUAL(rfov.has_vert, 0);
}


static void test_session_perspective_distortion_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
	float r1 = 0.01f, r2 = -0.02f, r3 = 0.003f, t1 = 0.0004f, t2 = -0.0005f;
	float rr1, rr2, rr3, rt1, rt2;

	/* Invalid args */
	wret = vmeta_session_perspective_distortion_write(
		NULL, sizeof(str), r1, r2, r3, t1, t2);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_perspective_distortion_read(
		NULL, &rr1, &rr2, &rr3, &rt1, &rt2);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_perspective_distortion_read(
		"1,2,3,4,5", NULL, &rr2, &rr3, &rt1, &rt2);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip */
	wret = vmeta_session_perspective_distortion_write(
		str, sizeof(str), r1, r2, r3, t1, t2);
	CU_ASSERT_TRUE(wret > 0);
	ret = vmeta_session_perspective_distortion_read(
		str, &rr1, &rr2, &rr3, &rt1, &rt2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(rr1, r1, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rr2, r2, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rr3, r3, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rt1, t1, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rt2, t2, 1e-5);

	/* Malformed string */
	ret = vmeta_session_perspective_distortion_read(
		"garbage", &rr1, &rr2, &rr3, &rt1, &rt2);
	CU_ASSERT_EQUAL(ret, -EPROTO);
}


static void test_session_fisheye_affine_matrix_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
	float c = 1.0f, d = 0.001f, e = -0.001f, f = 1.0f;
	float rc, rd, re, rf;

	/* Invalid args */
	wret = vmeta_session_fisheye_affine_matrix_write(
		NULL, sizeof(str), c, d, e, f);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_fisheye_affine_matrix_read(
		NULL, &rc, &rd, &re, &rf);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_fisheye_affine_matrix_read(
		"1,2,3,4", &rc, &rd, &re, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip */
	wret = vmeta_session_fisheye_affine_matrix_write(
		str, sizeof(str), c, d, e, f);
	CU_ASSERT_TRUE(wret > 0);
	ret = vmeta_session_fisheye_affine_matrix_read(str, &rc, &rd, &re, &rf);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(rc, c, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rd, d, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(re, e, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rf, f, 1e-5);

	/* Malformed string */
	ret = vmeta_session_fisheye_affine_matrix_read(
		"garbage", &rc, &rd, &re, &rf);
	CU_ASSERT_EQUAL(ret, -EPROTO);
}


static void test_session_fisheye_polynomial_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
	float p2 = 0.0001f, p3 = -0.0002f, p4 = 0.0003f;
	float rp2, rp3, rp4;

	/* Invalid args */
	wret = vmeta_session_fisheye_polynomial_write(
		NULL, sizeof(str), p2, p3, p4);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_fisheye_polynomial_read(NULL, &rp2, &rp3, &rp4);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_fisheye_polynomial_read(
		"0,1,1,2,3", &rp2, NULL, &rp4);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip: note the wire format always starts with "0,1," */
	wret = vmeta_session_fisheye_polynomial_write(
		str, sizeof(str), p2, p3, p4);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_PTR_NOT_NULL(strstr(str, "0,1,"));
	ret = vmeta_session_fisheye_polynomial_read(str, &rp2, &rp3, &rp4);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(rp2, p2, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rp3, p3, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rp4, p4, 1e-5);

	/* Malformed string (missing the mandatory "0,1," prefix) */
	ret = vmeta_session_fisheye_polynomial_read("1,2,3", &rp2, &rp3, &rp4);
	CU_ASSERT_EQUAL(ret, -EPROTO);
}


static void test_session_overlay_header_footer_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_OVERLAY_HEADER_FOOTER_MAX_LEN];
	float header_height = 0.05f, footer_height = 0.08f;
	float rheader, rfooter;

	/* Invalid args */
	wret = vmeta_session_overlay_header_footer_write(
		NULL, sizeof(str), header_height, footer_height);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_overlay_header_footer_read(
		NULL, &rheader, &rfooter);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_overlay_header_footer_read(
		"0.05,0.08", NULL, &rfooter);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_overlay_header_footer_read(
		"0.05,0.08", &rheader, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip */
	wret = vmeta_session_overlay_header_footer_write(
		str, sizeof(str), header_height, footer_height);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "0.050,0.080");
	ret = vmeta_session_overlay_header_footer_read(str, &rheader, &rfooter);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(rheader, header_height, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(rfooter, footer_height, 1e-4);

	/* Malformed string */
	ret = vmeta_session_overlay_header_footer_read(
		"garbage", &rheader, &rfooter);
	CU_ASSERT_EQUAL(ret, -EPROTO);
}


static void test_session_thermal_alignment_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN];
	struct vmeta_thermal_alignment align = {0};
	struct vmeta_thermal_alignment ralign;

	/* Invalid args */
	wret = vmeta_session_thermal_alignment_write(NULL, sizeof(str), &align);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	wret = vmeta_session_thermal_alignment_write(str, sizeof(str), NULL);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_thermal_alignment_read(NULL, &ralign);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_thermal_alignment_read("-1.355,0.609,89.730", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip (values from the format's doc example) */
	align.rotation.yaw = -1.355f;
	align.rotation.pitch = 0.609f;
	align.rotation.roll = 89.730f;
	wret = vmeta_session_thermal_alignment_write(str, sizeof(str), &align);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "-1.355,0.609,89.730");

	memset(&ralign, 0xff, sizeof(ralign));
	ret = vmeta_session_thermal_alignment_read(str, &ralign);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(ralign.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(ralign.rotation.yaw, align.rotation.yaw, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(
		ralign.rotation.pitch, align.rotation.pitch, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(ralign.rotation.roll, align.rotation.roll, 1e-3);

	/* Malformed string: valid cleared, no error returned */
	ralign.valid = 1;
	ret = vmeta_session_thermal_alignment_read("garbage", &ralign);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(ralign.valid, 0);
}


static void test_session_thermal_conversion_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
	struct vmeta_thermal_conversion conv = {0};
	struct vmeta_thermal_conversion rconv;

	/* Invalid args */
	wret = vmeta_session_thermal_conversion_write(NULL, sizeof(str), &conv);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	wret = vmeta_session_thermal_conversion_write(str, sizeof(str), NULL);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_thermal_conversion_read(NULL, &rconv);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_thermal_conversion_read("1,2,3,4,5,6,7,8", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip (values from the format's doc example) */
	conv.r = 1390082.947851f;
	conv.b = 1449.5f;
	conv.f = 1.0f;
	conv.o = 1476.356f;
	conv.tau_win = 0.8f;
	conv.t_win = 25.0f;
	conv.t_bg = 22.0f;
	conv.emissivity = 0.98f;
	wret = vmeta_session_thermal_conversion_write(str, sizeof(str), &conv);
	CU_ASSERT_TRUE(wret > 0);
	/* Note: 'r' is stored as a float but formatted with 6 decimals, so
	 * only the field separators/general shape are checked exactly; the
	 * numeric values themselves are checked below after reading back */
	CU_ASSERT_EQUAL(wret, (ssize_t)strlen(str));

	memset(&rconv, 0xff, sizeof(rconv));
	ret = vmeta_session_thermal_conversion_read(str, &rconv);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rconv.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(rconv.r, conv.r, 1.0);
	CU_ASSERT_DOUBLE_EQUAL(rconv.b, conv.b, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(rconv.f, conv.f, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(rconv.o, conv.o, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(rconv.tau_win, conv.tau_win, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(rconv.t_win, conv.t_win, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(rconv.t_bg, conv.t_bg, 1e-3);
	CU_ASSERT_DOUBLE_EQUAL(rconv.emissivity, conv.emissivity, 1e-3);

	/* Malformed string: valid cleared, no error returned */
	rconv.valid = 1;
	ret = vmeta_session_thermal_conversion_read("garbage", &rconv);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rconv.valid, 0);
}


static void test_session_thermal_scale_factor_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN];
	double value = 1.035156;
	double rvalue;

	/* Invalid args */
	wret = vmeta_session_thermal_scale_factor_write(
		NULL, sizeof(str), value);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_thermal_scale_factor_read(NULL, &rvalue);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_thermal_scale_factor_read("1.0", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Round trip (value from the format's doc example) */
	wret = vmeta_session_thermal_scale_factor_write(
		str, sizeof(str), value);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "1.035156");
	ret = vmeta_session_thermal_scale_factor_read(str, &rvalue);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(rvalue, value, 1e-6);

	/* Malformed string: value reset to 0, no error returned */
	rvalue = 42.0;
	ret = vmeta_session_thermal_scale_factor_read("garbage", &rvalue);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(rvalue, 0.0, 1e-9);
}


static void test_session_principal_point_write_read(void)
{
	ssize_t wret;
	int ret;
	char str[VMETA_SESSION_PRINCIPAL_POINT_MAX_LEN];
	struct vmeta_principal_point pp = {0};
	struct vmeta_principal_point rpp;

	/* Invalid args */
	wret = vmeta_session_principal_point_write(NULL, sizeof(str), &pp);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	wret = vmeta_session_principal_point_write(str, sizeof(str), NULL);
	CU_ASSERT_EQUAL(wret, -EINVAL);
	ret = vmeta_session_principal_point_read(NULL, &rpp);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_principal_point_read("0.1,0.2", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid principal point: nothing written */
	pp.valid = 0;
	memset(str, 0xaa, sizeof(str));
	wret = vmeta_session_principal_point_write(str, sizeof(str), &pp);
	CU_ASSERT_EQUAL(wret, 0);

	/* Round trip (values from the format's doc example) */
	pp.valid = 1;
	pp.position.x = 0.491170f;
	pp.position.y = 0.395359f;
	wret = vmeta_session_principal_point_write(str, sizeof(str), &pp);
	CU_ASSERT_TRUE(wret > 0);
	CU_ASSERT_STRING_EQUAL(str, "0.491170,0.395359");

	memset(&rpp, 0xff, sizeof(rpp));
	ret = vmeta_session_principal_point_read(str, &rpp);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rpp.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(rpp.position.x, pp.position.x, 1e-5);
	CU_ASSERT_DOUBLE_EQUAL(rpp.position.y, pp.position.y, 1e-5);

	/* Malformed string: position reset to 0, valid cleared, no error */
	rpp.valid = 1;
	ret = vmeta_session_principal_point_read("garbage", &rpp);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(rpp.valid, 0);
	CU_ASSERT_DOUBLE_EQUAL(rpp.position.x, 0.0, 1e-9);
	CU_ASSERT_DOUBLE_EQUAL(rpp.position.y, 0.0, 1e-9);
}


/* ---------------------------------------------------------------------- */
/* Group B: RTCP SDES streaming write/read                                */
/* ---------------------------------------------------------------------- */

struct sdes_item {
	enum vmeta_stream_sdes_type type;
	char value[256];
	char prefix[64];
	int has_prefix;
};


struct sdes_capture {
	struct sdes_item items[80];
	int count;
};


static void sdes_write_cb(enum vmeta_stream_sdes_type type,
			  const char *value,
			  const char *prefix,
			  void *userdata)
{
	struct sdes_capture *cap = userdata;

	CU_ASSERT_TRUE_FATAL(cap->count <
			     (int)(sizeof(cap->items) / sizeof(cap->items[0])));
	cap->items[cap->count].type = type;
	snprintf(cap->items[cap->count].value,
		 sizeof(cap->items[cap->count].value),
		 "%s",
		 value != NULL ? value : "");
	cap->items[cap->count].has_prefix = (prefix != NULL);
	snprintf(cap->items[cap->count].prefix,
		 sizeof(cap->items[cap->count].prefix),
		 "%s",
		 prefix != NULL ? prefix : "");
	cap->count++;
}


static const struct sdes_item *find_sdes_item(struct sdes_capture *cap,
					      enum vmeta_stream_sdes_type type,
					      const char *prefix)
{
	int i;
	for (i = 0; i < cap->count; i++) {
		if (cap->items[i].type != type)
			continue;
		if (prefix == NULL) {
			if (!cap->items[i].has_prefix)
				return &cap->items[i];
		} else if (cap->items[i].has_prefix &&
			   strcmp(cap->items[i].prefix, prefix) == 0) {
			return &cap->items[i];
		}
	}
	return NULL;
}


static void test_session_streaming_sdes_write(void)
{
	int ret;
	struct vmeta_session meta;
	struct sdes_capture cap;
	const struct sdes_item *item;
	char expected[256];

	/* Invalid args */
	ret = vmeta_session_streaming_sdes_write(NULL, &sdes_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	build_full_session_meta(&meta, 0);
	ret = vmeta_session_streaming_sdes_write(&meta, NULL, &cap);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Perspective camera model variant */
	build_full_session_meta(&meta, 0);
	memset(&cap, 0, sizeof(cap));
	ret = vmeta_session_streaming_sdes_write(&meta, &sdes_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, 0);

	item = find_sdes_item(&cap, VMETA_STRM_SDES_TYPE_CNAME, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.serial_number);

	item = find_sdes_item(&cap, VMETA_STRM_SDES_TYPE_NAME, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.friendly_name);

	item = find_sdes_item(&cap, VMETA_STRM_SDES_TYPE_TOOL, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.software_version);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_TAKEOFF_LOC);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_location_write(expected,
				     sizeof(expected),
				     VMETA_SESSION_LOCATION_ISO6709,
				     &meta.takeoff_loc);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap, VMETA_STRM_SDES_TYPE_LOC, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_location_write(expected,
				     sizeof(expected),
				     VMETA_SESSION_LOCATION_ISO6709,
				     &meta.location);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(
		&cap, VMETA_STRM_SDES_TYPE_PRIV, VMETA_STRM_SDES_KEY_MAKER);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.maker);

	item = find_sdes_item(
		&cap, VMETA_STRM_SDES_TYPE_PRIV, VMETA_STRM_SDES_KEY_MODEL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.model);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_MEDIA_DATE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_date_write(expected,
				 sizeof(expected),
				 meta.media_date,
				 meta.media_date_gmtoff);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_PICTURE_FOV);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_fov_write(expected, sizeof(expected), &meta.picture_fov);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_CAMERA_TYPE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value,
			       vmeta_camera_type_to_str(meta.camera_type));

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_CAMERA_MODEL_TYPE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(
		item->value,
		vmeta_camera_model_type_to_str(meta.camera_model.type));

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_PERSPECTIVE_DISTORTION);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_perspective_distortion_write(
		expected,
		sizeof(expected),
		meta.camera_model.perspective.distortion.r1,
		meta.camera_model.perspective.distortion.r2,
		meta.camera_model.perspective.distortion.r3,
		meta.camera_model.perspective.distortion.t1,
		meta.camera_model.perspective.distortion.t2);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	/* Fisheye-specific key must not be present in the perspective case */
	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_FISHEYE_AFFINE_MATRIX);
	CU_ASSERT_PTR_NULL(item);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_THERMAL_CAMSERIAL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.thermal.camserial);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_THERMAL_ALIGNMENT);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_thermal_alignment_write(
		expected, sizeof(expected), &meta.thermal.alignment);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_THERMAL_SCALE_FACTOR);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_thermal_scale_factor_write(
		expected, sizeof(expected), meta.thermal.scale_factor);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_FIRST_FRAME_CAPTURE_TS);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	snprintf(expected,
		 sizeof(expected),
		 "%" PRIu64,
		 meta.first_frame_capture_ts);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(
		&cap, VMETA_STRM_SDES_TYPE_PRIV, VMETA_STRM_SDES_KEY_MEDIA_ID);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	snprintf(expected, sizeof(expected), "%" PRIu32, meta.media_id);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_PRINCIPAL_POINT);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_principal_point_write(
		expected, sizeof(expected), &meta.principal_point);
	CU_ASSERT_STRING_EQUAL(item->value, expected);
}


static void test_session_streaming_sdes_write_fisheye_and_skip(void)
{
	int ret;
	struct vmeta_session meta;
	struct sdes_capture cap;
	const struct sdes_item *item;
	char expected[256];

	/* Fisheye camera model variant */
	build_full_session_meta(&meta, 1);
	memset(&cap, 0, sizeof(cap));
	ret = vmeta_session_streaming_sdes_write(&meta, &sdes_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, 0);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_FISHEYE_AFFINE_MATRIX);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_fisheye_affine_matrix_write(
		expected,
		sizeof(expected),
		meta.camera_model.fisheye.affine_matrix.c,
		meta.camera_model.fisheye.affine_matrix.d,
		meta.camera_model.fisheye.affine_matrix.e,
		meta.camera_model.fisheye.affine_matrix.f);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_FISHEYE_POLYNOMIAL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	vmeta_session_fisheye_polynomial_write(
		expected,
		sizeof(expected),
		meta.camera_model.fisheye.polynomial.p2,
		meta.camera_model.fisheye.polynomial.p3,
		meta.camera_model.fisheye.polynomial.p4);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	/* Perspective-specific key must not be present in the fisheye case */
	item = find_sdes_item(&cap,
			      VMETA_STRM_SDES_TYPE_PRIV,
			      VMETA_STRM_SDES_KEY_PERSPECTIVE_DISTORTION);
	CU_ASSERT_PTR_NULL(item);

	/* Fields left at their "unknown"/empty default must not be emitted */
	memset(&meta, 0, sizeof(meta));
	memset(&cap, 0, sizeof(cap));
	ret = vmeta_session_streaming_sdes_write(&meta, &sdes_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(cap.count, 0);
}


static void test_session_streaming_sdes_read(void)
{
	int ret;
	struct vmeta_session meta = {0};

	/* Invalid args */
	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_CNAME, NULL, NULL, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV, "value", NULL, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_CNAME, "value", NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Basic (non-PRIV) types */
	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_CNAME, "SN12345", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.serial_number, "SN12345");

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_NAME, "MyDrone", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.friendly_name, "MyDrone");

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_TOOL, "1.2.3", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.software_version, "1.2.3");

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_LOC,
		"+16.42850589-061.53569552+6.80/",
		NULL,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.location.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(meta.location.latitude, 16.42850589, 1e-6);

	/* Unknown/out of range type: no-op, returns 0 */
	ret = vmeta_session_streaming_sdes_read(
		(enum vmeta_stream_sdes_type)99, "value", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);

	/* PRIV items */
	ret = vmeta_session_streaming_sdes_read(VMETA_STRM_SDES_TYPE_PRIV,
						"Parrot",
						VMETA_STRM_SDES_KEY_MAKER,
						&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.maker, "Parrot");

	ret = vmeta_session_streaming_sdes_read(VMETA_STRM_SDES_TYPE_PRIV,
						"Anafi",
						VMETA_STRM_SDES_KEY_MODEL,
						&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.model, "Anafi");

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV,
		"+16.42850589-061.53569552+6.80/",
		VMETA_STRM_SDES_KEY_TAKEOFF_LOC,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.takeoff_loc.valid, 1);

	ret = vmeta_session_streaming_sdes_read(VMETA_STRM_SDES_TYPE_PRIV,
						"78.00,49.00",
						VMETA_STRM_SDES_KEY_PICTURE_FOV,
						&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.picture_fov.has_horz, 1);
	CU_ASSERT_EQUAL(meta.picture_fov.has_vert, 1);

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV,
		vmeta_camera_type_to_str(VMETA_CAMERA_TYPE_FRONT),
		VMETA_STRM_SDES_KEY_CAMERA_TYPE,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.camera_type, VMETA_CAMERA_TYPE_FRONT);

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV,
		"-1.355,0.609,89.730",
		VMETA_STRM_SDES_KEY_THERMAL_ALIGNMENT,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.thermal.alignment.valid, 1);
	CU_ASSERT_EQUAL(meta.has_thermal, 1);

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV,
		"123456789012",
		VMETA_STRM_SDES_KEY_FIRST_FRAME_CAPTURE_TS,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.first_frame_capture_ts, (uint64_t)123456789012ULL);

	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV,
		"0.491170,0.395359",
		VMETA_STRM_SDES_KEY_PRINCIPAL_POINT,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.principal_point.valid, 1);

	/* Unknown PRIV prefix: silently ignored */
	ret = vmeta_session_streaming_sdes_read(
		VMETA_STRM_SDES_TYPE_PRIV, "value", "unknown_prefix", &meta);
	CU_ASSERT_EQUAL(ret, 0);
}


/* ---------------------------------------------------------------------- */
/* Group C: SDP streaming write/read                                      */
/* ---------------------------------------------------------------------- */

struct sdp_item {
	enum vmeta_stream_sdp_type type;
	char value[256];
	char key[64];
	int has_key;
};


struct sdp_capture {
	struct sdp_item items[80];
	int count;
};


static void sdp_write_cb(enum vmeta_stream_sdp_type type,
			 const char *value,
			 const char *key,
			 void *userdata)
{
	struct sdp_capture *cap = userdata;

	CU_ASSERT_TRUE_FATAL(cap->count <
			     (int)(sizeof(cap->items) / sizeof(cap->items[0])));
	cap->items[cap->count].type = type;
	snprintf(cap->items[cap->count].value,
		 sizeof(cap->items[cap->count].value),
		 "%s",
		 value != NULL ? value : "");
	cap->items[cap->count].has_key = (key != NULL);
	snprintf(cap->items[cap->count].key,
		 sizeof(cap->items[cap->count].key),
		 "%s",
		 key != NULL ? key : "");
	cap->count++;
}


static const struct sdp_item *find_sdp_item(struct sdp_capture *cap,
					    enum vmeta_stream_sdp_type type,
					    const char *key)
{
	int i;
	for (i = 0; i < cap->count; i++) {
		if (cap->items[i].type != type)
			continue;
		if (key == NULL) {
			if (!cap->items[i].has_key)
				return &cap->items[i];
		} else if (cap->items[i].has_key &&
			   strcmp(cap->items[i].key, key) == 0) {
			return &cap->items[i];
		}
	}
	return NULL;
}


static void test_session_streaming_sdp_write_session_level(void)
{
	int ret;
	struct vmeta_session meta;
	struct sdp_capture cap;
	const struct sdp_item *item;

	/* Invalid args */
	ret = vmeta_session_streaming_sdp_write(NULL, 0, &sdp_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	build_full_session_meta(&meta, 0);
	ret = vmeta_session_streaming_sdp_write(&meta, 0, NULL, &cap);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	memset(&cap, 0, sizeof(cap));
	ret = vmeta_session_streaming_sdp_write(&meta, 0, &sdp_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, 0);

	item = find_sdp_item(&cap, VMETA_STRM_SDP_TYPE_SESSION_INFO, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.friendly_name);

	/* At session level, the title comes as SESSION_NAME, not MEDIA_INFO */
	item = find_sdp_item(&cap, VMETA_STRM_SDP_TYPE_SESSION_NAME, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.title);
	item = find_sdp_item(&cap, VMETA_STRM_SDP_TYPE_MEDIA_INFO, NULL);
	CU_ASSERT_PTR_NULL(item);

	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_MAKER);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.maker);

	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_SERIAL_NUMBER);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.serial_number);

	/* Shared (session-or-media) items also come through with type
	 * SESSION_ATTR when media_level is 0 */
	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_PICTURE_FOV);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);

	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_CAMERA_TYPE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value,
			       vmeta_camera_type_to_str(meta.camera_type));

	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_DEFAULT_MEDIA);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, "1");

	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_PRINCIPAL_POINT);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
}


static void test_session_streaming_sdp_write_media_level(void)
{
	int ret;
	struct vmeta_session meta;
	struct sdp_capture cap;
	const struct sdp_item *item;

	build_full_session_meta(&meta, 1);
	memset(&cap, 0, sizeof(cap));
	ret = vmeta_session_streaming_sdp_write(&meta, 1, &sdp_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, 0);

	/* Session-only items must not be emitted at media level */
	item = find_sdp_item(&cap, VMETA_STRM_SDP_TYPE_SESSION_INFO, NULL);
	CU_ASSERT_PTR_NULL(item);
	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			     VMETA_STRM_SDP_KEY_MAKER);
	CU_ASSERT_PTR_NULL(item);

	/* At media level, the title comes as MEDIA_INFO */
	item = find_sdp_item(&cap, VMETA_STRM_SDP_TYPE_MEDIA_INFO, NULL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.title);

	/* Shared items now come through with type MEDIA_ATTR */
	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_MEDIA_ATTR,
			     VMETA_STRM_SDP_KEY_PICTURE_FOV);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);

	item = find_sdp_item(&cap,
			     VMETA_STRM_SDP_TYPE_MEDIA_ATTR,
			     VMETA_STRM_SDP_KEY_FISHEYE_AFFINE_MATRIX);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
}


static void test_session_streaming_sdp_read(void)
{
	int ret;
	struct vmeta_session meta = {0};

	/* Invalid args */
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_SESSION_INFO, NULL, NULL, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_SESSION_ATTR, "value", NULL, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_MEDIA_ATTR, "value", NULL, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_SESSION_INFO, "value", NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Basic (non-attr) types */
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_SESSION_INFO, "MyDrone", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.friendly_name, "MyDrone");

	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_SESSION_NAME, "My video", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.title, "My video");

	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_MEDIA_INFO, "My media title", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.title, "My media title");

	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_SESSION_TOOL, "1.2.3", NULL, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.software_version, "1.2.3");

	/* SESSION_ATTR: a session-only key */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_streaming_sdp_read(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
					       "Parrot",
					       VMETA_STRM_SDP_KEY_MAKER,
					       &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.maker, "Parrot");

	/* SESSION_ATTR falls through to the shared handling block, so a
	 * "media" key also works with type SESSION_ATTR */
	ret = vmeta_session_streaming_sdp_read(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
					       "78.00,49.00",
					       VMETA_STRM_SDP_KEY_PICTURE_FOV,
					       &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.picture_fov.has_horz, 1);

	/* MEDIA_ATTR: a shared key works directly with this type too */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_MEDIA_ATTR,
		vmeta_camera_type_to_str(VMETA_CAMERA_TYPE_FRONT),
		VMETA_STRM_SDP_KEY_CAMERA_TYPE,
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.camera_type, VMETA_CAMERA_TYPE_FRONT);

	ret = vmeta_session_streaming_sdp_read(VMETA_STRM_SDP_TYPE_MEDIA_ATTR,
					       "1",
					       VMETA_STRM_SDP_KEY_DEFAULT_MEDIA,
					       &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.default_media, 1);

	/* Unknown key: silently ignored */
	ret = vmeta_session_streaming_sdp_read(
		VMETA_STRM_SDP_TYPE_MEDIA_ATTR, "value", "unknown_key", &meta);
	CU_ASSERT_EQUAL(ret, 0);
}


/* ---------------------------------------------------------------------- */
/* Group D: MP4 'meta'/'udta' recording write/read                        */
/* ---------------------------------------------------------------------- */

struct rec_item {
	enum vmeta_record_type type;
	char key[128];
	char value[256];
};


struct rec_capture {
	struct rec_item items[80];
	int count;
};


static void rec_write_cb(enum vmeta_record_type type,
			 const char *key,
			 const char *value,
			 void *userdata)
{
	struct rec_capture *cap = userdata;

	CU_ASSERT_TRUE_FATAL(cap->count <
			     (int)(sizeof(cap->items) / sizeof(cap->items[0])));
	cap->items[cap->count].type = type;
	snprintf(cap->items[cap->count].key,
		 sizeof(cap->items[cap->count].key),
		 "%s",
		 key != NULL ? key : "");
	snprintf(cap->items[cap->count].value,
		 sizeof(cap->items[cap->count].value),
		 "%s",
		 value != NULL ? value : "");
	cap->count++;
}


static const struct rec_item *find_rec_item(struct rec_capture *cap,
					    enum vmeta_record_type type,
					    const char *key)
{
	int i;
	for (i = 0; i < cap->count; i++) {
		if ((cap->items[i].type == type) &&
		    (strcmp(cap->items[i].key, key) == 0))
			return &cap->items[i];
	}
	return NULL;
}


static void test_session_recording_write(void)
{
	int ret;
	struct vmeta_session meta;
	struct rec_capture cap;
	const struct rec_item *item;
	char expected[256];

	/* Invalid args */
	ret = vmeta_session_recording_write(NULL, &rec_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	build_full_session_meta(&meta, 0);
	ret = vmeta_session_recording_write(&meta, NULL, &cap);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	memset(&cap, 0, sizeof(cap));
	ret = vmeta_session_recording_write(&meta, &rec_write_cb, &cap);
	CU_ASSERT_EQUAL(ret, 0);

	/* Friendly name is written to both UDTA and META */
	item = find_rec_item(
		&cap, VMETA_REC_UDTA, VMETA_REC_UDTA_KEY_FRIENDLY_NAME);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.friendly_name);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_FRIENDLY_NAME);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.friendly_name);

	/* Media date is also written to both */
	vmeta_session_date_write(expected,
				 sizeof(expected),
				 meta.media_date,
				 meta.media_date_gmtoff);
	item = find_rec_item(
		&cap, VMETA_REC_UDTA, VMETA_REC_UDTA_KEY_MEDIA_DATE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, expected);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_MEDIA_DATE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	/* Takeoff location is only written to META, in ISO6709 format */
	vmeta_session_location_write(expected,
				     sizeof(expected),
				     VMETA_SESSION_LOCATION_ISO6709,
				     &meta.takeoff_loc);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_TAKEOFF_LOC);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	/* Location is written as XYZ to a REC_XYZ item, and as ISO6709 to a
	 * REC_META item */
	vmeta_session_location_write(expected,
				     sizeof(expected),
				     VMETA_SESSION_LOCATION_XYZ,
				     &meta.location);
	item = find_rec_item(&cap, VMETA_REC_XYZ, VMETA_REC_UDTA_KEY_LOCATION);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, expected);
	vmeta_session_location_write(expected,
				     sizeof(expected),
				     VMETA_SESSION_LOCATION_ISO6709,
				     &meta.location);
	item = find_rec_item(&cap, VMETA_REC_META, VMETA_REC_META_KEY_LOCATION);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	/* model_id and build_id are META-only */
	item = find_rec_item(&cap, VMETA_REC_META, VMETA_REC_META_KEY_MODEL_ID);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.model_id);

	item = find_rec_item(&cap, VMETA_REC_META, VMETA_REC_META_KEY_BUILD_ID);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.build_id);

	/* Camera model (perspective variant) */
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_CAMERA_MODEL_TYPE);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(
		item->value,
		vmeta_camera_model_type_to_str(meta.camera_model.type));
	item = find_rec_item(&cap,
			     VMETA_REC_META,
			     VMETA_REC_META_KEY_PERSPECTIVE_DISTORTION);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);

	/* Overlay header-footer */
	vmeta_session_overlay_header_footer_write(
		expected,
		sizeof(expected),
		meta.overlay.header_footer.header_height,
		meta.overlay.header_footer.footer_height);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_HEADER_FOOTER);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, expected);

	/* Thermal sub-items */
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_THERMAL_CAMSERIAL);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	CU_ASSERT_STRING_EQUAL(item->value, meta.thermal.camserial);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_THERMAL_CONV_LOW);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_THERMAL_CONV_HIGH);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_THERMAL_SCALE_FACTOR);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);

	/* Principal point */
	item = find_rec_item(
		&cap, VMETA_REC_META, VMETA_REC_META_KEY_PRINCIPAL_POINT);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);

	/* Resource / media identifiers */
	item = find_rec_item(&cap, VMETA_REC_META, VMETA_REC_META_KEY_MEDIA_ID);
	CU_ASSERT_PTR_NOT_NULL_FATAL(item);
	snprintf(expected, sizeof(expected), "%" PRIu32, meta.media_id);
	CU_ASSERT_STRING_EQUAL(item->value, expected);
}


static void test_session_recording_read_meta_keys(void)
{
	int ret;
	struct vmeta_session meta;

	/* Invalid args */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(NULL, "value", &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_TITLE, NULL, &meta);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_TITLE, "value", NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_FRIENDLY_NAME, "MyDrone", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.friendly_name, "MyDrone");

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_TITLE, "My video", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.title, "My video");

	ret = vmeta_session_recording_read(VMETA_REC_META_KEY_MEDIA_DATE,
					   "2024-01-27T13:00:00+01:00",
					   &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(meta.media_date != 0);

	ret = vmeta_session_recording_read(VMETA_REC_META_KEY_TAKEOFF_LOC,
					   "+16.42850589-061.53569552+6.80/",
					   &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.takeoff_loc.valid, 1);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_LOCATION, "+48.8566+002.3522/", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.location.valid, 1);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_CAMERA_TYPE,
		vmeta_camera_type_to_str(VMETA_CAMERA_TYPE_FRONT),
		&meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.camera_type, VMETA_CAMERA_TYPE_FRONT);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_HEADER_FOOTER, "0.05,0.08", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.overlay.type, VMETA_OVERLAY_TYPE_HEADER_FOOTER);

	ret = vmeta_session_recording_read(VMETA_REC_META_KEY_THERMAL_ALIGNMENT,
					   "-1.355,0.609,89.730",
					   &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.thermal.alignment.valid, 1);
	CU_ASSERT_EQUAL(meta.has_thermal, 1);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_PRINCIPAL_POINT, "0.491170,0.395359", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.principal_point.valid, 1);

	ret = vmeta_session_recording_read(
		VMETA_REC_META_KEY_MEDIA_ID, "777", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(meta.media_id, (uint32_t)777);

	/* Unknown key: silently ignored */
	ret = vmeta_session_recording_read("com.unknown.key", "value", &meta);
	CU_ASSERT_EQUAL(ret, 0);
}


static void test_session_recording_read_udta_keys(void)
{
	int ret;
	struct vmeta_session meta;

	/* Friendly name with the "Parrot <model>" convention infers maker
	 * and model when they are not already set */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_FRIENDLY_NAME, "Parrot Anafi", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.friendly_name, "Parrot Anafi");
	CU_ASSERT_STRING_EQUAL(meta.maker, "Parrot");
	CU_ASSERT_STRING_EQUAL(meta.model, "Anafi");

	/* A friendly name without the "Parrot " prefix does not infer
	 * maker/model */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_FRIENDLY_NAME, "MyCustomName", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.friendly_name, "MyCustomName");
	CU_ASSERT_STRING_EQUAL(meta.maker, "");
	CU_ASSERT_STRING_EQUAL(meta.model, "");

	/* Title that looks like a date is also parsed as the run date */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_TITLE, "2024-01-27T13:00:00+01:00", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.title, "2024-01-27T13:00:00+01:00");
	CU_ASSERT_TRUE(meta.run_date != 0);

	/* Title that is not a date leaves run_date untouched */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_TITLE, "Some custom title", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.title, "Some custom title");
	CU_ASSERT_EQUAL(meta.run_date, (uint64_t)0);

	/* A plain (non-JSON) comment is copied verbatim */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_COMMENT, "A regular comment", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.comment, "A regular comment");

	/* "Already set" guards: UDTA maker/model/software_version/serial
	 * are only copied if not already set */
	memset(&meta, 0, sizeof(meta));
	snprintf(meta.maker, sizeof(meta.maker), "AlreadySet");
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_MAKER, "Parrot", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.maker, "AlreadySet");

	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_MAKER, "Parrot", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.maker, "Parrot");
}


static void test_session_recording_read_udta_json_comment(void)
{
	int ret;
	struct vmeta_session meta;
	const char *json_comment =
		"{\"software_version\":\"9.9.9\","
		"\"run_uuid\":\"11112222333344445555666677778888\","
		"\"takeoff_position\":\"+16.42850589-061.53569552+6.80/\","
		"\"media_date\":\"2024-01-27T13:00:00+01:00\","
		"\"picture_hfov\":78.5,"
		"\"picture_vfov\":49.2}";

	/* A comment that looks like a JSON object is parsed */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_COMMENT, json_comment, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	/* The JSON comment itself must not end up copied as a plain
	 * comment */
	CU_ASSERT_STRING_EQUAL(meta.comment, "");
	CU_ASSERT_STRING_EQUAL(meta.software_version, "9.9.9");
	CU_ASSERT_STRING_EQUAL(meta.run_id, "11112222333344445555666677778888");
	CU_ASSERT_EQUAL(meta.takeoff_loc.valid, 1);
	CU_ASSERT_TRUE(meta.media_date != 0);
	CU_ASSERT_EQUAL(meta.picture_fov.has_horz, 1);
	CU_ASSERT_EQUAL(meta.picture_fov.has_vert, 1);
	CU_ASSERT_DOUBLE_EQUAL(meta.picture_fov.horz, 78.5, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(meta.picture_fov.vert, 49.2, 1e-4);

	/* Fields already set are not overwritten by the JSON comment */
	memset(&meta, 0, sizeof(meta));
	snprintf(meta.software_version, sizeof(meta.software_version), "1.0.0");
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_COMMENT, json_comment, &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.software_version, "1.0.0");

	/* A value that starts with '{' and ends with '}' (so it is treated as
	 * a JSON comment) but uses single quotes instead of the standard
	 * double quotes for its key: verified against the real build that
	 * this codebase's json-c still tokenizes it without error (i.e.
	 * json_tokener_parse() does not return NULL here), so
	 * vmeta_session_recording_json_comment_read() succeeds; since none of
	 * the known JSON comment keys match "not_json" though, no field in
	 * meta ends up populated */
	memset(&meta, 0, sizeof(meta));
	ret = vmeta_session_recording_read(
		VMETA_REC_UDTA_KEY_COMMENT, "{'not_json':true}", &meta);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(meta.software_version, "");
	CU_ASSERT_STRING_EQUAL(meta.run_id, "");
	CU_ASSERT_EQUAL(meta.takeoff_loc.valid, 0);
	CU_ASSERT_TRUE(meta.media_date == 0);
}


/* ---------------------------------------------------------------------- */
/* Group E: to_json / to_str                                              */
/* ---------------------------------------------------------------------- */

static double json_get_double(struct json_object *jobj, const char *key)
{
	struct json_object *jval = NULL;
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, key, &jval));
	if (jval == NULL)
		return 0.;
	return json_object_get_double(jval);
}


static const char *json_get_str(struct json_object *jobj, const char *key)
{
	struct json_object *jval = NULL;
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, key, &jval));
	if (jval == NULL)
		return NULL;
	return json_object_get_string(jval);
}


static void test_session_to_json_perspective(void)
{
	int ret;
	struct vmeta_session meta;
	struct json_object *jobj;
	struct json_object *jval = NULL;
	struct json_object *jcam;
	struct json_object *jdist;

	/* Invalid args */
	jobj = json_object_new_object();
	ret = vmeta_session_to_json(NULL, jobj);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	build_full_session_meta(&meta, 0);
	ret = vmeta_session_to_json(&meta, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_session_to_json(&meta, jobj);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_STRING_EQUAL(json_get_str(jobj, "friendly_name"),
			       meta.friendly_name);
	CU_ASSERT_STRING_EQUAL(json_get_str(jobj, "maker"), meta.maker);
	CU_ASSERT_STRING_EQUAL(json_get_str(jobj, "model"), meta.model);
	CU_ASSERT_STRING_EQUAL(json_get_str(jobj, "camera_type"),
			       vmeta_camera_type_to_str(meta.camera_type));
	CU_ASSERT_EQUAL((uint32_t)json_get_double(jobj, "photo_count"),
			meta.photo_count);
	CU_ASSERT_STRING_EQUAL(json_get_str(jobj, "secure_cn"), meta.secure_cn);
	CU_ASSERT_EQUAL((uint32_t)json_get_double(jobj, "media_id"),
			meta.media_id);

	/* default_media is deliberately omitted */
	CU_ASSERT_FALSE(
		json_object_object_get_ex(jobj, "default_media", &jval));

	/* Nested camera_model object (perspective variant) */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "camera_model", &jcam));
	CU_ASSERT_PTR_NOT_NULL_FATAL(jcam);
	CU_ASSERT_STRING_EQUAL(
		json_get_str(jcam, "type"),
		vmeta_camera_model_type_to_str(meta.camera_model.type));
	CU_ASSERT_TRUE(json_object_object_get_ex(
		jcam, "perspective_distortion", &jdist));
	CU_ASSERT_PTR_NOT_NULL_FATAL(jdist);
	CU_ASSERT_DOUBLE_EQUAL(json_get_double(jdist, "r1"),
			       meta.camera_model.perspective.distortion.r1,
			       1e-5);
	CU_ASSERT_DOUBLE_EQUAL(json_get_double(jdist, "t2"),
			       meta.camera_model.perspective.distortion.t2,
			       1e-5);

	/* Nested overlay object */
	{
		struct json_object *jovl;
		struct json_object *jhf;
		CU_ASSERT_TRUE(
			json_object_object_get_ex(jobj, "overlay", &jovl));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jovl);
		CU_ASSERT_STRING_EQUAL(
			json_get_str(jovl, "type"),
			vmeta_overlay_type_to_str(meta.overlay.type));
		CU_ASSERT_TRUE(
			json_object_object_get_ex(jovl, "header_footer", &jhf));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jhf);
		CU_ASSERT_DOUBLE_EQUAL(json_get_double(jhf, "header_height"),
				       meta.overlay.header_footer.header_height,
				       1e-4);
	}

	/* Nested thermal object */
	{
		struct json_object *jth;
		struct json_object *jalign;
		struct json_object *jrot;
		CU_ASSERT_TRUE(
			json_object_object_get_ex(jobj, "thermal", &jth));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jth);
		CU_ASSERT_STRING_EQUAL(json_get_str(jth, "camserial"),
				       meta.thermal.camserial);
		CU_ASSERT_TRUE(
			json_object_object_get_ex(jth, "alignment", &jalign));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jalign);
		CU_ASSERT_TRUE(
			json_object_object_get_ex(jalign, "rotation", &jrot));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jrot);
		CU_ASSERT_DOUBLE_EQUAL(json_get_double(jrot, "yaw"),
				       meta.thermal.alignment.rotation.yaw,
				       1e-3);
	}

	/* Principal point */
	{
		struct json_object *jpp;
		CU_ASSERT_TRUE(json_object_object_get_ex(
			jobj, "principal_point", &jpp));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jpp);
		CU_ASSERT_DOUBLE_EQUAL(json_get_double(jpp, "x"),
				       meta.principal_point.position.x,
				       1e-5);
	}

	json_object_put(jobj);
}


static void test_session_to_json_fisheye(void)
{
	int ret;
	struct vmeta_session meta;
	struct json_object *jobj;
	struct json_object *jcam;
	struct json_object *jfam;
	struct json_object *jfp;

	build_full_session_meta(&meta, 1);
	jobj = json_object_new_object();
	ret = vmeta_session_to_json(&meta, jobj);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "camera_model", &jcam));
	CU_ASSERT_PTR_NOT_NULL_FATAL(jcam);
	CU_ASSERT_TRUE(json_object_object_get_ex(
		jcam, "fisheye_affine_matrix", &jfam));
	CU_ASSERT_PTR_NOT_NULL_FATAL(jfam);
	CU_ASSERT_DOUBLE_EQUAL(json_get_double(jfam, "c"),
			       meta.camera_model.fisheye.affine_matrix.c,
			       1e-5);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jcam, "fisheye_polynomial", &jfp));
	CU_ASSERT_PTR_NOT_NULL_FATAL(jfp);
	CU_ASSERT_DOUBLE_EQUAL(json_get_double(jfp, "p0"), 0., 1e-9);
	CU_ASSERT_DOUBLE_EQUAL(json_get_double(jfp, "p1"), 1., 1e-9);
	CU_ASSERT_DOUBLE_EQUAL(json_get_double(jfp, "p2"),
			       meta.camera_model.fisheye.polynomial.p2,
			       1e-5);

	/* The "symmetric" flag is added at the top level of the session
	 * JSON object, not nested inside camera_model */
	{
		struct json_object *jsym = NULL;
		CU_ASSERT_TRUE(
			json_object_object_get_ex(jobj, "symmetric", &jsym));
		CU_ASSERT_PTR_NOT_NULL_FATAL(jsym);
		CU_ASSERT_EQUAL(json_object_get_boolean(jsym), 1);
	}

	json_object_put(jobj);
}


static void test_session_to_str_perspective(void)
{
	int ret;
	struct vmeta_session meta;
	char str[4096];
	char line[256];

	/* Invalid args */
	ret = vmeta_session_to_str(NULL, str, sizeof(str));
	CU_ASSERT_EQUAL(ret, -EINVAL);
	build_full_session_meta(&meta, 0);
	ret = vmeta_session_to_str(&meta, NULL, sizeof(str));
	CU_ASSERT_EQUAL(ret, -EINVAL);

	memset(str, 0, sizeof(str));
	ret = vmeta_session_to_str(&meta, str, sizeof(str));
	CU_ASSERT_EQUAL(ret, 0);

	snprintf(line, sizeof(line), "friendly_name: %s\n", meta.friendly_name);
	CU_ASSERT_PTR_NOT_NULL(strstr(str, line));

	snprintf(line, sizeof(line), "maker: %s\n", meta.maker);
	CU_ASSERT_PTR_NOT_NULL(strstr(str, line));

	snprintf(line,
		 sizeof(line),
		 "camera_type: %s\n",
		 vmeta_camera_type_to_str(meta.camera_type));
	CU_ASSERT_PTR_NOT_NULL(strstr(str, line));

	snprintf(line,
		 sizeof(line),
		 "camera_model_type: %s\n",
		 vmeta_camera_model_type_to_str(meta.camera_model.type));
	CU_ASSERT_PTR_NOT_NULL(strstr(str, line));

	CU_ASSERT_PTR_NOT_NULL(
		strstr(str, "camera_model_perspective_distortion: "));

	snprintf(line,
		 sizeof(line),
		 "overlay_type: %s\n",
		 vmeta_overlay_type_to_str(meta.overlay.type));
	CU_ASSERT_PTR_NOT_NULL(strstr(str, line));

	CU_ASSERT_PTR_NOT_NULL(strstr(str, "thermal_metaversion: 2\n"));
	CU_ASSERT_PTR_NOT_NULL(strstr(str, "thermal_conv_low: "));

	snprintf(line,
		 sizeof(line),
		 "principal_point: " VMETA_SESSION_PRINCIPAL_POINT_FORMAT "\n",
		 meta.principal_point.position.x,
		 meta.principal_point.position.y);
	CU_ASSERT_PTR_NOT_NULL(strstr(str, line));

	/* Fisheye-specific line must not be present */
	CU_ASSERT_PTR_NULL(strstr(str, "camera_model_fisheye_affine_matrix"));
}


static void test_session_to_str_fisheye(void)
{
	int ret;
	struct vmeta_session meta;
	char str[4096];

	build_full_session_meta(&meta, 1);
	memset(str, 0, sizeof(str));
	ret = vmeta_session_to_str(&meta, str, sizeof(str));
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_PTR_NOT_NULL(
		strstr(str, "camera_model_fisheye_affine_matrix: "));
	CU_ASSERT_PTR_NOT_NULL(strstr(
		str, "camera_model_fisheye_affine_matrix_symmetric: 1\n"));
	CU_ASSERT_PTR_NOT_NULL(
		strstr(str, "camera_model_fisheye_polynomial: "));

	/* Perspective-specific line must not be present */
	CU_ASSERT_PTR_NULL(strstr(str, "camera_model_perspective_distortion"));
}


/* ---------------------------------------------------------------------- */
/* Group F: vmeta_session_to_proto() camera_model / overlay / picture_fov  */
/* branches, and exhaustive (data-driven) vmeta<->proto enum conversions   */
/* ---------------------------------------------------------------------- */

/**
 * test_session_proto_api() deliberately forces camera_model.type ==
 * VMETA_CAMERA_MODEL_TYPE_UNKNOWN and overlay.type == VMETA_OVERLAY_TYPE_NONE,
 * so vmeta_session_to_proto()'s camera_model (PERSPECTIVE and FISHEYE) and
 * overlay (HEADER_FOOTER) branches -- and every lazy-init accessor they call
 * (vmeta_session_proto_get_{picture_fov,camera_model,
 * perspective_camera_model,perspective_camera_model_distorsion,
 * fisheye_camera_model,fisheye_camera_model_affine_matrix,
 * fisheye_camera_model_affine_matrix_symmetric,
 * fisheye_camera_model_polynomial,overlay,overlay_header_footer}) -- are
 * never exercised there. build_full_session_meta() (used extensively by the
 * SDES/SDP/MP4/to_json tests above) already sets picture_fov, camera_model
 * (PERSPECTIVE or FISHEYE depending on its 'fisheye' argument, with
 * symmetric_valid set in the fisheye case) and a HEADER_FOOTER overlay, and
 * compare_session_proto() (via compare_vmeta_proto_camera_model()/
 * compare_vmeta_proto_overlay()) already asserts deeply on every one of
 * these fields -- so reusing both here exercises every currently-uncovered
 * branch/accessor at once, via the same real round-trip idiom
 * test_session_proto_api() already established.
 */
static void test_session_proto_camera_model_and_overlay(void)
{
	int ret;
	struct vmeta_session meta = {0};
	struct vmeta_session_proto *meta_proto = NULL;
	const Vmeta__SessionMetadata *proto_meta = NULL;

	/* Perspective camera model */
	build_full_session_meta(&meta, 0 /* fisheye */);
	ret = vmeta_session_to_proto(&meta, &meta_proto);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(meta_proto);
	if (meta_proto != NULL) {
		ret = vmeta_session_proto_get_unpacked(meta_proto, &proto_meta);
		CU_ASSERT_EQUAL(ret, 0);
		compare_session_proto(proto_meta, &meta);
		ret = vmeta_session_proto_release_unpacked(meta_proto,
							   proto_meta);
		CU_ASSERT_EQUAL(ret, 0);
		ret = vmeta_session_proto_destroy(meta_proto);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Fisheye camera model (with symmetric_valid set) */
	meta_proto = NULL;
	proto_meta = NULL;
	build_full_session_meta(&meta, 1 /* fisheye */);
	ret = vmeta_session_to_proto(&meta, &meta_proto);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL(meta_proto);
	if (meta_proto != NULL) {
		ret = vmeta_session_proto_get_unpacked(meta_proto, &proto_meta);
		CU_ASSERT_EQUAL(ret, 0);
		compare_session_proto(proto_meta, &meta);
		ret = vmeta_session_proto_release_unpacked(meta_proto,
							   proto_meta);
		CU_ASSERT_EQUAL(ret, 0);
		ret = vmeta_session_proto_destroy(meta_proto);
		CU_ASSERT_EQUAL(ret, 0);
	}
}


static void test_session_proto_camera_type_conversion(void)
{
	static const struct {
		enum vmeta_camera_type vmeta;
		Vmeta__CameraType proto;
	} cases[] = {
		{VMETA_CAMERA_TYPE_FRONT, VMETA__CAMERA_TYPE__CT_FRONT},
		{VMETA_CAMERA_TYPE_FRONT_STEREO,
		 VMETA__CAMERA_TYPE__CT_FRONT_STEREO},
		{VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT,
		 VMETA__CAMERA_TYPE__CT_FRONT_STEREO_LEFT},
		{VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT,
		 VMETA__CAMERA_TYPE__CT_FRONT_STEREO_RIGHT},
		{VMETA_CAMERA_TYPE_VERTICAL, VMETA__CAMERA_TYPE__CT_VERTICAL},
		{VMETA_CAMERA_TYPE_DISPARITY, VMETA__CAMERA_TYPE__CT_DISPARITY},
		{VMETA_CAMERA_TYPE_HORIZONTAL_STEREO,
		 VMETA__CAMERA_TYPE__CT_HORIZONTAL_STEREO},
		{VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT,
		 VMETA__CAMERA_TYPE__CT_HORIZONTAL_STEREO_LEFT},
		{VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT,
		 VMETA__CAMERA_TYPE__CT_HORIZONTAL_STEREO_RIGHT},
		{VMETA_CAMERA_TYPE_DOWN_STEREO,
		 VMETA__CAMERA_TYPE__CT_DOWN_STEREO},
		{VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT,
		 VMETA__CAMERA_TYPE__CT_DOWN_STEREO_LEFT},
		{VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT,
		 VMETA__CAMERA_TYPE__CT_DOWN_STEREO_RIGHT},
		{VMETA_CAMERA_TYPE_EXTERNAL, VMETA__CAMERA_TYPE__CT_EXTERNAL},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_EQUAL(vmeta_session_camera_type_vmeta_to_proto(
					cases[i].vmeta),
				cases[i].proto);
	}

	CU_ASSERT_EQUAL(vmeta_session_camera_type_vmeta_to_proto(
				VMETA_CAMERA_TYPE_UNKNOWN),
			VMETA__CAMERA_TYPE__CT_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_camera_type_vmeta_to_proto(
				(enum vmeta_camera_type)999),
			VMETA__CAMERA_TYPE__CT_UNKNOWN);
}


static void test_session_proto_video_mode_conversion(void)
{
	static const struct {
		enum vmeta_video_mode vmeta;
		Vmeta__VideoMode proto;
	} cases[] = {
		{VMETA_VIDEO_MODE_STANDARD, VMETA__VIDEO_MODE__VM_STANDARD},
		{VMETA_VIDEO_MODE_HYPERLAPSE, VMETA__VIDEO_MODE__VM_HYPERLAPSE},
		{VMETA_VIDEO_MODE_SLOWMOTION, VMETA__VIDEO_MODE__VM_SLOWMOTION},
		{VMETA_VIDEO_MODE_STREAMREC, VMETA__VIDEO_MODE__VM_STREAMREC},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_EQUAL(
			vmeta_session_video_mode_vmeta_to_proto(cases[i].vmeta),
			cases[i].proto);
	}

	CU_ASSERT_EQUAL(vmeta_session_video_mode_vmeta_to_proto(
				VMETA_VIDEO_MODE_UNKNOWN),
			VMETA__VIDEO_MODE__VM_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_video_mode_vmeta_to_proto(
				(enum vmeta_video_mode)999),
			VMETA__VIDEO_MODE__VM_UNKNOWN);
}


static void test_session_proto_video_stop_reason_conversion(void)
{
	static const struct {
		enum vmeta_video_stop_reason vmeta;
		Vmeta__VideoStopReason proto;
	} cases[] = {
		{VMETA_VIDEO_STOP_REASON_USER,
		 VMETA__VIDEO_STOP_REASON__VSR_USER},
		{VMETA_VIDEO_STOP_REASON_RECONFIGURATION,
		 VMETA__VIDEO_STOP_REASON__VSR_RECONFIGURATION},
		{VMETA_VIDEO_STOP_REASON_POOR_STORAGE_PERF,
		 VMETA__VIDEO_STOP_REASON__VSR_POOR_STORAGE_PERF},
		{VMETA_VIDEO_STOP_REASON_STORAGE_FULL,
		 VMETA__VIDEO_STOP_REASON__VSR_STORAGE_FULL},
		{VMETA_VIDEO_STOP_REASON_RECOVERY,
		 VMETA__VIDEO_STOP_REASON__VSR_RECOVERY},
		{VMETA_VIDEO_STOP_REASON_END_OF_STREAM,
		 VMETA__VIDEO_STOP_REASON__VSR_END_OF_STREAM},
		{VMETA_VIDEO_STOP_REASON_SHUTDOWN,
		 VMETA__VIDEO_STOP_REASON__VSR_SHUTDOWN},
		{VMETA_VIDEO_STOP_REASON_INTERNAL_ERROR,
		 VMETA__VIDEO_STOP_REASON__VSR_INTERNAL_ERROR},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_EQUAL(vmeta_session_video_stop_reason_vmeta_to_proto(
					cases[i].vmeta),
				cases[i].proto);
	}

	CU_ASSERT_EQUAL(vmeta_session_video_stop_reason_vmeta_to_proto(
				VMETA_VIDEO_STOP_REASON_UNKNOWN),
			VMETA__VIDEO_STOP_REASON__VSR_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_video_stop_reason_vmeta_to_proto(
				(enum vmeta_video_stop_reason)999),
			VMETA__VIDEO_STOP_REASON__VSR_UNKNOWN);
}


static void test_session_proto_dynamic_range_conversion(void)
{
	static const struct {
		enum vmeta_dynamic_range vmeta;
		Vmeta__DynamicRange proto;
	} cases[] = {
		{VMETA_DYNAMIC_RANGE_SDR, VMETA__DYNAMIC_RANGE__DR_SDR},
		{VMETA_DYNAMIC_RANGE_HDR8, VMETA__DYNAMIC_RANGE__DR_HDR8},
		{VMETA_DYNAMIC_RANGE_HDR10, VMETA__DYNAMIC_RANGE__DR_HDR10},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_EQUAL(vmeta_session_dynamic_range_vmeta_to_proto(
					cases[i].vmeta),
				cases[i].proto);
	}

	CU_ASSERT_EQUAL(vmeta_session_dynamic_range_vmeta_to_proto(
				VMETA_DYNAMIC_RANGE_UNKNOWN),
			VMETA__DYNAMIC_RANGE__DR_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_dynamic_range_vmeta_to_proto(
				(enum vmeta_dynamic_range)999),
			VMETA__DYNAMIC_RANGE__DR_UNKNOWN);
}


static void test_session_proto_photo_mode_conversion(void)
{
	static const struct {
		enum vmeta_photo_mode vmeta;
		Vmeta__PhotoMode proto;
	} cases[] = {
		{VMETA_PHOTO_MODE_SINGLE, VMETA__PHOTO_MODE__PM_SINGLE},
		{VMETA_PHOTO_MODE_BRACKETING, VMETA__PHOTO_MODE__PM_BRACKETING},
		{VMETA_PHOTO_MODE_BURST, VMETA__PHOTO_MODE__PM_BURST},
		{VMETA_PHOTO_MODE_TIMELAPSE, VMETA__PHOTO_MODE__PM_TIMELAPSE},
		{VMETA_PHOTO_MODE_GPSLAPSE, VMETA__PHOTO_MODE__PM_GPSLAPSE},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		/* vmeta -> proto */
		CU_ASSERT_EQUAL(
			vmeta_session_photo_mode_vmeta_to_proto(cases[i].vmeta),
			cases[i].proto);
		/* proto -> vmeta (round trip) */
		CU_ASSERT_EQUAL(
			vmeta_session_photo_mode_proto_to_vmeta(cases[i].proto),
			cases[i].vmeta);
	}

	CU_ASSERT_EQUAL(vmeta_session_photo_mode_vmeta_to_proto(
				VMETA_PHOTO_MODE_UNKNOWN),
			VMETA__PHOTO_MODE__PHOTO_MODE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_photo_mode_vmeta_to_proto(
				(enum vmeta_photo_mode)999),
			VMETA__PHOTO_MODE__PHOTO_MODE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_photo_mode_proto_to_vmeta(
				VMETA__PHOTO_MODE__PHOTO_MODE_UNKNOWN),
			VMETA_PHOTO_MODE_UNKNOWN);
	CU_ASSERT_EQUAL(
		vmeta_session_photo_mode_proto_to_vmeta((Vmeta__PhotoMode)999),
		VMETA_PHOTO_MODE_UNKNOWN);
}


static void test_session_proto_panorama_type_conversion(void)
{
	static const struct {
		enum vmeta_panorama_type vmeta;
		Vmeta__PanoramaType proto;
	} cases[] = {
		{VMETA_PANORAMA_TYPE_NONE, VMETA__PANORAMA_TYPE__PT_NONE},
		{VMETA_PANORAMA_TYPE_HORIZONTAL_180,
		 VMETA__PANORAMA_TYPE__PT_HORIZONTAL_180},
		{VMETA_PANORAMA_TYPE_VERTICAL_180,
		 VMETA__PANORAMA_TYPE__PT_VERTICAL_180},
		{VMETA_PANORAMA_TYPE_SPHERICAL,
		 VMETA__PANORAMA_TYPE__PT_SPHERICAL},
		{VMETA_PANORAMA_TYPE_SUPER_WIDE,
		 VMETA__PANORAMA_TYPE__PT_SUPER_WIDE},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_EQUAL(vmeta_session_panorama_type_vmeta_to_proto(
					cases[i].vmeta),
				cases[i].proto);
		CU_ASSERT_EQUAL(vmeta_session_panorama_type_proto_to_vmeta(
					cases[i].proto),
				cases[i].vmeta);
	}

	CU_ASSERT_EQUAL(vmeta_session_panorama_type_vmeta_to_proto(
				VMETA_PANORAMA_TYPE_UNKNOWN),
			VMETA__PANORAMA_TYPE__PT_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_panorama_type_vmeta_to_proto(
				(enum vmeta_panorama_type)999),
			VMETA__PANORAMA_TYPE__PT_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_panorama_type_proto_to_vmeta(
				VMETA__PANORAMA_TYPE__PT_UNKNOWN),
			VMETA_PANORAMA_TYPE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_panorama_type_proto_to_vmeta(
				(Vmeta__PanoramaType)999),
			VMETA_PANORAMA_TYPE_UNKNOWN);
}


static void test_session_proto_tone_mapping_conversion(void)
{
	static const struct {
		enum vmeta_tone_mapping vmeta;
		Vmeta__ToneMapping proto;
	} cases[] = {
		{VMETA_TONE_MAPPING_STANDARD, VMETA__TONE_MAPPING__TM_STANDARD},
		{VMETA_TONE_MAPPING_P_LOG, VMETA__TONE_MAPPING__TM_P_LOG},
	};
	size_t i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CU_ASSERT_EQUAL(vmeta_session_tone_mapping_vmeta_to_proto(
					cases[i].vmeta),
				cases[i].proto);
	}

	CU_ASSERT_EQUAL(vmeta_session_tone_mapping_vmeta_to_proto(
				VMETA_TONE_MAPPING_UNKNOWN),
			VMETA__TONE_MAPPING__TM_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_session_tone_mapping_vmeta_to_proto(
				(enum vmeta_tone_mapping)999),
			VMETA__TONE_MAPPING__TM_UNKNOWN);
}


CU_TestInfo s_session_tests[] = {
	{(char *)"session_size", &test_session_size},
	{(char *)"session_cmp", &test_session_cmp},
	{(char *)"session_merge_metadata", &test_session_merge_metadata},
	{(char *)"session_is_valid", &test_session_is_valid},
	{(char *)"session_proto_api", &test_session_proto_api},
	{(char *)"session_to_json", &test_session_to_json},
	{(char *)"session_date_write", &test_session_date_write},
	{(char *)"session_recording_read", &test_session_recording_read},
	{(char *)"session_recording_write_table_driven",
	 &test_session_recording_write_table_driven},
	{(char *)"session_recording_date", &test_session_recording_date},
	{(char *)"session_recording_location",
	 &test_session_recording_location},
	{(char *)"session_recording_fov", &test_session_recording_fov},
	{(char *)"session_recording_camera_model",
	 &test_session_recording_camera_model},
	{(char *)"session_recording_overlay", &test_session_recording_overlay},
	{(char *)"session_recording_principal_point",
	 &test_session_recording_principal_point},
	{(char *)"session_recording_thermal", &test_session_recording_thermal},
	{(char *)"session_date_read", &test_session_date_read},
	{(char *)"session_location_write_read",
	 &test_session_location_write_read},
	{(char *)"session_fov_write_read", &test_session_fov_write_read},
	{(char *)"session_perspective_distortion_write_read",
	 &test_session_perspective_distortion_write_read},
	{(char *)"session_fisheye_affine_matrix_write_read",
	 &test_session_fisheye_affine_matrix_write_read},
	{(char *)"session_fisheye_polynomial_write_read",
	 &test_session_fisheye_polynomial_write_read},
	{(char *)"session_overlay_header_footer_write_read",
	 &test_session_overlay_header_footer_write_read},
	{(char *)"session_thermal_alignment_write_read",
	 &test_session_thermal_alignment_write_read},
	{(char *)"session_thermal_conversion_write_read",
	 &test_session_thermal_conversion_write_read},
	{(char *)"session_thermal_scale_factor_write_read",
	 &test_session_thermal_scale_factor_write_read},
	{(char *)"session_principal_point_write_read",
	 &test_session_principal_point_write_read},
	{(char *)"session_streaming_sdes_write",
	 &test_session_streaming_sdes_write},
	{(char *)"session_streaming_sdes_write_fisheye_and_skip",
	 &test_session_streaming_sdes_write_fisheye_and_skip},
	{(char *)"session_streaming_sdes_read",
	 &test_session_streaming_sdes_read},
	{(char *)"session_streaming_sdp_write_session_level",
	 &test_session_streaming_sdp_write_session_level},
	{(char *)"session_streaming_sdp_write_media_level",
	 &test_session_streaming_sdp_write_media_level},
	{(char *)"session_streaming_sdp_read",
	 &test_session_streaming_sdp_read},
	{(char *)"session_recording_write", &test_session_recording_write},
	{(char *)"session_recording_read_meta_keys",
	 &test_session_recording_read_meta_keys},
	{(char *)"session_recording_read_udta_keys",
	 &test_session_recording_read_udta_keys},
	{(char *)"session_recording_read_udta_json_comment",
	 &test_session_recording_read_udta_json_comment},
	{(char *)"session_to_json_perspective",
	 &test_session_to_json_perspective},
	{(char *)"session_to_json_fisheye", &test_session_to_json_fisheye},
	{(char *)"session_to_str_perspective",
	 &test_session_to_str_perspective},
	{(char *)"session_to_str_fisheye", &test_session_to_str_fisheye},
	{(char *)"session_proto_camera_model_and_overlay",
	 &test_session_proto_camera_model_and_overlay},
	{(char *)"session_proto_camera_type_conversion",
	 &test_session_proto_camera_type_conversion},
	{(char *)"session_proto_video_mode_conversion",
	 &test_session_proto_video_mode_conversion},
	{(char *)"session_proto_video_stop_reason_conversion",
	 &test_session_proto_video_stop_reason_conversion},
	{(char *)"session_proto_dynamic_range_conversion",
	 &test_session_proto_dynamic_range_conversion},
	{(char *)"session_proto_photo_mode_conversion",
	 &test_session_proto_photo_mode_conversion},
	{(char *)"session_proto_panorama_type_conversion",
	 &test_session_proto_panorama_type_conversion},
	{(char *)"session_proto_tone_mapping_conversion",
	 &test_session_proto_tone_mapping_conversion},
	CU_TEST_INFO_NULL,
};
