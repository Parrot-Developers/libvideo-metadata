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
	long media_date_gmtoff;
	uint64_t run_date;
	long run_date_gmtoff;
	char run_id[33];
	uint64_t boot_date;
	long boot_date_gmtoff;
	char boot_id[33];
	uint64_t flight_date;
	long flight_date_gmtoff;
	char flight_id[33];
	char custom_id[80];
	struct vmeta_location takeoff_loc;
	struct vmeta_location location;
	struct vmeta_fov picture_fov;
	struct vmeta_thermal thermal;
	uint32_t has_thermal : 1;
	uint32_t default_media : 1;
	enum vmeta_camera_type camera_type;
	enum vmeta_camera_subtype camera_subtype;
	enum vmeta_camera_spectrum camera_spectrum;
	char camera_serial_number[VMETA_SESSION_CAMERA_SERIAL_PATTERN_MAX_LEN];
	struct vmeta_camera_model camera_model;
	struct vmeta_overlay overlay;
	struct vmeta_principal_point principal_point;
	enum vmeta_video_mode video_mode;
	enum vmeta_video_stop_reason video_stop_reason;
	enum vmeta_dynamic_range dynamic_range;
	enum vmeta_tone_mapping tone_mapping;
	uint64_t first_frame_capture_ts;
	uint64_t first_frame_sample_index;
	uint32_t media_id;
	uint32_t resource_index;
} s_vmeta_session_test_size;


static void fill_vmeta_with(struct vmeta_session *meta, int val)
{
	memset(meta, val, sizeof(*meta));
	meta->friendly_name[39] = '\0';
	meta->maker[39] = '\0';
	meta->model[39] = '\0';
	meta->model_id[4] = '\0';
	meta->serial_number[31] = '\0';
	meta->software_version[19] = '\0';
	meta->build_id[79] = '\0';
	meta->title[79] = '\0';
	meta->comment[99] = '\0';
	meta->copyright[79] = '\0';
	meta->run_id[32] = '\0';
	meta->boot_id[32] = '\0';
	meta->flight_id[32] = '\0';
	meta->custom_id[79] = '\0';
	meta->camera_serial_number[sizeof(meta->camera_serial_number) - 1] =
		'\0';
}


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
	CU_ASSERT_EQUAL(proto->media_date_gmtoff,
			(int32_t)meta->media_date_gmtoff);
	CU_ASSERT_EQUAL(proto->boot_date, meta->boot_date);
	CU_ASSERT_EQUAL(proto->boot_date_gmtoff,
			(int32_t)meta->boot_date_gmtoff);
	CU_ASSERT_STRING_EQUAL(proto->boot_id, meta->boot_id);
	CU_ASSERT_EQUAL(proto->flight_date, meta->flight_date);
	CU_ASSERT_EQUAL(proto->flight_date_gmtoff,
			(int32_t)meta->flight_date_gmtoff);
	CU_ASSERT_STRING_EQUAL(proto->flight_id, meta->flight_id);
	CU_ASSERT_STRING_EQUAL(proto->custom_id, meta->custom_id);
	/* takeoff loc */
	compare_vmeta_proto_location(
		&meta->takeoff_loc, proto->takeoff_location, true);
	/* picture_fov */
	if (meta->picture_fov.has_horz && meta->picture_fov.has_vert) {
		struct vmeta_xy picture_fov = {
			.x = meta->picture_fov.horz,
			.y = meta->picture_fov.vert,
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

	ret = vmeta_session_proto_get_packed_size(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

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

	ret = vmeta_session_proto_get_packed_size(meta_proto);
	CU_ASSERT(ret > 0);
	packed_len = ret;

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


CU_TestInfo s_session_tests[] = {
	{(char *)"session_size", &test_session_size},
	{(char *)"session_cmp", &test_session_cmp},
	{(char *)"session_merge_metadata", &test_session_merge_metadata},
	{(char *)"session_is_valid", &test_session_is_valid},
	{(char *)"session_proto_api", &test_session_proto_api},
	CU_TEST_INFO_NULL,
};
