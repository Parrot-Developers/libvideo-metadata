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

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include <json-c/json.h>


/* "Parrot Video Metadata" v2 wire sizes (bytes), hand derived from
 * src/vmeta_frame_v2.c / src/vmeta_frame.c's field-by-field write() calls:
 * base = 4 (id+len header) + 52 (fields) = 56
 * timestamp extension = 4 (id+len header) + 8 (u64) = 12
 * followme extension = 4 (id+len header) + 12 (lat/lon/alt) + 2 (mode/anim)
 *                       + 2 (reserved1/2) + 8 (reserved3/4) = 28
 * base + both extensions = 56 + 12 + 28 = 96 = VMETA_FRAME_V2_MAX_SIZE */
#define V2_SIZE_BASE_ONLY 56
#define V2_SIZE_WITH_TIMESTAMP 68
#define V2_SIZE_WITH_FOLLOWME 84
#define V2_SIZE_WITH_BOTH 96


static void fill_base(struct vmeta_frame_v2_base *base)
{
	memset(base, 0, sizeof(*base));

	/* All fractional values below are exact sums of negative powers of
	 * two so that the fixed-point wire encoding (shift <= 22 everywhere
	 * in this format) never truncates them: round-trip comparisons can
	 * therefore be made bit-exact rather than only "within granularity",
	 * and the same fixture is reused verbatim to build an exact-string
	 * oracle for the to_csv() tests. */
	base->drone_quat.w = 0.5f;
	base->drone_quat.x = 0.25f;
	base->drone_quat.y = -0.25f;
	base->drone_quat.z = 0.125f;

	base->location.latitude = 1.5;
	base->location.longitude = -2.5;
	/* Not part of the v2 wire format at all (see vmeta_frame_v2_read()):
	 * always comes back as NAN after a round-trip. */
	base->location.altitude_wgs84ellipsoid = NAN;
	base->location.altitude_egm96amsl = 4.25;
	base->location.horizontal_accuracy = 0.5f;
	base->location.vertical_accuracy = 0.25f;
	base->location.sv_count = 7;
	base->location.valid = 1;

	base->ground_distance = 6.5;

	base->speed.north = 1.5f;
	base->speed.east = -1.5f;
	base->speed.down = 0.5f;

	base->air_speed = 2.5f;

	base->frame_quat.w = 0.5f;
	base->frame_quat.x = -0.5f;
	base->frame_quat.y = 0.5f;
	base->frame_quat.z = -0.5f;

	base->camera_pan = 1.25f;
	base->camera_tilt = -1.25f;
	base->exposure_time = 0.5f;
	base->gain = 100;

	base->wifi_rssi = -50;
	base->battery_percentage = 80;

	/* Deliberately non-default bitfields/enums */
	base->binning = 1;
	base->animation = 1;
	base->state = VMETA_FLYING_STATE_EMERGENCY_LANDING;
	base->mode = VMETA_PILOTING_MODE_UNKNOWN;
}


static void fill_timestamp(struct vmeta_frame_ext_timestamp *ts)
{
	ts->frame_timestamp = UINT64_C(123456789012);
}


static void fill_followme(struct vmeta_frame_ext_followme *fm)
{
	memset(fm, 0, sizeof(*fm));

	fm->target.latitude = 10.5;
	fm->target.longitude = -20.5;
	fm->target.altitude_wgs84ellipsoid = NAN;
	fm->target.altitude_egm96amsl = 100.0;
	fm->target.horizontal_accuracy = 1.0f;
	fm->target.vertical_accuracy = 2.0f;
	fm->target.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
	fm->target.valid = 1;

	fm->enabled = 1;
	fm->mode = 0;
	fm->angle_locked = 1;
	fm->animation = VMETA_FOLLOWME_ANIM_ORBIT;
}


static void compare_base_v2(struct vmeta_frame_v2_base *b1,
			    struct vmeta_frame_v2_base *b2)
{
	CU_ASSERT_PTR_NOT_NULL(b1);
	CU_ASSERT_PTR_NOT_NULL(b2);
	if (!b1 || !b2)
		return;

	compare_vmeta_quaternion(&b1->drone_quat, &b2->drone_quat);
	compare_vmeta_location(&b1->location, &b2->location, true);
	CU_ASSERT_DOUBLE_EQUAL(
		b1->ground_distance, b2->ground_distance, granularity(16));
	compare_vmeta_ned(&b1->speed, &b2->speed);
	CU_ASSERT_DOUBLE_EQUAL(b1->air_speed, b2->air_speed, granularity(8));
	compare_vmeta_quaternion(&b1->frame_quat, &b2->frame_quat);
	CU_ASSERT_DOUBLE_EQUAL(b1->camera_pan, b2->camera_pan, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		b1->camera_tilt, b2->camera_tilt, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		b1->exposure_time, b2->exposure_time, granularity(8));
	CU_ASSERT_EQUAL(b1->gain, b2->gain);
	CU_ASSERT_EQUAL(b1->wifi_rssi, b2->wifi_rssi);
	CU_ASSERT_EQUAL(b1->battery_percentage, b2->battery_percentage);
	CU_ASSERT_EQUAL(b1->binning, b2->binning);
	CU_ASSERT_EQUAL(b1->animation, b2->animation);
	CU_ASSERT_EQUAL(b1->state, b2->state);
	CU_ASSERT_EQUAL(b1->mode, b2->mode);
}


static void compare_timestamp_v2(struct vmeta_frame_ext_timestamp *t1,
				 struct vmeta_frame_ext_timestamp *t2)
{
	CU_ASSERT_PTR_NOT_NULL(t1);
	CU_ASSERT_PTR_NOT_NULL(t2);
	if (!t1 || !t2)
		return;

	CU_ASSERT_EQUAL(t1->frame_timestamp, t2->frame_timestamp);
}


static void compare_followme_v2(struct vmeta_frame_ext_followme *f1,
				struct vmeta_frame_ext_followme *f2)
{
	CU_ASSERT_PTR_NOT_NULL(f1);
	CU_ASSERT_PTR_NOT_NULL(f2);
	if (!f1 || !f2)
		return;

	/* Target location has no sv_count on the wire (always forced back
	 * to VMETA_LOCATION_INVALID_SV_COUNT by
	 * vmeta_frame_ext_followme_read()), and its altitude uses a 16-bit
	 * shift, same as automation/lfic targets in v3 -- hence
	 * include_sv_count=false here. */
	compare_vmeta_location(&f1->target, &f2->target, false);
	CU_ASSERT_EQUAL(f1->enabled, f2->enabled);
	CU_ASSERT_EQUAL(f1->mode, f2->mode);
	CU_ASSERT_EQUAL(f1->angle_locked, f2->angle_locked);
	CU_ASSERT_EQUAL(f1->animation, f2->animation);
}


static void do_write_read(bool with_ts, bool with_fm, size_t expected_size)
{
	struct vmeta_frame_v2 in;
	struct vmeta_frame_v2 out;
	uint8_t buf[VMETA_FRAME_V2_MAX_SIZE];
	struct vmeta_buffer vb;
	int res;

	memset(&in, 0, sizeof(in));
	fill_base(&in.base);

	in.has_timestamp = with_ts ? 1 : 0;
	if (with_ts)
		fill_timestamp(&in.timestamp);

	in.has_followme = with_fm ? 1 : 0;
	if (with_fm)
		fill_followme(&in.followme);

	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v2_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, expected_size);

	vb.len = vb.pos;
	vb.pos = 0;
	memset(&out, 0xff, sizeof(out));
	res = vmeta_frame_v2_read(&vb, &out);
	CU_ASSERT_EQUAL(res, 0);

	compare_base_v2(&in.base, &out.base);
	CU_ASSERT_EQUAL(out.has_timestamp, in.has_timestamp);
	CU_ASSERT_EQUAL(out.has_followme, in.has_followme);
	if (with_ts)
		compare_timestamp_v2(&in.timestamp, &out.timestamp);
	if (with_fm)
		compare_followme_v2(&in.followme, &out.followme);

	/* Characterize a real limitation of this legacy wire format: it has
	 * no fields at all for location horizontal/vertical accuracy, so
	 * they are unconditionally lost (read back as 0) regardless of what
	 * was written. */
	CU_ASSERT_EQUAL(out.base.location.horizontal_accuracy, 0.f);
	CU_ASSERT_EQUAL(out.base.location.vertical_accuracy, 0.f);
}


static void test_write_read_base_only(void)
{
	do_write_read(false, false, V2_SIZE_BASE_ONLY);
}


static void test_write_read_timestamp_only(void)
{
	do_write_read(true, false, V2_SIZE_WITH_TIMESTAMP);
}


static void test_write_read_followme_only(void)
{
	do_write_read(false, true, V2_SIZE_WITH_FOLLOWME);
}


static void test_write_read_both(void)
{
	do_write_read(true, true, V2_SIZE_WITH_BOTH);
	CU_ASSERT_EQUAL(V2_SIZE_WITH_BOTH, VMETA_FRAME_V2_MAX_SIZE);
}


static void test_write_buffer_too_small(void)
{
	struct vmeta_frame_v2 in;
	struct vmeta_buffer vb;
	uint8_t *buf;
	int res;
	size_t i;
	/* Every size below is strictly smaller than the 56 bytes required
	 * for a base-only frame: all must fail cleanly (negative errno)
	 * without ever writing past an exactly-sized allocation. */
	static const size_t sizes[] = {0, 1, 2, 3, 4, 10, 40, 55};

	memset(&in, 0, sizeof(in));
	fill_base(&in.base);

	for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
		buf = malloc(sizes[i] > 0 ? sizes[i] : 1);
		CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
		vmeta_buffer_set_data(&vb, buf, sizes[i], 0);
		res = vmeta_frame_v2_write(&vb, &in);
		CU_ASSERT_EQUAL(res, -ENOBUFS);
		free(buf);
	}
}


static void test_write_null_args(void)
{
	struct vmeta_frame_v2 in;
	struct vmeta_buffer vb;
	uint8_t buf[VMETA_FRAME_V2_MAX_SIZE];
	int res;

	memset(&in, 0, sizeof(in));
	fill_base(&in.base);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v2_write(NULL, &in);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v2_write(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_read_null_args(void)
{
	struct vmeta_frame_v2 out;
	struct vmeta_buffer vb;
	uint8_t buf[VMETA_FRAME_V2_MAX_SIZE];

	memset(buf, 0, sizeof(buf));
	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);

	CU_ASSERT_EQUAL(vmeta_frame_v2_read(NULL, &out), -EINVAL);
	CU_ASSERT_EQUAL(vmeta_frame_v2_read(&vb, NULL), -EINVAL);
}


static void test_read_buffer_empty(void)
{
	struct vmeta_frame_v2 out;
	struct vmeta_buffer vb;
	uint8_t buf[1] = {0};
	int res;

	vmeta_buffer_set_cdata(&vb, buf, 0, 0);
	res = vmeta_frame_v2_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -ENOBUFS);
}


static void test_read_buffer_claims_more_than_available(void)
{
	struct vmeta_frame_v2 out;
	struct vmeta_buffer vb;
	/* Well-formed id + len (13, i.e. a base-only frame claiming 56
	 * bytes), but the buffer itself is only 4 bytes long. */
	uint8_t buf[4] = {0x50, 0x32, 0x00, 0x0d};
	int res;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v2_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_read_bad_id(void)
{
	struct vmeta_frame_v2 out;
	struct vmeta_buffer vb;
	uint8_t buf[VMETA_FRAME_V2_MAX_SIZE];
	int res;

	memset(buf, 0, sizeof(buf));
	/* Wrong id (0x1234 instead of 0x5032) */
	buf[0] = 0x12;
	buf[1] = 0x34;
	buf[2] = 0x00;
	buf[3] = 0x0d;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v2_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_read_extension_truncated(void)
{
	struct vmeta_frame_v2 out;
	struct vmeta_buffer vb;
	uint8_t buf[64];
	int res;

	memset(buf, 0, sizeof(buf));

	/* Outer header: id=0x5032, and a len of 15, i.e. claiming the whole
	 * frame is exactly 15*4+4 = 64 bytes -- matching this (truncated)
	 * buffer's actual size, so the top-level length check in
	 * vmeta_frame_v2_read() passes and base-field reading (offsets
	 * [4..55], left zeroed here) succeeds. */
	buf[0] = 0x50;
	buf[1] = 0x32;
	buf[2] = 0x00;
	buf[3] = 0x0f;

	/* A timestamp extension header (offsets [56..59]) claiming len=2,
	 * i.e. 12 bytes total (4 header + 8 byte u64 payload) -- but only 4
	 * more bytes are actually available in the buffer (offsets
	 * [60..63]), 8 bytes short of what the extension's own header
	 * claims. This must be caught by the per-extension length check
	 * inside the read loop, distinct from (and only reachable past) the
	 * top-level frame-length check above. */
	buf[56] = 0x45;
	buf[57] = 0x31;
	buf[58] = 0x00;
	buf[59] = 0x02;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v2_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_to_json_full(void)
{
	struct vmeta_frame_v2 meta;
	struct json_object *jobj;
	struct json_object *sub;
	struct json_object *target;
	struct json_object *val;
	int res;

	memset(&meta, 0, sizeof(meta));
	fill_base(&meta.base);
	meta.has_timestamp = 1;
	fill_timestamp(&meta.timestamp);
	meta.has_followme = 1;
	fill_followme(&meta.followme);

	/* Deliberately exercise every conditional branch of
	 * vmeta_json_add_location() on the base location: NaN altitude
	 * (absent), a normal altitude (present), a zero accuracy (absent),
	 * a non-zero accuracy (present), and a valid sv_count (present). */
	meta.base.location.altitude_wgs84ellipsoid = NAN;
	meta.base.location.altitude_egm96amsl = 123.5;
	meta.base.location.horizontal_accuracy = 0.f;
	meta.base.location.vertical_accuracy = 2.5f;
	meta.base.location.sv_count = 12;
	meta.base.location.valid = 1;

	jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	res = vmeta_frame_v2_to_json(&meta, jobj);
	CU_ASSERT_EQUAL(res, 0);

	/* drone_quat */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "drone_quat", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	json_object_object_get_ex(sub, "w", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.drone_quat.w, 1e-6);
	json_object_object_get_ex(sub, "x", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.drone_quat.x, 1e-6);
	json_object_object_get_ex(sub, "y", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.drone_quat.y, 1e-6);
	json_object_object_get_ex(sub, "z", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.drone_quat.z, 1e-6);

	/* location */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "location", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "latitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.location.latitude, 1e-6);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "longitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(val),
			       meta.base.location.longitude,
			       1e-6);
	CU_ASSERT_FALSE(json_object_object_get_ex(
		sub, "altitude_wgs84ellipsoid", &val));
	CU_ASSERT_TRUE(
		json_object_object_get_ex(sub, "altitude_egm96amsl", &val));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(val), 123.5, 1e-6);
	CU_ASSERT_FALSE(
		json_object_object_get_ex(sub, "horizontal_accuracy", &val));
	CU_ASSERT_TRUE(
		json_object_object_get_ex(sub, "vertical_accuracy", &val));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(val), 2.5, 1e-6);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "sv_count", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), 12);

	/* ground_distance */
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jobj, "ground_distance", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.ground_distance, 1e-6);

	/* speed (ned) */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "speed", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	json_object_object_get_ex(sub, "north", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.speed.north, 1e-6);
	json_object_object_get_ex(sub, "east", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.speed.east, 1e-6);
	json_object_object_get_ex(sub, "down", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.speed.down, 1e-6);

	/* air_speed */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "air_speed", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.air_speed, 1e-6);

	/* frame_quat */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "frame_quat", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	json_object_object_get_ex(sub, "w", &val);
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.frame_quat.w, 1e-6);

	/* camera_pan / camera_tilt / exposure_time / gain */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "camera_pan", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.camera_pan, 1e-6);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "camera_tilt", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.camera_tilt, 1e-6);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "exposure_time", &val));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(val), meta.base.exposure_time, 1e-6);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "gain", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), meta.base.gain);

	/* wifi_rssi / battery_percentage */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "wifi_rssi", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), meta.base.wifi_rssi);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jobj, "battery_percentage", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), meta.base.battery_percentage);

	/* binning / animation (bitfields) */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "binning", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), 1);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "animation", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), 1);

	/* state / mode (enums, as strings) */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "state", &val));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(val),
			       vmeta_flying_state_str(meta.base.state));
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "mode", &val));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(val),
			       vmeta_piloting_mode_str(meta.base.mode));

	/* timestamp extension */
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jobj, "frame_timestamp", &val));
	CU_ASSERT_EQUAL((uint64_t)json_object_get_int64(val),
			meta.timestamp.frame_timestamp);

	/* followme extension */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "followme", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "target", &target));
	CU_ASSERT_PTR_NOT_NULL_FATAL(target);
	CU_ASSERT_TRUE(json_object_object_get_ex(target, "latitude", &val));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(val),
			       meta.followme.target.latitude,
			       1e-6);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "enabled", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), meta.followme.enabled);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "mode", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), meta.followme.mode);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "angle_locked", &val));
	CU_ASSERT_EQUAL(json_object_get_int(val), meta.followme.angle_locked);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "animation", &val));
	CU_ASSERT_STRING_EQUAL(
		json_object_get_string(val),
		vmeta_followme_anim_str(meta.followme.animation));

	json_object_put(jobj);
}


static void test_to_json_minimal(void)
{
	struct vmeta_frame_v2 meta;
	struct json_object *jobj;
	struct json_object *val;
	int res;

	/* has_timestamp=0, has_followme=0, location.valid=0, state=LANDED
	 * (0), mode=MANUAL (0) */
	memset(&meta, 0, sizeof(meta));

	jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	res = vmeta_frame_v2_to_json(&meta, jobj);
	CU_ASSERT_EQUAL(res, 0);

	/* An invalid location emits no "location" key at all */
	CU_ASSERT_FALSE(json_object_object_get_ex(jobj, "location", &val));
	/* Absent extensions emit no corresponding key at all */
	CU_ASSERT_FALSE(
		json_object_object_get_ex(jobj, "frame_timestamp", &val));
	CU_ASSERT_FALSE(json_object_object_get_ex(jobj, "followme", &val));

	/* Unconditional (non-extension, non-location) fields are still
	 * always present */
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "drone_quat", &val));
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "state", &val));
	CU_ASSERT_STRING_EQUAL(
		json_object_get_string(val),
		vmeta_flying_state_str(VMETA_FLYING_STATE_LANDED));
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "mode", &val));
	CU_ASSERT_STRING_EQUAL(
		json_object_get_string(val),
		vmeta_piloting_mode_str(VMETA_PILOTING_MODE_MANUAL));

	json_object_put(jobj);
}


/*
 * The following three helpers re-derive vmeta_frame_v2_to_csv()'s expected
 * output directly from the format strings in src/vmeta_csv.c /
 * src/vmeta_frame_v2.c (copied verbatim below), rather than calling those
 * (non-exported, hidden-visibility) functions directly -- vmeta_csv_add_*()
 * live in a private header (src/vmeta_csv.h) with no VMETA_API/export
 * attribute, so under this library's -fvisibility=hidden they are not
 * callable from a test binary that only links libvideo-metadata.so (same
 * class of gotcha as libjpeg-parrot's parser_read_*() functions).
 */
static size_t
csv_ref_quaternion(char *str, size_t maxlen, const struct vmeta_quaternion *val)
{
	size_t len = 0;
	len += snprintf(str,
			maxlen,
			"%.5f %.5f %.5f %.5f",
			val->w,
			val->x,
			val->y,
			val->z);
	return len;
}


static size_t csv_ref_ned(char *str, size_t maxlen, const struct vmeta_ned *val)
{
	size_t len = 0;
	len += snprintf(str,
			maxlen,
			"%.3f %.3f %.3f",
			val->north,
			val->east,
			val->down);
	return len;
}


static size_t
csv_ref_location(char *str, size_t maxlen, const struct vmeta_location *val)
{
	size_t len = 0;

	if (val->valid) {
		len += snprintf(
			str,
			maxlen,
			"%d %.8lf %.8lf %.2lf %.2lf %.2f %.2f %d",
			val->valid,
			val->latitude,
			val->longitude,
			val->altitude_wgs84ellipsoid,
			val->altitude_egm96amsl,
			val->horizontal_accuracy,
			val->vertical_accuracy,
			(val->sv_count != VMETA_LOCATION_INVALID_SV_COUNT)
				? val->sv_count
				: 0);
	} else {
		/* NOTE: mirrors vmeta_csv_add_location()'s "invalid" branch
		 * verbatim -- it has only ONE altitude placeholder (not two
		 * like the "valid" branch above), so an invalid location's
		 * CSV row silently has one fewer field than a valid one's. */
		len += snprintf(str,
				maxlen,
				"%d %.8lf %.8lf %.2lf %.2f %.2f %d",
				0,
				0.,
				0.,
				0.,
				0.,
				0.,
				0);
	}

	return len;
}


/* Only ever call this with a "maxlen" comfortably larger than the true
 * output length (see the safety note in test_to_csv_truncated()). */
static size_t
build_expected_csv(const struct vmeta_frame_v2 *meta, char *str, size_t maxlen)
{
	size_t len = 0;

	len += csv_ref_quaternion(
		str + len, maxlen - len, &meta->base.drone_quat);
	len += snprintf(str + len, maxlen - len, " ");
	len += csv_ref_location(str + len, maxlen - len, &meta->base.location);
	len += snprintf(
		str + len, maxlen - len, " %.2f ", meta->base.ground_distance);
	len += csv_ref_ned(str + len, maxlen - len, &meta->base.speed);
	len += snprintf(
		str + len, maxlen - len, " %.3f ", meta->base.air_speed);

	len += csv_ref_quaternion(
		str + len, maxlen - len, &meta->base.frame_quat);
	len += snprintf(str + len,
			maxlen - len,
			" %.4f %.4f %.4f %d",
			meta->base.camera_pan,
			meta->base.camera_tilt,
			meta->base.exposure_time,
			meta->base.gain);

	len += snprintf(str + len,
			maxlen - len,
			" %d %d",
			meta->base.wifi_rssi,
			meta->base.battery_percentage);

	len += snprintf(str + len,
			maxlen - len,
			" %d %d %d %d",
			meta->base.binning,
			meta->base.animation,
			meta->base.state,
			meta->base.mode);

	if (meta->has_timestamp) {
		len += snprintf(str + len,
				maxlen - len,
				" %" PRIu64,
				meta->timestamp.frame_timestamp);
	} else {
		len += snprintf(str + len, maxlen - len, " 0");
	}

	if (meta->has_followme) {
		len += snprintf(str + len, maxlen - len, " ");
		len += csv_ref_location(
			str + len, maxlen - len, &meta->followme.target);
		len += snprintf(str + len,
				maxlen - len,
				" %d %d %d %d",
				meta->followme.enabled,
				meta->followme.mode,
				meta->followme.angle_locked,
				meta->followme.animation);
	} else {
		struct vmeta_location loc;
		memset(&loc, 0, sizeof(loc));
		len += snprintf(str + len, maxlen - len, " ");
		len += csv_ref_location(str + len, maxlen - len, &loc);
		len += snprintf(
			str + len, maxlen - len, " %d %d %d %d", 0, 0, 0, 0);
	}

	return len;
}


static void test_to_csv_full(void)
{
	struct vmeta_frame_v2 meta;
	char actual[512];
	char expected[512];
	size_t actual_len;
	size_t expected_len;

	memset(&meta, 0, sizeof(meta));
	fill_base(&meta.base);
	meta.has_timestamp = 1;
	fill_timestamp(&meta.timestamp);
	meta.has_followme = 1;
	fill_followme(&meta.followme);

	expected_len = build_expected_csv(&meta, expected, sizeof(expected));
	actual_len = vmeta_frame_v2_to_csv(&meta, actual, sizeof(actual));

	CU_ASSERT_EQUAL(actual_len, expected_len);
	CU_ASSERT_STRING_EQUAL(actual, expected);
}


static void test_to_csv_minimal(void)
{
	struct vmeta_frame_v2 meta;
	char actual[512];
	char expected[512];
	size_t actual_len;
	size_t expected_len;

	/* has_timestamp=0, has_followme=0, base.location.valid=0 */
	memset(&meta, 0, sizeof(meta));

	expected_len = build_expected_csv(&meta, expected, sizeof(expected));
	actual_len = vmeta_frame_v2_to_csv(&meta, actual, sizeof(actual));

	CU_ASSERT_EQUAL(actual_len, expected_len);
	CU_ASSERT_STRING_EQUAL(actual, expected);
}


static void test_to_csv_truncated(void)
{
	struct vmeta_frame_v2 meta;
	char full[512];
	char truncated[512];
	size_t full_len;
	size_t res;
	size_t maxlen;
	size_t safe_prefix;
	/* The very last thing vmeta_frame_v2_to_csv() writes on the
	 * has_followme=1 path is " %d %d %d %d" with followme.{enabled,
	 * mode,angle_locked,animation} = {1,0,1,1} (see fill_followme()),
	 * i.e. the fixed 8-character string below. */
	static const char last_field[] = " 1 0 1 1";
	const size_t last_field_len = sizeof(last_field) - 1;

	memset(&meta, 0, sizeof(meta));
	fill_base(&meta.base);
	meta.has_timestamp = 1;
	fill_timestamp(&meta.timestamp);
	meta.has_followme = 1;
	fill_followme(&meta.followme);

	full_len = vmeta_frame_v2_to_csv(&meta, full, sizeof(full));
	CU_ASSERT_FATAL(full_len > last_field_len);
	CU_ASSERT_FATAL(full_len < sizeof(truncated));
	CU_ASSERT_EQUAL(memcmp(full + full_len - last_field_len,
			       last_field,
			       last_field_len),
			0);

	/* Truncate by exactly one byte: this is the only maxlen guaranteed
	 * safe to give vmeta_frame_v2_to_csv(). vmeta_frame_v2_to_csv() and
	 * the VMETA_STR_PRINT/VMETA_STR_SPACE macros it (and vmeta_csv.c's
	 * helpers) are built on never clamp their running "len" to "maxlen"
	 * between fields -- each call blindly does
	 * "len += snprintf(str + len, maxlen - len, ...)". The moment one
	 * field's true (untruncated) length pushes "len" above "maxlen",
	 * every subsequent call computes "maxlen - len" as an
	 * unsigned/size_t underflow (a huge value) together with a "str +
	 * len" pointer that is already past the caller's buffer -- a real
	 * heap-buffer-overflow footgun. Truncating by only the final
	 * character is provably safe (every "len" value before the last
	 * field call stays <= maxlen, see build_expected_csv()), so that is
	 * all that is exercised here; a smaller maxlen is deliberately not
	 * tested since it would corrupt memory instead of failing a single
	 * assertion. */
	maxlen = full_len - 1;
	safe_prefix = full_len - last_field_len;

	memset(truncated, 0x7f, sizeof(truncated));
	res = vmeta_frame_v2_to_csv(&meta, truncated, maxlen);

	/* The reported length always reflects the untruncated size, matching
	 * snprintf()'s own truncation convention (every VMETA_STR_PRINT call
	 * accumulates snprintf()'s return value, not the actual bytes
	 * written). */
	CU_ASSERT_EQUAL(res, full_len);

	/* Everything strictly before the final field is unaffected by the
	 * truncation (byte-for-byte identical to the untruncated run) */
	CU_ASSERT_EQUAL(memcmp(truncated, full, safe_prefix), 0);

	/* The string is still properly nul-terminated within maxlen bytes */
	CU_ASSERT_PTR_NOT_NULL(memchr(truncated, '\0', maxlen));

	/* Nothing was written past the given maxlen */
	CU_ASSERT_EQUAL((unsigned char)truncated[maxlen], 0x7f);
}


static const char *v2_csv_header_expected =
	"drone_quat_w drone_quat_x drone_quat_y drone_quat_z "
	"location_valid location_latitude location_longitude "
	"location_altitude_wgs84ellipsoid location_altitude_egm96amsl "
	"location_horizontal_accuracy location_vertical_accuracy "
	"location_sv_count ground_distance speed_north speed_east "
	"speed_down air_speed frame_quat_w frame_quat_x frame_quat_y "
	"frame_quat_z camera_pan camera_tilt exposure_time gain "
	"wifi_rssi battery_percentage "
	"binning animation state mode "
	"frame_timestamp "
	"followme_target_valid followme_target_latitude "
	"followme_target_longitude "
	"followme_target_altitude_wgs84ellipsoid "
	"followme_target_altitude_egm96amsl "
	"followme_target_sv_count "
	"followme_enabled followme_mode "
	"followme_angle_locked followme_animation";


static void test_csv_header(void)
{
	char buf[1024];
	size_t res;

	memset(buf, 0, sizeof(buf));
	res = vmeta_frame_v2_csv_header(buf, sizeof(buf));

	CU_ASSERT_EQUAL(res, strlen(v2_csv_header_expected));
	CU_ASSERT_STRING_EQUAL(buf, v2_csv_header_expected);
}


static void test_csv_header_truncated(void)
{
	char buf[1024];
	size_t maxlen;
	size_t res;

	/* A single VMETA_STR_PRINT() call (unlike to_csv()'s many chained
	 * ones): any maxlen is safe here, no cascading-underflow risk. */
	maxlen = strlen(v2_csv_header_expected) - 5;
	memset(buf, 0x7f, sizeof(buf));

	res = vmeta_frame_v2_csv_header(buf, maxlen);

	CU_ASSERT_EQUAL(res, strlen(v2_csv_header_expected));
	CU_ASSERT_EQUAL(memcmp(buf, v2_csv_header_expected, maxlen - 1), 0);
	CU_ASSERT_EQUAL(buf[maxlen - 1], '\0');
	CU_ASSERT_EQUAL((unsigned char)buf[maxlen], 0x7f);
}


CU_TestInfo s_v2_tests[] = {
	{(char *)"vmeta v2 write/read base only", &test_write_read_base_only},
	{(char *)"vmeta v2 write/read with timestamp",
	 &test_write_read_timestamp_only},
	{(char *)"vmeta v2 write/read with followme",
	 &test_write_read_followme_only},
	{(char *)"vmeta v2 write/read with timestamp and followme",
	 &test_write_read_both},
	{(char *)"vmeta v2 write buffer too small",
	 &test_write_buffer_too_small},
	{(char *)"vmeta v2 write null args", &test_write_null_args},
	{(char *)"vmeta v2 read null args", &test_read_null_args},
	{(char *)"vmeta v2 read empty buffer", &test_read_buffer_empty},
	{(char *)"vmeta v2 read buffer claims more than available",
	 &test_read_buffer_claims_more_than_available},
	{(char *)"vmeta v2 read bad id", &test_read_bad_id},
	{(char *)"vmeta v2 read extension truncated",
	 &test_read_extension_truncated},
	{(char *)"vmeta v2 to_json full", &test_to_json_full},
	{(char *)"vmeta v2 to_json minimal", &test_to_json_minimal},
	{(char *)"vmeta v2 to_csv full", &test_to_csv_full},
	{(char *)"vmeta v2 to_csv minimal", &test_to_csv_minimal},
	{(char *)"vmeta v2 to_csv truncated", &test_to_csv_truncated},
	{(char *)"vmeta v2 csv_header", &test_csv_header},
	{(char *)"vmeta v2 csv_header truncated", &test_csv_header_truncated},
	CU_TEST_INFO_NULL,
};
