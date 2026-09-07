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

/**
 * Real, assertion-based CUnit coverage for the legacy "Parrot Video
 * Streaming/Recording Metadata" v1 wire format (src/vmeta_frame_v1.c):
 *   - struct vmeta_frame_v1_streaming_basic
 *   - struct vmeta_frame_v1_streaming_extended
 *   - struct vmeta_frame_v1_recording
 *
 * Wire-format notes (from reading src/vmeta_frame_v1.c directly, not just
 * the public header):
 *   - "basic" and "extended" both use a 4-byte TLV-ish framing: a fixed
 *     16-bit id (VMETA_FRAME_V1_STREAMING_ID, 0x5031) followed by a 16-bit
 *     "len" (number of remaining 32-bit words). Read validates the id and
 *     validates buf->len against the declared len *before* reading any
 *     field, so a too-short read buffer fails with -EPROTO, not -ENOBUFS.
 *   - "recording" has NO id/len framing at all: it is a flat sequence of
 *     fields with no size validation on write, and no up-front length
 *     check on read - so a too-short buffer for recording fails with
 *     -ENOBUFS (from the underlying vmeta_buffer_read()), not -EPROTO.
 *   - Fixed-point shifts actually used (verified against the CHECK(...)
 *     call list in each write()/read() pair):
 *       - euler (yaw/pitch/roll), camera_pan, camera_tilt, quaternion
 *         (w/x/y/z): f32_i16 shift 12
 *       - exposure_time: f32_i16 shift 8
 *       - location.latitude/longitude: f64_i32 shift 20
 *       - location.altitude_egm96amsl (packed manually into
 *         gps_altitude_and_sv_count): effectively shift 8
 *       - altitude (relative to take-off), distance_from_home: f64_i32
 *         shift 16
 *       - speed (vmeta_xyz): f32_i16 shift 8
 *   - location.altitude_wgs84ellipsoid is *never* put on the wire: read()
 *     unconditionally sets it to NAN, regardless of what was written.
 *   - Location validity is not an explicit wire bit either: write() forces
 *     lat/lon (and both altitudes) to the sentinel 500.0 when
 *     meta->location.valid is 0 (vmeta_location_adjust_write()); read()
 *     infers valid = (latitude != 500.0 && longitude != 500.0)
 *     (vmeta_location_adjust_read()) and, when invalid, forces
 *     lat/lon/both-altitudes/both-accuracies to NAN and sv_count to
 *     VMETA_LOCATION_INVALID_SV_COUNT - *regardless* of what sv_count was
 *     actually written (the sv_count written to the wire always comes from
 *     the *original*, unadjusted meta->location.sv_count).
 *   - binning/animation/state/mode are packed as
 *     state_byte = (binning<<7)&0x80 | (state&0x7f) and
 *     mode_byte = (animation<<7)&0x80 | (mode&0x7f).
 *   - to_json()/to_csv()/csv_header() have NO NULL-argument guards (no
 *     ULOG_ERRNO_RETURN_ERR_IF at the top, unlike write()/read()), so they
 *     are only exercised here with valid pointers.
 *   - vmeta_frame_v1_streaming_basic_csv_header() just forwards to
 *     vmeta_frame_v1_streaming_extended_csv_header().
 *   - vmeta_csv_add_location() (src/vmeta_csv.c) has an asymmetric format
 *     string bug: the "valid" branch prints 8 fields (valid, lat, lon,
 *     wgs84 altitude, egm96 altitude, horizontal accuracy, vertical
 *     accuracy, sv_count) but the "invalid" branch's format string only
 *     has 7 conversions ("%d %.8lf %.8lf %.2lf %.2f %.2f %d") - i.e. one
 *     of the two altitude fields is silently dropped from the CSV output
 *     when a location is invalid. vmeta_frame_v1_streaming_basic_to_csv()
 *     always takes this "invalid" branch (it feeds a zeroed, non-valid
 *     struct vmeta_location, since the basic format has no location
 *     field at all), so this file's expected-CSV builder for "basic"
 *     reproduces the buggy 7-field form exactly, to match reality.
 *   - VMETA_STR_PRINT() is "len += snprintf(...)": snprintf's return value
 *     is the number of characters that *would* have been written, not the
 *     number actually written. In multi-call helpers like to_csv() (which
 *     chains several VMETA_STR_PRINT()/vmeta_csv_add_*() calls, each fed
 *     "maxlen - len" as its own remaining size), once the accumulated
 *     "would-be" len exceeds the caller-supplied maxlen, "maxlen - len"
 *     (both size_t) underflows to a huge value: from that point on,
 *     further internal snprintf() calls are effectively unbounded (no
 *     longer actually truncated) even though the *overall* to_csv() return
 *     value keeps reporting the full, untruncated logical length as if
 *     maxlen had been respected throughout. This is real, reproduced
 *     below (test_*_to_csv_small_maxlen()) using destination buffers sized
 *     well above the true (short, known) full output so the underflow
 *     cannot walk off the end of the real allocation - only demonstrating
 *     the logical inconsistency, never a real OOB write.
 */

#include "vmeta_test.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include <json-c/json.h>

#include <video-metadata/vmeta_frame_v1.h>


/* Comfortably larger than any real v1 CSV line (all under ~200 bytes) so
 * that the "maxlen underflow" quirk described above can never walk off the
 * end of the real allocation, however small the maxlen argument is. */
#define CSV_SAFE_BUFLEN 512


/*
 * ------------------------------------------------------------------
 * Streaming basic
 * ------------------------------------------------------------------
 */

static void fill_basic(struct vmeta_frame_v1_streaming_basic *m)
{
	memset(m, 0, sizeof(*m));
	m->drone_attitude.yaw = 0.5f;
	m->drone_attitude.pitch = -1.0f;
	m->drone_attitude.roll = 0.25f;
	m->frame_quat.w = 0.9f;
	m->frame_quat.x = 0.1f;
	m->frame_quat.y = -0.2f;
	m->frame_quat.z = 0.3f;
	m->camera_pan = 0.15f;
	m->camera_tilt = -0.35f;
	m->exposure_time = 12.5f;
	m->gain = 400;
	m->wifi_rssi = -55;
	m->battery_percentage = 77;
}


static void compare_basic(const struct vmeta_frame_v1_streaming_basic *a,
			  const struct vmeta_frame_v1_streaming_basic *b)
{
	CU_ASSERT_PTR_NOT_NULL_FATAL(a);
	CU_ASSERT_PTR_NOT_NULL_FATAL(b);

	CU_ASSERT_DOUBLE_EQUAL(
		a->drone_attitude.yaw, b->drone_attitude.yaw, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->drone_attitude.pitch,
			       b->drone_attitude.pitch,
			       granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->drone_attitude.roll,
			       b->drone_attitude.roll,
			       granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.w, b->frame_quat.w, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.x, b->frame_quat.x, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.y, b->frame_quat.y, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.z, b->frame_quat.z, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->camera_pan, b->camera_pan, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->camera_tilt, b->camera_tilt, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->exposure_time, b->exposure_time, granularity(8));
	CU_ASSERT_EQUAL(a->gain, b->gain);
	CU_ASSERT_EQUAL(a->wifi_rssi, b->wifi_rssi);
	CU_ASSERT_EQUAL(a->battery_percentage, b->battery_percentage);
}


static size_t
build_expected_basic_csv(const struct vmeta_frame_v1_streaming_basic *m,
			 char *out,
			 size_t outlen)
{
	size_t len = 0;

	len += snprintf(out + len,
			outlen - len,
			"%.4f %.4f %.4f",
			m->drone_attitude.yaw,
			m->drone_attitude.pitch,
			m->drone_attitude.roll);
	len += snprintf(out + len, outlen - len, " ");
	/* Zeroed, non-valid struct vmeta_location -> vmeta_csv_add_location()'s
	 * "invalid" branch, which (see file header comment) only has 7
	 * conversions, not 8. */
	len += snprintf(out + len,
			outlen - len,
			"%d %.8lf %.8lf %.2lf %.2f %.2f %d",
			0,
			0.,
			0.,
			0.,
			0.,
			0.,
			0);
	len += snprintf(out + len, outlen - len, " %.2f %.2f ", 0., 0.);
	/* Zeroed struct vmeta_xyz */
	len += snprintf(out + len, outlen - len, "%.3f %.3f %.3f", 0., 0., 0.);
	len += snprintf(out + len, outlen - len, " ");
	len += snprintf(out + len,
			outlen - len,
			"%.5f %.5f %.5f %.5f",
			m->frame_quat.w,
			m->frame_quat.x,
			m->frame_quat.y,
			m->frame_quat.z);
	len += snprintf(out + len,
			outlen - len,
			" %.4f %.4f %.4f %d",
			m->camera_pan,
			m->camera_tilt,
			m->exposure_time,
			m->gain);
	len += snprintf(out + len,
			outlen - len,
			" %d %d",
			m->wifi_rssi,
			m->battery_percentage);
	len += snprintf(out + len, outlen - len, " %d %d %d %d", 0, 0, 0, 0);

	return len;
}


static const char *s_basic_extended_csv_header =
	"drone_attitude_yaw drone_attitude_pitch drone_attitude_roll "
	"location_valid location_latitude location_longitude "
	"location_altitude_wgs84ellipsoid location_altitude_egm96amsl "
	"location_horizontal_accuracy location_vertical_accuracy "
	"location_sv_count altitude distance_from_home "
	"speed_x speed_y speed_z "
	"frame_quat_w frame_quat_x frame_quat_y frame_quat_z "
	"camera_pan camera_tilt exposure_time gain "
	"wifi_rssi battery_percentage "
	"binning animation state mode";


static void test_basic_write_read(void)
{
	struct vmeta_frame_v1_streaming_basic in;
	struct vmeta_frame_v1_streaming_basic out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_BASIC_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_basic(&in);

	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_basic_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, (size_t)VMETA_FRAME_V1_STREAMING_BASIC_SIZE);

	/* Check the id/len framing bytes explicitly: id = 0x5031, len =
	 * (28 - 4) / 4 = 6, both big-endian on the wire. */
	CU_ASSERT_EQUAL(buf[0], 0x50);
	CU_ASSERT_EQUAL(buf[1], 0x31);
	CU_ASSERT_EQUAL(buf[2], 0x00);
	CU_ASSERT_EQUAL(buf[3], 0x06);

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	memset(&out, 0xaa, sizeof(out));
	res = vmeta_frame_v1_streaming_basic_read(&vb, &out);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, (size_t)VMETA_FRAME_V1_STREAMING_BASIC_SIZE);

	compare_basic(&in, &out);
}


static void test_basic_write_too_small(void)
{
	struct vmeta_frame_v1_streaming_basic in;
	uint8_t buf[10];
	struct vmeta_buffer vb;
	int res;

	fill_basic(&in);

	/* 10 bytes is enough for id+len+yaw+pitch+roll (10 bytes exactly),
	 * but not for the next field (camera_pan) -> deterministic
	 * -ENOBUFS from vmeta_buffer_write(). */
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_basic_write(&vb, &in);
	CU_ASSERT_EQUAL(res, -ENOBUFS);
}


static void test_basic_write_null(void)
{
	struct vmeta_frame_v1_streaming_basic in;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_BASIC_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_basic(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v1_streaming_basic_write(NULL, &in);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v1_streaming_basic_write(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_basic_read_null(void)
{
	struct vmeta_frame_v1_streaming_basic out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_BASIC_SIZE] = {0};
	struct vmeta_buffer vb;
	int res;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v1_streaming_basic_read(NULL, &out);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v1_streaming_basic_read(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_basic_read_too_small(void)
{
	struct vmeta_frame_v1_streaming_basic in;
	struct vmeta_frame_v1_streaming_basic out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_BASIC_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_basic(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_basic_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	/* Declare a shorter buffer length than the real allocation (the real
	 * allocation still has all the bytes, so this cannot read out of
	 * bounds); the declared len (6 words = 28 bytes) vs. the shortened
	 * buf->len fails the explicit "bad length" check in read() with
	 * -EPROTO, before any field is actually read. */
	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf) - 8, 0);
	res = vmeta_frame_v1_streaming_basic_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_basic_read_bad_id(void)
{
	struct vmeta_frame_v1_streaming_basic in;
	struct vmeta_frame_v1_streaming_basic out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_BASIC_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_basic(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_basic_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	/* Corrupt the id field */
	buf[0] = 0x12;
	buf[1] = 0x34;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_basic_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_basic_to_json(void)
{
	struct vmeta_frame_v1_streaming_basic m;
	struct json_object *jobj;
	struct json_object *sub;
	struct json_object *tmp;
	int res;

	fill_basic(&m);

	jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	res = vmeta_frame_v1_streaming_basic_to_json(&m, jobj);
	CU_ASSERT_EQUAL(res, 0);

	/* "basic" has no location field at all */
	CU_ASSERT_FALSE(json_object_object_get_ex(jobj, "location", &tmp));

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "drone_attitude", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "yaw", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp),
			       (double)m.drone_attitude.yaw,
			       1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "pitch", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp),
			       (double)m.drone_attitude.pitch,
			       1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "roll", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp),
			       (double)m.drone_attitude.roll,
			       1e-9);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "frame_quat", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "w", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.frame_quat.w, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "x", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.frame_quat.x, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "y", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.frame_quat.y, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "z", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.frame_quat.z, 1e-9);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "camera_pan", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.camera_pan, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "camera_tilt", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.camera_tilt, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "exposure_time", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.exposure_time, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "gain", &tmp));
	CU_ASSERT_EQUAL(json_object_get_int(tmp), m.gain);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "wifi_rssi", &tmp));
	CU_ASSERT_EQUAL(json_object_get_int(tmp), m.wifi_rssi);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jobj, "battery_percentage", &tmp));
	CU_ASSERT_EQUAL(json_object_get_int(tmp), m.battery_percentage);

	json_object_put(jobj);
}


static void test_basic_to_csv(void)
{
	struct vmeta_frame_v1_streaming_basic m;
	char expected[CSV_SAFE_BUFLEN];
	char actual[CSV_SAFE_BUFLEN];
	size_t expected_len;
	size_t actual_len;

	fill_basic(&m);

	expected_len = build_expected_basic_csv(&m, expected, sizeof(expected));
	actual_len = vmeta_frame_v1_streaming_basic_to_csv(
		&m, actual, sizeof(actual));

	CU_ASSERT_EQUAL(actual_len, expected_len);
	CU_ASSERT_STRING_EQUAL(actual, expected);
}


static void test_basic_to_csv_small_maxlen(void)
{
	struct vmeta_frame_v1_streaming_basic m;
	char full[CSV_SAFE_BUFLEN];
	char truncated[CSV_SAFE_BUFLEN];
	size_t full_len;
	size_t truncated_len;

	fill_basic(&m);
	memset(full, 0, sizeof(full));
	memset(truncated, 0, sizeof(truncated));

	full_len =
		vmeta_frame_v1_streaming_basic_to_csv(&m, full, sizeof(full));

	/* maxlen = 8 is smaller than even the first (euler) field's output;
	 * the underlying buffer is still CSV_SAFE_BUFLEN so this cannot
	 * write out of bounds (see file header comment on the maxlen
	 * underflow quirk). */
	truncated_len = vmeta_frame_v1_streaming_basic_to_csv(&m, truncated, 8);

	/* The returned length does NOT reflect the small maxlen: it is the
	 * same "logical" length as the untruncated call. */
	CU_ASSERT_EQUAL(truncated_len, full_len);

	/* But the buffer contents were genuinely truncated: it is a valid,
	 * NUL-terminated C string, strictly shorter than the full output,
	 * and a prefix of it. */
	CU_ASSERT_TRUE(strlen(truncated) < strlen(full));
	CU_ASSERT_EQUAL(strncmp(truncated, full, strlen(truncated)), 0);
	CU_ASSERT_TRUE(strlen(truncated) < 8);
}


static void test_basic_csv_header(void)
{
	char actual[CSV_SAFE_BUFLEN];
	char actual_basic[CSV_SAFE_BUFLEN];
	size_t len;
	size_t len_basic;

	len = vmeta_frame_v1_streaming_extended_csv_header(actual,
							   sizeof(actual));
	CU_ASSERT_EQUAL(len, strlen(s_basic_extended_csv_header));
	CU_ASSERT_STRING_EQUAL(actual, s_basic_extended_csv_header);

	/* basic's csv_header() just forwards to extended's */
	len_basic = vmeta_frame_v1_streaming_basic_csv_header(
		actual_basic, sizeof(actual_basic));
	CU_ASSERT_EQUAL(len_basic, len);
	CU_ASSERT_STRING_EQUAL(actual_basic, actual);
}


static void test_basic_csv_header_small_maxlen(void)
{
	char truncated[CSV_SAFE_BUFLEN];
	size_t full_len;
	size_t truncated_len;

	memset(truncated, 0, sizeof(truncated));

	full_len = strlen(s_basic_extended_csv_header);

	truncated_len =
		vmeta_frame_v1_streaming_basic_csv_header(truncated, 10);

	/* Single snprintf() call: standard, well-behaved truncation
	 * semantics - the returned length is still the full logical length
	 * (that's how snprintf() works), but the buffer content is properly
	 * truncated to 9 characters + NUL. */
	CU_ASSERT_EQUAL(truncated_len, full_len);
	CU_ASSERT_EQUAL(strlen(truncated), 9);
	CU_ASSERT_EQUAL(strncmp(truncated, s_basic_extended_csv_header, 9), 0);
}


/*
 * ------------------------------------------------------------------
 * Streaming extended
 * ------------------------------------------------------------------
 */

static void fill_extended(struct vmeta_frame_v1_streaming_extended *m)
{
	memset(m, 0, sizeof(*m));
	m->drone_attitude.yaw = 0.5f;
	m->drone_attitude.pitch = -1.0f;
	m->drone_attitude.roll = 0.25f;

	m->location.latitude = 48.8566;
	m->location.longitude = 2.3522;
	m->location.altitude_wgs84ellipsoid = 100.0;
	m->location.altitude_egm96amsl = 95.5;
	m->location.horizontal_accuracy = 1.5f;
	m->location.vertical_accuracy = 2.5f;
	m->location.sv_count = 12;
	m->location.valid = 1;

	m->altitude = 50.25;
	m->distance_from_home = 123.75;

	m->speed.x = 1.5f;
	m->speed.y = -2.5f;
	m->speed.z = 0.5f;

	m->frame_quat.w = 0.9f;
	m->frame_quat.x = 0.1f;
	m->frame_quat.y = -0.2f;
	m->frame_quat.z = 0.3f;
	m->camera_pan = 0.15f;
	m->camera_tilt = -0.35f;
	m->exposure_time = 12.5f;
	m->gain = 400;
	m->wifi_rssi = -55;
	m->battery_percentage = 77;

	m->binning = 1;
	m->animation = 1;
	m->state = VMETA_FLYING_STATE_FLYING;
	m->mode = VMETA_PILOTING_MODE_MAGIC_CARPET;
}


static void
compare_extended_common(const struct vmeta_frame_v1_streaming_extended *a,
			const struct vmeta_frame_v1_streaming_extended *b)
{
	CU_ASSERT_DOUBLE_EQUAL(
		a->drone_attitude.yaw, b->drone_attitude.yaw, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->drone_attitude.pitch,
			       b->drone_attitude.pitch,
			       granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->drone_attitude.roll,
			       b->drone_attitude.roll,
			       granularity(12));

	CU_ASSERT_DOUBLE_EQUAL(a->altitude, b->altitude, granularity(16));
	CU_ASSERT_DOUBLE_EQUAL(
		a->distance_from_home, b->distance_from_home, granularity(16));

	CU_ASSERT_DOUBLE_EQUAL(a->speed.x, b->speed.x, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(a->speed.y, b->speed.y, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(a->speed.z, b->speed.z, granularity(8));

	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.w, b->frame_quat.w, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.x, b->frame_quat.x, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.y, b->frame_quat.y, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.z, b->frame_quat.z, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->camera_pan, b->camera_pan, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->camera_tilt, b->camera_tilt, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->exposure_time, b->exposure_time, granularity(8));
	CU_ASSERT_EQUAL(a->gain, b->gain);
	CU_ASSERT_EQUAL(a->wifi_rssi, b->wifi_rssi);
	CU_ASSERT_EQUAL(a->battery_percentage, b->battery_percentage);

	CU_ASSERT_EQUAL(a->binning, b->binning);
	CU_ASSERT_EQUAL(a->animation, b->animation);
	CU_ASSERT_EQUAL(a->state, b->state);
	CU_ASSERT_EQUAL(a->mode, b->mode);
}


static size_t
build_expected_extended_csv(const struct vmeta_frame_v1_streaming_extended *m,
			    char *out,
			    size_t outlen)
{
	size_t len = 0;

	len += snprintf(out + len,
			outlen - len,
			"%.4f %.4f %.4f",
			m->drone_attitude.yaw,
			m->drone_attitude.pitch,
			m->drone_attitude.roll);
	len += snprintf(out + len, outlen - len, " ");
	/* location.valid is set in the fixture -> the "valid" branch of
	 * vmeta_csv_add_location(), 8 conversions. */
	len += snprintf(out + len,
			outlen - len,
			"%d %.8lf %.8lf %.2lf %.2lf %.2f %.2f %d",
			m->location.valid,
			m->location.latitude,
			m->location.longitude,
			m->location.altitude_wgs84ellipsoid,
			m->location.altitude_egm96amsl,
			m->location.horizontal_accuracy,
			m->location.vertical_accuracy,
			m->location.sv_count);
	len += snprintf(out + len,
			outlen - len,
			" %.2f %.2f ",
			m->altitude,
			m->distance_from_home);
	len += snprintf(out + len,
			outlen - len,
			"%.3f %.3f %.3f",
			m->speed.x,
			m->speed.y,
			m->speed.z);
	len += snprintf(out + len, outlen - len, " ");
	len += snprintf(out + len,
			outlen - len,
			"%.5f %.5f %.5f %.5f",
			m->frame_quat.w,
			m->frame_quat.x,
			m->frame_quat.y,
			m->frame_quat.z);
	len += snprintf(out + len,
			outlen - len,
			" %.4f %.4f %.4f %d",
			m->camera_pan,
			m->camera_tilt,
			m->exposure_time,
			m->gain);
	len += snprintf(out + len,
			outlen - len,
			" %d %d",
			m->wifi_rssi,
			m->battery_percentage);
	len += snprintf(out + len,
			outlen - len,
			" %d %d %d %d",
			m->binning,
			m->animation,
			m->state,
			m->mode);

	return len;
}


static void test_extended_write_read(void)
{
	struct vmeta_frame_v1_streaming_extended in;
	struct vmeta_frame_v1_streaming_extended out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_extended(&in);

	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_extended_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, (size_t)VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE);

	CU_ASSERT_EQUAL(buf[0], 0x50);
	CU_ASSERT_EQUAL(buf[1], 0x31);
	CU_ASSERT_EQUAL(buf[2], 0x00);
	CU_ASSERT_EQUAL(buf[3], 0x0d); /* (56 - 4) / 4 = 13 */

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	memset(&out, 0xaa, sizeof(out));
	res = vmeta_frame_v1_streaming_extended_read(&vb, &out);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, (size_t)VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE);

	compare_extended_common(&in, &out);

	/* Location: valid, so lat/lon/sv_count/egm96 altitude round-trip
	 * (within their respective granularities); wgs84 altitude is never
	 * on the wire, always NAN on read. */
	CU_ASSERT_EQUAL(out.location.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(
		out.location.latitude, in.location.latitude, granularity(20));
	CU_ASSERT_DOUBLE_EQUAL(
		out.location.longitude, in.location.longitude, granularity(20));
	CU_ASSERT_TRUE(isnan(out.location.altitude_wgs84ellipsoid));
	CU_ASSERT_DOUBLE_EQUAL(out.location.altitude_egm96amsl,
			       in.location.altitude_egm96amsl,
			       granularity(8));
	CU_ASSERT_EQUAL(out.location.sv_count, in.location.sv_count);
}


static void test_extended_write_read_invalid_location(void)
{
	struct vmeta_frame_v1_streaming_extended in;
	struct vmeta_frame_v1_streaming_extended out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_extended(&in);
	in.location.valid = 0;
	/* On the wire, sv_count is taken from the *original* (unadjusted)
	 * meta->location.sv_count even when the location is invalid; the
	 * read side is expected to force it back to
	 * VMETA_LOCATION_INVALID_SV_COUNT regardless. Use a real value here
	 * to prove that override actually happens. */
	in.location.sv_count = 7;

	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_extended_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	memset(&out, 0xaa, sizeof(out));
	res = vmeta_frame_v1_streaming_extended_read(&vb, &out);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(out.location.valid, 0);
	CU_ASSERT_TRUE(isnan(out.location.latitude));
	CU_ASSERT_TRUE(isnan(out.location.longitude));
	CU_ASSERT_TRUE(isnan(out.location.altitude_wgs84ellipsoid));
	CU_ASSERT_TRUE(isnan(out.location.altitude_egm96amsl));
	CU_ASSERT_TRUE(isnan(out.location.horizontal_accuracy));
	CU_ASSERT_TRUE(isnan(out.location.vertical_accuracy));
	CU_ASSERT_EQUAL(out.location.sv_count, VMETA_LOCATION_INVALID_SV_COUNT);
}


static void test_extended_write_too_small(void)
{
	struct vmeta_frame_v1_streaming_extended in;
	uint8_t buf[10];
	struct vmeta_buffer vb;
	int res;

	fill_extended(&in);

	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_extended_write(&vb, &in);
	CU_ASSERT_EQUAL(res, -ENOBUFS);
}


static void test_extended_write_null(void)
{
	struct vmeta_frame_v1_streaming_extended in;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_extended(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v1_streaming_extended_write(NULL, &in);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v1_streaming_extended_write(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_extended_read_null(void)
{
	struct vmeta_frame_v1_streaming_extended out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE] = {0};
	struct vmeta_buffer vb;
	int res;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v1_streaming_extended_read(NULL, &out);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v1_streaming_extended_read(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_extended_read_too_small(void)
{
	struct vmeta_frame_v1_streaming_extended in;
	struct vmeta_frame_v1_streaming_extended out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_extended(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_extended_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf) - 8, 0);
	res = vmeta_frame_v1_streaming_extended_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_extended_read_bad_id(void)
{
	struct vmeta_frame_v1_streaming_extended in;
	struct vmeta_frame_v1_streaming_extended out;
	uint8_t buf[VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_extended(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_extended_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	buf[0] = 0x00;
	buf[1] = 0x00;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_streaming_extended_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -EPROTO);
}


static void test_extended_to_json(void)
{
	struct vmeta_frame_v1_streaming_extended m;
	struct json_object *jobj;
	struct json_object *sub;
	struct json_object *tmp;
	int res;

	fill_extended(&m);

	jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	res = vmeta_frame_v1_streaming_extended_to_json(&m, jobj);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "location", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "latitude", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), m.location.latitude, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "longitude", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), m.location.longitude, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(
		sub, "altitude_wgs84ellipsoid", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp),
			       m.location.altitude_wgs84ellipsoid,
			       1e-9);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(sub, "altitude_egm96amsl", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp),
			       m.location.altitude_egm96amsl,
			       1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "sv_count", &tmp));
	CU_ASSERT_EQUAL(json_object_get_int(tmp), m.location.sv_count);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "altitude", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp), m.altitude, 1e-9);
	CU_ASSERT_TRUE(
		json_object_object_get_ex(jobj, "distance_from_home", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), m.distance_from_home, 1e-9);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "speed", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "x", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.speed.x, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "y", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.speed.y, 1e-9);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "z", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), (double)m.speed.z, 1e-9);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "binning", &tmp));
	CU_ASSERT_EQUAL(json_object_get_int(tmp), m.binning);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "animation", &tmp));
	CU_ASSERT_EQUAL(json_object_get_int(tmp), m.animation);
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "state", &tmp));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(tmp),
			       vmeta_flying_state_str(m.state));
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "mode", &tmp));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(tmp),
			       vmeta_piloting_mode_str(m.mode));

	json_object_put(jobj);
}


static void test_extended_to_json_invalid_location(void)
{
	struct vmeta_frame_v1_streaming_extended m;
	struct json_object *jobj;
	struct json_object *tmp;
	int res;

	fill_extended(&m);
	m.location.valid = 0;

	jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	res = vmeta_frame_v1_streaming_extended_to_json(&m, jobj);
	CU_ASSERT_EQUAL(res, 0);

	/* vmeta_json_add_location() returns early without adding anything
	 * when val->valid is 0 */
	CU_ASSERT_FALSE(json_object_object_get_ex(jobj, "location", &tmp));

	json_object_put(jobj);
}


static void test_extended_to_csv(void)
{
	struct vmeta_frame_v1_streaming_extended m;
	char expected[CSV_SAFE_BUFLEN];
	char actual[CSV_SAFE_BUFLEN];
	size_t expected_len;
	size_t actual_len;

	fill_extended(&m);

	expected_len =
		build_expected_extended_csv(&m, expected, sizeof(expected));
	actual_len = vmeta_frame_v1_streaming_extended_to_csv(
		&m, actual, sizeof(actual));

	CU_ASSERT_EQUAL(actual_len, expected_len);
	CU_ASSERT_STRING_EQUAL(actual, expected);
}


static void test_extended_to_csv_small_maxlen(void)
{
	struct vmeta_frame_v1_streaming_extended m;
	char full[CSV_SAFE_BUFLEN];
	char truncated[CSV_SAFE_BUFLEN];
	size_t full_len;
	size_t truncated_len;

	fill_extended(&m);
	memset(full, 0, sizeof(full));
	memset(truncated, 0, sizeof(truncated));

	full_len = vmeta_frame_v1_streaming_extended_to_csv(
		&m, full, sizeof(full));
	truncated_len =
		vmeta_frame_v1_streaming_extended_to_csv(&m, truncated, 8);

	CU_ASSERT_EQUAL(truncated_len, full_len);
	CU_ASSERT_TRUE(strlen(truncated) < strlen(full));
	CU_ASSERT_EQUAL(strncmp(truncated, full, strlen(truncated)), 0);
	CU_ASSERT_TRUE(strlen(truncated) < 8);
}


static void test_extended_csv_header(void)
{
	char actual[CSV_SAFE_BUFLEN];
	size_t len;

	len = vmeta_frame_v1_streaming_extended_csv_header(actual,
							   sizeof(actual));
	CU_ASSERT_EQUAL(len, strlen(s_basic_extended_csv_header));
	CU_ASSERT_STRING_EQUAL(actual, s_basic_extended_csv_header);
}


/*
 * ------------------------------------------------------------------
 * Recording
 * ------------------------------------------------------------------
 */

static void fill_recording(struct vmeta_frame_v1_recording *m)
{
	memset(m, 0, sizeof(*m));
	m->drone_attitude.yaw = 0.5f;
	m->drone_attitude.pitch = -1.0f;
	m->drone_attitude.roll = 0.25f;

	m->location.latitude = 48.8566;
	m->location.longitude = 2.3522;
	m->location.altitude_wgs84ellipsoid = 100.0;
	m->location.altitude_egm96amsl = 95.5;
	m->location.horizontal_accuracy = 1.5f;
	m->location.vertical_accuracy = 2.5f;
	m->location.sv_count = 12;
	m->location.valid = 1;

	m->altitude = 50.25;
	m->distance_from_home = 123.75;

	m->speed.x = 1.5f;
	m->speed.y = -2.5f;
	m->speed.z = 0.5f;

	m->frame_timestamp = UINT64_C(1234567890123);

	m->frame_quat.w = 0.9f;
	m->frame_quat.x = 0.1f;
	m->frame_quat.y = -0.2f;
	m->frame_quat.z = 0.3f;
	m->camera_pan = 0.15f;
	m->camera_tilt = -0.35f;
	m->exposure_time = 12.5f;
	m->gain = 400;
	m->wifi_rssi = -55;
	m->battery_percentage = 77;

	m->binning = 1;
	m->animation = 1;
	m->state = VMETA_FLYING_STATE_FLYING;
	m->mode = VMETA_PILOTING_MODE_MAGIC_CARPET;
}


static void compare_recording_common(const struct vmeta_frame_v1_recording *a,
				     const struct vmeta_frame_v1_recording *b)
{
	CU_ASSERT_EQUAL(a->frame_timestamp, b->frame_timestamp);

	CU_ASSERT_DOUBLE_EQUAL(
		a->drone_attitude.yaw, b->drone_attitude.yaw, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->drone_attitude.pitch,
			       b->drone_attitude.pitch,
			       granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->drone_attitude.roll,
			       b->drone_attitude.roll,
			       granularity(12));

	CU_ASSERT_DOUBLE_EQUAL(a->altitude, b->altitude, granularity(16));
	CU_ASSERT_DOUBLE_EQUAL(
		a->distance_from_home, b->distance_from_home, granularity(16));

	CU_ASSERT_DOUBLE_EQUAL(a->speed.x, b->speed.x, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(a->speed.y, b->speed.y, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(a->speed.z, b->speed.z, granularity(8));

	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.w, b->frame_quat.w, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.x, b->frame_quat.x, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.y, b->frame_quat.y, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->frame_quat.z, b->frame_quat.z, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->camera_pan, b->camera_pan, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(a->camera_tilt, b->camera_tilt, granularity(12));
	CU_ASSERT_DOUBLE_EQUAL(
		a->exposure_time, b->exposure_time, granularity(8));
	CU_ASSERT_EQUAL(a->gain, b->gain);
	CU_ASSERT_EQUAL(a->wifi_rssi, b->wifi_rssi);
	CU_ASSERT_EQUAL(a->battery_percentage, b->battery_percentage);

	CU_ASSERT_EQUAL(a->binning, b->binning);
	CU_ASSERT_EQUAL(a->animation, b->animation);
	CU_ASSERT_EQUAL(a->state, b->state);
	CU_ASSERT_EQUAL(a->mode, b->mode);
}


static size_t
build_expected_recording_csv(const struct vmeta_frame_v1_recording *m,
			     char *out,
			     size_t outlen)
{
	size_t len = 0;

	len += snprintf(out + len,
			outlen - len,
			"%.4f %.4f %.4f",
			m->drone_attitude.yaw,
			m->drone_attitude.pitch,
			m->drone_attitude.roll);
	len += snprintf(out + len, outlen - len, " ");
	len += snprintf(out + len,
			outlen - len,
			"%d %.8lf %.8lf %.2lf %.2lf %.2f %.2f %d",
			m->location.valid,
			m->location.latitude,
			m->location.longitude,
			m->location.altitude_wgs84ellipsoid,
			m->location.altitude_egm96amsl,
			m->location.horizontal_accuracy,
			m->location.vertical_accuracy,
			m->location.sv_count);
	len += snprintf(out + len,
			outlen - len,
			" %.2f %.2f ",
			m->altitude,
			m->distance_from_home);
	len += snprintf(out + len,
			outlen - len,
			"%.3f %.3f %.3f",
			m->speed.x,
			m->speed.y,
			m->speed.z);
	len += snprintf(
		out + len, outlen - len, " %" PRIu64 " ", m->frame_timestamp);
	len += snprintf(out + len,
			outlen - len,
			"%.5f %.5f %.5f %.5f",
			m->frame_quat.w,
			m->frame_quat.x,
			m->frame_quat.y,
			m->frame_quat.z);
	len += snprintf(out + len,
			outlen - len,
			" %.4f %.4f %.4f %d",
			m->camera_pan,
			m->camera_tilt,
			m->exposure_time,
			m->gain);
	len += snprintf(out + len,
			outlen - len,
			" %d %d",
			m->wifi_rssi,
			m->battery_percentage);
	len += snprintf(out + len,
			outlen - len,
			" %d %d %d %d",
			m->binning,
			m->animation,
			m->state,
			m->mode);

	return len;
}


static const char *s_recording_csv_header =
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
	"binning animation state mode";


static void test_recording_write_read(void)
{
	struct vmeta_frame_v1_recording in;
	struct vmeta_frame_v1_recording out;
	uint8_t buf[VMETA_FRAME_V1_RECORDING_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_recording(&in);

	/* No id/len framing at all: the first 8 bytes are the raw big-endian
	 * frame_timestamp. */
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_recording_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, (size_t)VMETA_FRAME_V1_RECORDING_SIZE);

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	memset(&out, 0xaa, sizeof(out));
	res = vmeta_frame_v1_recording_read(&vb, &out);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(vb.pos, (size_t)VMETA_FRAME_V1_RECORDING_SIZE);

	compare_recording_common(&in, &out);

	CU_ASSERT_EQUAL(out.location.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(
		out.location.latitude, in.location.latitude, granularity(20));
	CU_ASSERT_DOUBLE_EQUAL(
		out.location.longitude, in.location.longitude, granularity(20));
	CU_ASSERT_TRUE(isnan(out.location.altitude_wgs84ellipsoid));
	CU_ASSERT_DOUBLE_EQUAL(out.location.altitude_egm96amsl,
			       in.location.altitude_egm96amsl,
			       granularity(8));
	CU_ASSERT_EQUAL(out.location.sv_count, in.location.sv_count);
}


static void test_recording_write_read_invalid_location(void)
{
	struct vmeta_frame_v1_recording in;
	struct vmeta_frame_v1_recording out;
	uint8_t buf[VMETA_FRAME_V1_RECORDING_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_recording(&in);
	in.location.valid = 0;
	in.location.sv_count = 3;

	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_recording_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);
	memset(&out, 0xaa, sizeof(out));
	res = vmeta_frame_v1_recording_read(&vb, &out);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(out.location.valid, 0);
	CU_ASSERT_TRUE(isnan(out.location.latitude));
	CU_ASSERT_TRUE(isnan(out.location.longitude));
	CU_ASSERT_EQUAL(out.location.sv_count, VMETA_LOCATION_INVALID_SV_COUNT);
}


static void test_recording_write_too_small(void)
{
	struct vmeta_frame_v1_recording in;
	uint8_t buf[4];
	struct vmeta_buffer vb;
	int res;

	fill_recording(&in);

	/* First field is the 8-byte frame_timestamp: 4 bytes is never
	 * enough -> deterministic -ENOBUFS on the very first write. */
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_recording_write(&vb, &in);
	CU_ASSERT_EQUAL(res, -ENOBUFS);
}


static void test_recording_write_null(void)
{
	struct vmeta_frame_v1_recording in;
	uint8_t buf[VMETA_FRAME_V1_RECORDING_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_recording(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v1_recording_write(NULL, &in);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v1_recording_write(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_recording_read_null(void)
{
	struct vmeta_frame_v1_recording out;
	uint8_t buf[VMETA_FRAME_V1_RECORDING_SIZE] = {0};
	struct vmeta_buffer vb;
	int res;

	vmeta_buffer_set_cdata(&vb, buf, sizeof(buf), 0);

	res = vmeta_frame_v1_recording_read(NULL, &out);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_v1_recording_read(&vb, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_recording_read_too_small(void)
{
	struct vmeta_frame_v1_recording in;
	struct vmeta_frame_v1_recording out;
	uint8_t buf[VMETA_FRAME_V1_RECORDING_SIZE];
	struct vmeta_buffer vb;
	int res;

	fill_recording(&in);
	vmeta_buffer_set_data(&vb, buf, sizeof(buf), 0);
	res = vmeta_frame_v1_recording_write(&vb, &in);
	CU_ASSERT_EQUAL(res, 0);

	/* Unlike basic/extended, recording has no id/len framing and no
	 * up-front length check: a too-short buffer fails from within
	 * vmeta_buffer_read() itself, with -ENOBUFS, once the read cursor
	 * would run past the declared (shortened) buf->len. The real
	 * underlying allocation is untouched/still full-sized, so this
	 * cannot read out of bounds. */
	vmeta_buffer_set_cdata(&vb, buf, 30, 0);
	res = vmeta_frame_v1_recording_read(&vb, &out);
	CU_ASSERT_EQUAL(res, -ENOBUFS);
}


static void test_recording_to_json(void)
{
	struct vmeta_frame_v1_recording m;
	struct json_object *jobj;
	struct json_object *sub;
	struct json_object *tmp;
	int res;

	fill_recording(&m);

	jobj = json_object_new_object();
	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);

	res = vmeta_frame_v1_recording_to_json(&m, jobj);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "location", &sub));
	CU_ASSERT_PTR_NOT_NULL_FATAL(sub);
	CU_ASSERT_TRUE(json_object_object_get_ex(sub, "latitude", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(
		json_object_get_double(tmp), m.location.latitude, 1e-9);

	CU_ASSERT_TRUE(
		json_object_object_get_ex(jobj, "frame_timestamp", &tmp));
	CU_ASSERT_EQUAL((uint64_t)json_object_get_int64(tmp),
			m.frame_timestamp);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "altitude", &tmp));
	CU_ASSERT_DOUBLE_EQUAL(json_object_get_double(tmp), m.altitude, 1e-9);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "state", &tmp));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(tmp),
			       vmeta_flying_state_str(m.state));
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "mode", &tmp));
	CU_ASSERT_STRING_EQUAL(json_object_get_string(tmp),
			       vmeta_piloting_mode_str(m.mode));

	json_object_put(jobj);
}


static void test_recording_to_csv(void)
{
	struct vmeta_frame_v1_recording m;
	char expected[CSV_SAFE_BUFLEN];
	char actual[CSV_SAFE_BUFLEN];
	size_t expected_len;
	size_t actual_len;

	fill_recording(&m);

	expected_len =
		build_expected_recording_csv(&m, expected, sizeof(expected));
	actual_len =
		vmeta_frame_v1_recording_to_csv(&m, actual, sizeof(actual));

	CU_ASSERT_EQUAL(actual_len, expected_len);
	CU_ASSERT_STRING_EQUAL(actual, expected);
}


static void test_recording_to_csv_small_maxlen(void)
{
	struct vmeta_frame_v1_recording m;
	char full[CSV_SAFE_BUFLEN];
	char truncated[CSV_SAFE_BUFLEN];
	size_t full_len;
	size_t truncated_len;

	fill_recording(&m);
	memset(full, 0, sizeof(full));
	memset(truncated, 0, sizeof(truncated));

	full_len = vmeta_frame_v1_recording_to_csv(&m, full, sizeof(full));
	truncated_len = vmeta_frame_v1_recording_to_csv(&m, truncated, 8);

	CU_ASSERT_EQUAL(truncated_len, full_len);
	CU_ASSERT_TRUE(strlen(truncated) < strlen(full));
	CU_ASSERT_EQUAL(strncmp(truncated, full, strlen(truncated)), 0);
	CU_ASSERT_TRUE(strlen(truncated) < 8);
}


static void test_recording_csv_header(void)
{
	char actual[CSV_SAFE_BUFLEN];
	size_t len;

	len = vmeta_frame_v1_recording_csv_header(actual, sizeof(actual));
	CU_ASSERT_EQUAL(len, strlen(s_recording_csv_header));
	CU_ASSERT_STRING_EQUAL(actual, s_recording_csv_header);
}


static void test_recording_csv_header_small_maxlen(void)
{
	char truncated[CSV_SAFE_BUFLEN];
	size_t full_len;
	size_t truncated_len;

	memset(truncated, 0, sizeof(truncated));

	full_len = strlen(s_recording_csv_header);
	truncated_len = vmeta_frame_v1_recording_csv_header(truncated, 10);

	CU_ASSERT_EQUAL(truncated_len, full_len);
	CU_ASSERT_EQUAL(strlen(truncated), 9);
	CU_ASSERT_EQUAL(strncmp(truncated, s_recording_csv_header, 9), 0);
}


CU_TestInfo s_v1_tests[] = {
	/* streaming basic */
	{(char *)"vmeta v1 streaming basic write/read", &test_basic_write_read},
	{(char *)"vmeta v1 streaming basic write too small buffer",
	 &test_basic_write_too_small},
	{(char *)"vmeta v1 streaming basic write NULL args",
	 &test_basic_write_null},
	{(char *)"vmeta v1 streaming basic read NULL args",
	 &test_basic_read_null},
	{(char *)"vmeta v1 streaming basic read too small buffer",
	 &test_basic_read_too_small},
	{(char *)"vmeta v1 streaming basic read bad id",
	 &test_basic_read_bad_id},
	{(char *)"vmeta v1 streaming basic to_json", &test_basic_to_json},
	{(char *)"vmeta v1 streaming basic to_csv", &test_basic_to_csv},
	{(char *)"vmeta v1 streaming basic to_csv small maxlen",
	 &test_basic_to_csv_small_maxlen},
	{(char *)"vmeta v1 streaming basic csv_header", &test_basic_csv_header},
	{(char *)"vmeta v1 streaming basic csv_header small maxlen",
	 &test_basic_csv_header_small_maxlen},

	/* streaming extended */
	{(char *)"vmeta v1 streaming extended write/read",
	 &test_extended_write_read},
	{(char *)"vmeta v1 streaming extended write/read invalid location",
	 &test_extended_write_read_invalid_location},
	{(char *)"vmeta v1 streaming extended write too small buffer",
	 &test_extended_write_too_small},
	{(char *)"vmeta v1 streaming extended write NULL args",
	 &test_extended_write_null},
	{(char *)"vmeta v1 streaming extended read NULL args",
	 &test_extended_read_null},
	{(char *)"vmeta v1 streaming extended read too small buffer",
	 &test_extended_read_too_small},
	{(char *)"vmeta v1 streaming extended read bad id",
	 &test_extended_read_bad_id},
	{(char *)"vmeta v1 streaming extended to_json", &test_extended_to_json},
	{(char *)"vmeta v1 streaming extended to_json invalid location",
	 &test_extended_to_json_invalid_location},
	{(char *)"vmeta v1 streaming extended to_csv", &test_extended_to_csv},
	{(char *)"vmeta v1 streaming extended to_csv small maxlen",
	 &test_extended_to_csv_small_maxlen},
	{(char *)"vmeta v1 streaming extended csv_header",
	 &test_extended_csv_header},

	/* recording */
	{(char *)"vmeta v1 recording write/read", &test_recording_write_read},
	{(char *)"vmeta v1 recording write/read invalid location",
	 &test_recording_write_read_invalid_location},
	{(char *)"vmeta v1 recording write too small buffer",
	 &test_recording_write_too_small},
	{(char *)"vmeta v1 recording write NULL args",
	 &test_recording_write_null},
	{(char *)"vmeta v1 recording read NULL args",
	 &test_recording_read_null},
	{(char *)"vmeta v1 recording read too small buffer",
	 &test_recording_read_too_small},
	{(char *)"vmeta v1 recording to_json", &test_recording_to_json},
	{(char *)"vmeta v1 recording to_csv", &test_recording_to_csv},
	{(char *)"vmeta v1 recording to_csv small maxlen",
	 &test_recording_to_csv_small_maxlen},
	{(char *)"vmeta v1 recording csv_header", &test_recording_csv_header},
	{(char *)"vmeta v1 recording csv_header small maxlen",
	 &test_recording_csv_header_small_maxlen},

	CU_TEST_INFO_NULL,
};
