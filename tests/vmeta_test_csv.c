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

/*
 * Real, assertion-based CUnit coverage for two previously 0%-covered files:
 *
 *  - src/vmeta_csv.c: the vmeta_csv_add_* CSV-formatting helpers
 *    (vmeta_csv_add_location/_quaternion/_euler/_xyz/_ned/_thermal_spot).
 *    Every helper is a thin wrapper around a single VMETA_STR_PRINT()
 *    (i.e. a single snprintf() call), so the expected output string content
 *    is derived directly from the exact format string / argument order read
 *    from src/vmeta_csv.c, using "nice" (exactly binary-representable)
 *    float/double literals so the %f/%lf renderings below are exact and not
 *    subject to rounding ambiguity.
 *
 *  - src/vmeta_proto.c: the two vmeta_camera_subtype_{vmeta_to_proto,
 *    proto_to_vmeta} enum conversions.
 *
 * Only "vmeta_csv.h" is included directly (it is a private header, declaring
 * functions with no public entry point, exactly like vmeta_json.h in
 * vmeta_test_json.c); the vmeta_proto.c functions are declared in the public
 * include/video-metadata/vmeta_proto.h, transitively reachable from
 * "vmeta_test.h" -> <video-metadata/vmeta.h> -> vmeta_frame.h ->
 * vmeta_frame_proto.h -> vmeta_proto.h, exactly the same way
 * tests/vmeta_test_proto.c already reaches the Vmeta__* protobuf-c types
 * without any extra include.
 */

#include "vmeta_test.h"

#include <string.h>

#include "vmeta_csv.h"

#define CSV_SENTINEL_BYTE 0xAA


/*
 * Fill 'buf' with a sentinel byte pattern before a truncating call, so that
 * any write past the requested 'maxlen' can be detected afterwards.
 */
static void sentinel_fill(char *buf, size_t buflen)
{
	memset(buf, CSV_SENTINEL_BYTE, buflen);
}


/*
 * Common assertions for a "maxlen too small" call: given the untruncated
 * 'full' string already produced (and validated) with a large buffer, redo
 * the same call into 'buf' (of size 'buflen', pre-filled with the sentinel
 * pattern) with a small 'maxlen' (< buflen), and check:
 *  - the return value is the *untruncated* length (snprintf()-style: the
 *    length that would have been written had the buffer been large enough),
 *    matching VMETA_STR_PRINT()'s "_len += snprintf(...)" implementation;
 *  - the first (maxlen - 1) characters written match the untruncated string;
 *  - a NUL terminator is written at buf[maxlen - 1];
 *  - nothing at or past buf[maxlen] was touched.
 */
static void check_truncated_result(const char *full,
				   size_t full_res,
				   char *buf,
				   size_t buflen,
				   size_t maxlen,
				   size_t trunc_res)
{
	size_t i;

	CU_ASSERT_EQUAL(trunc_res, full_res);
	CU_ASSERT_EQUAL(strncmp(buf, full, maxlen - 1), 0);
	CU_ASSERT_EQUAL(buf[maxlen - 1], '\0');
	for (i = maxlen; i < buflen; i++)
		CU_ASSERT_EQUAL((unsigned char)buf[i], CSV_SENTINEL_BYTE);
}


/* vmeta_csv_add_quaternion: "%.5f %.5f %.5f %.5f" of w, x, y, z */
static void test_csv_add_quaternion(void)
{
	struct vmeta_quaternion val = {
		.w = 1.5f,
		.x = -2.25f,
		.y = 0.125f,
		.z = 3.75f,
	};
	char buf[64];
	size_t res;

	res = vmeta_csv_add_quaternion(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "1.50000 -2.25000 0.12500 3.75000");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_quaternion_truncated(void)
{
	struct vmeta_quaternion val = {
		.w = 1.5f,
		.x = -2.25f,
		.y = 0.125f,
		.z = 3.75f,
	};
	char full[64];
	char buf[64];
	size_t maxlen = 6;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_quaternion(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(full, "1.50000 -2.25000 0.12500 3.75000");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_quaternion(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


/* vmeta_csv_add_euler: "%.4f %.4f %.4f" of yaw, pitch, roll */
static void test_csv_add_euler(void)
{
	struct vmeta_euler val = {
		.yaw = 1.5f,
		.pitch = -2.25f,
		.roll = 0.125f,
	};
	char buf[64];
	size_t res;

	res = vmeta_csv_add_euler(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "1.5000 -2.2500 0.1250");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_euler_truncated(void)
{
	struct vmeta_euler val = {
		.yaw = 1.5f,
		.pitch = -2.25f,
		.roll = 0.125f,
	};
	char full[64];
	char buf[64];
	size_t maxlen = 5;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_euler(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(full, "1.5000 -2.2500 0.1250");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_euler(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


/* vmeta_csv_add_xyz: "%.3f %.3f %.3f" of x, y, z */
static void test_csv_add_xyz(void)
{
	struct vmeta_xyz val = {
		.x = 1.5f,
		.y = -2.25f,
		.z = 0.125f,
	};
	char buf[64];
	size_t res;

	res = vmeta_csv_add_xyz(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "1.500 -2.250 0.125");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_xyz_truncated(void)
{
	struct vmeta_xyz val = {
		.x = 1.5f,
		.y = -2.25f,
		.z = 0.125f,
	};
	char full[64];
	char buf[64];
	size_t maxlen = 4;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_xyz(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(full, "1.500 -2.250 0.125");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_xyz(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


/* vmeta_csv_add_ned: "%.3f %.3f %.3f" of north, east, down */
static void test_csv_add_ned(void)
{
	struct vmeta_ned val = {
		.north = 1.5f,
		.east = -2.25f,
		.down = 0.125f,
	};
	char buf[64];
	size_t res;

	res = vmeta_csv_add_ned(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "1.500 -2.250 0.125");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_ned_truncated(void)
{
	struct vmeta_ned val = {
		.north = 1.5f,
		.east = -2.25f,
		.down = 0.125f,
	};
	char full[64];
	char buf[64];
	size_t maxlen = 3;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_ned(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(full, "1.500 -2.250 0.125");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_ned(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


/*
 * vmeta_csv_add_thermal_spot: valid == 1 uses
 * "%d %.5f %.5f %.5f %" PRId32 with valid/x/y/temp/value.
 */
static void test_csv_add_thermal_spot_valid(void)
{
	struct vmeta_thermal_spot val = {
		.x = 0.5f,
		.y = 0.25f,
		.temp = 310.5f,
		.value = 12345,
		.valid = 1,
	};
	char buf[64];
	size_t res;

	res = vmeta_csv_add_thermal_spot(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "1 0.50000 0.25000 310.50000 12345");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_thermal_spot_valid_truncated(void)
{
	struct vmeta_thermal_spot val = {
		.x = 0.5f,
		.y = 0.25f,
		.temp = 310.5f,
		.value = 12345,
		.valid = 1,
	};
	char full[64];
	char buf[64];
	size_t maxlen = 8;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_thermal_spot(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(full, "1 0.50000 0.25000 310.50000 12345");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_thermal_spot(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


/*
 * vmeta_csv_add_thermal_spot: valid == 0 always renders the fixed
 * "0 0.00000 0.00000 0.00000 0" placeholder string, regardless of the
 * actual (ignored) field values.
 */
static void test_csv_add_thermal_spot_invalid(void)
{
	struct vmeta_thermal_spot val = {
		.x = 0.9f,
		.y = 0.9f,
		.temp = 999.f,
		.value = 999,
		.valid = 0,
	};
	char buf[64];
	size_t res;

	res = vmeta_csv_add_thermal_spot(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "0 0.00000 0.00000 0.00000 0");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


/*
 * vmeta_csv_add_location: valid == 1 uses
 * "%d %.8lf %.8lf %.2lf %.2lf %.2f %.2f %d" with valid/latitude/longitude/
 * altitude_wgs84ellipsoid/altitude_egm96amsl/horizontal_accuracy/
 * vertical_accuracy/sv_count (or 0 if sv_count == VMETA_LOCATION_INVALID_
 * SV_COUNT).
 */
static void test_csv_add_location_valid_full(void)
{
	struct vmeta_location val = {
		.latitude = 48.5,
		.longitude = -2.25,
		.altitude_wgs84ellipsoid = 100.25,
		.altitude_egm96amsl = 90.5,
		.horizontal_accuracy = 1.5f,
		.vertical_accuracy = 2.25f,
		.sv_count = 12,
		.valid = 1,
	};
	char buf[80];
	size_t res;

	res = vmeta_csv_add_location(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(
		buf, "1 48.50000000 -2.25000000 100.25 90.50 1.50 2.25 12");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_location_valid_truncated(void)
{
	struct vmeta_location val = {
		.latitude = 48.5,
		.longitude = -2.25,
		.altitude_wgs84ellipsoid = 100.25,
		.altitude_egm96amsl = 90.5,
		.horizontal_accuracy = 1.5f,
		.vertical_accuracy = 2.25f,
		.sv_count = 12,
		.valid = 1,
	};
	char full[80];
	char buf[80];
	size_t maxlen = 10;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_location(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(
		full, "1 48.50000000 -2.25000000 100.25 90.50 1.50 2.25 12");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_location(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


/*
 * vmeta_csv_add_location: valid == 1 but sv_count ==
 * VMETA_LOCATION_INVALID_SV_COUNT renders 0 in the last field instead of
 * the (meaningless) raw sv_count value.
 */
static void test_csv_add_location_valid_invalid_sv_count(void)
{
	struct vmeta_location val = {
		.latitude = 48.5,
		.longitude = -2.25,
		.altitude_wgs84ellipsoid = 100.25,
		.altitude_egm96amsl = 90.5,
		.horizontal_accuracy = 1.5f,
		.vertical_accuracy = 2.25f,
		.sv_count = VMETA_LOCATION_INVALID_SV_COUNT,
		.valid = 1,
	};
	char buf[80];
	size_t res;

	res = vmeta_csv_add_location(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(
		buf, "1 48.50000000 -2.25000000 100.25 90.50 1.50 2.25 0");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


/*
 * vmeta_csv_add_location: valid == 0 always renders a fixed placeholder
 * string, regardless of the actual (ignored) field values.
 *
 * Note: unlike every other vmeta_csv_add_* helper (and unlike this same
 * function's own valid == 1 branch, which emits 8 space-separated fields),
 * the valid == 0 branch's format string ("%d %.8lf %.8lf %.2lf %.2f %.2f
 * %d") only has 7 conversion specifiers -- one of the two %.2lf altitude
 * fields present in the valid branch is missing here. This looks like a
 * genuine (if harmless, since it is a fixed placeholder either way)
 * asymmetry/bug in src/vmeta_csv.c; this test pins down the *actual*
 * observed behavior rather than the presumably-intended 8-field output.
 */
static void test_csv_add_location_invalid(void)
{
	struct vmeta_location val = {
		.latitude = 99.9,
		.longitude = 99.9,
		.altitude_wgs84ellipsoid = 99.9,
		.altitude_egm96amsl = 99.9,
		.horizontal_accuracy = 99.9f,
		.vertical_accuracy = 99.9f,
		.sv_count = 99,
		.valid = 0,
	};
	char buf[80];
	size_t res;

	res = vmeta_csv_add_location(buf, sizeof(buf), &val);
	CU_ASSERT_STRING_EQUAL(buf, "0 0.00000000 0.00000000 0.00 0.00 0.00 0");
	CU_ASSERT_EQUAL(res, strlen(buf));
}


static void test_csv_add_location_invalid_truncated(void)
{
	struct vmeta_location val = {
		.latitude = 99.9,
		.longitude = 99.9,
		.altitude_wgs84ellipsoid = 99.9,
		.altitude_egm96amsl = 99.9,
		.horizontal_accuracy = 99.9f,
		.vertical_accuracy = 99.9f,
		.sv_count = 99,
		.valid = 0,
	};
	char full[80];
	char buf[80];
	size_t maxlen = 5;
	size_t res_full, res_trunc;

	res_full = vmeta_csv_add_location(full, sizeof(full), &val);
	CU_ASSERT_STRING_EQUAL(full,
			       "0 0.00000000 0.00000000 0.00 0.00 0.00 0");

	sentinel_fill(buf, sizeof(buf));
	res_trunc = vmeta_csv_add_location(buf, maxlen, &val);

	check_truncated_result(
		full, res_full, buf, sizeof(buf), maxlen, res_trunc);
}


CU_TestInfo s_csv_tests[] = {
	{(char *)"csv_add_quaternion", &test_csv_add_quaternion},
	{(char *)"csv_add_quaternion_truncated",
	 &test_csv_add_quaternion_truncated},
	{(char *)"csv_add_euler", &test_csv_add_euler},
	{(char *)"csv_add_euler_truncated", &test_csv_add_euler_truncated},
	{(char *)"csv_add_xyz", &test_csv_add_xyz},
	{(char *)"csv_add_xyz_truncated", &test_csv_add_xyz_truncated},
	{(char *)"csv_add_ned", &test_csv_add_ned},
	{(char *)"csv_add_ned_truncated", &test_csv_add_ned_truncated},
	{(char *)"csv_add_thermal_spot_valid",
	 &test_csv_add_thermal_spot_valid},
	{(char *)"csv_add_thermal_spot_valid_truncated",
	 &test_csv_add_thermal_spot_valid_truncated},
	{(char *)"csv_add_thermal_spot_invalid",
	 &test_csv_add_thermal_spot_invalid},
	{(char *)"csv_add_location_valid_full",
	 &test_csv_add_location_valid_full},
	{(char *)"csv_add_location_valid_truncated",
	 &test_csv_add_location_valid_truncated},
	{(char *)"csv_add_location_valid_invalid_sv_count",
	 &test_csv_add_location_valid_invalid_sv_count},
	{(char *)"csv_add_location_invalid", &test_csv_add_location_invalid},
	{(char *)"csv_add_location_invalid_truncated",
	 &test_csv_add_location_invalid_truncated},
	CU_TEST_INFO_NULL,
};


/*
 * vmeta_proto.c: vmeta_camera_subtype_{vmeta_to_proto,proto_to_vmeta}.
 *
 * Table-driven round trip over every named enum value on both sides
 * (mirroring the exact switch cases in src/vmeta_proto.c), plus a
 * dedicated test for the "unrecognized input" default case on each
 * direction.
 */
struct camera_subtype_pair {
	enum vmeta_camera_subtype vmeta;
	Vmeta__CameraSubtype proto;
};

static const struct camera_subtype_pair s_camera_subtype_pairs[] = {
	{VMETA_CAMERA_SUBTYPE_UNKNOWN, VMETA__CAMERA_SUBTYPE__CST_UNKNOWN},
	{VMETA_CAMERA_SUBTYPE_LEFT, VMETA__CAMERA_SUBTYPE__CST_LEFT},
	{VMETA_CAMERA_SUBTYPE_RIGHT, VMETA__CAMERA_SUBTYPE__CST_RIGHT},
	{VMETA_CAMERA_SUBTYPE_WIDE, VMETA__CAMERA_SUBTYPE__CST_WIDE},
	{VMETA_CAMERA_SUBTYPE_TELE, VMETA__CAMERA_SUBTYPE__CST_TELE},
	{VMETA_CAMERA_SUBTYPE_DISPARITY, VMETA__CAMERA_SUBTYPE__CST_DISPARITY},
	{VMETA_CAMERA_SUBTYPE_DEPTH, VMETA__CAMERA_SUBTYPE__CST_DEPTH},
};

#define CAMERA_SUBTYPE_PAIR_COUNT                                              \
	(sizeof(s_camera_subtype_pairs) / sizeof(s_camera_subtype_pairs[0]))


static void test_proto_camera_subtype_vmeta_to_proto(void)
{
	size_t i;

	for (i = 0; i < CAMERA_SUBTYPE_PAIR_COUNT; i++) {
		Vmeta__CameraSubtype res = vmeta_camera_subtype_vmeta_to_proto(
			s_camera_subtype_pairs[i].vmeta);
		CU_ASSERT_EQUAL(res, s_camera_subtype_pairs[i].proto);
	}
}


static void test_proto_camera_subtype_proto_to_vmeta(void)
{
	size_t i;

	for (i = 0; i < CAMERA_SUBTYPE_PAIR_COUNT; i++) {
		enum vmeta_camera_subtype res =
			vmeta_camera_subtype_proto_to_vmeta(
				s_camera_subtype_pairs[i].proto);
		CU_ASSERT_EQUAL(res, s_camera_subtype_pairs[i].vmeta);
	}
}


static void test_proto_camera_subtype_round_trip(void)
{
	size_t i;

	for (i = 0; i < CAMERA_SUBTYPE_PAIR_COUNT; i++) {
		enum vmeta_camera_subtype orig =
			s_camera_subtype_pairs[i].vmeta;
		Vmeta__CameraSubtype proto =
			vmeta_camera_subtype_vmeta_to_proto(orig);
		enum vmeta_camera_subtype back =
			vmeta_camera_subtype_proto_to_vmeta(proto);
		CU_ASSERT_EQUAL(back, orig);
	}
}


/*
 * Values with no matching switch case fall through to the default branch
 * on each side, which yields the respective "unknown" sentinel.
 */
static void test_proto_camera_subtype_vmeta_to_proto_unknown_input(void)
{
	Vmeta__CameraSubtype res = vmeta_camera_subtype_vmeta_to_proto(
		(enum vmeta_camera_subtype)99);
	CU_ASSERT_EQUAL(res, VMETA__CAMERA_SUBTYPE__CST_UNKNOWN);
}


static void test_proto_camera_subtype_proto_to_vmeta_unknown_input(void)
{
	enum vmeta_camera_subtype res =
		vmeta_camera_subtype_proto_to_vmeta((Vmeta__CameraSubtype)99);
	CU_ASSERT_EQUAL(res, VMETA_CAMERA_SUBTYPE_UNKNOWN);
}


CU_TestInfo s_proto_conv_tests[] = {
	{(char *)"proto_camera_subtype_vmeta_to_proto",
	 &test_proto_camera_subtype_vmeta_to_proto},
	{(char *)"proto_camera_subtype_proto_to_vmeta",
	 &test_proto_camera_subtype_proto_to_vmeta},
	{(char *)"proto_camera_subtype_round_trip",
	 &test_proto_camera_subtype_round_trip},
	{(char *)"proto_camera_subtype_vmeta_to_proto_unknown_input",
	 &test_proto_camera_subtype_vmeta_to_proto_unknown_input},
	{(char *)"proto_camera_subtype_proto_to_vmeta_unknown_input",
	 &test_proto_camera_subtype_proto_to_vmeta_unknown_input},
	CU_TEST_INFO_NULL,
};
