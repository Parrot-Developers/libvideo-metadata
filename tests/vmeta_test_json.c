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
 * Real, assertion-based CUnit coverage for src/vmeta_json.c, i.e. the
 * non-inline struct-level vmeta_json_add_* helpers (vmeta_json_add_location,
 * _quaternion, _euler, _xyz, _xy, _ned, _fov, _thermal_conversion and
 * _thermal_spot). The primitive-type helpers (vmeta_json_add_bool/_int/
 * _int64/_double/_str) are 'static inline' in src/vmeta_json.h itself, not
 * part of vmeta_json.c, so they are only exercised here incidentally, as
 * building blocks of the struct-level helpers under test.
 */

#include "vmeta_test.h"

#include <math.h>

#include <json-c/json.h>

#include "vmeta_json.h"

#define JSON_GRANULARITY (0.0000001)


/* Small helper: fetch a double field and assert it is present */
static double get_double(struct json_object *jobj, const char *key)
{
	struct json_object *jval = NULL;
	int found = json_object_object_get_ex(jobj, key, &jval);
	CU_ASSERT_TRUE(found);
	if (!found)
		return 0.;
	return json_object_get_double(jval);
}


static int get_int(struct json_object *jobj, const char *key)
{
	struct json_object *jval = NULL;
	int found = json_object_object_get_ex(jobj, key, &jval);
	CU_ASSERT_TRUE(found);
	if (!found)
		return 0;
	return json_object_get_int(jval);
}


static void assert_key_absent(struct json_object *jobj, const char *key)
{
	struct json_object *jval = NULL;
	CU_ASSERT_FALSE(json_object_object_get_ex(jobj, key, &jval));
}


static void assert_key_present(struct json_object *jobj, const char *key)
{
	struct json_object *jval = NULL;
	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, key, &jval));
}


/* vmeta_json_add_quaternion: always writes an object with w/x/y/z */
static void test_json_add_quaternion(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_quaternion val = {
		.w = 0.5f,
		.x = -0.25f,
		.y = 0.125f,
		.z = 1.0f,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_quaternion(jobj, "quat", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "quat", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "w"), (double)val.w, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "x"), (double)val.x, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "y"), (double)val.y, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "z"), (double)val.z, JSON_GRANULARITY);

	json_object_put(jobj);
}


/* vmeta_json_add_euler: always writes an object with yaw/pitch/roll */
static void test_json_add_euler(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_euler val = {
		.yaw = 1.5f,
		.pitch = -2.25f,
		.roll = 3.125f,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_euler(jobj, "euler", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "euler", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "yaw"), (double)val.yaw, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "pitch"), (double)val.pitch, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "roll"), (double)val.roll, JSON_GRANULARITY);

	json_object_put(jobj);
}


/* vmeta_json_add_xyz: always writes an object with x/y/z */
static void test_json_add_xyz(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_xyz val = {
		.x = 1.5f,
		.y = -2.5f,
		.z = 4.25f,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_xyz(jobj, "xyz", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "xyz", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "x"), (double)val.x, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "y"), (double)val.y, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "z"), (double)val.z, JSON_GRANULARITY);

	json_object_put(jobj);
}


/* vmeta_json_add_xy: always writes an object with x/y (no z) */
static void test_json_add_xy(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_xy val = {
		.x = -3.5f,
		.y = 6.25f,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_xy(jobj, "xy", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "xy", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "x"), (double)val.x, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "y"), (double)val.y, JSON_GRANULARITY);

	/* Only x and y are expected in a vmeta_xy object */
	assert_key_absent(jval, "z");

	json_object_put(jobj);
}


/* vmeta_json_add_ned: always writes an object with north/east/down */
static void test_json_add_ned(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_ned val = {
		.north = 10.5f,
		.east = -20.25f,
		.down = 1.0f,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_ned(jobj, "ned", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "ned", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "north"), (double)val.north, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "east"), (double)val.east, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "down"), (double)val.down, JSON_GRANULARITY);

	json_object_put(jobj);
}


/* vmeta_json_add_fov: both has_horz/has_vert set writes both fields */
static void test_json_add_fov_both(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_fov val = {
		.horz = 90.5f,
		.vert = 60.25f,
		.has_horz = 1,
		.has_vert = 1,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_fov(jobj, "fov", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "fov", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "horz"), (double)val.horz, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "vert"), (double)val.vert, JSON_GRANULARITY);

	json_object_put(jobj);
}


/* vmeta_json_add_fov: only has_horz set writes only "horz" */
static void test_json_add_fov_horz_only(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_fov val = {
		.horz = 45.f,
		.vert = 0.f,
		.has_horz = 1,
		.has_vert = 0,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_fov(jobj, "fov", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "fov", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	assert_key_present(jval, "horz");
	assert_key_absent(jval, "vert");

	json_object_put(jobj);
}


/* vmeta_json_add_fov: only has_vert set writes only "vert" */
static void test_json_add_fov_vert_only(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_fov val = {
		.horz = 0.f,
		.vert = 33.f,
		.has_horz = 0,
		.has_vert = 1,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_fov(jobj, "fov", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "fov", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	assert_key_absent(jval, "horz");
	assert_key_present(jval, "vert");

	json_object_put(jobj);
}


/* vmeta_json_add_fov: neither has_horz nor has_vert => nothing written */
static void test_json_add_fov_none(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_fov val = {
		.horz = 0.f,
		.vert = 0.f,
		.has_horz = 0,
		.has_vert = 0,
	};
	int res;

	res = vmeta_json_add_fov(jobj, "fov", &val);
	CU_ASSERT_EQUAL(res, 0);

	assert_key_absent(jobj, "fov");
	CU_ASSERT_EQUAL(json_object_object_length(jobj), 0);

	json_object_put(jobj);
}


/* vmeta_json_add_location: valid == 0 => nothing written at all */
static void test_json_add_location_invalid(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_location val = {
		.latitude = 48.8566,
		.longitude = 2.3522,
		.altitude_wgs84ellipsoid = 100.,
		.altitude_egm96amsl = 90.,
		.horizontal_accuracy = 1.5f,
		.vertical_accuracy = 2.5f,
		.sv_count = 12,
		.valid = 0,
	};
	int res;

	res = vmeta_json_add_location(jobj, "location", &val);
	CU_ASSERT_EQUAL(res, 0);

	assert_key_absent(jobj, "location");
	CU_ASSERT_EQUAL(json_object_object_length(jobj), 0);

	json_object_put(jobj);
}


/*
 * vmeta_json_add_location: valid == 1, all optional fields populated
 * (finite altitudes, non-zero accuracies, real sv_count) => every field
 * present.
 */
static void test_json_add_location_valid_full(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_location val = {
		.latitude = 48.8566,
		.longitude = 2.3522,
		.altitude_wgs84ellipsoid = 100.25,
		.altitude_egm96amsl = 90.5,
		.horizontal_accuracy = 1.5f,
		.vertical_accuracy = 2.5f,
		.sv_count = 12,
		.valid = 1,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_location(jobj, "location", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "location", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "latitude"), val.latitude, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "longitude"), val.longitude, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(get_double(jval, "altitude_wgs84ellipsoid"),
			       val.altitude_wgs84ellipsoid,
			       JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(get_double(jval, "altitude_egm96amsl"),
			       val.altitude_egm96amsl,
			       JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(get_double(jval, "horizontal_accuracy"),
			       (double)val.horizontal_accuracy,
			       JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(get_double(jval, "vertical_accuracy"),
			       (double)val.vertical_accuracy,
			       JSON_GRANULARITY);
	CU_ASSERT_EQUAL(get_int(jval, "sv_count"), val.sv_count);

	json_object_put(jobj);
}


/*
 * vmeta_json_add_location: valid == 1 but every optional field is at its
 * "unknown" sentinel (NaN altitudes, zero accuracies, invalid sv_count)
 * => only latitude/longitude are written.
 */
static void test_json_add_location_valid_minimal(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_location val = {
		.latitude = -33.8688,
		.longitude = 151.2093,
		.altitude_wgs84ellipsoid = NAN,
		.altitude_egm96amsl = NAN,
		.horizontal_accuracy = 0.f,
		.vertical_accuracy = 0.f,
		.sv_count = VMETA_LOCATION_INVALID_SV_COUNT,
		.valid = 1,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_location(jobj, "location", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "location", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "latitude"), val.latitude, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "longitude"), val.longitude, JSON_GRANULARITY);

	assert_key_absent(jval, "altitude_wgs84ellipsoid");
	assert_key_absent(jval, "altitude_egm96amsl");
	assert_key_absent(jval, "horizontal_accuracy");
	assert_key_absent(jval, "vertical_accuracy");
	assert_key_absent(jval, "sv_count");

	/* Only latitude and longitude should be present */
	CU_ASSERT_EQUAL(json_object_object_length(jval), 2);

	json_object_put(jobj);
}


/* vmeta_json_add_thermal_conversion: valid == 0 => nothing written */
static void test_json_add_thermal_conversion_invalid(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_thermal_conversion val = {
		.r = 1.f,
		.b = 2.f,
		.f = 3.f,
		.o = 4.f,
		.tau_win = 0.9f,
		.t_win = 20.f,
		.t_bg = 15.f,
		.emissivity = 0.95f,
		.valid = 0,
	};
	int res;

	res = vmeta_json_add_thermal_conversion(jobj, "conv", &val);
	CU_ASSERT_EQUAL(res, 0);

	assert_key_absent(jobj, "conv");
	CU_ASSERT_EQUAL(json_object_object_length(jobj), 0);

	json_object_put(jobj);
}


/* vmeta_json_add_thermal_conversion: valid == 1 => all fields written */
static void test_json_add_thermal_conversion_valid(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_thermal_conversion val = {
		.r = 1.5f,
		.b = 2.5f,
		.f = 3.5f,
		.o = 4.5f,
		.tau_win = 0.9f,
		.t_win = 20.25f,
		.t_bg = 15.75f,
		.emissivity = 0.95f,
		.valid = 1,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_thermal_conversion(jobj, "conv", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "conv", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "r"), (double)val.r, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "b"), (double)val.b, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "f"), (double)val.f, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "o"), (double)val.o, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(get_double(jval, "tau_win"),
			       (double)val.tau_win,
			       JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "t_win"), (double)val.t_win, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "t_bg"), (double)val.t_bg, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(get_double(jval, "emissivity"),
			       (double)val.emissivity,
			       JSON_GRANULARITY);

	json_object_put(jobj);
}


/* vmeta_json_add_thermal_spot: valid == 0 => nothing written */
static void test_json_add_thermal_spot_invalid(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_thermal_spot val = {
		.x = 0.5f,
		.y = 0.5f,
		.temp = 310.f,
		.value = 1234,
		.valid = 0,
	};
	int res;

	res = vmeta_json_add_thermal_spot(jobj, "spot", &val);
	CU_ASSERT_EQUAL(res, 0);

	assert_key_absent(jobj, "spot");
	CU_ASSERT_EQUAL(json_object_object_length(jobj), 0);

	json_object_put(jobj);
}


/* vmeta_json_add_thermal_spot: valid == 1 => x/y/temp/value all written */
static void test_json_add_thermal_spot_valid(void)
{
	struct json_object *jobj = json_object_new_object();
	struct vmeta_thermal_spot val = {
		.x = 0.25f,
		.y = 0.75f,
		.temp = 305.5f,
		.value = 4321,
		.valid = 1,
	};
	struct json_object *jval = NULL;
	int res;

	res = vmeta_json_add_thermal_spot(jobj, "spot", &val);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_TRUE(json_object_object_get_ex(jobj, "spot", &jval));
	CU_ASSERT_PTR_NOT_NULL(jval);

	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "x"), (double)val.x, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "y"), (double)val.y, JSON_GRANULARITY);
	CU_ASSERT_DOUBLE_EQUAL(
		get_double(jval, "temp"), (double)val.temp, JSON_GRANULARITY);
	CU_ASSERT_EQUAL(get_int(jval, "value"), val.value);

	json_object_put(jobj);
}


CU_TestInfo s_json_tests[] = {
	{(char *)"json_add_quaternion", &test_json_add_quaternion},
	{(char *)"json_add_euler", &test_json_add_euler},
	{(char *)"json_add_xyz", &test_json_add_xyz},
	{(char *)"json_add_xy", &test_json_add_xy},
	{(char *)"json_add_ned", &test_json_add_ned},
	{(char *)"json_add_fov_both", &test_json_add_fov_both},
	{(char *)"json_add_fov_horz_only", &test_json_add_fov_horz_only},
	{(char *)"json_add_fov_vert_only", &test_json_add_fov_vert_only},
	{(char *)"json_add_fov_none", &test_json_add_fov_none},
	{(char *)"json_add_location_invalid", &test_json_add_location_invalid},
	{(char *)"json_add_location_valid_full",
	 &test_json_add_location_valid_full},
	{(char *)"json_add_location_valid_minimal",
	 &test_json_add_location_valid_minimal},
	{(char *)"json_add_thermal_conversion_invalid",
	 &test_json_add_thermal_conversion_invalid},
	{(char *)"json_add_thermal_conversion_valid",
	 &test_json_add_thermal_conversion_valid},
	{(char *)"json_add_thermal_spot_invalid",
	 &test_json_add_thermal_spot_invalid},
	{(char *)"json_add_thermal_spot_valid",
	 &test_json_add_thermal_spot_valid},
	CU_TEST_INFO_NULL,
};
