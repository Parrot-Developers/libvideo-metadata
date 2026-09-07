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
#include <ctype.h>
#include <float.h>

#define TEST_EPSILON (0.000001f)


#define RESET_VALUES(_meta1, _copy, _meta2)                                    \
	do {                                                                   \
		(_copy) = (meta1);                                             \
		(_meta2) = (_copy);                                            \
	} while (0)

/**
 * Helper to test float comparison robustness:
 * 1. Equality within epsilon
 * 2. Inequality outside epsilon
 * 3. Sign parity for zeros (-0.0 vs 0.0)
 * 4. NaN behavior (should not be equal)
 */
#define ASSERT_DOUBLE_CMP_ROBUST(_cmp_func, _meta1, _meta2, _field, _epsilon)  \
	do {                                                                   \
		__typeof__(_meta1) _meta1_copy;                                \
		/* 1. Perfect equality */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
		/* 2. Equality within _epsilon */                              \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + _epsilon;             \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
		/* 3. Inequality outside _epsilon */                           \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + (_epsilon * 2.0);     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 0);      \
		/* 4. Zero sign parity */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta1_copy)._field = 0.0;                                    \
		(_meta2)._field = -0.0;                                        \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
		/* 5. NaN (Never equal) */                                     \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta1_copy)._field = NAN;                                    \
		(_meta2)._field = NAN;                                         \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 0);      \
	} while (0)


#define ASSERT_DOUBLE_CMP_TREAT_NAN_AS_UNKNOWN(                                \
	_cmp_func, _meta1, _meta2, _field, _epsilon)                           \
	do {                                                                   \
		__typeof__(_meta1) _meta1_copy;                                \
		/* 1. Perfect equality */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
		/* 2. Equality within _epsilon */                              \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + _epsilon;             \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
		/* 3. Inequality outside _epsilon */                           \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + (_epsilon * 2.0);     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 0);      \
		/* 4. Zero sign parity */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta1_copy)._field = 0.0;                                    \
		(_meta2)._field = -0.0;                                        \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
		/* 5. NaN (treated as equal) */                                \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta1_copy)._field = NAN;                                    \
		(_meta2)._field = NAN;                                         \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
	} while (0)


static const double EPS = DBL_EPSILON;


static bool quat_are_equal(const struct vmeta_quaternion *q1,
			   const struct vmeta_quaternion *q2,
			   float threshold)
{
	if (!q1 || !q2)
		return false;

	return (fabsf(q1->w - q2->w) <= threshold) &&
	       (fabsf(q1->x - q2->x) <= threshold) &&
	       (fabsf(q1->y - q2->y) <= threshold) &&
	       (fabsf(q1->z - q2->z) <= threshold);
}


static bool euler_are_equal(const struct vmeta_euler *e1,
			    const struct vmeta_euler *e2,
			    float threshold)
{
	if (!e1 || !e2)
		return false;

	return (fabsf(e1->psi - e2->psi) <= threshold) &&
	       (fabsf(e1->theta - e2->theta) <= threshold) &&
	       (fabsf(e1->phi - e2->phi) <= threshold);
}


static void test_euler_to_quat(void)
{
	const struct vmeta_euler euler = {
		.psi = 0.02f,
		.theta = -1.22f,
		.phi = 0.87f,
	};

	struct vmeta_quaternion quat_res = {0};

	const struct vmeta_quaternion quat_comp = {
		.w = 0.740862984384455f,
		.x = 0.350586006705661f,
		.y = -0.516036337738762f,
		.z = 0.248833254217165f,
	};

	vmeta_euler_to_quat(&euler, &quat_res);
	bool comp = quat_are_equal(&quat_res, &quat_comp, TEST_EPSILON);
	CU_ASSERT_TRUE(comp);
}


static void test_quat_to_euler(void)
{
	bool comp;
	struct vmeta_euler euler_res;

	/* Quaternion whose three angles stay in [-pi/2, pi/2], and
	 * without singularities */
	const struct vmeta_quaternion quat1 = {
		.w = 0.740862984384455f,
		.x = 0.350586006705661f,
		.y = -0.516036337738762f,
		.z = 0.248833254217165f,
	};
	const struct vmeta_euler euler_comp1 = {
		.psi = 0.02f,
		.theta = -1.22f,
		.phi = 0.87f,
	};
	memset(&euler_res, 0, sizeof(euler_res));
	vmeta_quat_to_euler(&quat1, &euler_res);
	comp = euler_are_equal(&euler_res, &euler_comp1, TEST_EPSILON);
	CU_ASSERT_TRUE(comp);

	/* Quaternion with at least one angle out of [-pi/2, pi/2], and
	 * without singularities */
	const struct vmeta_quaternion quat2 = {
		.w = -0.497493760429674f,
		.x = -0.502493739596367f,
		.y = -0.502493739596367f,
		.z = 0.497493760429674f,
	};
	const struct vmeta_euler euler_comp2 = {
		.psi = M_PI / 2.f,
		.theta = M_PI / 2.f - 0.01f,
		.phi = M_PI,
	};
	memset(&euler_res, 0, sizeof(euler_res));
	vmeta_quat_to_euler(&quat2, &euler_res);
	/* Precision is not great here; MATLAB gives a different
	 * result, due to another implementation of atan2 */
	comp = euler_are_equal(&euler_res, &euler_comp2, TEST_EPSILON * 10.f);
	CU_ASSERT_TRUE(comp);

	/* Quaternions at singularities */
	const struct vmeta_quaternion quat3 = {
		/* pi/2 around Y, then 2pi/3 around X */
		.w = 0.353553390593274f,
		.x = 0.612372435695794f,
		.y = 0.353553390593274f,
		.z = -0.612372435695794f,
	};
	const struct vmeta_quaternion quat4 = {
		/* -pi/2 around Y, then 2pi/3 around X */
		.w = 0.353553390593274f,
		.x = 0.612372435695794f,
		.y = -0.353553390593274f,
		.z = 0.612372435695794f,
	};
	const struct vmeta_euler euler_comp3 = {
		.psi = 0.f,
		.theta = M_PI / 2.f,
		.phi = 2.f * M_PI / 3.f,
	};
	const struct vmeta_euler euler_comp4 = {
		.psi = 0.f,
		.theta = -M_PI / 2.f,
		.phi = 2.f * M_PI / 3.f,
	};
	memset(&euler_res, 0, sizeof(euler_res));
	vmeta_quat_to_euler(&quat3, &euler_res);
	comp = euler_are_equal(&euler_res, &euler_comp3, TEST_EPSILON);
	CU_ASSERT_TRUE(comp);
	memset(&euler_res, 0, sizeof(euler_res));
	vmeta_quat_to_euler(&quat4, &euler_res);
	comp = euler_are_equal(&euler_res, &euler_comp4, TEST_EPSILON);
	CU_ASSERT_TRUE(comp);
}


#define MAKE_TYPE_SPLIT_MAP(t1, t2, st2, ret)                                  \
	{                                                                      \
		VMETA_CAMERA_TYPE_##t1, VMETA_CAMERA_TYPE_##t2,                \
			VMETA_CAMERA_SUBTYPE_##st2, ret                        \
	}

#define MAKE_TYPE_SPLIT_MAP_OK(t1, t2, st2) MAKE_TYPE_SPLIT_MAP(t1, t2, st2, 0)

#define MAKE_TYPE_SPLIT_MAP_KO(t1) MAKE_TYPE_SPLIT_MAP(t1, t1, UNKNOWN, -ENOENT)


static void test_camera_type_split_subtype(void)
{
	int ret;
	enum vmeta_camera_type got_type;
	enum vmeta_camera_subtype got_subtype;

	static struct {
		enum vmeta_camera_type type;
		enum vmeta_camera_type expected_type;
		enum vmeta_camera_subtype expected_subtype;
		int ret;
	} type_split_map[] = {
		/* KO */
		MAKE_TYPE_SPLIT_MAP_KO(FRONT),
		MAKE_TYPE_SPLIT_MAP_KO(FRONT_STEREO),
		MAKE_TYPE_SPLIT_MAP_KO(HORIZONTAL_STEREO),
		MAKE_TYPE_SPLIT_MAP_KO(DOWN_STEREO),
		MAKE_TYPE_SPLIT_MAP_KO(UNKNOWN),
		/* Types that can never appear as a ".combined" entry in
		 * s_camera_type_map[] (vmeta_utils.c), so they always fall
		 * through to the -ENOENT passthrough branch */
		MAKE_TYPE_SPLIT_MAP_KO(VERTICAL),
		MAKE_TYPE_SPLIT_MAP_KO(DISPARITY),
		MAKE_TYPE_SPLIT_MAP_KO(EXTERNAL),

		/* OK */
		MAKE_TYPE_SPLIT_MAP_OK(FRONT_STEREO_LEFT, FRONT_STEREO, LEFT),
		MAKE_TYPE_SPLIT_MAP_OK(FRONT_STEREO_RIGHT, FRONT_STEREO, RIGHT),
		MAKE_TYPE_SPLIT_MAP_OK(
			HORIZONTAL_STEREO_LEFT, HORIZONTAL_STEREO, LEFT),
		MAKE_TYPE_SPLIT_MAP_OK(
			HORIZONTAL_STEREO_RIGHT, HORIZONTAL_STEREO, RIGHT),
		MAKE_TYPE_SPLIT_MAP_OK(DOWN_STEREO_LEFT, DOWN_STEREO, LEFT),
		MAKE_TYPE_SPLIT_MAP_OK(DOWN_STEREO_RIGHT, DOWN_STEREO, RIGHT),
	};

	ret = vmeta_camera_type_split_subtype(
		VMETA_CAMERA_TYPE_UNKNOWN, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_camera_type_split_subtype(
		VMETA_CAMERA_TYPE_UNKNOWN, &got_type, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_camera_type_split_subtype(
		VMETA_CAMERA_TYPE_UNKNOWN, NULL, &got_subtype);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	for (size_t i = 0; i < SIZEOF_ARRAY(type_split_map); i++) {
		ret = vmeta_camera_type_split_subtype(
			type_split_map[i].type, &got_type, &got_subtype);

		CU_ASSERT_EQUAL(ret, type_split_map[i].ret);
		CU_ASSERT_EQUAL(got_type, type_split_map[i].expected_type);
		CU_ASSERT_EQUAL(got_subtype,
				type_split_map[i].expected_subtype);
	}
}


#define MAKE_TYPE_SUBTYPE_COMBINE_MAP(t1, st1, t2, st2, ret)                   \
	{                                                                      \
		VMETA_CAMERA_TYPE_##t1, VMETA_CAMERA_SUBTYPE_##st1,            \
			VMETA_CAMERA_TYPE_##t2, VMETA_CAMERA_SUBTYPE_##st2,    \
			ret                                                    \
	}

#define MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(t1, st1, t2, st2)                     \
	MAKE_TYPE_SUBTYPE_COMBINE_MAP(t1, st1, t2, st2, 0)

#define MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(t1, st1)                              \
	MAKE_TYPE_SUBTYPE_COMBINE_MAP(t1, st1, t1, st1, -ENOENT)


static void test_camera_type_combine_subtype(void)
{
	int ret;
	enum vmeta_camera_type got_type;
	enum vmeta_camera_subtype got_subtype;

	static struct {
		enum vmeta_camera_type type;
		enum vmeta_camera_subtype subtype;
		enum vmeta_camera_type expected_type;
		enum vmeta_camera_subtype expected_subtype;
		int ret;
	} type_subtype_combine_map[] = {
		/* KO */
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT, LEFT),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT, RIGHT),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT_STEREO_LEFT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT_STEREO_LEFT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(HORIZONTAL_STEREO_RIGHT,
						 RIGHT),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(DOWN_STEREO_LEFT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(HORIZONTAL_STEREO_RIGHT, LEFT),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT_STEREO, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(FRONT_STEREO, WIDE),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(HORIZONTAL_STEREO, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(HORIZONTAL_STEREO, TELE),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(DOWN_STEREO, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(DOWN_STEREO, WIDE),
		/* Types that can never appear as a ".generic" entry in
		 * s_camera_type_map[] (vmeta_utils.c), so they always fall
		 * through to the -ENOENT passthrough branch, subtype passed
		 * through unchanged */
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(VERTICAL, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(DISPARITY, WIDE),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_KO(EXTERNAL, TELE),

		/* OK */
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(
			FRONT_STEREO, LEFT, FRONT_STEREO_LEFT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(
			FRONT_STEREO, RIGHT, FRONT_STEREO_RIGHT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(HORIZONTAL_STEREO,
						 LEFT,
						 HORIZONTAL_STEREO_LEFT,
						 UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(HORIZONTAL_STEREO,
						 RIGHT,
						 HORIZONTAL_STEREO_RIGHT,
						 UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(
			DOWN_STEREO, LEFT, DOWN_STEREO_LEFT, UNKNOWN),
		MAKE_TYPE_SUBTYPE_COMBINE_MAP_OK(
			DOWN_STEREO, RIGHT, DOWN_STEREO_RIGHT, UNKNOWN),
	};

	ret = vmeta_camera_type_combine_subtype(VMETA_CAMERA_TYPE_UNKNOWN,
						VMETA_CAMERA_SUBTYPE_UNKNOWN,
						NULL,
						NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_camera_type_combine_subtype(VMETA_CAMERA_TYPE_UNKNOWN,
						VMETA_CAMERA_SUBTYPE_UNKNOWN,
						&got_type,
						NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = vmeta_camera_type_combine_subtype(VMETA_CAMERA_TYPE_UNKNOWN,
						VMETA_CAMERA_SUBTYPE_UNKNOWN,
						NULL,
						&got_subtype);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	for (size_t i = 0; i < SIZEOF_ARRAY(type_subtype_combine_map); i++) {
		ret = vmeta_camera_type_combine_subtype(
			type_subtype_combine_map[i].type,
			type_subtype_combine_map[i].subtype,
			&got_type,
			&got_subtype);
		CU_ASSERT_EQUAL(ret, type_subtype_combine_map[i].ret);
		CU_ASSERT_EQUAL(got_type,
				type_subtype_combine_map[i].expected_type);
		CU_ASSERT_EQUAL(got_subtype,
				type_subtype_combine_map[i].expected_subtype);
	}
}


static void test_camera_type_split_combine_roundtrip(void)
{
	int ret;
	enum vmeta_camera_type split_type;
	enum vmeta_camera_subtype split_subtype;
	enum vmeta_camera_type combined_type;
	enum vmeta_camera_subtype combined_subtype;

	static const enum vmeta_camera_type combined_types[] = {
		VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT,
		VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT,
		VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT,
		VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT,
		VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT,
		VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT,
	};

	/* For every type that is a valid ".combined" entry in
	 * s_camera_type_map[] (vmeta_utils.c), splitting then re-combining
	 * must yield back the original type, with subtype UNKNOWN */
	for (size_t i = 0; i < SIZEOF_ARRAY(combined_types); i++) {
		ret = vmeta_camera_type_split_subtype(
			combined_types[i], &split_type, &split_subtype);
		CU_ASSERT_EQUAL(ret, 0);

		ret = vmeta_camera_type_combine_subtype(split_type,
							split_subtype,
							&combined_type,
							&combined_subtype);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_EQUAL(combined_type, combined_types[i]);
		CU_ASSERT_EQUAL(combined_subtype, VMETA_CAMERA_SUBTYPE_UNKNOWN);
	}
}


#define MAKE_TYPE_SUBTYPE_CMP_PAIR(t1, st1, t2, st2, val)                      \
	{                                                                      \
		VMETA_CAMERA_TYPE_##t1, VMETA_CAMERA_SUBTYPE_##st1,            \
			VMETA_CAMERA_TYPE_##t2, VMETA_CAMERA_SUBTYPE_##st2,    \
			val                                                    \
	}


static void test_camera_type_subtype_pair_cmp(void)
{
	int ret;


	static struct {
		enum vmeta_camera_type type1;
		enum vmeta_camera_subtype subtype1;
		enum vmeta_camera_type type2;
		enum vmeta_camera_subtype subtype2;
		bool identical;
	} type_subtype_cmp_pairs[] = {
		/* Indentical */
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			FRONT, UNKNOWN, FRONT, UNKNOWN, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			EXTERNAL, UNKNOWN, EXTERNAL, UNKNOWN, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(FRONT, WIDE, FRONT, WIDE, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO,
					   UNKNOWN,
					   HORIZONTAL_STEREO,
					   UNKNOWN,
					   true),

		/* Indentical, equivalent */
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			FRONT_STEREO, LEFT, FRONT_STEREO_LEFT, UNKNOWN, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			FRONT_STEREO, RIGHT, FRONT_STEREO_RIGHT, UNKNOWN, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO,
					   LEFT,
					   HORIZONTAL_STEREO_LEFT,
					   UNKNOWN,
					   true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO,
					   RIGHT,
					   HORIZONTAL_STEREO_RIGHT,
					   UNKNOWN,
					   true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			DOWN_STEREO, LEFT, DOWN_STEREO_LEFT, UNKNOWN, true),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			DOWN_STEREO, RIGHT, DOWN_STEREO_RIGHT, UNKNOWN, true),

		/* Not indentical */
		MAKE_TYPE_SUBTYPE_CMP_PAIR(FRONT, UNKNOWN, FRONT, LEFT, false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(FRONT, UNKNOWN, FRONT, RIGHT, false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(FRONT, LEFT, FRONT, UNKNOWN, false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(FRONT, LEFT, FRONT, RIGHT, false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(
			FRONT, UNKNOWN, EXTERNAL, UNKNOWN, false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO_LEFT,
					   UNKNOWN,
					   HORIZONTAL_STEREO,
					   UNKNOWN,
					   false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO_LEFT,
					   UNKNOWN,
					   HORIZONTAL_STEREO_RIGHT,
					   UNKNOWN,
					   false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO_LEFT,
					   UNKNOWN,
					   DOWN_STEREO_LEFT,
					   UNKNOWN,
					   false),
		MAKE_TYPE_SUBTYPE_CMP_PAIR(HORIZONTAL_STEREO,
					   RIGHT,
					   HORIZONTAL_STEREO_RIGHT,
					   RIGHT,
					   false),
	};

	for (size_t i = 0; i < SIZEOF_ARRAY(type_subtype_cmp_pairs); i++) {
		ret = vmeta_camera_type_subtype_pair_cmp(
			type_subtype_cmp_pairs[i].type1,
			type_subtype_cmp_pairs[i].subtype1,
			type_subtype_cmp_pairs[i].type2,
			type_subtype_cmp_pairs[i].subtype2);
		CU_ASSERT_EQUAL(ret,
				type_subtype_cmp_pairs[i].identical ? 1 : 0);

		ret = vmeta_camera_type_subtype_pair_cmp(
			type_subtype_cmp_pairs[i].type2,
			type_subtype_cmp_pairs[i].subtype2,
			type_subtype_cmp_pairs[i].type1,
			type_subtype_cmp_pairs[i].subtype1);
		CU_ASSERT_EQUAL(ret,
				type_subtype_cmp_pairs[i].identical ? 1 : 0);
	}
}


static void test_vmeta_euler_cmp(void)
{
	struct vmeta_euler meta1 = {};
	struct vmeta_euler meta2 = {};

	ASSERT_DOUBLE_CMP_ROBUST(vmeta_euler_cmp, meta1, meta2, yaw, EPS);
	ASSERT_DOUBLE_CMP_ROBUST(vmeta_euler_cmp, meta1, meta2, pitch, EPS);
	ASSERT_DOUBLE_CMP_ROBUST(vmeta_euler_cmp, meta1, meta2, roll, EPS);
}


static void test_vmeta_thermal_alignment_cmp(void)
{
	struct vmeta_thermal_alignment meta1;
	struct vmeta_thermal_alignment meta2;

	memset(&meta1, 0, sizeof(meta1));
	meta2 = meta1;

	/* euler part tested previously */

	CU_ASSERT_EQUAL(vmeta_thermal_alignment_cmp(&meta1, &meta2), 1);

	meta1.valid = 1;
	CU_ASSERT_EQUAL(vmeta_thermal_alignment_cmp(&meta1, &meta2), 0);

	meta2.valid = 1;
	CU_ASSERT_EQUAL(vmeta_thermal_alignment_cmp(&meta1, &meta2), 1);

	meta2.valid = 2;
	CU_ASSERT_EQUAL(vmeta_thermal_alignment_cmp(&meta1, &meta2), 1);
}


static void test_vmeta_thermal_conversion_cmp(void)
{
	struct vmeta_thermal_conversion meta1;
	struct vmeta_thermal_conversion meta2;

	memset(&meta1, 0, sizeof(meta1));
	meta2 = meta1;

	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 1);

	meta1.valid = 1;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);
	meta2.valid = 1;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 1);

	meta2.valid = 2;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 1);

	meta2 = meta1;
	meta2.r = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.b = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.f = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.o = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.tau_win = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.t_win = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.t_bg = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.emissivity = 1.;
	CU_ASSERT_EQUAL(vmeta_thermal_conversion_cmp(&meta1, &meta2), 0);
}


static void test_vmeta_thermal_cmp(void)
{
	struct vmeta_thermal meta1;
	struct vmeta_thermal meta2;

	memset(&meta1, 0, sizeof(meta1));
	meta2 = meta1;

	/* alignment, conv_low, conv_high tested previously */

	CU_ASSERT_EQUAL(vmeta_thermal_cmp(&meta1, &meta2), 1);

	meta2.metaversion = 1;
	CU_ASSERT_EQUAL(vmeta_thermal_cmp(&meta1, &meta2), 0);

	/* Test equality of content */
	meta2 = meta1;
	snprintf(meta1.camserial, sizeof(meta1.camserial), "SN12345");
	snprintf(meta2.camserial, sizeof(meta2.camserial), "SN12345");
	CU_ASSERT_EQUAL(vmeta_thermal_cmp(&meta1, &meta2), 1);

	/* Test difference in length (prefix) */
	snprintf(meta2.camserial, sizeof(meta2.camserial), "SN123456");
	CU_ASSERT_EQUAL(vmeta_thermal_cmp(&meta1, &meta2), 0);

	/* Test case sensitivity */
	snprintf(meta1.camserial, sizeof(meta1.camserial), "Abc");
	snprintf(meta2.camserial, sizeof(meta2.camserial), "abc");
	CU_ASSERT_EQUAL(vmeta_thermal_cmp(&meta1, &meta2), 0);

	ASSERT_DOUBLE_CMP_ROBUST(
		vmeta_thermal_cmp, meta1, meta2, scale_factor, EPS);
}


static void test_vmeta_fov_cmp(void)
{
	struct vmeta_fov meta1 = {};
	struct vmeta_fov meta2 = {};

	CU_ASSERT_EQUAL(vmeta_fov_cmp(&meta1, &meta2), 1);

	meta1.has_horz = 1;
	CU_ASSERT_EQUAL(vmeta_fov_cmp(&meta1, &meta2), 0);
	meta1.has_vert = 1.;
	CU_ASSERT_EQUAL(vmeta_fov_cmp(&meta1, &meta2), 0);
	meta1.has_horz = 0;
	CU_ASSERT_EQUAL(vmeta_fov_cmp(&meta1, &meta2), 0);
	meta1.has_horz = 1;
	meta2 = meta1;

	ASSERT_DOUBLE_CMP_ROBUST(vmeta_fov_cmp, meta1, meta2, vert, EPS);
	ASSERT_DOUBLE_CMP_ROBUST(vmeta_fov_cmp, meta1, meta2, horz, EPS);
}


static void test_vmeta_location_cmp(void)
{
	struct vmeta_location meta1 = {};
	struct vmeta_location meta2 = {};

	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 1);
	meta1.valid = 1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);
	meta2.valid = 1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 1);
	meta2.valid = 2;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 1);

	meta2.latitude = 1.;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.longitude = 1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	ASSERT_DOUBLE_CMP_TREAT_NAN_AS_UNKNOWN(
		vmeta_location_cmp, meta1, meta2, altitude_wgs84ellipsoid, EPS);
	ASSERT_DOUBLE_CMP_TREAT_NAN_AS_UNKNOWN(
		vmeta_location_cmp, meta1, meta2, altitude_egm96amsl, EPS);

	meta2 = meta1;
	meta2.horizontal_accuracy = 1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.vertical_accuracy = 1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.sv_count = 1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 0);

	meta1.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
	meta2 = meta1;
	CU_ASSERT_EQUAL(vmeta_location_cmp(&meta1, &meta2), 1);
}


static void test_vmeta_overlay_cmp(void)
{
	struct vmeta_overlay meta1;
	struct vmeta_overlay meta2;

	memset(&meta1, 0, sizeof(meta1));
	meta2 = meta1;

	CU_ASSERT_EQUAL(vmeta_overlay_cmp(&meta1, &meta2), 1);
	meta1.type = VMETA_OVERLAY_TYPE_HEADER_FOOTER;
	CU_ASSERT_EQUAL(vmeta_overlay_cmp(&meta1, &meta2), 0);
	meta2.type = VMETA_OVERLAY_TYPE_HEADER_FOOTER;
	CU_ASSERT_EQUAL(vmeta_overlay_cmp(&meta1, &meta2), 1);

	meta2.header_footer.header_height = 1.;
	CU_ASSERT_EQUAL(vmeta_overlay_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.header_footer.footer_height = 1.;
	CU_ASSERT_EQUAL(vmeta_overlay_cmp(&meta1, &meta2), 0);
}


static void test_vmeta_camera_model_cmp(void)
{
	struct vmeta_camera_model meta1;
	struct vmeta_camera_model meta2;

	memset(&meta1, 0, sizeof(meta1));
	meta2 = meta1;

	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 1);
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 1);
	meta1.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);
	meta2.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 1);

	meta2.perspective.distortion.r1 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.perspective.distortion.r2 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.perspective.distortion.r3 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.perspective.distortion.t1 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.perspective.distortion.t2 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta1.type = VMETA_CAMERA_MODEL_TYPE_FISHEYE;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);
	meta2.type = VMETA_CAMERA_MODEL_TYPE_FISHEYE;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 1);

	meta2.fisheye.affine_matrix.c = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.affine_matrix.d = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.affine_matrix.e = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.affine_matrix.f = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.affine_matrix.symmetric_valid = 0;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 1);

	meta2 = meta1;
	meta2.fisheye.affine_matrix.symmetric_valid = 1;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.polynomial.p2 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.polynomial.p3 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	meta2 = meta1;
	meta2.fisheye.polynomial.p4 = 1.;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 0);

	/* Set to perspective and sync */
	meta1.type = VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	meta1.perspective.distortion.r1 = 1.0f;
	meta2 = meta1;
	meta2.fisheye.polynomial.p4 = 0.5f;
	CU_ASSERT_EQUAL(vmeta_camera_model_cmp(&meta1, &meta2), 1);
}


/*
 * vmeta_quat_to_euler_zyx(): a distinct rotation-order variant of
 * vmeta_quat_to_euler(), previously entirely uncovered.
 */
static void test_vmeta_quat_to_euler_zyx(void)
{
	struct vmeta_euler euler_res;

	/* NULL arguments must not crash */
	vmeta_quat_to_euler_zyx(NULL, &euler_res);
	{
		const struct vmeta_quaternion q = {.w = 1.f};
		vmeta_quat_to_euler_zyx(&q, NULL);
	}

	/* Zero quaternion -> NaN (same convention as vmeta_quat_to_euler) */
	{
		const struct vmeta_quaternion quat_zero = {0};
		memset(&euler_res, 0, sizeof(euler_res));
		vmeta_quat_to_euler_zyx(&quat_zero, &euler_res);
		CU_ASSERT_TRUE(isnan(euler_res.psi));
		CU_ASSERT_TRUE(isnan(euler_res.theta));
		CU_ASSERT_TRUE(isnan(euler_res.phi));
	}

	/* Identity quaternion -> all angles zero */
	{
		const struct vmeta_quaternion quat_id = {.w = 1.f};
		const struct vmeta_euler expected = {0};
		memset(&euler_res, 0, sizeof(euler_res));
		vmeta_quat_to_euler_zyx(&quat_id, &euler_res);
		CU_ASSERT_TRUE(
			euler_are_equal(&euler_res, &expected, TEST_EPSILON));
	}

	/* Small x-only rotation: no singularity, |phi| <= pi/2, so the
	 * zyx-specific post-processing does not kick in and the result
	 * matches vmeta_quat_to_euler() exactly. */
	{
		float a = 0.5f;
		const struct vmeta_quaternion quat = {.w = cosf(a / 2.f),
						      .x = sinf(a / 2.f)};
		struct vmeta_euler euler_zyx, euler_xyz;
		memset(&euler_zyx, 0, sizeof(euler_zyx));
		memset(&euler_xyz, 0, sizeof(euler_xyz));
		vmeta_quat_to_euler_zyx(&quat, &euler_zyx);
		vmeta_quat_to_euler(&quat, &euler_xyz);
		CU_ASSERT_TRUE(
			euler_are_equal(&euler_zyx, &euler_xyz, TEST_EPSILON));
		CU_ASSERT_DOUBLE_EQUAL(euler_zyx.phi, a, TEST_EPSILON);
	}

	/* Large x-only rotation: |phi| > pi/2 triggers the zyx-specific
	 * yaw/pitch/roll renormalization branch. */
	{
		float a = 2.5f;
		const struct vmeta_quaternion quat = {.w = cosf(a / 2.f),
						      .x = sinf(a / 2.f)};
		const struct vmeta_euler expected = {
			.psi = (float)M_PI,
			.theta = (float)-M_PI,
			.phi = a - (float)M_PI,
		};
		memset(&euler_res, 0, sizeof(euler_res));
		vmeta_quat_to_euler_zyx(&quat, &euler_res);
		CU_ASSERT_TRUE(euler_are_equal(
			&euler_res, &expected, TEST_EPSILON * 10.f));
		CU_ASSERT_TRUE(fabsf(euler_res.phi) <=
			       (float)M_PI / 2.f + TEST_EPSILON);
	}

	/* Singularities: same quaternions as test_quat_to_euler()'s
	 * quat3/quat4, but the zyx variant swaps yaw/roll at the
	 * singularity instead of leaving roll non-zero. */
	{
		const struct vmeta_quaternion quat3 = {
			.w = 0.353553390593274f,
			.x = 0.612372435695794f,
			.y = 0.353553390593274f,
			.z = -0.612372435695794f,
		};
		const struct vmeta_euler expected3 = {
			.psi = -2.f * (float)M_PI / 3.f,
			.theta = (float)M_PI / 2.f,
			.phi = 0.f,
		};
		memset(&euler_res, 0, sizeof(euler_res));
		vmeta_quat_to_euler_zyx(&quat3, &euler_res);
		CU_ASSERT_TRUE(euler_are_equal(
			&euler_res, &expected3, TEST_EPSILON * 10.f));

		const struct vmeta_quaternion quat4 = {
			.w = 0.353553390593274f,
			.x = 0.612372435695794f,
			.y = -0.353553390593274f,
			.z = 0.612372435695794f,
		};
		const struct vmeta_euler expected4 = {
			.psi = 2.f * (float)M_PI / 3.f,
			.theta = -(float)M_PI / 2.f,
			.phi = 0.f,
		};
		memset(&euler_res, 0, sizeof(euler_res));
		vmeta_quat_to_euler_zyx(&quat4, &euler_res);
		CU_ASSERT_TRUE(euler_are_equal(
			&euler_res, &expected4, TEST_EPSILON * 10.f));
	}
}


/*
 * vmeta_hash_joaat_str(): Jenkins one-at-a-time hash, previously entirely
 * uncovered. Expected hash values below were computed with an independent
 * re-implementation of the exact same algorithm (32-bit wraparound
 * add/shift/xor per character, then the 3 finalization steps).
 */
static void test_vmeta_hash_joaat_str(void)
{
	int res;
	uint32_t hash;
	uint32_t hash_lower, hash_upper;

	res = vmeta_hash_joaat_str(NULL, &hash);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_hash_joaat_str("abc", NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* Empty string: the per-character loop never executes, and the
	 * finalization steps are all no-ops on a zero accumulator */
	hash = 0xffffffff;
	res = vmeta_hash_joaat_str("", &hash);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(hash, 0);

	res = vmeta_hash_joaat_str("vmeta", &hash);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(hash, 528729306U);

	res = vmeta_hash_joaat_str("Hello, World!", &hash);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(hash, 847757641U);

	/* Deterministic: same input always gives the same hash */
	res = vmeta_hash_joaat_str("vmeta", &hash);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(hash, 528729306U);

	/* Case sensitivity: differently-cased strings hash differently */
	CU_ASSERT_EQUAL(vmeta_hash_joaat_str("abc", &hash_lower), 0);
	CU_ASSERT_EQUAL(vmeta_hash_joaat_str("ABC", &hash_upper), 0);
	CU_ASSERT_NOT_EQUAL(hash_lower, hash_upper);
}


/* Copy src into dst, upper-casing every character (helper for the
 * case-insensitive enum<->string round-trip tests below). */
static void str_toupper_copy(char *dst, size_t dst_size, const char *src)
{
	size_t i;
	for (i = 0; src[i] != '\0' && i + 1 < dst_size; i++)
		dst[i] = (char)toupper((unsigned char)src[i]);
	dst[i] = '\0';
}


static void test_vmeta_camera_type_str(void)
{
	static const struct {
		enum vmeta_camera_type val;
		const char *str;
	} map[] = {
		{VMETA_CAMERA_TYPE_FRONT, "front"},
		{VMETA_CAMERA_TYPE_FRONT_STEREO, "front-stereo"},
		{VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT, "front-stereo-left"},
		{VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT, "front-stereo-right"},
		{VMETA_CAMERA_TYPE_VERTICAL, "vertical"},
		{VMETA_CAMERA_TYPE_DISPARITY, "disparity"},
		{VMETA_CAMERA_TYPE_HORIZONTAL_STEREO, "horizontal-stereo"},
		{VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT,
		 "horizontal-stereo-left"},
		{VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT,
		 "horizontal-stereo-right"},
		{VMETA_CAMERA_TYPE_DOWN_STEREO, "down-stereo"},
		{VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT, "down-stereo-left"},
		{VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT, "down-stereo-right"},
		{VMETA_CAMERA_TYPE_EXTERNAL, "external"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_camera_type_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_camera_type_from_str(str), map[i].val);

		/* from_str is case-insensitive */
		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_camera_type_from_str(upper), map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_type_to_str(VMETA_CAMERA_TYPE_UNKNOWN), "unknown");
	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_type_to_str((enum vmeta_camera_type)9999),
		"unknown");

	CU_ASSERT_EQUAL(vmeta_camera_type_from_str(NULL),
			VMETA_CAMERA_TYPE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_camera_type_from_str("bogus"),
			VMETA_CAMERA_TYPE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_camera_type_from_str(""),
			VMETA_CAMERA_TYPE_UNKNOWN);
}


static void test_vmeta_camera_subtype_str(void)
{
	static const struct {
		enum vmeta_camera_subtype val;
		const char *str;
	} map[] = {
		{VMETA_CAMERA_SUBTYPE_LEFT, "left"},
		{VMETA_CAMERA_SUBTYPE_RIGHT, "right"},
		{VMETA_CAMERA_SUBTYPE_WIDE, "wide"},
		{VMETA_CAMERA_SUBTYPE_TELE, "tele"},
		{VMETA_CAMERA_SUBTYPE_DISPARITY, "disparity"},
		{VMETA_CAMERA_SUBTYPE_DEPTH, "depth"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_camera_subtype_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_camera_subtype_from_str(str), map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_camera_subtype_from_str(upper),
				map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_subtype_to_str(VMETA_CAMERA_SUBTYPE_UNKNOWN),
		"unknown");
	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_subtype_to_str((enum vmeta_camera_subtype)9999),
		"unknown");

	CU_ASSERT_EQUAL(vmeta_camera_subtype_from_str(NULL),
			VMETA_CAMERA_SUBTYPE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_camera_subtype_from_str("bogus"),
			VMETA_CAMERA_SUBTYPE_UNKNOWN);
}


static void test_vmeta_camera_spectrum_str(void)
{
	static const struct {
		enum vmeta_camera_spectrum val;
		const char *str;
	} map[] = {
		{VMETA_CAMERA_SPECTRUM_VISIBLE, "visible"},
		{VMETA_CAMERA_SPECTRUM_THERMAL, "thermal"},
		{VMETA_CAMERA_SPECTRUM_BLENDED, "blended"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_camera_spectrum_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_camera_spectrum_from_str(str),
				map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_camera_spectrum_from_str(upper),
				map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_spectrum_to_str(VMETA_CAMERA_SPECTRUM_UNKNOWN),
		"unknown");
	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_spectrum_to_str((enum vmeta_camera_spectrum)9999),
		"unknown");

	CU_ASSERT_EQUAL(vmeta_camera_spectrum_from_str(NULL),
			VMETA_CAMERA_SPECTRUM_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_camera_spectrum_from_str("bogus"),
			VMETA_CAMERA_SPECTRUM_UNKNOWN);
}


static void test_vmeta_camera_model_type_str(void)
{
	static const struct {
		enum vmeta_camera_model_type val;
		const char *str;
	} map[] = {
		{VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE, "perspective"},
		{VMETA_CAMERA_MODEL_TYPE_FISHEYE, "fisheye"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_camera_model_type_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_camera_model_type_from_str(str),
				map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_camera_model_type_from_str(upper),
				map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_camera_model_type_to_str(VMETA_CAMERA_MODEL_TYPE_UNKNOWN),
		"unknown");
	CU_ASSERT_STRING_EQUAL(vmeta_camera_model_type_to_str(
				       (enum vmeta_camera_model_type)9999),
			       "unknown");

	CU_ASSERT_EQUAL(vmeta_camera_model_type_from_str(NULL),
			VMETA_CAMERA_MODEL_TYPE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_camera_model_type_from_str("bogus"),
			VMETA_CAMERA_MODEL_TYPE_UNKNOWN);
}


static void test_vmeta_overlay_type_str(void)
{
	/* No vmeta_overlay_type_from_str() exists for this enum */
	CU_ASSERT_STRING_EQUAL(
		vmeta_overlay_type_to_str(VMETA_OVERLAY_TYPE_NONE), "none");
	CU_ASSERT_STRING_EQUAL(
		vmeta_overlay_type_to_str(VMETA_OVERLAY_TYPE_HEADER_FOOTER),
		"header_footer");
	CU_ASSERT_STRING_EQUAL(
		vmeta_overlay_type_to_str((enum vmeta_overlay_type)9999),
		"unknown");
}


static void test_vmeta_video_mode_str(void)
{
	static const struct {
		enum vmeta_video_mode val;
		const char *str;
	} map[] = {
		{VMETA_VIDEO_MODE_STANDARD, "standard"},
		{VMETA_VIDEO_MODE_HYPERLAPSE, "hyperlapse"},
		{VMETA_VIDEO_MODE_SLOWMOTION, "slowmotion"},
		{VMETA_VIDEO_MODE_STREAMREC, "streamrec"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_video_mode_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_video_mode_from_str(str), map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_video_mode_from_str(upper), map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_video_mode_to_str(VMETA_VIDEO_MODE_UNKNOWN), "unknown");
	CU_ASSERT_STRING_EQUAL(
		vmeta_video_mode_to_str((enum vmeta_video_mode)9999),
		"unknown");

	CU_ASSERT_EQUAL(vmeta_video_mode_from_str(NULL),
			VMETA_VIDEO_MODE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_video_mode_from_str("bogus"),
			VMETA_VIDEO_MODE_UNKNOWN);
}


static void test_vmeta_video_stop_reason_str(void)
{
	static const struct {
		enum vmeta_video_stop_reason val;
		const char *str;
	} map[] = {
		{VMETA_VIDEO_STOP_REASON_USER, "user"},
		{VMETA_VIDEO_STOP_REASON_RECONFIGURATION, "reconfiguration"},
		{VMETA_VIDEO_STOP_REASON_POOR_STORAGE_PERF,
		 "poor-storage-perf"},
		{VMETA_VIDEO_STOP_REASON_STORAGE_FULL, "storage-full"},
		{VMETA_VIDEO_STOP_REASON_RECOVERY, "recovery"},
		{VMETA_VIDEO_STOP_REASON_END_OF_STREAM, "end-of-stream"},
		{VMETA_VIDEO_STOP_REASON_SHUTDOWN, "shutdown"},
		{VMETA_VIDEO_STOP_REASON_INTERNAL_ERROR, "internal-error"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_video_stop_reason_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_video_stop_reason_from_str(str),
				map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_video_stop_reason_from_str(upper),
				map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_video_stop_reason_to_str(VMETA_VIDEO_STOP_REASON_UNKNOWN),
		"unknown");
	CU_ASSERT_STRING_EQUAL(vmeta_video_stop_reason_to_str(
				       (enum vmeta_video_stop_reason)9999),
			       "unknown");

	CU_ASSERT_EQUAL(vmeta_video_stop_reason_from_str(NULL),
			VMETA_VIDEO_STOP_REASON_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_video_stop_reason_from_str("bogus"),
			VMETA_VIDEO_STOP_REASON_UNKNOWN);
}


static void test_vmeta_photo_mode_str(void)
{
	/* Unlike most other enum<->string pairs in this file,
	 * vmeta_photo_mode_from_str() is case-sensitive (strcmp, not
	 * strcasecmp) AND only recognizes all-lowercase spellings, while
	 * vmeta_photo_mode_to_str() returns capitalized strings (documented
	 * in the source as required "for parsing in libphoto-metadata").
	 * The two are therefore NOT round-trip partners for this enum:
	 * from_str(to_str(x)) != x. This is deliberately exercised below
	 * instead of assumed away. */
	static const struct {
		enum vmeta_photo_mode val;
		const char *to_str_expected;
		const char *from_str_input;
	} map[] = {
		{VMETA_PHOTO_MODE_SINGLE, "Single", "single"},
		{VMETA_PHOTO_MODE_BRACKETING, "Bracketing", "bracketing"},
		{VMETA_PHOTO_MODE_BURST, "Burst", "burst"},
		{VMETA_PHOTO_MODE_TIMELAPSE, "TimeLapse", "timelapse"},
		{VMETA_PHOTO_MODE_GPSLAPSE, "GPSLapse", "gpslapse"},
		{VMETA_PHOTO_MODE_PANORAMA, "Panorama", "panorama"},
	};

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		CU_ASSERT_STRING_EQUAL(vmeta_photo_mode_to_str(map[i].val),
				       map[i].to_str_expected);
		CU_ASSERT_EQUAL(
			vmeta_photo_mode_from_str(map[i].from_str_input),
			map[i].val);

		/* Confirm the to_str() output does NOT round-trip through
		 * from_str() whenever the casing differs */
		if (strcmp(map[i].to_str_expected, map[i].from_str_input) !=
		    0) {
			CU_ASSERT_EQUAL(vmeta_photo_mode_from_str(
						map[i].to_str_expected),
					VMETA_PHOTO_MODE_UNKNOWN);
		}
	}

	CU_ASSERT_EQUAL(vmeta_photo_mode_from_str(NULL),
			VMETA_PHOTO_MODE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_photo_mode_from_str("bogus"),
			VMETA_PHOTO_MODE_UNKNOWN);
	CU_ASSERT_STRING_EQUAL(
		vmeta_photo_mode_to_str((enum vmeta_photo_mode)9999),
		"unknown");
}


static void test_vmeta_panorama_type_str(void)
{
	/* Unlike most other enum<->string pairs in this file,
	 * vmeta_panorama_type_from_str() is case-sensitive (strcmp), but
	 * since vmeta_panorama_type_to_str() also returns all-lowercase
	 * strings, the two do round-trip here. */
	static const struct {
		enum vmeta_panorama_type val;
		const char *str;
	} map[] = {
		{VMETA_PANORAMA_TYPE_NONE, "none"},
		{VMETA_PANORAMA_TYPE_HORIZONTAL_180, "horizontal-180"},
		{VMETA_PANORAMA_TYPE_VERTICAL_180, "vertical-180"},
		{VMETA_PANORAMA_TYPE_SPHERICAL, "spherical"},
		{VMETA_PANORAMA_TYPE_SUPER_WIDE, "super-wide"},
	};

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		CU_ASSERT_STRING_EQUAL(vmeta_panorama_type_to_str(map[i].val),
				       map[i].str);
		CU_ASSERT_EQUAL(vmeta_panorama_type_from_str(map[i].str),
				map[i].val);
	}

	/* Case-sensitivity check */
	CU_ASSERT_EQUAL(vmeta_panorama_type_from_str("SPHERICAL"),
			VMETA_PANORAMA_TYPE_UNKNOWN);

	CU_ASSERT_EQUAL(vmeta_panorama_type_from_str(NULL),
			VMETA_PANORAMA_TYPE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_panorama_type_from_str("bogus"),
			VMETA_PANORAMA_TYPE_UNKNOWN);
	CU_ASSERT_STRING_EQUAL(
		vmeta_panorama_type_to_str((enum vmeta_panorama_type)9999),
		"unknown");
}


static void test_vmeta_dynamic_range_str(void)
{
	static const struct {
		enum vmeta_dynamic_range val;
		const char *str;
	} map[] = {
		{VMETA_DYNAMIC_RANGE_SDR, "sdr"},
		{VMETA_DYNAMIC_RANGE_HDR8, "hdr8"},
		{VMETA_DYNAMIC_RANGE_HDR10, "hdr10"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_dynamic_range_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_dynamic_range_from_str(str), map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_dynamic_range_from_str(upper),
				map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_dynamic_range_to_str(VMETA_DYNAMIC_RANGE_UNKNOWN),
		"unknown");
	CU_ASSERT_STRING_EQUAL(
		vmeta_dynamic_range_to_str((enum vmeta_dynamic_range)9999),
		"unknown");

	CU_ASSERT_EQUAL(vmeta_dynamic_range_from_str(NULL),
			VMETA_DYNAMIC_RANGE_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_dynamic_range_from_str("bogus"),
			VMETA_DYNAMIC_RANGE_UNKNOWN);
}


static void test_vmeta_tone_mapping_str(void)
{
	static const struct {
		enum vmeta_tone_mapping val;
		const char *str;
	} map[] = {
		{VMETA_TONE_MAPPING_STANDARD, "standard"},
		{VMETA_TONE_MAPPING_P_LOG, "p-log"},
	};
	char upper[64];

	for (size_t i = 0; i < SIZEOF_ARRAY(map); i++) {
		const char *str = vmeta_tone_mapping_to_str(map[i].val);
		CU_ASSERT_STRING_EQUAL(str, map[i].str);
		CU_ASSERT_EQUAL(vmeta_tone_mapping_from_str(str), map[i].val);

		str_toupper_copy(upper, sizeof(upper), str);
		CU_ASSERT_EQUAL(vmeta_tone_mapping_from_str(upper), map[i].val);
	}

	CU_ASSERT_STRING_EQUAL(
		vmeta_tone_mapping_to_str(VMETA_TONE_MAPPING_UNKNOWN),
		"unknown");
	CU_ASSERT_STRING_EQUAL(
		vmeta_tone_mapping_to_str((enum vmeta_tone_mapping)9999),
		"unknown");

	CU_ASSERT_EQUAL(vmeta_tone_mapping_from_str(NULL),
			VMETA_TONE_MAPPING_UNKNOWN);
	CU_ASSERT_EQUAL(vmeta_tone_mapping_from_str("bogus"),
			VMETA_TONE_MAPPING_UNKNOWN);
}


/*
 * Helper for the vmeta_frame_get_*() PROTO-path tests below: create a fresh
 * VMETA_FRAME_TYPE_PROTO frame and return it locked in read-write mode via
 * *tm, ready for the writer helpers declared in vmeta_frame_proto.h to
 * populate. Caller must release the rw lock (vmeta_frame_proto_release_
 * unpacked_rw()) before calling any vmeta_frame_get_*() function on the
 * frame (those take the read-only lock internally), and must
 * vmeta_frame_unref() the frame when done.
 */
static struct vmeta_frame *
utils_test_new_proto_frame_rw(Vmeta__TimedMetadata **tm)
{
	int res;
	struct vmeta_frame *frame = NULL;

	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(frame);

	res = vmeta_frame_proto_get_unpacked_rw(frame, tm);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(*tm);

	return frame;
}


static void test_vmeta_frame_get_frame_utc_timestamp(void)
{
	int res;
	uint64_t ts;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	Vmeta__CameraMetadata *camera;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;

	res = vmeta_frame_get_frame_utc_timestamp(NULL, &ts);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_frame_utc_timestamp(&frame_none, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_frame_utc_timestamp(&frame_none, &ts);
	CU_ASSERT_EQUAL(res, -ENOENT);
	CU_ASSERT_EQUAL(ts, 0);

	res = vmeta_frame_get_frame_utc_timestamp(&frame_bad, &ts);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* PROTO, no camera at all -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_frame_utc_timestamp(frame, &ts);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, camera present but utc_timestamp == 0 -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	camera = vmeta_frame_proto_get_camera(tm);
	CU_ASSERT_PTR_NOT_NULL(camera);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_frame_utc_timestamp(frame, &ts);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, camera present with a nonzero utc_timestamp -> success */
	frame = utils_test_new_proto_frame_rw(&tm);
	camera = vmeta_frame_proto_get_camera(tm);
	CU_ASSERT_PTR_NOT_NULL(camera);
	camera->utc_timestamp = 123456789ULL;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_frame_utc_timestamp(frame, &ts);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(ts, 123456789ULL);
	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_camera_spectrum(void)
{
	int res;
	enum vmeta_camera_spectrum spectrum;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	Vmeta__CameraMetadata *camera;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;

	res = vmeta_frame_get_camera_spectrum(NULL, &spectrum);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_camera_spectrum(&frame_none, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_camera_spectrum(&frame_none, &spectrum);
	CU_ASSERT_EQUAL(res, -ENOENT);
	CU_ASSERT_EQUAL(spectrum, VMETA_CAMERA_SPECTRUM_UNKNOWN);

	res = vmeta_frame_get_camera_spectrum(&frame_bad, &spectrum);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* PROTO, no camera -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_camera_spectrum(frame, &spectrum);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, camera with spectrum set -> success */
	frame = utils_test_new_proto_frame_rw(&tm);
	camera = vmeta_frame_proto_get_camera(tm);
	CU_ASSERT_PTR_NOT_NULL(camera);
	camera->spectrum = VMETA__CAMERA_SPECTRUM__CS_THERMAL;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_camera_spectrum(frame, &spectrum);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(spectrum, VMETA_CAMERA_SPECTRUM_THERMAL);
	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_camera_subtype(void)
{
	int res;
	enum vmeta_camera_subtype subtype;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	Vmeta__CameraMetadata *camera;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;

	res = vmeta_frame_get_camera_subtype(NULL, &subtype);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_camera_subtype(&frame_none, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_camera_subtype(&frame_none, &subtype);
	CU_ASSERT_EQUAL(res, -ENOENT);
	CU_ASSERT_EQUAL(subtype, VMETA_CAMERA_SUBTYPE_UNKNOWN);

	res = vmeta_frame_get_camera_subtype(&frame_bad, &subtype);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* PROTO, no camera -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_camera_subtype(frame, &subtype);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, camera with subtype set -> success */
	frame = utils_test_new_proto_frame_rw(&tm);
	camera = vmeta_frame_proto_get_camera(tm);
	CU_ASSERT_PTR_NOT_NULL(camera);
	camera->subtype = VMETA__CAMERA_SUBTYPE__CST_WIDE;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_camera_subtype(frame, &subtype);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(subtype, VMETA_CAMERA_SUBTYPE_WIDE);
	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_thermal_mask(void)
{
	int res;
	struct vmeta_rectf mask;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	Vmeta__ThermalMetadata *thermal;
	Vmeta__Rectf *rectf;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;

	res = vmeta_frame_get_thermal_mask(NULL, &mask);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_thermal_mask(&frame_none, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_thermal_mask(&frame_none, &mask);
	CU_ASSERT_EQUAL(res, -ENOENT);

	res = vmeta_frame_get_thermal_mask(&frame_bad, &mask);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* PROTO, no thermal metadata at all -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_thermal_mask(frame, &mask);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, thermal present but no mask -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	thermal = vmeta_frame_proto_get_thermal(tm);
	CU_ASSERT_PTR_NOT_NULL(thermal);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_thermal_mask(frame, &mask);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, thermal mask fully set -> success */
	frame = utils_test_new_proto_frame_rw(&tm);
	thermal = vmeta_frame_proto_get_thermal(tm);
	CU_ASSERT_PTR_NOT_NULL(thermal);
	rectf = vmeta_frame_proto_get_thermal_mask(thermal);
	CU_ASSERT_PTR_NOT_NULL(rectf);
	rectf->x = 0.1f;
	rectf->y = 0.2f;
	rectf->width = 0.3f;
	rectf->height = 0.4f;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_thermal_mask(frame, &mask);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_DOUBLE_EQUAL(mask.left, 0.1, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(mask.top, 0.2, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(mask.width, 0.3, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(mask.height, 0.4, TEST_EPSILON);
	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_color_matrix(void)
{
	int res;
	size_t count;
	double matrix[9];
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	Vmeta__PhotoMetadata *photo;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;

	res = vmeta_frame_get_color_matrix(NULL, matrix, &count);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_color_matrix(&frame_none, matrix, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_color_matrix(&frame_none, matrix, &count);
	CU_ASSERT_EQUAL(res, -ENOENT);

	res = vmeta_frame_get_color_matrix(&frame_bad, matrix, &count);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* PROTO, no photo metadata -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_color_matrix(frame, matrix, &count);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, photo with a 3x3 (9-element) color matrix */
	frame = utils_test_new_proto_frame_rw(&tm);
	photo = vmeta_frame_proto_get_photo(tm);
	CU_ASSERT_PTR_NOT_NULL(photo);
	for (size_t i = 0; i < 9; i++) {
		double *entry =
			vmeta_frame_proto_get_color_matrix_1_by_index(photo, i);
		CU_ASSERT_PTR_NOT_NULL(entry);
		*entry = (double)i + 1.0;
	}
	vmeta_frame_proto_release_unpacked_rw(frame, tm);

	/* count-only query: matrix == NULL just reports the size needed */
	count = 0;
	res = vmeta_frame_get_color_matrix(frame, NULL, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 9);

	/* Buffer too small -> -ENOBUFS, count updated to the required size */
	count = 4;
	res = vmeta_frame_get_color_matrix(frame, matrix, &count);
	CU_ASSERT_EQUAL(res, -ENOBUFS);
	CU_ASSERT_EQUAL(count, 9);

	/* Buffer large enough -> full copy */
	count = 9;
	memset(matrix, 0, sizeof(matrix));
	res = vmeta_frame_get_color_matrix(frame, matrix, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 9);
	for (size_t i = 0; i < 9; i++)
		CU_ASSERT_DOUBLE_EQUAL(
			matrix[i], (double)i + 1.0, TEST_EPSILON);

	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_lfic_count(void)
{
	int res;
	size_t count;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	struct vmeta_frame frame_v3;
	Vmeta__LFICMetadata *lfic0;
	Vmeta__LFICMetadata *lfic1;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;
	memset(&frame_v3, 0, sizeof(frame_v3));
	frame_v3.type = VMETA_FRAME_TYPE_V3;

	res = vmeta_frame_get_lfic_count(NULL, &count);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_count(&frame_none, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	/* Unlike most other getters, NONE/V1/V2 leave count at 0 with a
	 * success return, not -ENOENT */
	res = vmeta_frame_get_lfic_count(&frame_none, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 0);

	res = vmeta_frame_get_lfic_count(&frame_bad, &count);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* V3, no lfic -> count 0 */
	res = vmeta_frame_get_lfic_count(&frame_v3, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 0);

	/* V3, has_lfic -> count 1 */
	frame_v3.v3.has_lfic = 1;
	res = vmeta_frame_get_lfic_count(&frame_v3, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 1);

	/* PROTO, no lfic entries -> count 0 */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_lfic_count(frame, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 0);
	vmeta_frame_unref(frame);

	/* PROTO, 2 lfic entries -> count 2 */
	frame = utils_test_new_proto_frame_rw(&tm);
	lfic0 = vmeta_frame_proto_get_lfic_by_index(tm, 0);
	CU_ASSERT_PTR_NOT_NULL(lfic0);
	lfic1 = vmeta_frame_proto_get_lfic_by_index(tm, 1);
	CU_ASSERT_PTR_NOT_NULL(lfic1);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_lfic_count(frame, &count);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(count, 2);
	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_lfic_by_index(void)
{
	int res;
	struct vmeta_location loc;
	float x, y;
	enum vmeta_lfic_type type;
	double estimated_precision, grid_precision;
	float h_tang_acc, h_rad_acc;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	struct vmeta_frame frame_v3;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	Vmeta__LFICMetadata *lfic;
	Vmeta__Location *ploc;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;
	memset(&frame_v3, 0, sizeof(frame_v3));
	frame_v3.type = VMETA_FRAME_TYPE_V3;

	res = vmeta_frame_get_lfic_by_index(
		NULL, 0, &loc, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_index(
		&frame_none, 0, NULL, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_index(
		&frame_none, 0, &loc, NULL, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_index(
		&frame_none, 0, &loc, &x, NULL, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_index(
		&frame_none, 0, &loc, &x, &y, NULL, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_lfic_by_index(
		&frame_none, 0, &loc, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);

	res = vmeta_frame_get_lfic_by_index(
		&frame_bad, 0, &loc, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* V3, no lfic -> -ENOENT */
	res = vmeta_frame_get_lfic_by_index(
		&frame_v3, 0, &loc, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);

	/* V3, has_lfic but index != 0 -> -ENOENT */
	frame_v3.v3.has_lfic = 1;
	frame_v3.v3.lfic.target_x = 0.25f;
	frame_v3.v3.lfic.target_y = 0.75f;
	frame_v3.v3.lfic.target_location.latitude = 48.8;
	frame_v3.v3.lfic.target_location.longitude = 2.3;
	frame_v3.v3.lfic.estimated_precision = 3.5;
	frame_v3.v3.lfic.grid_precision = 1.5;
	res = vmeta_frame_get_lfic_by_index(&frame_v3,
					    1,
					    &loc,
					    &x,
					    &y,
					    &type,
					    &estimated_precision,
					    &grid_precision,
					    &h_tang_acc,
					    &h_rad_acc);
	CU_ASSERT_EQUAL(res, -ENOENT);

	/* V3, has_lfic, index 0 -> success */
	res = vmeta_frame_get_lfic_by_index(&frame_v3,
					    0,
					    &loc,
					    &x,
					    &y,
					    &type,
					    &estimated_precision,
					    &grid_precision,
					    &h_tang_acc,
					    &h_rad_acc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(type, VMETA_LFIC_TYPE_COT);
	CU_ASSERT_DOUBLE_EQUAL(x, 0.25, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(y, 0.75, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(loc.latitude, 48.8, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(estimated_precision, 3.5, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(grid_precision, 1.5, TEST_EPSILON);

	/* PROTO, index out of range on an empty list -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_lfic_by_index(
		frame, 0, &loc, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* PROTO, index 0 populated with a location and USER type */
	frame = utils_test_new_proto_frame_rw(&tm);
	lfic = vmeta_frame_proto_get_lfic_by_index(tm, 0);
	CU_ASSERT_PTR_NOT_NULL(lfic);
	lfic->x = 0.1f;
	lfic->y = 0.2f;
	lfic->grid_precision = 4.0;
	lfic->type = VMETA__LFIC_TYPE__LFIC_TYPE_USER;
	lfic->horizontal_tangential_accuracy = 0.5f;
	lfic->horizontal_radial_accuracy = 0.6f;
	ploc = vmeta_frame_proto_get_lfic_location(lfic);
	CU_ASSERT_PTR_NOT_NULL(ploc);
	ploc->latitude = 10.0;
	ploc->longitude = 20.0;
	ploc->horizontal_accuracy = 2.5f;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);

	res = vmeta_frame_get_lfic_by_index(frame,
					    0,
					    &loc,
					    &x,
					    &y,
					    &type,
					    &estimated_precision,
					    &grid_precision,
					    &h_tang_acc,
					    &h_rad_acc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(type, VMETA_LFIC_TYPE_USER);
	CU_ASSERT_DOUBLE_EQUAL(x, 0.1, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(y, 0.2, TEST_EPSILON);
	CU_ASSERT_EQUAL(loc.valid, 1);
	CU_ASSERT_DOUBLE_EQUAL(loc.latitude, 10.0, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(loc.longitude, 20.0, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(estimated_precision, 2.5, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(grid_precision, 4.0, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(h_tang_acc, 0.5, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(h_rad_acc, 0.6, TEST_EPSILON);

	/* Index out of range on a populated frame -> -ENOENT */
	res = vmeta_frame_get_lfic_by_index(
		frame, 5, &loc, &x, &y, &type, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);

	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_get_lfic_by_type(void)
{
	int res;
	struct vmeta_location loc;
	float x, y;
	double estimated_precision, grid_precision;
	float h_tang_acc, h_rad_acc;
	struct vmeta_frame frame_none;
	struct vmeta_frame frame_bad;
	struct vmeta_frame frame_v3;
	Vmeta__TimedMetadata *tm;
	struct vmeta_frame *frame;
	Vmeta__LFICMetadata *cot;
	Vmeta__LFICMetadata *user;

	memset(&frame_none, 0, sizeof(frame_none));
	frame_none.type = VMETA_FRAME_TYPE_NONE;
	memset(&frame_bad, 0, sizeof(frame_bad));
	frame_bad.type = (enum vmeta_frame_type)9999;
	memset(&frame_v3, 0, sizeof(frame_v3));
	frame_v3.type = VMETA_FRAME_TYPE_V3;

	res = vmeta_frame_get_lfic_by_type(NULL,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_type(&frame_none,
					   VMETA_LFIC_TYPE_COT,
					   NULL,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_type(&frame_none,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   NULL,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vmeta_frame_get_lfic_by_type(&frame_none,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   NULL,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	res = vmeta_frame_get_lfic_by_type(&frame_none,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);

	res = vmeta_frame_get_lfic_by_type(&frame_bad,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	/* V3 only ever supports the COT type */
	frame_v3.v3.has_lfic = 1;
	frame_v3.v3.lfic.target_x = 0.4f;
	frame_v3.v3.lfic.target_y = 0.6f;
	res = vmeta_frame_get_lfic_by_type(&frame_v3,
					   VMETA_LFIC_TYPE_USER,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);

	res = vmeta_frame_get_lfic_by_type(&frame_v3,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   &y,
					   &estimated_precision,
					   &grid_precision,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_DOUBLE_EQUAL(x, 0.4, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(y, 0.6, TEST_EPSILON);

	/* PROTO with two lfic entries of different types: look up by type */
	frame = utils_test_new_proto_frame_rw(&tm);
	cot = vmeta_frame_proto_get_lfic_by_index(tm, 0);
	CU_ASSERT_PTR_NOT_NULL(cot);
	cot->type = VMETA__LFIC_TYPE__LFIC_TYPE_COT;
	cot->x = 0.11f;
	cot->y = 0.22f;
	user = vmeta_frame_proto_get_lfic_by_index(tm, 1);
	CU_ASSERT_PTR_NOT_NULL(user);
	user->type = VMETA__LFIC_TYPE__LFIC_TYPE_USER;
	user->x = 0.33f;
	user->y = 0.44f;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);

	res = vmeta_frame_get_lfic_by_type(frame,
					   VMETA_LFIC_TYPE_USER,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   &h_tang_acc,
					   &h_rad_acc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_DOUBLE_EQUAL(x, 0.33, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(y, 0.44, TEST_EPSILON);

	res = vmeta_frame_get_lfic_by_type(frame,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_DOUBLE_EQUAL(x, 0.11, TEST_EPSILON);

	vmeta_frame_unref(frame);

	/* PROTO, requested type absent from a non-empty list -> -ENOENT */
	frame = utils_test_new_proto_frame_rw(&tm);
	user = vmeta_frame_proto_get_lfic_by_index(tm, 0);
	CU_ASSERT_PTR_NOT_NULL(user);
	user->type = VMETA__LFIC_TYPE__LFIC_TYPE_USER;
	user->x = 0.77f;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_lfic_by_type(frame,
					   VMETA_LFIC_TYPE_COT,
					   &loc,
					   &x,
					   &y,
					   NULL,
					   NULL,
					   NULL,
					   NULL);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* vmeta_frame_get_lfic() always looks up type COT: empty list */
	frame = utils_test_new_proto_frame_rw(&tm);
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_lfic(frame,
				   &loc,
				   &x,
				   &y,
				   &estimated_precision,
				   &grid_precision,
				   &h_tang_acc,
				   &h_rad_acc);
	CU_ASSERT_EQUAL(res, -ENOENT);
	vmeta_frame_unref(frame);

	/* vmeta_frame_get_lfic(), single COT entry -> success */
	frame = utils_test_new_proto_frame_rw(&tm);
	cot = vmeta_frame_proto_get_lfic_by_index(tm, 0);
	CU_ASSERT_PTR_NOT_NULL(cot);
	cot->type = VMETA__LFIC_TYPE__LFIC_TYPE_COT;
	cot->x = 0.55f;
	cot->y = 0.66f;
	vmeta_frame_proto_release_unpacked_rw(frame, tm);
	res = vmeta_frame_get_lfic(frame,
				   &loc,
				   &x,
				   &y,
				   &estimated_precision,
				   &grid_precision,
				   &h_tang_acc,
				   &h_rad_acc);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_DOUBLE_EQUAL(x, 0.55, TEST_EPSILON);
	CU_ASSERT_DOUBLE_EQUAL(y, 0.66, TEST_EPSILON);
	vmeta_frame_unref(frame);
}


static void test_utils_enum_str_helpers(void)
{
	size_t i;

	static const struct {
		enum vmeta_flying_state val;
		const char *str;
	} flying_state_cases[] = {
		{VMETA_FLYING_STATE_LANDED, "LANDED"},
		{VMETA_FLYING_STATE_TAKINGOFF, "TAKINGOFF"},
		{VMETA_FLYING_STATE_HOVERING, "HOVERING"},
		{VMETA_FLYING_STATE_FLYING, "FLYING"},
		{VMETA_FLYING_STATE_LANDING, "LANDING"},
		{VMETA_FLYING_STATE_EMERGENCY, "EMERGENCY"},
		{VMETA_FLYING_STATE_USER_TAKEOFF, "USER_TAKEOFF"},
		{VMETA_FLYING_STATE_MOTOR_RAMPING, "MOTOR_RAMPING"},
		{VMETA_FLYING_STATE_EMERGENCY_LANDING, "EMERGENCY_LANDING"},
	};
	for (i = 0;
	     i < sizeof(flying_state_cases) / sizeof(flying_state_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_flying_state_str(flying_state_cases[i].val),
			flying_state_cases[i].str);
	CU_ASSERT_STRING_EQUAL(
		vmeta_flying_state_str((enum vmeta_flying_state)999),
		"UNKNOWN");

	static const struct {
		enum vmeta_lfic_type val;
		const char *str;
	} lfic_type_cases[] = {
		{VMETA_LFIC_TYPE_COT, "COT"},
		{VMETA_LFIC_TYPE_USER, "USER"},
	};
	for (i = 0; i < sizeof(lfic_type_cases) / sizeof(lfic_type_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_lfic_type_str(lfic_type_cases[i].val),
			lfic_type_cases[i].str);
	CU_ASSERT_STRING_EQUAL(vmeta_lfic_type_str((enum vmeta_lfic_type)999),
			       "UNKNOWN");

	static const struct {
		enum vmeta_piloting_mode val;
		const char *str;
	} piloting_mode_cases[] = {
		{VMETA_PILOTING_MODE_MANUAL, "MANUAL"},
		{VMETA_PILOTING_MODE_RETURN_HOME, "RETURN_HOME"},
		{VMETA_PILOTING_MODE_FLIGHT_PLAN, "FLIGHT_PLAN"},
		{VMETA_PILOTING_MODE_TRACKING, "TRACKING"},
		{VMETA_PILOTING_MODE_MAGIC_CARPET, "MAGIC_CARPET"},
		{VMETA_PILOTING_MODE_MOVE_TO, "MOVE_TO"},
		/* VMETA_PILOTING_MODE_UNKNOWN has its own explicit case, but
		 * it falls through to the same "UNKNOWN" string as an
		 * unrecognized value would via 'default' */
		{VMETA_PILOTING_MODE_UNKNOWN, "UNKNOWN"},
	};
	for (i = 0;
	     i < sizeof(piloting_mode_cases) / sizeof(piloting_mode_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_piloting_mode_str(piloting_mode_cases[i].val),
			piloting_mode_cases[i].str);
	CU_ASSERT_STRING_EQUAL(
		vmeta_piloting_mode_str((enum vmeta_piloting_mode)999),
		"UNKNOWN");

	static const struct {
		enum vmeta_followme_anim val;
		const char *str;
	} followme_anim_cases[] = {
		{VMETA_FOLLOWME_ANIM_NONE, "NONE"},
		{VMETA_FOLLOWME_ANIM_ORBIT, "ORBIT"},
		{VMETA_FOLLOWME_ANIM_BOOMERANG, "BOOMERANG"},
		{VMETA_FOLLOWME_ANIM_PARABOLA, "PARABOLA"},
		{VMETA_FOLLOWME_ANIM_ZENITH, "ZENITH"},
	};
	for (i = 0;
	     i < sizeof(followme_anim_cases) / sizeof(followme_anim_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_followme_anim_str(followme_anim_cases[i].val),
			followme_anim_cases[i].str);
	CU_ASSERT_STRING_EQUAL(
		vmeta_followme_anim_str((enum vmeta_followme_anim)999),
		"UNKNOWN");

	static const struct {
		enum vmeta_automation_anim val;
		const char *str;
	} automation_anim_cases[] = {
		{VMETA_AUTOMATION_ANIM_NONE, "NONE"},
		{VMETA_AUTOMATION_ANIM_ORBIT, "ORBIT"},
		{VMETA_AUTOMATION_ANIM_BOOMERANG, "BOOMERANG"},
		{VMETA_AUTOMATION_ANIM_PARABOLA, "PARABOLA"},
		{VMETA_AUTOMATION_ANIM_DOLLY_SLIDE, "DOLLY_SLIDE"},
		{VMETA_AUTOMATION_ANIM_DOLLY_ZOOM, "DOLLY_ZOOM"},
		{VMETA_AUTOMATION_ANIM_REVEAL_VERT, "REVEAL_VERT"},
		{VMETA_AUTOMATION_ANIM_REVEAL_HORZ, "REVEAL_HORZ"},
		{VMETA_AUTOMATION_ANIM_PANORAMA_HORZ, "PANORAMA_HORZ"},
		{VMETA_AUTOMATION_ANIM_CANDLE, "CANDLE"},
		{VMETA_AUTOMATION_ANIM_FLIP_FRONT, "FLIP_FRONT"},
		{VMETA_AUTOMATION_ANIM_FLIP_BACK, "FLIP_BACK"},
		{VMETA_AUTOMATION_ANIM_FLIP_LEFT, "FLIP_LEFT"},
		{VMETA_AUTOMATION_ANIM_FLIP_RIGHT, "FLIP_RIGHT"},
		{VMETA_AUTOMATION_ANIM_TWISTUP, "TWISTUP"},
		{VMETA_AUTOMATION_ANIM_POSITION_TWISTUP, "POSITION_TWISTUP"},
	};
	for (i = 0; i < sizeof(automation_anim_cases) /
				sizeof(automation_anim_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_automation_anim_str(automation_anim_cases[i].val),
			automation_anim_cases[i].str);
	CU_ASSERT_STRING_EQUAL(
		vmeta_automation_anim_str((enum vmeta_automation_anim)999),
		"UNKNOWN");

	static const struct {
		enum vmeta_thermal_calib_state val;
		const char *str;
	} thermal_calib_state_cases[] = {
		{VMETA_THERMAL_CALIB_STATE_DONE, "DONE"},
		{VMETA_THERMAL_CALIB_STATE_REQUESTED, "REQUESTED"},
		{VMETA_THERMAL_CALIB_STATE_IN_PROGRESS, "IN_PROGRESS"},
	};
	for (i = 0; i < sizeof(thermal_calib_state_cases) /
				sizeof(thermal_calib_state_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_thermal_calib_state_str(
				thermal_calib_state_cases[i].val),
			thermal_calib_state_cases[i].str);
	CU_ASSERT_STRING_EQUAL(vmeta_thermal_calib_state_str(
				       (enum vmeta_thermal_calib_state)999),
			       "UNKNOWN");

	static const struct {
		enum vmeta_frame_type val;
		const char *str;
	} frame_type_cases[] = {
		{VMETA_FRAME_TYPE_NONE, "NONE"},
		{VMETA_FRAME_TYPE_V1_RECORDING, "V1_REC"},
		{VMETA_FRAME_TYPE_V1_STREAMING_BASIC, "V1_STRM_BASIC"},
		{VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED, "V1_STRM_EXTENDED"},
		{VMETA_FRAME_TYPE_V2, "V2"},
		{VMETA_FRAME_TYPE_V3, "V3"},
		{VMETA_FRAME_TYPE_PROTO, "PROTO"},
	};
	for (i = 0; i < sizeof(frame_type_cases) / sizeof(frame_type_cases[0]);
	     i++)
		CU_ASSERT_STRING_EQUAL(
			vmeta_frame_type_str(frame_type_cases[i].val),
			frame_type_cases[i].str);
	CU_ASSERT_STRING_EQUAL(vmeta_frame_type_str((enum vmeta_frame_type)999),
			       "UNKNOWN");
}


static void test_utils_frame_to_csv_and_csv_header(void)
{
	static const enum vmeta_frame_type types[] = {
		VMETA_FRAME_TYPE_V1_RECORDING,
		VMETA_FRAME_TYPE_V1_STREAMING_BASIC,
		VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED,
		VMETA_FRAME_TYPE_V2,
		VMETA_FRAME_TYPE_V3,
	};
	size_t i;
	int res;
	char buf[512];
	char expected[512];
	ssize_t ret, expected_ret;
	struct vmeta_frame *frame;

	/* NULL argument checks */
	ret = vmeta_frame_to_csv(NULL, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -EINVAL);
	res = vmeta_frame_new(VMETA_FRAME_TYPE_NONE, &frame);
	CU_ASSERT_EQUAL(res, 0);
	ret = vmeta_frame_to_csv(frame, NULL, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -EINVAL);
	vmeta_frame_unref(frame);

	ret = vmeta_frame_csv_header(VMETA_FRAME_TYPE_V3, NULL, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* VMETA_FRAME_TYPE_NONE: nothing written, returns 0 */
	res = vmeta_frame_new(VMETA_FRAME_TYPE_NONE, &frame);
	CU_ASSERT_EQUAL(res, 0);
	ret = vmeta_frame_to_csv(frame, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, 0);
	vmeta_frame_unref(frame);
	ret = vmeta_frame_csv_header(VMETA_FRAME_TYPE_NONE, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, 0);

	/* PROTO: not implemented -> -ENOSYS */
	res = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &frame);
	CU_ASSERT_EQUAL(res, 0);
	ret = vmeta_frame_to_csv(frame, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -ENOSYS);
	vmeta_frame_unref(frame);
	ret = vmeta_frame_csv_header(VMETA_FRAME_TYPE_PROTO, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* Unknown type -> -ENOSYS */
	{
		struct vmeta_frame bad_frame;
		memset(&bad_frame, 0, sizeof(bad_frame));
		bad_frame.type = (enum vmeta_frame_type)999;
		ret = vmeta_frame_to_csv(&bad_frame, buf, sizeof(buf));
		CU_ASSERT_EQUAL(ret, -ENOSYS);
	}
	ret = vmeta_frame_csv_header(
		(enum vmeta_frame_type)999, buf, sizeof(buf));
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* Real formats: the dispatcher must forward verbatim (same return
	 * value and content) to the corresponding format-specific
	 * to_csv()/csv_header() function, called directly here on the same
	 * (zeroed) sub-structure for comparison */
	for (i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
		res = vmeta_frame_new(types[i], &frame);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_PTR_NOT_NULL(frame);

		ret = vmeta_frame_to_csv(frame, buf, sizeof(buf));

		switch (types[i]) {
		case VMETA_FRAME_TYPE_V1_RECORDING:
			expected_ret = vmeta_frame_v1_recording_to_csv(
				&frame->v1_rec, expected, sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
			expected_ret = vmeta_frame_v1_streaming_basic_to_csv(
				&frame->v1_strm_basic,
				expected,
				sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
			expected_ret = vmeta_frame_v1_streaming_extended_to_csv(
				&frame->v1_strm_ext,
				expected,
				sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V2:
			expected_ret = vmeta_frame_v2_to_csv(
				&frame->v2, expected, sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V3:
			expected_ret = vmeta_frame_v3_to_csv(
				&frame->v3, expected, sizeof(expected));
			break;
		default:
			CU_FAIL("unexpected type in test table");
			expected_ret = -1;
			break;
		}
		CU_ASSERT_EQUAL(ret, expected_ret);
		if (ret >= 0 && expected_ret >= 0)
			CU_ASSERT_STRING_EQUAL(buf, expected);

		ret = vmeta_frame_csv_header(types[i], buf, sizeof(buf));
		switch (types[i]) {
		case VMETA_FRAME_TYPE_V1_RECORDING:
			expected_ret = vmeta_frame_v1_recording_csv_header(
				expected, sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
			expected_ret =
				vmeta_frame_v1_streaming_basic_csv_header(
					expected, sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
			expected_ret =
				vmeta_frame_v1_streaming_extended_csv_header(
					expected, sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V2:
			expected_ret = vmeta_frame_v2_csv_header(
				expected, sizeof(expected));
			break;
		case VMETA_FRAME_TYPE_V3:
			expected_ret = vmeta_frame_v3_csv_header(
				expected, sizeof(expected));
			break;
		default:
			CU_FAIL("unexpected type in test table");
			expected_ret = -1;
			break;
		}
		CU_ASSERT_EQUAL(ret, expected_ret);
		if (ret >= 0 && expected_ret >= 0)
			CU_ASSERT_STRING_EQUAL(buf, expected);

		vmeta_frame_unref(frame);
	}
}


static void test_utils_frame_get_mime_type(void)
{
	CU_ASSERT_PTR_NULL(vmeta_frame_get_mime_type(VMETA_FRAME_TYPE_NONE));
	CU_ASSERT_PTR_NULL(
		vmeta_frame_get_mime_type(VMETA_FRAME_TYPE_V1_STREAMING_BASIC));
	CU_ASSERT_PTR_NULL(vmeta_frame_get_mime_type(
		VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED));
	CU_ASSERT_STRING_EQUAL(
		vmeta_frame_get_mime_type(VMETA_FRAME_TYPE_V1_RECORDING),
		VMETA_FRAME_V1_RECORDING_MIME_TYPE);
	CU_ASSERT_STRING_EQUAL(vmeta_frame_get_mime_type(VMETA_FRAME_TYPE_V2),
			       VMETA_FRAME_V2_MIME_TYPE);
	CU_ASSERT_STRING_EQUAL(vmeta_frame_get_mime_type(VMETA_FRAME_TYPE_V3),
			       VMETA_FRAME_V3_MIME_TYPE);
	CU_ASSERT_STRING_EQUAL(
		vmeta_frame_get_mime_type(VMETA_FRAME_TYPE_PROTO),
		VMETA_FRAME_PROTO_MIME_TYPE);
	CU_ASSERT_PTR_NULL(
		vmeta_frame_get_mime_type((enum vmeta_frame_type)999));
}


static void test_vmeta_frame_proto_get_packed_size(void)
{
	int res;
	ssize_t ret;
	struct vmeta_frame *frame;
	struct vmeta_frame frame_v3;
	Vmeta__TimedMetadata *tm;
	Vmeta__CameraMetadata *camera;
	const uint8_t *buf;
	size_t len;

	/* Bad args */
	ret = vmeta_frame_proto_get_packed_size(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Wrong frame type -> -EPROTO */
	memset(&frame_v3, 0, sizeof(frame_v3));
	frame_v3.type = VMETA_FRAME_TYPE_V3;
	ret = vmeta_frame_proto_get_packed_size(&frame_v3);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	/* Unpacked path: get_packed_size() computes the size on the fly via
	 * vmeta__timed_metadata__get_packed_size() */
	frame = utils_test_new_proto_frame_rw(&tm);
	camera = vmeta_frame_proto_get_camera(tm);
	CU_ASSERT_PTR_NOT_NULL(camera);
	camera->timestamp = 123456;
	res = vmeta_frame_proto_release_unpacked_rw(frame, tm);
	CU_ASSERT_EQUAL(res, 0);

	ret = vmeta_frame_proto_get_packed_size(frame);
	CU_ASSERT(ret > 0);

	/* Packed path: after get_buffer() forces a pack, get_packed_size()
	 * must return the already-known packed length directly */
	res = vmeta_frame_proto_get_buffer(frame, &buf, &len);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL((size_t)ret, len);

	ret = vmeta_frame_proto_get_packed_size(frame);
	CU_ASSERT_EQUAL((size_t)ret, len);

	res = vmeta_frame_proto_release_buffer(frame, buf);
	CU_ASSERT_EQUAL(res, 0);

	vmeta_frame_unref(frame);
}


static void test_vmeta_frame_proto_get_camera_local_position(void)
{
	struct vmeta_frame *frame;
	Vmeta__TimedMetadata *tm;
	Vmeta__CameraMetadata *camera;
	Vmeta__Vector3 *pos;
	Vmeta__Vector3 *pos2;
	const Vmeta__TimedMetadata *ro_tm;
	int res;

	CU_ASSERT_PTR_NULL(vmeta_frame_proto_get_camera_local_position(NULL));

	frame = utils_test_new_proto_frame_rw(&tm);
	camera = vmeta_frame_proto_get_camera(tm);
	CU_ASSERT_PTR_NOT_NULL(camera);

	pos = vmeta_frame_proto_get_camera_local_position(camera);
	CU_ASSERT_PTR_NOT_NULL(pos);
	/* Idempotent: a second call on the same camera returns the same
	 * lazily-allocated pointer, not a fresh one */
	pos2 = vmeta_frame_proto_get_camera_local_position(camera);
	CU_ASSERT_PTR_EQUAL(pos, pos2);

	pos->x = 1.5;
	pos->y = -2.5;
	pos->z = 3.5;

	res = vmeta_frame_proto_release_unpacked_rw(frame, tm);
	CU_ASSERT_EQUAL(res, 0);

	/* Round trip through a fresh unpacked-read view */
	res = vmeta_frame_proto_get_unpacked(frame, &ro_tm);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL(ro_tm->camera);
	CU_ASSERT_PTR_NOT_NULL(ro_tm->camera->local_position);
	CU_ASSERT_DOUBLE_EQUAL(ro_tm->camera->local_position->x, 1.5, 1e-9);
	CU_ASSERT_DOUBLE_EQUAL(ro_tm->camera->local_position->y, -2.5, 1e-9);
	CU_ASSERT_DOUBLE_EQUAL(ro_tm->camera->local_position->z, 3.5, 1e-9);
	res = vmeta_frame_proto_release_unpacked(frame, ro_tm);
	CU_ASSERT_EQUAL(res, 0);

	vmeta_frame_unref(frame);
}


CU_TestInfo s_utils_tests[] = {
	{(char *)"euler to quat", &test_euler_to_quat},
	{(char *)"quat to euler", &test_quat_to_euler},
	{(char *)"camera_type_split_subtype", &test_camera_type_split_subtype},
	{(char *)"camera_type_combine_subtype",
	 &test_camera_type_combine_subtype},
	{(char *)"camera_type_subtype_pair_cmp",
	 &test_camera_type_subtype_pair_cmp},
	{(char *)"camera_type_split_combine_roundtrip",
	 &test_camera_type_split_combine_roundtrip},
	{(char *)"euler cmp", &test_vmeta_euler_cmp},
	{(char *)"thermal alignment cmp", &test_vmeta_thermal_alignment_cmp},
	{(char *)"thermal conversion cmp", &test_vmeta_thermal_conversion_cmp},
	{(char *)"thermal cmp", &test_vmeta_thermal_cmp},
	{(char *)"fov cmp", &test_vmeta_fov_cmp},
	{(char *)"location cmp", &test_vmeta_location_cmp},
	{(char *)"overlay cmp", &test_vmeta_overlay_cmp},
	{(char *)"camera model cmp", &test_vmeta_camera_model_cmp},
	{(char *)"quat to euler (zyx)", &test_vmeta_quat_to_euler_zyx},
	{(char *)"hash joaat str", &test_vmeta_hash_joaat_str},
	{(char *)"camera_type str", &test_vmeta_camera_type_str},
	{(char *)"camera_subtype str", &test_vmeta_camera_subtype_str},
	{(char *)"camera_spectrum str", &test_vmeta_camera_spectrum_str},
	{(char *)"camera_model_type str", &test_vmeta_camera_model_type_str},
	{(char *)"overlay_type str", &test_vmeta_overlay_type_str},
	{(char *)"video_mode str", &test_vmeta_video_mode_str},
	{(char *)"video_stop_reason str", &test_vmeta_video_stop_reason_str},
	{(char *)"photo_mode str", &test_vmeta_photo_mode_str},
	{(char *)"panorama_type str", &test_vmeta_panorama_type_str},
	{(char *)"dynamic_range str", &test_vmeta_dynamic_range_str},
	{(char *)"tone_mapping str", &test_vmeta_tone_mapping_str},
	{(char *)"frame get frame_utc_timestamp",
	 &test_vmeta_frame_get_frame_utc_timestamp},
	{(char *)"frame get camera_spectrum",
	 &test_vmeta_frame_get_camera_spectrum},
	{(char *)"frame get camera_subtype",
	 &test_vmeta_frame_get_camera_subtype},
	{(char *)"frame get thermal_mask", &test_vmeta_frame_get_thermal_mask},
	{(char *)"frame get color_matrix", &test_vmeta_frame_get_color_matrix},
	{(char *)"frame get lfic_count", &test_vmeta_frame_get_lfic_count},
	{(char *)"frame get lfic_by_index",
	 &test_vmeta_frame_get_lfic_by_index},
	{(char *)"frame get lfic_by_type / lfic",
	 &test_vmeta_frame_get_lfic_by_type},
	{(char *)"utils enum str helpers", &test_utils_enum_str_helpers},
	{(char *)"utils frame to_csv / csv_header dispatch",
	 &test_utils_frame_to_csv_and_csv_header},
	{(char *)"utils frame get_mime_type", &test_utils_frame_get_mime_type},
	{(char *)"frame proto get_packed_size",
	 &test_vmeta_frame_proto_get_packed_size},
	{(char *)"frame proto get_camera_local_position",
	 &test_vmeta_frame_proto_get_camera_local_position},
	CU_TEST_INFO_NULL,
};
