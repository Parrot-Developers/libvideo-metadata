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
                                                                               \
		/* 1. Perfect equality */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
                                                                               \
		/* 2. Equality within _epsilon */                              \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + _epsilon;             \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
                                                                               \
		/* 3. Inequality outside _epsilon */                           \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + (_epsilon * 2.0);     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 0);      \
                                                                               \
		/* 4. Zero sign parity */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta1_copy)._field = 0.0;                                    \
		(_meta2)._field = -0.0;                                        \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
                                                                               \
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
                                                                               \
		/* 1. Perfect equality */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
                                                                               \
		/* 2. Equality within _epsilon */                              \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + _epsilon;             \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
                                                                               \
		/* 3. Inequality outside _epsilon */                           \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta2)._field = (_meta1_copy)._field + (_epsilon * 2.0);     \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 0);      \
                                                                               \
		/* 4. Zero sign parity */                                      \
		RESET_VALUES(_meta1, _meta1_copy, _meta2);                     \
		(_meta1_copy)._field = 0.0;                                    \
		(_meta2)._field = -0.0;                                        \
		CU_ASSERT_EQUAL(_cmp_func(&(_meta1_copy), &(_meta2)), 1);      \
                                                                               \
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
		if (type_subtype_cmp_pairs[i].identical) {
			CU_ASSERT_EQUAL(ret, 1);
		} else {
			CU_ASSERT_EQUAL(ret, 0);
		}

		ret = vmeta_camera_type_subtype_pair_cmp(
			type_subtype_cmp_pairs[i].type2,
			type_subtype_cmp_pairs[i].subtype2,
			type_subtype_cmp_pairs[i].type1,
			type_subtype_cmp_pairs[i].subtype1);
		if (type_subtype_cmp_pairs[i].identical) {
			CU_ASSERT_EQUAL(ret, 1);
		} else {
			CU_ASSERT_EQUAL(ret, 0);
		}
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


CU_TestInfo s_utils_tests[] = {
	{(char *)"euler to quat", &test_euler_to_quat},
	{(char *)"quat to euler", &test_quat_to_euler},
	{(char *)"camera_type_split_subtype", &test_camera_type_split_subtype},
	{(char *)"camera_type_combine_subtype",
	 &test_camera_type_combine_subtype},
	{(char *)"camera_type_subtype_pair_cmp",
	 &test_camera_type_subtype_pair_cmp},
	{(char *)"euler cmp", &test_vmeta_euler_cmp},
	{(char *)"thermal alignment cmp", &test_vmeta_thermal_alignment_cmp},
	{(char *)"thermal conversion cmp", &test_vmeta_thermal_conversion_cmp},
	{(char *)"thermal cmp", &test_vmeta_thermal_cmp},
	{(char *)"fov cmp", &test_vmeta_fov_cmp},
	{(char *)"location cmp", &test_vmeta_location_cmp},
	{(char *)"overlay cmp", &test_vmeta_overlay_cmp},
	{(char *)"camera model cmp", &test_vmeta_camera_model_cmp},
	CU_TEST_INFO_NULL,
};
