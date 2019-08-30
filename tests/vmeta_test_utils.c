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

#define TEST_EPSILON (0.000001f)


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


CU_TestInfo s_utils_tests[] = {
	{(char *)"euler to quat", &test_euler_to_quat},
	{(char *)"quat to euler", &test_quat_to_euler},
	CU_TEST_INFO_NULL,
};
