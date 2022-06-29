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

#ifndef _VMETA_TEST_H_
#define _VMETA_TEST_H_

#include <video-metadata/vmeta.h>

#include <futils/futils.h>

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>
#include <inttypes.h>
#include <stdbool.h>

#define ULOG_TAG vmeta_test
#include <ulog.h>


#define VMETA_ASSERT_BOTH_NULL_NOTNULL(p1, p2)                                 \
	CU_ASSERT((p1 == NULL && p2 == NULL) || (p1 && p2))

#define MONKEY_TEST_COUNT 1000

extern CU_TestInfo s_utils_tests[];
extern CU_TestInfo s_proto_tests[];
extern CU_TestInfo s_proto_monkey[];
extern CU_TestInfo s_proto_gen[];
extern CU_TestInfo s_v3_tests[];
extern CU_TestInfo s_v3_monkey[];
extern CU_TestInfo s_v3_gen[];

/**
 * Since the v1, v2 and v3 format stores floats and doubles as fixed point
 * values, we need to handle a granularity for comparisons, based on the shift
 * value applied when encoding the source value.
 */
static inline double granularity(unsigned int shift)
{
	return 1. / pow(2, shift);
}

/* Comparison functions */
void compare_vmeta_quaternion(struct vmeta_quaternion *q1,
			      struct vmeta_quaternion *q2);
void compare_proto_quaternion(Vmeta__Quaternion *q1, Vmeta__Quaternion *q2);
void compare_vmeta_proto_quaternion(struct vmeta_quaternion *q1,
				    Vmeta__Quaternion *q2);
void compare_vmeta_euler(struct vmeta_euler *e1, struct vmeta_euler *e2);
void compare_vmeta_location(struct vmeta_location *l1,
			    struct vmeta_location *l2,
			    bool include_sv_count);
void compare_proto_location(Vmeta__Location *l1, Vmeta__Location *l2);
void compare_vmeta_proto_location(struct vmeta_location *l1,
				  Vmeta__Location *l2,
				  bool include_sv_count);
void compare_vmeta_ned(struct vmeta_ned *n1, struct vmeta_ned *n2);
void compare_proto_ned(Vmeta__NED *n1, Vmeta__NED *n2);
void compare_vmeta_proto_ned(struct vmeta_ned *n1, Vmeta__NED *n2);
void compare_proto_vector3(Vmeta__Vector3 *v1, Vmeta__Vector3 *v2);
void compare_vmeta_thermal_spot(struct vmeta_thermal_spot *t1,
				struct vmeta_thermal_spot *t2);
void compare_proto_thermal_spot(Vmeta__ThermalSpot *t1, Vmeta__ThermalSpot *t2);
void compare_vmeta_proto_thermal_spot(struct vmeta_thermal_spot *t1,
				      Vmeta__ThermalSpot *t2);
void compare_proto_bounding_box(Vmeta__BoundingBox *b1, Vmeta__BoundingBox *b2);
void compare_flying_state(enum vmeta_flying_state f1, Vmeta__FlyingState f2);
void compare_piloting_mode(enum vmeta_piloting_mode p1, Vmeta__PilotingMode p2);
void compare_animation_enum(enum vmeta_automation_anim a1, Vmeta__Animation a2);
void compare_thermal_calib_state(enum vmeta_thermal_calib_state t1,
				 Vmeta__ThermalCalibrationState t2);

void compare_vmeta_frame_getters(struct vmeta_frame *f1,
				 struct vmeta_frame *f2);
void compare_vmeta_frame_v3_getters(struct vmeta_frame *f);
void compare_vmeta_frame_proto_getters(struct vmeta_frame *f);


#endif /* _FUTILS_TEST_H_ */
