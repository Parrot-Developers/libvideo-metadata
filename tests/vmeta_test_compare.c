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


void compare_vmeta_quaternion(struct vmeta_quaternion *q1,
			      struct vmeta_quaternion *q2)
{
	CU_ASSERT_PTR_NOT_NULL(q1);
	CU_ASSERT_PTR_NOT_NULL(q2);
	if (!q1 || !q2)
		return;

	CU_ASSERT_DOUBLE_EQUAL(q1->w, q2->w, granularity(14));
	CU_ASSERT_DOUBLE_EQUAL(q1->x, q2->x, granularity(14));
	CU_ASSERT_DOUBLE_EQUAL(q1->y, q2->y, granularity(14));
	CU_ASSERT_DOUBLE_EQUAL(q1->z, q2->z, granularity(14));
}


void compare_proto_quaternion(Vmeta__Quaternion *q1, Vmeta__Quaternion *q2)
{
	VMETA_ASSERT_BOTH_NULL_NOTNULL(q1, q2);
	if (!q1 || !q2)
		return;

	CU_ASSERT_EQUAL(q1->w, q2->w);
	CU_ASSERT_EQUAL(q1->x, q2->x);
	CU_ASSERT_EQUAL(q1->y, q2->y);
	CU_ASSERT_EQUAL(q1->z, q2->z);
}


void compare_vmeta_proto_quaternion(struct vmeta_quaternion *q1,
				    Vmeta__Quaternion *q2)
{
	CU_ASSERT_PTR_NOT_NULL(q1);
	CU_ASSERT_PTR_NOT_NULL(q2);
	if (!q1 || !q2)
		return;

	CU_ASSERT_DOUBLE_EQUAL(q1->w, q2->w, granularity(14));
	CU_ASSERT_DOUBLE_EQUAL(q1->x, q2->x, granularity(14));
	CU_ASSERT_DOUBLE_EQUAL(q1->y, q2->y, granularity(14));
	CU_ASSERT_DOUBLE_EQUAL(q1->z, q2->z, granularity(14));
}


void compare_vmeta_euler(struct vmeta_euler *e1, struct vmeta_euler *e2)
{
	CU_ASSERT_PTR_NOT_NULL(e1);
	CU_ASSERT_PTR_NOT_NULL(e2);
	if (!e1 || !e2)
		return;

	/* Test with a lower granularity than compare_quaternion to take the
	 * quat->euler conversion precision loss into account */
	int gr = 7;
	CU_ASSERT_DOUBLE_EQUAL(e1->phi, e2->phi, granularity(gr));
	CU_ASSERT_DOUBLE_EQUAL(e1->theta, e2->theta, granularity(gr));
	CU_ASSERT_DOUBLE_EQUAL(e1->psi, e2->psi, granularity(gr));
	CU_ASSERT_DOUBLE_EQUAL(e1->roll, e2->roll, granularity(gr));
	CU_ASSERT_DOUBLE_EQUAL(e1->pitch, e2->pitch, granularity(gr));
	CU_ASSERT_DOUBLE_EQUAL(e1->yaw, e2->yaw, granularity(gr));
}


void compare_vmeta_location(struct vmeta_location *l1,
			    struct vmeta_location *l2,
			    bool include_sv_count)
{
	CU_ASSERT_PTR_NOT_NULL(l1);
	CU_ASSERT_PTR_NOT_NULL(l2);
	if (!l1 || !l2)
		return;

	CU_ASSERT_EQUAL(l1->valid, l2->valid);
	if (!l1->valid || !l2->valid)
		return;

	CU_ASSERT_DOUBLE_EQUAL(l1->latitude, l2->latitude, granularity(22));
	CU_ASSERT_DOUBLE_EQUAL(l1->longitude, l2->longitude, granularity(22));

	/* Altitude granularity is 8 if sv_count is included, 16 otherwise */
	if (include_sv_count) {
		CU_ASSERT_DOUBLE_EQUAL(
			l1->altitude, l2->altitude, granularity(8));
		CU_ASSERT_EQUAL(l1->sv_count, l2->sv_count);
	} else {
		CU_ASSERT_DOUBLE_EQUAL(
			l1->altitude, l2->altitude, granularity(16));
	}
}


void compare_proto_location(Vmeta__Location *l1, Vmeta__Location *l2)
{
	VMETA_ASSERT_BOTH_NULL_NOTNULL(l1, l2);
	if (!l1 || !l2)
		return;

	CU_ASSERT_EQUAL(l1->altitude, l2->altitude);
	CU_ASSERT_EQUAL(l1->latitude, l2->latitude);
	CU_ASSERT_EQUAL(l1->longitude, l2->longitude);
	CU_ASSERT_EQUAL(l1->sv_count, l2->sv_count);
}


void compare_vmeta_proto_location(struct vmeta_location *l1,
				  Vmeta__Location *l2,
				  bool include_sv_count)
{
	CU_ASSERT_PTR_NOT_NULL(l1);
	if (!l1)
		return;

	if (l1->valid) {
		/* Keep braces due to CU_ASSERT_xxx macros */
		CU_ASSERT_PTR_NOT_NULL(l2);
	} else {
		CU_ASSERT_PTR_NULL(l2);
	}
	if (!l2)
		return;

	CU_ASSERT_DOUBLE_EQUAL(l1->latitude, l2->latitude, granularity(22));
	CU_ASSERT_DOUBLE_EQUAL(l1->longitude, l2->longitude, granularity(22));

	/* Altitude granularity is 8 if sv_count is included, 16 otherwise */
	if (include_sv_count) {
		CU_ASSERT_DOUBLE_EQUAL(
			l1->altitude, l2->altitude, granularity(8));
		CU_ASSERT_EQUAL(l1->sv_count, l2->sv_count);
	} else {
		CU_ASSERT_DOUBLE_EQUAL(
			l1->altitude, l2->altitude, granularity(16));
	}
}


void compare_vmeta_ned(struct vmeta_ned *n1, struct vmeta_ned *n2)
{
	CU_ASSERT_PTR_NOT_NULL(n1);
	CU_ASSERT_PTR_NOT_NULL(n2);
	if (!n1 || !n2)
		return;

	CU_ASSERT_DOUBLE_EQUAL(n1->north, n2->north, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(n1->east, n2->east, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(n1->down, n2->down, granularity(8));
}


void compare_proto_ned(Vmeta__NED *n1, Vmeta__NED *n2)
{
	VMETA_ASSERT_BOTH_NULL_NOTNULL(n1, n2);
	if (!n1 || !n2)
		return;

	CU_ASSERT_EQUAL(n1->north, n2->north);
	CU_ASSERT_EQUAL(n1->east, n2->east);
	CU_ASSERT_EQUAL(n1->down, n2->down);
}

void compare_vmeta_proto_ned(struct vmeta_ned *n1, Vmeta__NED *n2)
{
	CU_ASSERT_PTR_NOT_NULL(n1);
	CU_ASSERT_PTR_NOT_NULL(n2);
	if (!n1 || !n2)
		return;

	CU_ASSERT_DOUBLE_EQUAL(n1->north, n2->north, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(n1->east, n2->east, granularity(8));
	CU_ASSERT_DOUBLE_EQUAL(n1->down, n2->down, granularity(8));
}


void compare_proto_vector3(Vmeta__Vector3 *v1, Vmeta__Vector3 *v2)
{
	VMETA_ASSERT_BOTH_NULL_NOTNULL(v1, v2);
	if (!v1 || !v2)
		return;

	CU_ASSERT_EQUAL(v1->x, v2->x);
	CU_ASSERT_EQUAL(v1->y, v2->y);
	CU_ASSERT_EQUAL(v1->z, v2->z);
}


void compare_vmeta_thermal_spot(struct vmeta_thermal_spot *t1,
				struct vmeta_thermal_spot *t2)
{
	CU_ASSERT_PTR_NOT_NULL(t1);
	CU_ASSERT_PTR_NOT_NULL(t2);
	if (!t1 || !t2)
		return;

	CU_ASSERT_EQUAL(t1->valid, t2->valid);
	if (!t1->valid || !t2->valid)
		return;

	CU_ASSERT_DOUBLE_EQUAL(t1->x, t2->x, granularity(5));
	CU_ASSERT_DOUBLE_EQUAL(t1->y, t2->y, granularity(5));
	CU_ASSERT_DOUBLE_EQUAL(t1->temp, t2->temp, granularity(5));
}


void compare_proto_thermal_spot(Vmeta__ThermalSpot *t1, Vmeta__ThermalSpot *t2)
{
	VMETA_ASSERT_BOTH_NULL_NOTNULL(t1, t2);
	if (!t1 || !t2)
		return;

	CU_ASSERT_EQUAL(t1->x, t2->x);
	CU_ASSERT_EQUAL(t1->y, t2->y);
	CU_ASSERT_EQUAL(t1->temp, t2->temp);
}


void compare_vmeta_proto_thermal_spot(struct vmeta_thermal_spot *t1,
				      Vmeta__ThermalSpot *t2)
{
	CU_ASSERT_PTR_NOT_NULL(t1);
	if (!t1)
		return;

	if (t1->valid) {
		/* Keep braces due to CU_ASSERT_xxx macros */
		CU_ASSERT_PTR_NOT_NULL(t2);
	} else {
		CU_ASSERT_PTR_NULL(t2);
	}
	if (!t2)
		return;

	CU_ASSERT_DOUBLE_EQUAL(t1->x, t2->x, granularity(5));
	CU_ASSERT_DOUBLE_EQUAL(t1->y, t2->y, granularity(5));
	CU_ASSERT_DOUBLE_EQUAL(t1->temp, t2->temp, granularity(5));
}


void compare_proto_bounding_box(Vmeta__BoundingBox *b1, Vmeta__BoundingBox *b2)
{
	VMETA_ASSERT_BOTH_NULL_NOTNULL(b1, b2);
	if (!b1 || !b2)
		return;

	CU_ASSERT_EQUAL(b1->object_class, b2->object_class);
	CU_ASSERT_EQUAL(b1->confidence, b2->confidence);
	CU_ASSERT_EQUAL(b1->height, b2->height);
	CU_ASSERT_EQUAL(b1->width, b2->width);
	CU_ASSERT_EQUAL(b1->x, b2->x);
	CU_ASSERT_EQUAL(b1->y, b2->y);
	CU_ASSERT_EQUAL(b1->uid, b2->uid);
}


void compare_flying_state(enum vmeta_flying_state f1, Vmeta__FlyingState f2)
{
	Vmeta__FlyingState f1c = vmeta_frame_flying_state_vmeta_to_proto(f1);
	enum vmeta_flying_state f2c =
		vmeta_frame_flying_state_proto_to_vmeta(f2);
	CU_ASSERT_EQUAL(f1, f2c);
	CU_ASSERT_EQUAL(f1c, f2);
}


void compare_piloting_mode(enum vmeta_piloting_mode p1, Vmeta__PilotingMode p2)
{
	Vmeta__PilotingMode p1c = vmeta_frame_piloting_mode_vmeta_to_proto(p1);
	enum vmeta_piloting_mode p2c =
		vmeta_frame_piloting_mode_proto_to_vmeta(p2);
	CU_ASSERT_EQUAL(p1, p2c);
	CU_ASSERT_EQUAL(p1c, p2);
}


void compare_animation_enum(enum vmeta_automation_anim a1, Vmeta__Animation a2)
{
	Vmeta__Animation a1c = vmeta_frame_automation_anim_vmeta_to_proto(a1);
	enum vmeta_automation_anim a2c =
		vmeta_frame_automation_anim_proto_to_vmeta(a2);
	CU_ASSERT_EQUAL(a1, a2c);
	CU_ASSERT_EQUAL(a1c, a2);
}


void compare_thermal_calib_state(enum vmeta_thermal_calib_state t1,
				 Vmeta__ThermalCalibrationState t2)
{
	Vmeta__ThermalCalibrationState t1c =
		vmeta_frame_thermal_calib_state_vmeta_to_proto(t1);
	enum vmeta_thermal_calib_state t2c =
		vmeta_frame_thermal_calib_state_proto_to_vmeta(t2);
	CU_ASSERT_EQUAL(t1, t2c);
	CU_ASSERT_EQUAL(t1c, t2);
}


void compare_vmeta_frame_getters(struct vmeta_frame *f1, struct vmeta_frame *f2)
{
	int err1, err2;
	struct vmeta_location loc1, loc2;
	struct vmeta_ned ned1, ned2;
	struct vmeta_euler eul1, eul2;
	struct vmeta_quaternion q1, q2;
	float fl1, fl2;
	double d1, d2;
	uint8_t u8_1, u8_2;
	uint16_t u16_1, u16_2;
	uint32_t u32_1, u32_2;
	uint64_t u64_1, u64_2;
	int8_t i8_1, i8_2;
	enum vmeta_flying_state s1, s2;
	enum vmeta_piloting_mode m1, m2;

	CU_ASSERT_PTR_NOT_NULL(f1);
	CU_ASSERT_PTR_NOT_NULL(f2);
	if (!f1 || !f2)
		return;

	err1 = vmeta_frame_get_location(f1, &loc1);
	err2 = vmeta_frame_get_location(f2, &loc2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_location(&loc1, &loc2, true);

	err1 = vmeta_frame_get_speed_ned(f1, &ned1);
	err2 = vmeta_frame_get_speed_ned(f2, &ned2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_ned(&ned1, &ned2);

	/* Air speed is not available on proto */
	if (f1->type != VMETA_FRAME_TYPE_PROTO &&
	    f2->type != VMETA_FRAME_TYPE_PROTO) {
		err1 = vmeta_frame_get_air_speed(f1, &fl1);
		err2 = vmeta_frame_get_air_speed(f2, &fl2);
		CU_ASSERT_EQUAL(err1, err2);
		if (err1 == 0 && err2 == 0)
			CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(8));
	}

	err1 = vmeta_frame_get_ground_distance(f1, &d1);
	err2 = vmeta_frame_get_ground_distance(f2, &d2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(d1, d2, granularity(16));

	err1 = vmeta_frame_get_drone_euler(f1, &eul1);
	err2 = vmeta_frame_get_drone_euler(f2, &eul2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_euler(&eul1, &eul2);

	err1 = vmeta_frame_get_drone_quat(f1, &q1);
	err2 = vmeta_frame_get_drone_quat(f2, &q2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_quaternion(&q1, &q2);

	err1 = vmeta_frame_get_frame_euler(f1, &eul1);
	err2 = vmeta_frame_get_frame_euler(f2, &eul2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_euler(&eul1, &eul2);

	err1 = vmeta_frame_get_frame_quat(f1, &q1);
	err2 = vmeta_frame_get_frame_quat(f2, &q2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_quaternion(&q1, &q2);

	err1 = vmeta_frame_get_frame_base_euler(f1, &eul1);
	err2 = vmeta_frame_get_frame_base_euler(f2, &eul2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_euler(&eul1, &eul2);

	err1 = vmeta_frame_get_frame_base_quat(f1, &q1);
	err2 = vmeta_frame_get_frame_base_quat(f2, &q2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		compare_vmeta_quaternion(&q1, &q2);

	err1 = vmeta_frame_get_frame_timestamp(f1, &u64_1);
	err2 = vmeta_frame_get_frame_timestamp(f2, &u64_2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(u64_1, u64_2);

	err1 = vmeta_frame_get_camera_pan(f1, &fl1);
	err2 = vmeta_frame_get_camera_pan(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(8));

	err1 = vmeta_frame_get_camera_tilt(f1, &fl1);
	err2 = vmeta_frame_get_camera_tilt(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(8));

	err1 = vmeta_frame_get_exposure_time(f1, &fl1);
	err2 = vmeta_frame_get_exposure_time(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(8));

	err1 = vmeta_frame_get_gain(f1, &u16_1);
	err2 = vmeta_frame_get_gain(f2, &u16_2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(u16_1, u16_2);

	err1 = vmeta_frame_get_awb_r_gain(f1, &fl1);
	err2 = vmeta_frame_get_awb_r_gain(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(14));

	err1 = vmeta_frame_get_awb_b_gain(f1, &fl1);
	err2 = vmeta_frame_get_awb_b_gain(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(14));

	err1 = vmeta_frame_get_picture_h_fov(f1, &fl1);
	err2 = vmeta_frame_get_picture_h_fov(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(8));

	err1 = vmeta_frame_get_picture_v_fov(f1, &fl1);
	err2 = vmeta_frame_get_picture_v_fov(f2, &fl2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_DOUBLE_EQUAL(fl1, fl2, granularity(8));

	err1 = vmeta_frame_get_link_goodput(f1, &u32_1);
	err2 = vmeta_frame_get_link_goodput(f2, &u32_2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(u32_1, u32_2);

	err1 = vmeta_frame_get_link_quality(f1, &u8_1);
	err2 = vmeta_frame_get_link_quality(f2, &u8_2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(u8_1, u8_2);

	err1 = vmeta_frame_get_wifi_rssi(f1, &i8_1);
	err2 = vmeta_frame_get_wifi_rssi(f2, &i8_2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(i8_1, i8_2);

	err1 = vmeta_frame_get_battery_percentage(f1, &u8_1);
	err2 = vmeta_frame_get_battery_percentage(f2, &u8_2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(u8_1, u8_2);

	err1 = vmeta_frame_get_flying_state(f1, &s1);
	err2 = vmeta_frame_get_flying_state(f2, &s2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(s1, s2);

	err1 = vmeta_frame_get_piloting_mode(f1, &m1);
	err2 = vmeta_frame_get_piloting_mode(f2, &m2);
	CU_ASSERT_EQUAL(err1, err2);
	if (err1 == 0 && err2 == 0)
		CU_ASSERT_EQUAL(m1, m2);
}


void compare_vmeta_frame_v3_getters(struct vmeta_frame *f)
{
	int err, expected;
	struct vmeta_location loc;
	struct vmeta_ned ned;
	struct vmeta_euler eul;
	struct vmeta_quaternion q;
	float fl;
	double d;
	uint8_t u8;
	uint16_t u16;
	uint32_t u32;
	uint64_t u64;
	int8_t i8;
	enum vmeta_flying_state state;
	enum vmeta_piloting_mode mode;

	CU_ASSERT_PTR_NOT_NULL(f);
	if (!f)
		return;

	CU_ASSERT_EQUAL(f->type, VMETA_FRAME_TYPE_V3);
	if (f->type != VMETA_FRAME_TYPE_V3)
		return;

	err = vmeta_frame_get_location(f, &loc);
	expected = f->v3.base.location.valid ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_location(&loc, &f->v3.base.location, true);

	err = vmeta_frame_get_speed_ned(f, &ned);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_ned(&ned, &f->v3.base.speed);

	err = vmeta_frame_get_air_speed(f, &fl);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, f->v3.base.air_speed, granularity(8));

	err = vmeta_frame_get_ground_distance(f, &d);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			d, f->v3.base.ground_distance, granularity(16));

	err = vmeta_frame_get_drone_euler(f, &eul);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0) {
		struct vmeta_euler eul2;
		vmeta_quat_to_euler(&f->v3.base.drone_quat, &eul2);
		compare_vmeta_euler(&eul, &eul2);
	}

	err = vmeta_frame_get_drone_quat(f, &q);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_quaternion(&q, &f->v3.base.drone_quat);

	err = vmeta_frame_get_frame_euler(f, &eul);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0) {
		struct vmeta_euler eul2;
		vmeta_quat_to_euler(&f->v3.base.frame_quat, &eul2);
		compare_vmeta_euler(&eul, &eul2);
	}

	err = vmeta_frame_get_frame_quat(f, &q);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_quaternion(&q, &f->v3.base.frame_quat);

	err = vmeta_frame_get_frame_base_euler(f, &eul);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0) {
		struct vmeta_euler eul2;
		vmeta_quat_to_euler(&f->v3.base.frame_base_quat, &eul2);
		compare_vmeta_euler(&eul, &eul2);
	}

	err = vmeta_frame_get_frame_base_quat(f, &q);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_quaternion(&q, &f->v3.base.frame_base_quat);

	err = vmeta_frame_get_frame_timestamp(f, &u64);
	expected = f->v3.has_timestamp ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u64, f->v3.timestamp.frame_timestamp);

	err = vmeta_frame_get_camera_pan(f, &fl);
	expected = -ENOENT;
	CU_ASSERT_EQUAL(err, expected);

	err = vmeta_frame_get_camera_tilt(f, &fl);
	expected = -ENOENT;
	CU_ASSERT_EQUAL(err, expected);

	err = vmeta_frame_get_exposure_time(f, &fl);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, f->v3.base.exposure_time, granularity(8));

	err = vmeta_frame_get_gain(f, &u16);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u16, f->v3.base.gain);

	err = vmeta_frame_get_awb_r_gain(f, &fl);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, f->v3.base.awb_r_gain, granularity(14));

	err = vmeta_frame_get_awb_b_gain(f, &fl);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, f->v3.base.awb_b_gain, granularity(14));

	err = vmeta_frame_get_picture_h_fov(f, &fl);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, f->v3.base.picture_hfov, granularity(8));

	err = vmeta_frame_get_picture_v_fov(f, &fl);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, f->v3.base.picture_vfov, granularity(8));

	err = vmeta_frame_get_link_goodput(f, &u32);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u32, f->v3.base.link_goodput);

	err = vmeta_frame_get_link_quality(f, &u8);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u8, f->v3.base.link_quality);

	err = vmeta_frame_get_wifi_rssi(f, &i8);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(i8, f->v3.base.wifi_rssi);

	err = vmeta_frame_get_battery_percentage(f, &u8);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u8, f->v3.base.battery_percentage);

	err = vmeta_frame_get_flying_state(f, &state);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(state, f->v3.base.state);

	err = vmeta_frame_get_piloting_mode(f, &mode);
	expected = 0;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(mode, f->v3.base.mode);
}


void compare_vmeta_frame_proto_getters(struct vmeta_frame *f)
{
	int err, expected;
	struct vmeta_location loc;
	struct vmeta_ned ned;
	struct vmeta_euler eul;
	struct vmeta_quaternion q;
	float fl;
	double d;
	uint8_t u8;
	uint16_t u16;
	uint32_t u32;
	uint64_t u64;
	int8_t i8;
	enum vmeta_flying_state state;
	enum vmeta_piloting_mode mode;
	const Vmeta__TimedMetadata *proto;
	size_t index;

	CU_ASSERT_PTR_NOT_NULL(f);
	if (!f)
		return;

	CU_ASSERT_EQUAL(f->type, VMETA_FRAME_TYPE_PROTO);
	if (f->type != VMETA_FRAME_TYPE_PROTO)
		return;

	err = vmeta_frame_proto_get_unpacked(f, &proto);
	CU_ASSERT_EQUAL(err, 0);
	if (err != 0)
		return;

	err = vmeta_frame_get_location(f, &loc);
	expected = (proto->drone && proto->drone->location) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_proto_location(
			&loc, proto->drone->location, true);

	err = vmeta_frame_get_speed_ned(f, &ned);
	expected = (proto->drone && proto->drone->speed) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_proto_ned(&ned, proto->drone->speed);

	err = vmeta_frame_get_air_speed(f, &fl);
	expected = -ENOENT;
	CU_ASSERT_EQUAL(err, expected);

	err = vmeta_frame_get_ground_distance(f, &d);
	expected = proto->drone ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			d, proto->drone->ground_distance, granularity(16));

	err = vmeta_frame_get_drone_euler(f, &eul);
	expected = (proto->drone && proto->drone->quat) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0) {
		struct vmeta_quaternion q2 = {
			.w = proto->drone->quat->w,
			.x = proto->drone->quat->x,
			.y = proto->drone->quat->y,
			.z = proto->drone->quat->z,
		};
		struct vmeta_euler eul2;
		vmeta_quat_to_euler(&q2, &eul2);
		compare_vmeta_euler(&eul, &eul2);
	}

	err = vmeta_frame_get_drone_quat(f, &q);
	expected = (proto->drone && proto->drone->quat) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_proto_quaternion(&q, proto->drone->quat);

	err = vmeta_frame_get_frame_euler(f, &eul);
	expected = (proto->camera && proto->camera->quat) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0) {
		struct vmeta_quaternion q2 = {
			.w = proto->camera->quat->w,
			.x = proto->camera->quat->x,
			.y = proto->camera->quat->y,
			.z = proto->camera->quat->z,
		};
		struct vmeta_euler eul2;
		vmeta_quat_to_euler(&q2, &eul2);
		compare_vmeta_euler(&eul, &eul2);
	}

	err = vmeta_frame_get_frame_quat(f, &q);
	expected = (proto->camera && proto->camera->quat) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_proto_quaternion(&q, proto->camera->quat);

	err = vmeta_frame_get_frame_base_euler(f, &eul);
	expected = (proto->camera && proto->camera->base_quat) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0) {
		struct vmeta_quaternion q2 = {
			.w = proto->camera->base_quat->w,
			.x = proto->camera->base_quat->x,
			.y = proto->camera->base_quat->y,
			.z = proto->camera->base_quat->z,
		};
		struct vmeta_euler eul2;
		vmeta_quat_to_euler(&q2, &eul2);
		compare_vmeta_euler(&eul, &eul2);
	}

	err = vmeta_frame_get_frame_base_quat(f, &q);
	expected = (proto->camera && proto->camera->base_quat) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_vmeta_proto_quaternion(&q, proto->camera->base_quat);

	err = vmeta_frame_get_frame_timestamp(f, &u64);
	expected = (proto->camera && proto->camera->timestamp) ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u64, proto->camera->timestamp);

	err = vmeta_frame_get_camera_pan(f, &fl);
	expected = -ENOENT;
	CU_ASSERT_EQUAL(err, expected);

	err = vmeta_frame_get_camera_tilt(f, &fl);
	expected = -ENOENT;
	CU_ASSERT_EQUAL(err, expected);

	err = vmeta_frame_get_exposure_time(f, &fl);
	expected = proto->camera ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, proto->camera->exposure_time, granularity(8));

	err = vmeta_frame_get_gain(f, &u16);
	expected = proto->camera ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u16, proto->camera->iso_gain);

	err = vmeta_frame_get_awb_r_gain(f, &fl);
	expected = proto->camera ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, proto->camera->awb_r_gain, granularity(14));

	err = vmeta_frame_get_awb_b_gain(f, &fl);
	expected = proto->camera ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, proto->camera->awb_b_gain, granularity(14));

	err = vmeta_frame_get_picture_h_fov(f, &fl);
	expected = proto->camera ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, proto->camera->hfov * 180. / M_PI, granularity(8));

	err = vmeta_frame_get_picture_v_fov(f, &fl);
	expected = proto->camera ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_DOUBLE_EQUAL(
			fl, proto->camera->vfov * 180. / M_PI, granularity(8));

	err = vmeta_frame_get_link_goodput(f, &u32);
	expected = -ENOENT;
	for (index = 0; index < proto->n_links; index++) {
		if (proto->links[index]->protocol_case ==
		    VMETA__LINK_METADATA__PROTOCOL_WIFI) {
			expected = 0;
			break;
		}
	}
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u32, proto->links[index]->wifi->goodput);

	err = vmeta_frame_get_link_quality(f, &u8);
	expected = -ENOENT;
	for (index = 0; index < proto->n_links; index++) {
		if (proto->links[index]->protocol_case ==
		    VMETA__LINK_METADATA__PROTOCOL_WIFI) {
			expected = 0;
			break;
		}
	}
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u8, proto->links[index]->wifi->quality);

	err = vmeta_frame_get_wifi_rssi(f, &i8);
	expected = -ENOENT;
	for (index = 0; index < proto->n_links; index++) {
		if (proto->links[index]->protocol_case ==
		    VMETA__LINK_METADATA__PROTOCOL_WIFI) {
			expected = 0;
			break;
		}
	}
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(i8, proto->links[index]->wifi->rssi);

	err = vmeta_frame_get_battery_percentage(f, &u8);
	expected = proto->drone ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		CU_ASSERT_EQUAL(u8, proto->drone->battery_percentage);

	err = vmeta_frame_get_flying_state(f, &state);
	expected = proto->drone ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_flying_state(state, proto->drone->flying_state);

	err = vmeta_frame_get_piloting_mode(f, &mode);
	expected = proto->drone ? 0 : -ENOENT;
	CU_ASSERT_EQUAL(err, expected);
	if (err == 0)
		compare_piloting_mode(mode, proto->drone->piloting_mode);

	vmeta_frame_proto_release_unpacked(f, proto);
}
