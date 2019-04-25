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

#include "vmeta_priv.h"

ULOG_DECLARE_TAG(vmeta);


void vmeta_euler_to_quat(const struct vmeta_euler *euler,
			 struct vmeta_quaternion *quat)
{
	if ((euler == NULL) || (quat == NULL))
		return;

	float c1, c2, c3, s1, s2, s3, psi, theta, phi;
	float qw, qx, qy, qz, n;
	phi = euler->phi;
	theta = euler->theta;
	psi = euler->psi;
	c1 = cosf(phi / 2);
	s1 = sinf(phi / 2);
	c2 = cosf(theta / 2);
	s2 = sinf(theta / 2);
	c3 = cosf(psi / 2);
	s3 = sinf(psi / 2);
	qw = c1 * c2 * c3 + s1 * s2 * s3;
	qx = s1 * c2 * c3 - c1 * s2 * s3;
	qy = c1 * s2 * c3 + s1 * c2 * s3;
	qz = c1 * c2 * s3 - s1 * s2 * c3;
	n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
	if (n != 0.) {
		qw /= n;
		qx /= n;
		qy /= n;
		qz /= n;
	}

	quat->w = qw;
	quat->x = qx;
	quat->y = qy;
	quat->z = qz;
}


void vmeta_quat_to_euler(const struct vmeta_quaternion *quat,
			 struct vmeta_euler *euler)
{
	if ((quat == NULL) || (euler == NULL))
		return;

	float psi, theta, phi;
	float qw, qx, qy, qz;
	qw = quat->w;
	qx = quat->x;
	qy = quat->y;
	qz = quat->z;
	phi = atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
	theta = asinf(2 * (qw * qy - qz * qx));
	psi = atan2f(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

	euler->phi = phi;
	euler->theta = theta;
	euler->psi = psi;
}


int vmeta_frame_get_location(const struct vmeta_frame *meta,
			     struct vmeta_location *loc)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);
	memset(loc, 0, sizeof(*loc));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*loc = meta->v1_strm_ext.location;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*loc = meta->v1_rec.location;
		break;

	case VMETA_FRAME_TYPE_V2:
		*loc = meta->v2.base.location;
		break;

	case VMETA_FRAME_TYPE_V3:
		*loc = meta->v3.base.location;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_speed_ned(const struct vmeta_frame *meta,
			      struct vmeta_ned *speed)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(speed == NULL, EINVAL);
	memset(speed, 0, sizeof(*speed));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		/* TODO: should be converted to NED */
		speed->north = meta->v1_strm_ext.speed.x;
		speed->east = meta->v1_strm_ext.speed.y;
		speed->down = meta->v1_strm_ext.speed.z;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		/* TODO: should be converted to NED */
		speed->north = meta->v1_rec.speed.x;
		speed->east = meta->v1_rec.speed.y;
		speed->down = meta->v1_rec.speed.z;
		break;

	case VMETA_FRAME_TYPE_V2:
		*speed = meta->v2.base.speed;
		break;

	case VMETA_FRAME_TYPE_V3:
		*speed = meta->v3.base.speed;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_air_speed(const struct vmeta_frame *meta, float *speed)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(speed == NULL, EINVAL);
	*speed = -1.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V2:
		*speed = meta->v2.base.air_speed;
		break;

	case VMETA_FRAME_TYPE_V3:
		*speed = meta->v3.base.air_speed;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_ground_distance(const struct vmeta_frame *meta,
				    double *dist)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dist == NULL, EINVAL);
	*dist = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*dist = meta->v1_strm_ext.altitude;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*dist = meta->v1_rec.altitude;
		break;

	case VMETA_FRAME_TYPE_V2:
		*dist = meta->v2.base.ground_distance;
		break;

	case VMETA_FRAME_TYPE_V3:
		*dist = meta->v3.base.ground_distance;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_drone_euler(const struct vmeta_frame *meta,
				struct vmeta_euler *euler)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(euler == NULL, EINVAL);
	memset(euler, 0, sizeof(*euler));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*euler = meta->v1_strm_basic.drone_attitude;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*euler = meta->v1_strm_ext.drone_attitude;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*euler = meta->v1_rec.drone_attitude;
		break;

	case VMETA_FRAME_TYPE_V2:
		vmeta_quat_to_euler(&meta->v2.base.drone_quat, euler);
		break;

	case VMETA_FRAME_TYPE_V3:
		vmeta_quat_to_euler(&meta->v3.base.drone_quat, euler);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_drone_quat(const struct vmeta_frame *meta,
			       struct vmeta_quaternion *quat)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(quat == NULL, EINVAL);
	memset(quat, 0, sizeof(*quat));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		vmeta_euler_to_quat(&meta->v1_strm_basic.drone_attitude, quat);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		vmeta_euler_to_quat(&meta->v1_strm_ext.drone_attitude, quat);
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		vmeta_euler_to_quat(&meta->v1_rec.drone_attitude, quat);
		break;

	case VMETA_FRAME_TYPE_V2:
		*quat = meta->v2.base.drone_quat;
		break;

	case VMETA_FRAME_TYPE_V3:
		*quat = meta->v3.base.drone_quat;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_euler(const struct vmeta_frame *meta,
				struct vmeta_euler *euler)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(euler == NULL, EINVAL);
	memset(euler, 0, sizeof(*euler));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		vmeta_quat_to_euler(&meta->v1_strm_basic.frame_quat, euler);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		vmeta_quat_to_euler(&meta->v1_strm_ext.frame_quat, euler);
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		vmeta_quat_to_euler(&meta->v1_rec.frame_quat, euler);
		break;

	case VMETA_FRAME_TYPE_V2:
		vmeta_quat_to_euler(&meta->v2.base.frame_quat, euler);
		break;

	case VMETA_FRAME_TYPE_V3:
		vmeta_quat_to_euler(&meta->v3.base.frame_quat, euler);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_quat(const struct vmeta_frame *meta,
			       struct vmeta_quaternion *quat)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(quat == NULL, EINVAL);
	memset(quat, 0, sizeof(*quat));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*quat = meta->v1_strm_basic.frame_quat;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*quat = meta->v1_strm_ext.frame_quat;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*quat = meta->v1_rec.frame_quat;
		break;

	case VMETA_FRAME_TYPE_V2:
		*quat = meta->v2.base.frame_quat;
		break;

	case VMETA_FRAME_TYPE_V3:
		*quat = meta->v3.base.frame_quat;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_base_euler(const struct vmeta_frame *meta,
				     struct vmeta_euler *euler)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(euler == NULL, EINVAL);
	memset(euler, 0, sizeof(*euler));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		vmeta_quat_to_euler(&meta->v3.base.frame_base_quat, euler);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_base_quat(const struct vmeta_frame *meta,
				    struct vmeta_quaternion *quat)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(quat == NULL, EINVAL);
	memset(quat, 0, sizeof(*quat));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*quat = meta->v3.base.frame_base_quat;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_timestamp(const struct vmeta_frame *meta,
				    uint64_t *timestamp)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(timestamp == NULL, EINVAL);
	*timestamp = 0;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*timestamp = meta->v1_rec.frame_timestamp;
		break;

	case VMETA_FRAME_TYPE_V2:
		if (meta->v2.has_timestamp)
			*timestamp = meta->v2.timestamp.frame_timestamp;
		else
			res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		if (meta->v3.has_timestamp)
			*timestamp = meta->v3.timestamp.frame_timestamp;
		else
			res = -ENOENT;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_camera_pan(const struct vmeta_frame *meta, float *pan)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pan == NULL, EINVAL);
	*pan = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V3:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*pan = meta->v1_strm_basic.camera_pan;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*pan = meta->v1_strm_ext.camera_pan;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*pan = meta->v1_rec.camera_pan;
		break;

	case VMETA_FRAME_TYPE_V2:
		*pan = meta->v2.base.camera_pan;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_camera_tilt(const struct vmeta_frame *meta, float *tilt)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(tilt == NULL, EINVAL);
	*tilt = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V3:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*tilt = meta->v1_strm_basic.camera_tilt;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*tilt = meta->v1_strm_ext.camera_tilt;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*tilt = meta->v1_rec.camera_tilt;
		break;

	case VMETA_FRAME_TYPE_V2:
		*tilt = meta->v2.base.camera_tilt;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_exposure_time(const struct vmeta_frame *meta, float *exp)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(exp == NULL, EINVAL);
	*exp = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*exp = meta->v1_strm_basic.exposure_time;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*exp = meta->v1_strm_ext.exposure_time;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*exp = meta->v1_rec.exposure_time;
		break;

	case VMETA_FRAME_TYPE_V2:
		*exp = meta->v2.base.exposure_time;
		break;

	case VMETA_FRAME_TYPE_V3:
		*exp = meta->v3.base.exposure_time;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_gain(const struct vmeta_frame *meta, uint16_t *gain)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(gain == NULL, EINVAL);
	*gain = 0;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*gain = meta->v1_strm_basic.gain;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*gain = meta->v1_strm_ext.gain;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*gain = meta->v1_rec.gain;
		break;

	case VMETA_FRAME_TYPE_V2:
		*gain = meta->v2.base.gain;
		break;

	case VMETA_FRAME_TYPE_V3:
		*gain = meta->v3.base.gain;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_awb_r_gain(const struct vmeta_frame *meta, float *gain)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(gain == NULL, EINVAL);
	*gain = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*gain = meta->v3.base.awb_r_gain;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_awb_b_gain(const struct vmeta_frame *meta, float *gain)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(gain == NULL, EINVAL);
	*gain = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*gain = meta->v3.base.awb_b_gain;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_picture_h_fov(const struct vmeta_frame *meta, float *fov)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fov == NULL, EINVAL);
	*fov = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*fov = meta->v3.base.picture_hfov;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_picture_v_fov(const struct vmeta_frame *meta, float *fov)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fov == NULL, EINVAL);
	*fov = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*fov = meta->v3.base.picture_vfov;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_link_goodput(const struct vmeta_frame *meta,
				 uint32_t *goodput)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(goodput == NULL, EINVAL);
	*goodput = 0;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*goodput = meta->v3.base.link_goodput;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_link_quality(const struct vmeta_frame *meta,
				 uint8_t *quality)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(quality == NULL, EINVAL);
	*quality = 0;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		*quality = meta->v3.base.link_quality;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_wifi_rssi(const struct vmeta_frame *meta, int8_t *rssi)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(rssi == NULL, EINVAL);
	*rssi = 0;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*rssi = meta->v1_strm_basic.wifi_rssi;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*rssi = meta->v1_strm_ext.wifi_rssi;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*rssi = meta->v1_rec.wifi_rssi;
		break;

	case VMETA_FRAME_TYPE_V2:
		*rssi = meta->v2.base.wifi_rssi;
		break;

	case VMETA_FRAME_TYPE_V3:
		*rssi = meta->v3.base.wifi_rssi;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_battery_pencentage(const struct vmeta_frame *meta,
				       uint8_t *bat)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(bat == NULL, EINVAL);
	*bat = 255;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		*bat = meta->v1_strm_basic.battery_percentage;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*bat = meta->v1_strm_ext.battery_percentage;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*bat = meta->v1_rec.battery_percentage;
		break;

	case VMETA_FRAME_TYPE_V2:
		*bat = meta->v2.base.battery_percentage;
		break;

	case VMETA_FRAME_TYPE_V3:
		*bat = meta->v3.base.battery_percentage;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_flying_state(const struct vmeta_frame *meta,
				 enum vmeta_flying_state *state)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(state == NULL, EINVAL);
	*state = VMETA_FLYING_STATE_LANDED;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*state = meta->v1_strm_ext.state;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*state = meta->v1_rec.state;
		break;

	case VMETA_FRAME_TYPE_V2:
		*state = meta->v2.base.state;
		break;

	case VMETA_FRAME_TYPE_V3:
		*state = meta->v3.base.state;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_piloting_mode(const struct vmeta_frame *meta,
				  enum vmeta_piloting_mode *mode)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mode == NULL, EINVAL);
	*mode = VMETA_PILOTING_MODE_MANUAL;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		*mode = meta->v1_strm_ext.mode;
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		*mode = meta->v1_rec.mode;
		break;

	case VMETA_FRAME_TYPE_V2:
		*mode = meta->v2.base.mode;
		break;

	case VMETA_FRAME_TYPE_V3:
		*mode = meta->v3.base.mode;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}
