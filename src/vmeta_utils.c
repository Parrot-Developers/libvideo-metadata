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

#define SINGULARITY_RADIUS (0.00001f)


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

	if ((quat->w == 0) && (quat->x == 0) && (quat->y == 0) &&
	    (quat->z == 0)) {
		euler->psi = NAN;
		euler->theta = NAN;
		euler->phi = NAN;
		return;
	}

	float w, x, y, z, sqw, sqx, sqy, sqz, psign, s2;

	w = quat->w;
	x = quat->x;
	y = quat->y;
	z = quat->z;
	sqw = w * w;
	sqx = x * x;
	sqy = y * y;
	sqz = z * z;
	psign = -1.f;
	s2 = 2.f * (w * y - z * x);

	/* Test singularities */
	if (s2 < (-1.f + SINGULARITY_RADIUS)) {
		euler->psi = 0.f;
		euler->theta = -M_PI / 2.f;
		euler->phi = atan2f(2.f * (psign * z * y + w * x),
				    sqw + sqy - sqz - sqx);
	} else if (s2 > (1.f - SINGULARITY_RADIUS)) {
		euler->psi = 0.f;
		euler->theta = M_PI / 2.f;
		euler->phi = atan2f(2.f * (psign * z * y + w * x),
				    sqw + sqy - sqz - sqx);
	} else {
		euler->psi = -atan2f(-2.f * (w * z - psign * y * x),
				     sqw + sqx - sqz - sqy);
		euler->theta = asinf(s2);
		euler->phi = atan2f(2.f * (w * x - psign * z * y),
				    sqw + sqz - sqy - sqx);
	}
}

void vmeta_quat_rotate_vector(const struct vmeta_quaternion *q,
			      struct vmeta_xyz *vector)
{
	struct vmeta_xyz tmp;

	if (!q || !vector)
		return;

	tmp.x = (1.f - 2.f * (q->y * q->y + q->z * q->z)) * vector->x +
		(2.f * q->x * q->y - 2.f * q->w * q->z) * vector->y +
		(2.f * q->w * q->y + 2.f * q->x * q->z) * vector->z;
	tmp.y = (2.f * q->x * q->y + 2.f * q->w * q->z) * vector->x +
		(1.f - 2.f * (q->x * q->x + q->z * q->z)) * vector->y +
		(-2.f * q->w * q->x + 2.f * q->y * q->z) * vector->z;
	tmp.z = (-2.f * q->w * q->y + 2.f * q->x * q->z) * vector->x +
		(2.f * q->w * q->x + 2.f * q->y * q->z) * vector->y +
		(1.f - 2.f * (q->x * q->x + q->y * q->y)) * vector->z;

	vector->x = tmp.x;
	vector->y = tmp.y;
	vector->z = tmp.z;
}

void vmeta_quat_conjugate(struct vmeta_quaternion *q)
{
	if (!q)
		return;

	q->x = -q->x;
	q->y = -q->y;
	q->z = -q->z;
}

int vmeta_frame_get_location(struct vmeta_frame *meta,
			     struct vmeta_location *loc)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone || !tm->drone->location) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		loc->altitude_wgs84ellipsoid =
			tm->drone->location->altitude_wgs84ellipsoid;
		if (loc->altitude_wgs84ellipsoid == 0.)
			loc->altitude_wgs84ellipsoid = NAN;
		loc->altitude_egm96amsl =
			tm->drone->location->altitude_egm96amsl;
		if (loc->altitude_egm96amsl == 0.)
			loc->altitude_egm96amsl = NAN;
		loc->latitude = tm->drone->location->latitude;
		loc->longitude = tm->drone->location->longitude;
		loc->horizontal_accuracy =
			tm->drone->location->horizontal_accuracy;
		loc->vertical_accuracy = tm->drone->location->vertical_accuracy;
		loc->sv_count = tm->drone->location->sv_count;
		loc->valid = 1;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	if (res == 0 && loc->valid == 0)
		res = -ENOENT;
	return res;
}


int vmeta_frame_get_speed_ned(struct vmeta_frame *meta, struct vmeta_ned *speed)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone || !tm->drone->speed) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		speed->north = tm->drone->speed->north;
		speed->east = tm->drone->speed->east;
		speed->down = tm->drone->speed->down;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_air_speed(struct vmeta_frame *meta, float *speed)
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

	case VMETA_FRAME_TYPE_PROTO:
		res = -ENOENT;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_ground_distance(struct vmeta_frame *meta, double *dist)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*dist = tm->drone->ground_distance;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_drone_euler(struct vmeta_frame *meta,
				struct vmeta_euler *euler)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	struct vmeta_quaternion tmp;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone || !tm->drone->quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		tmp.w = tm->drone->quat->w;
		tmp.x = tm->drone->quat->x;
		tmp.y = tm->drone->quat->y;
		tmp.z = tm->drone->quat->z;
		vmeta_quat_to_euler(&tmp, euler);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_drone_quat(struct vmeta_frame *meta,
			       struct vmeta_quaternion *quat)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone || !tm->drone->quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		quat->w = tm->drone->quat->w;
		quat->x = tm->drone->quat->x;
		quat->y = tm->drone->quat->y;
		quat->z = tm->drone->quat->z;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_euler(struct vmeta_frame *meta,
				struct vmeta_euler *euler)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	struct vmeta_quaternion tmp;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera || !tm->camera->quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		tmp.w = tm->camera->quat->w;
		tmp.x = tm->camera->quat->x;
		tmp.y = tm->camera->quat->y;
		tmp.z = tm->camera->quat->z;
		vmeta_quat_to_euler(&tmp, euler);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_quat(struct vmeta_frame *meta,
			       struct vmeta_quaternion *quat)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera || !tm->camera->quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		quat->w = tm->camera->quat->w;
		quat->x = tm->camera->quat->x;
		quat->y = tm->camera->quat->y;
		quat->z = tm->camera->quat->z;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_base_euler(struct vmeta_frame *meta,
				     struct vmeta_euler *euler)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	struct vmeta_quaternion tmp;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera || !tm->camera->base_quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		tmp.w = tm->camera->base_quat->w;
		tmp.x = tm->camera->base_quat->x;
		tmp.y = tm->camera->base_quat->y;
		tmp.z = tm->camera->base_quat->z;
		vmeta_quat_to_euler(&tmp, euler);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_base_quat(struct vmeta_frame *meta,
				    struct vmeta_quaternion *quat)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera || !tm->camera->base_quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		quat->w = tm->camera->base_quat->w;
		quat->x = tm->camera->base_quat->x;
		quat->y = tm->camera->base_quat->y;
		quat->z = tm->camera->base_quat->z;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_timestamp(struct vmeta_frame *meta,
				    uint64_t *timestamp)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*timestamp = tm->camera->timestamp;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	if (res == 0 && *timestamp == 0)
		res = -ENOENT;
	return res;
}


int vmeta_frame_get_camera_location(struct vmeta_frame *meta,
				    struct vmeta_location *loc)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);
	memset(loc, 0, sizeof(*loc));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
	case VMETA_FRAME_TYPE_V3:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera || !tm->camera->location) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		loc->altitude_wgs84ellipsoid =
			tm->camera->location->altitude_wgs84ellipsoid;
		if (loc->altitude_wgs84ellipsoid == 0.)
			loc->altitude_wgs84ellipsoid = NAN;
		loc->altitude_egm96amsl =
			tm->camera->location->altitude_egm96amsl;
		if (loc->altitude_egm96amsl == 0.)
			loc->altitude_egm96amsl = NAN;
		loc->latitude = tm->camera->location->latitude;
		loc->longitude = tm->camera->location->longitude;
		loc->horizontal_accuracy =
			tm->camera->location->horizontal_accuracy;
		loc->vertical_accuracy =
			tm->camera->location->vertical_accuracy;
		loc->sv_count = tm->camera->location->sv_count;
		loc->valid = 1;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	if (res == 0 && loc->valid == 0)
		res = -ENOENT;
	return res;
}


int vmeta_frame_get_camera_principal_point(struct vmeta_frame *meta,
					   struct vmeta_xy *vec)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(vec == NULL, EINVAL);
	memset(vec, 0, sizeof(*vec));

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
	case VMETA_FRAME_TYPE_V3:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera || !tm->camera->principal_point) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		vec->x = tm->camera->principal_point->x;
		vec->y = tm->camera->principal_point->y;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_camera_pan(struct vmeta_frame *meta, float *pan)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pan == NULL, EINVAL);
	*pan = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V3:
	case VMETA_FRAME_TYPE_PROTO:
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


int vmeta_frame_get_camera_tilt(struct vmeta_frame *meta, float *tilt)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(tilt == NULL, EINVAL);
	*tilt = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V3:
	case VMETA_FRAME_TYPE_PROTO:
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


int vmeta_frame_get_exposure_time(struct vmeta_frame *meta, float *exp)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*exp = tm->camera->exposure_time;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_gain(struct vmeta_frame *meta, uint16_t *gain)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*gain = tm->camera->iso_gain;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_awb_r_gain(struct vmeta_frame *meta, float *gain)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*gain = tm->camera->awb_r_gain;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_awb_b_gain(struct vmeta_frame *meta, float *gain)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*gain = tm->camera->awb_b_gain;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_picture_h_fov(struct vmeta_frame *meta, float *fov)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*fov = tm->camera->hfov * 180. / M_PI;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_picture_v_fov(struct vmeta_frame *meta, float *fov)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*fov = tm->camera->vfov * 180. / M_PI;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_link_goodput(struct vmeta_frame *meta, uint32_t *goodput)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (tm->n_links == 0) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		res = -ENOENT;
		for (size_t i = 0; i < tm->n_links; i++) {
			Vmeta__LinkMetadata *link = tm->links[i];
			if (link->protocol_case !=
			    VMETA__LINK_METADATA__PROTOCOL_WIFI)
				continue;
			res = 0;
			*goodput = link->wifi->goodput;
			break;
		}
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_link_quality(struct vmeta_frame *meta, uint8_t *quality)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (tm->n_links == 0) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		res = -ENOENT;
		for (size_t i = 0; i < tm->n_links; i++) {
			Vmeta__LinkMetadata *link = tm->links[i];
			if (link->protocol_case !=
			    VMETA__LINK_METADATA__PROTOCOL_WIFI)
				continue;
			res = 0;
			*quality = link->wifi->quality;
			break;
		}
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_wifi_rssi(struct vmeta_frame *meta, int8_t *rssi)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (tm->n_links == 0) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		res = -ENOENT;
		for (size_t i = 0; i < tm->n_links; i++) {
			Vmeta__LinkMetadata *link = tm->links[i];
			if (link->protocol_case !=
			    VMETA__LINK_METADATA__PROTOCOL_WIFI)
				continue;
			res = 0;
			*rssi = link->wifi->rssi;
			break;
		}
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_battery_percentage(struct vmeta_frame *meta, uint8_t *bat)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*bat = tm->drone->battery_percentage;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_flying_state(struct vmeta_frame *meta,
				 enum vmeta_flying_state *state)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*state = vmeta_frame_flying_state_proto_to_vmeta(
			tm->drone->flying_state);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_piloting_mode(struct vmeta_frame *meta,
				  enum vmeta_piloting_mode *mode)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
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

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if (!tm->drone) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*mode = vmeta_frame_piloting_mode_proto_to_vmeta(
			tm->drone->piloting_mode);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


enum vmeta_camera_type vmeta_camera_type_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_CAMERA_TYPE_UNKNOWN;

	if (strcasecmp(str, "front") == 0) {
		return VMETA_CAMERA_TYPE_FRONT;
	} else if (strcasecmp(str, "front-stereo-left") == 0) {
		return VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT;
	} else if (strcasecmp(str, "front-stereo-right") == 0) {
		return VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT;
	} else if (strcasecmp(str, "vertical") == 0) {
		return VMETA_CAMERA_TYPE_VERTICAL;
	} else if (strcasecmp(str, "disparity") == 0) {
		return VMETA_CAMERA_TYPE_DISPARITY;
	} else {
		ULOGW("%s: unknown camera type '%s'", __func__, str);
		return VMETA_CAMERA_TYPE_UNKNOWN;
	}
}


const char *vmeta_camera_type_to_str(enum vmeta_camera_type val)
{
	switch (val) {
	case VMETA_CAMERA_TYPE_FRONT:
		return "front";
	case VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT:
		return "front-stereo-left";
	case VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT:
		return "front-stereo-right";
	case VMETA_CAMERA_TYPE_VERTICAL:
		return "vertical";
	case VMETA_CAMERA_TYPE_DISPARITY:
		return "disparity";
	default:
		return "unknown";
	}
}


enum vmeta_camera_model_type vmeta_camera_model_type_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_CAMERA_MODEL_TYPE_UNKNOWN;

	if (strcasecmp(str, "perspective") == 0) {
		return VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE;
	} else if (strcasecmp(str, "fisheye") == 0) {
		return VMETA_CAMERA_MODEL_TYPE_FISHEYE;
	} else {
		ULOGW("%s: unknown camera model type '%s'", __func__, str);
		return VMETA_CAMERA_MODEL_TYPE_UNKNOWN;
	}
}


const char *vmeta_camera_model_type_to_str(enum vmeta_camera_model_type val)
{
	switch (val) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE:
		return "perspective";
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE:
		return "fisheye";
	default:
		return "unknown";
	}
}


enum vmeta_video_mode vmeta_video_mode_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_VIDEO_MODE_UNKNOWN;

	if (strcasecmp(str, "standard") == 0) {
		return VMETA_VIDEO_MODE_STANDARD;
	} else if (strcasecmp(str, "hyperlapse") == 0) {
		return VMETA_VIDEO_MODE_HYPERLAPSE;
	} else if (strcasecmp(str, "slowmotion") == 0) {
		return VMETA_VIDEO_MODE_SLOWMOTION;
	} else {
		ULOGW("%s: unknown video mode '%s'", __func__, str);
		return VMETA_VIDEO_MODE_UNKNOWN;
	}
}


const char *vmeta_video_mode_to_str(enum vmeta_video_mode val)
{
	switch (val) {
	case VMETA_VIDEO_MODE_STANDARD:
		return "standard";
	case VMETA_VIDEO_MODE_HYPERLAPSE:
		return "hyperlapse";
	case VMETA_VIDEO_MODE_SLOWMOTION:
		return "slowmotion";
	default:
		return "unknown";
	}
}


enum vmeta_video_stop_reason vmeta_video_stop_reason_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_VIDEO_STOP_REASON_UNKNOWN;

	if (strcasecmp(str, "user") == 0) {
		return VMETA_VIDEO_STOP_REASON_USER;
	} else if (strcasecmp(str, "reconfiguration") == 0) {
		return VMETA_VIDEO_STOP_REASON_RECONFIGURATION;
	} else if (strcasecmp(str, "poor-storage-perf") == 0) {
		return VMETA_VIDEO_STOP_REASON_POOR_STORAGE_PERF;
	} else if (strcasecmp(str, "storage-full") == 0) {
		return VMETA_VIDEO_STOP_REASON_STORAGE_FULL;
	} else if (strcasecmp(str, "recovery") == 0) {
		return VMETA_VIDEO_STOP_REASON_RECOVERY;
	} else if (strcasecmp(str, "end-of-stream") == 0) {
		return VMETA_VIDEO_STOP_REASON_END_OF_STREAM;
	} else if (strcasecmp(str, "shutdown") == 0) {
		return VMETA_VIDEO_STOP_REASON_SHUTDOWN;
	} else {
		ULOGW("%s: unknown stop reason '%s'", __func__, str);
		return VMETA_VIDEO_STOP_REASON_UNKNOWN;
	}
}


const char *vmeta_video_stop_reason_to_str(enum vmeta_video_stop_reason val)
{
	switch (val) {
	case VMETA_VIDEO_STOP_REASON_USER:
		return "user";
	case VMETA_VIDEO_STOP_REASON_RECONFIGURATION:
		return "reconfiguration";
	case VMETA_VIDEO_STOP_REASON_POOR_STORAGE_PERF:
		return "poor-storage-perf";
	case VMETA_VIDEO_STOP_REASON_STORAGE_FULL:
		return "storage-full";
	case VMETA_VIDEO_STOP_REASON_RECOVERY:
		return "recovery";
	case VMETA_VIDEO_STOP_REASON_END_OF_STREAM:
		return "end-of-stream";
	case VMETA_VIDEO_STOP_REASON_SHUTDOWN:
		return "shutdown";
	default:
		return "unknown";
	}
}


const char *vmeta_link_type_to_str(Vmeta__LinkType val)
{
	switch (val) {
	case VMETA__LINK_TYPE__LINK_TYPE_LO:
		return "lo";
	case VMETA__LINK_TYPE__LINK_TYPE_LAN:
		return "lan";
	case VMETA__LINK_TYPE__LINK_TYPE_WLAN:
		return "wlan";
	case VMETA__LINK_TYPE__LINK_TYPE_CELLULAR:
		return "cellular";
	case VMETA__LINK_TYPE__LINK_TYPE_UNKNOWN:
	default:
		return "unknown";
	}
}


const char *vmeta_link_status_to_str(Vmeta__LinkStatus val)
{
	switch (val) {
	case VMETA__LINK_STATUS__LINK_STATUS_DOWN:
		return "down";
	case VMETA__LINK_STATUS__LINK_STATUS_UP:
		return "up";
	case VMETA__LINK_STATUS__LINK_STATUS_RUNNING:
		return "running";
	case VMETA__LINK_STATUS__LINK_STATUS_READY:
		return "ready";
	case VMETA__LINK_STATUS__LINK_STATUS_CONNECTING:
		return "connecting";
	case VMETA__LINK_STATUS__LINK_STATUS_ERROR:
		return "error";
	default:
		return "unknown";
	}
}


enum vmeta_dynamic_range vmeta_dynamic_range_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_DYNAMIC_RANGE_UNKNOWN;

	if (strcasecmp(str, "sdr") == 0) {
		return VMETA_DYNAMIC_RANGE_SDR;
	} else if (strcasecmp(str, "hdr8") == 0) {
		return VMETA_DYNAMIC_RANGE_HDR8;
	} else if (strcasecmp(str, "hdr10") == 0) {
		return VMETA_DYNAMIC_RANGE_HDR10;
	} else {
		ULOGW("%s: unknown dynamic range '%s'", __func__, str);
		return VMETA_DYNAMIC_RANGE_UNKNOWN;
	}
}


const char *vmeta_dynamic_range_to_str(enum vmeta_dynamic_range val)
{
	switch (val) {
	case VMETA_DYNAMIC_RANGE_SDR:
		return "sdr";
	case VMETA_DYNAMIC_RANGE_HDR8:
		return "hdr8";
	case VMETA_DYNAMIC_RANGE_HDR10:
		return "hdr10";
	default:
		return "unknown";
	}
}


enum vmeta_tone_mapping vmeta_tone_mapping_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_TONE_MAPPING_UNKNOWN;

	if (strcasecmp(str, "standard") == 0) {
		return VMETA_TONE_MAPPING_STANDARD;
	} else if (strcasecmp(str, "p-log") == 0) {
		return VMETA_TONE_MAPPING_P_LOG;
	} else {
		ULOGW("%s: unknown tone mapping '%s'", __func__, str);
		return VMETA_TONE_MAPPING_UNKNOWN;
	}
}


const char *vmeta_tone_mapping_to_str(enum vmeta_tone_mapping val)
{
	switch (val) {
	case VMETA_TONE_MAPPING_STANDARD:
		return "standard";
	case VMETA_TONE_MAPPING_P_LOG:
		return "p-log";
	default:
		return "unknown";
	}
}


int vmeta_location_horiz_distance(const struct vmeta_location *loc1,
				  const struct vmeta_location *loc2,
				  float *horizontal_distance)
{
	ULOG_ERRNO_RETURN_ERR_IF(loc1 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!loc1->valid, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc2 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!loc2->valid, EINVAL);

	double tmp, hdist;
	double r_earth = 6371000.; /* earth radius */
	double deg_to_rad = M_PI / 180.;
	double lat1 = loc1->latitude * deg_to_rad;
	double lon1 = loc1->longitude * deg_to_rad;
	double lat2 = loc2->latitude * deg_to_rad;
	double lon2 = loc2->longitude * deg_to_rad;

	tmp = sin((lat2 - lat1) / 2) * sin((lat2 - lat1) / 2) +
	      cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) *
		      sin((lon2 - lon1) / 2);
	hdist = r_earth * 2 * atan2(sqrt(tmp), sqrt(1. - tmp));

	/* Horizontal distance */
	if (horizontal_distance)
		*horizontal_distance = hdist;

	return 0;
}


int vmeta_location_delta(const struct vmeta_location *loc1,
			 const struct vmeta_location *loc2,
			 float *distance,
			 float *horizontal_distance,
			 double *bearing,
			 double *elevation,
			 struct vmeta_xyz *cartesian_delta)
{
	ULOG_ERRNO_RETURN_ERR_IF(loc1 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!loc1->valid, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc2 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!loc2->valid, EINVAL);

	int ret;
	float horiz_dist;
	double deg_to_rad = M_PI / 180.;
	double lat1 = loc1->latitude;
	double lon1 = loc1->longitude;
	double lat2 = loc2->latitude;
	double lon2 = loc2->longitude;
	double alt_diff, _bearing, x, y;

	if (!isnan(loc2->altitude_wgs84ellipsoid) &&
	    !isnan(loc1->altitude_wgs84ellipsoid)) {
		alt_diff = loc2->altitude_wgs84ellipsoid -
			   loc1->altitude_wgs84ellipsoid;
	} else if (!isnan(loc2->altitude_egm96amsl) &&
		   !isnan(loc1->altitude_egm96amsl)) {
		alt_diff = loc2->altitude_egm96amsl - loc1->altitude_egm96amsl;
	} else {
		ULOGE("incompatible altitude references");
		return -EPROTO;
	}

	ret = vmeta_location_horiz_distance(loc1, loc2, &horiz_dist);
	if (ret < 0)
		return ret;

	lat1 *= deg_to_rad;
	lon1 *= deg_to_rad;
	lat2 *= deg_to_rad;
	lon2 *= deg_to_rad;

	x = cos(lat2) * sin(lon2 - lon1);
	y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
	_bearing = atan2(x, y);

	/* Horizontal distance */
	if (horizontal_distance)
		*horizontal_distance = horiz_dist;

	/* Bearing angle */
	if (bearing)
		*bearing = _bearing;

	/* Elevation */
	if (elevation)
		*elevation = atan2(alt_diff, horiz_dist);

	/* Distance */
	if (distance)
		*distance = sqrt(horiz_dist * horiz_dist + alt_diff * alt_diff);

	/* Location difference in cartesian coordinates */
	if (cartesian_delta)
		vmeta_geo_to_ned(loc2, loc1, cartesian_delta);

	return 0;
}


static int perspective_transform(const struct vmeta_session *session_meta,
				 const struct vmeta_xy *principal_point,
				 float theta,
				 float phi,
				 float hfov,
				 float aspect_ratio,
				 float *x,
				 float *y)
{
	float tan_theta, tan_theta_pow2, tan_theta_pow4, tan_theta_pow6;
	float _x, _y, xh, yh, focal_norm;

	/* Check angles validity  */
	if (theta >= M_PI_2 || hfov >= M_PI || hfov <= 0.f)
		return -ERANGE;

	/* Precompute powers of tan(theta) */
	tan_theta = tanf(theta);
	tan_theta_pow2 = tan_theta * tan_theta;
	tan_theta_pow4 = tan_theta_pow2 * tan_theta_pow2;
	tan_theta_pow6 = tan_theta_pow4 * tan_theta_pow2;

	/* Compute xh and yh */
	xh = tan_theta * cosf(phi);
	yh = tan_theta * sinf(phi);

	/* Compute normalized focal length and aspect ratio */
	focal_norm = 0.5 / tanf(hfov / 2.f);

	/* Apply distorsion model */
	_x = (1 +
	      session_meta->camera_model.perspective.distortion.r1 *
		      tan_theta_pow2 +
	      session_meta->camera_model.perspective.distortion.r2 *
		      tan_theta_pow4 +
	      session_meta->camera_model.perspective.distortion.r3 *
		      tan_theta_pow6) *
		     xh +
	     2 * session_meta->camera_model.perspective.distortion.t1 * xh *
		     yh +
	     session_meta->camera_model.perspective.distortion.t2 *
		     (tan_theta_pow2 + 2 * xh * xh);
	_y = (1 +
	      session_meta->camera_model.perspective.distortion.r1 *
		      tan_theta_pow2 +
	      session_meta->camera_model.perspective.distortion.r2 *
		      tan_theta_pow4 +
	      session_meta->camera_model.perspective.distortion.r3 *
		      tan_theta_pow6) *
		     yh +
	     2 * session_meta->camera_model.perspective.distortion.t2 * xh *
		     yh +
	     session_meta->camera_model.perspective.distortion.t1 *
		     (tan_theta_pow2 + 2 * yh * yh);

	/* Scale by focal and shift to top left corner */
	_x = focal_norm * _x + principal_point->x;
	_y = focal_norm * _y + principal_point->y;

	/* Normalize y coordinate */
	_y *= aspect_ratio;

	*x = _x;
	*y = _y;

	return 0;
}


static int fisheye_transform(const struct vmeta_session *session_meta,
			     const struct vmeta_xy *principal_point,
			     float theta,
			     float phi,
			     float aspect_ratio,
			     float *x,
			     float *y)
{
	float theta_norm, theta_norm_pow2, theta_norm_pow3, theta_norm_pow4;
	float rho, xh, yh, _x, _y;

	/* Precompute power of theta (normalized by pi/2) */
	theta_norm = 2.f / M_PI * theta;
	theta_norm_pow2 = theta_norm * theta_norm;
	theta_norm_pow3 = theta_norm_pow2 * theta_norm;
	theta_norm_pow4 = theta_norm_pow3 * theta_norm;

	/* Apply radial distorsion model */
	rho = theta_norm +
	      session_meta->camera_model.fisheye.polynomial.p2 *
		      theta_norm_pow2 +
	      session_meta->camera_model.fisheye.polynomial.p3 *
		      theta_norm_pow3 +
	      session_meta->camera_model.fisheye.polynomial.p4 *
		      theta_norm_pow4;

	/* Compute xh and yh */
	xh = rho * cosf(phi);
	yh = rho * sinf(phi);

	/* Scale and shift */
	_x = session_meta->camera_model.fisheye.affine_matrix.c * xh +
	     session_meta->camera_model.fisheye.affine_matrix.d * yh +
	     principal_point->x;
	_y = session_meta->camera_model.fisheye.affine_matrix.e * xh +
	     session_meta->camera_model.fisheye.affine_matrix.f * yh +
	     principal_point->y;

	/* Normalize y coordinate */
	_y *= aspect_ratio;

	*x = _x;
	*y = _y;

	return 0;
}


static double modulo_d(double num, double den)
{
	double rest;

	/* Handle case (num < 0) / (+/-inf): cannot define positive rest */
	if ((num < 0.0) && isinf(den))
		return NAN;

	/* Truncated division */
	rest = fmod(num, den);

	/* Handle -0.0 */
	if ((rest == 0.0) && (signbit(rest) != 0))
		return 0.0;

	/* Ensure that rest is positive: if not replace by rest + |den| */
	if ((!isnan(rest)) && (rest < 0.0)) {
		rest += fabs(den);

		/* Handle risk of numerical error for <0 rests close to 0 */
		if (rest < fabs(den))
			return rest;
		return 0.0;
	}

	return rest;
}


double vmeta_wrapto_d(double input, double bound)
{
	double res;

	/* Euclidean division by 2*bound */
	res = modulo_d(input, bound + bound);

	/* Enforce result in ]-bound, bound] */
	if (res > bound) {
		res -= (bound + bound);

		/* Handle risk of numerical error when close to the bound */
		if (res > -bound)
			return res;
		return bound;
	}

	return res;
}


void vmeta_geo_to_ned(const struct vmeta_location *loc,
		      const struct vmeta_location *ned_origin,
		      struct vmeta_xyz *ned_loc)
{
	double delta;
	static const double deg_to_rad = M_PI / 180.0;
	static const double r_earth = 6371000.0;

	if (!loc || !ned_origin || !ned_loc)
		return;

	/* Approximation: spherical Earth */
	delta = loc->latitude - ned_origin->latitude;
	ned_loc->x =
		(float)(vmeta_wrapto_d(deg_to_rad * delta, M_PI) * r_earth);

	delta = loc->longitude - ned_origin->longitude;
	ned_loc->y = (float)(vmeta_wrapto_d(deg_to_rad * delta, M_PI) *
			     r_earth * cos(deg_to_rad * ned_origin->latitude));

	/* For altitude, try using WGS84 ellipsoid if input data is valid,
	 * otherwise use EGM96 */
	ned_loc->z = (float)(ned_origin->altitude_wgs84ellipsoid -
			     loc->altitude_wgs84ellipsoid);
	if (isnan(ned_loc->z))
		ned_loc->z = (float)(ned_origin->altitude_egm96amsl -
				     loc->altitude_egm96amsl);
}


int vmeta_frame_ltic(const struct vmeta_session *session_meta,
		     struct vmeta_frame *frame_meta,
		     float aspect_ratio,
		     const struct vmeta_location *loc,
		     float *screen_x,
		     float *screen_y,
		     float *horizontal_distance,
		     float *distance)
{
	int ret;
	struct vmeta_location cam_loc = {};
	struct vmeta_quaternion cam_in_ned_q = {};
	struct vmeta_quaternion ned_in_cam_q = {};
	struct vmeta_xyz delta_ned = {};
	struct vmeta_xyz delta_cam = {};
	struct vmeta_xy cam_principal_point = {};
	double bearing = 0., elevation = 0.;
	float dist = 0., hdist = 0.;
	float hfov = 0.;
	float theta = 0., phi = 0.;
	float x = 0., y = 0., r = 0.;
	static const double deg_to_rad = M_PI / 180.0;

	ULOG_ERRNO_RETURN_ERR_IF(session_meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame_meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(aspect_ratio <= 0.f, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!loc->valid, EINVAL);

	/* Get camera location from metadata (fall back to drone location
	 * if unavailable) */
	ret = vmeta_frame_get_camera_location(frame_meta, &cam_loc);
	if (ret == -ENOENT)
		ret = vmeta_frame_get_location(frame_meta, &cam_loc);
	if (ret < 0)
		return ret;

	/* Extract drone to location difference */
	ret = vmeta_location_delta(
		&cam_loc, loc, &dist, &hdist, &bearing, &elevation, &delta_ned);
	if (ret < 0)
		return ret;

	/* Get camera horizontal FOV from metadata */
	ret = vmeta_frame_get_picture_h_fov(frame_meta, &hfov);
	if (ret < 0)
		return ret;
	hfov *= deg_to_rad;

	/* Extract camera quaternion from metadata */
	ret = vmeta_frame_get_frame_quat(frame_meta, &cam_in_ned_q);
	if (ret < 0)
		return ret;

	/* Extract camera principal point from metadata */
	ret = vmeta_frame_get_camera_principal_point(frame_meta,
						     &cam_principal_point);
	if (ret == -ENOENT) {
		/* Fall back to the image center */
		cam_principal_point.x = 0.5;
		cam_principal_point.y = 0.5 / aspect_ratio;
	} else if (ret < 0) {
		return ret;
	}

	/* Compute NED frame expressed in camera frame */
	ned_in_cam_q = cam_in_ned_q;
	vmeta_quat_conjugate(&ned_in_cam_q);

	/* Compute delta location in camera frame */
	delta_cam = delta_ned;
	vmeta_quat_rotate_vector(&ned_in_cam_q, &delta_cam);

	/* Compute polar angle (theta, with respect to optical axis)
	 * and azimuthal angle (phi) */
	r = sqrtf(delta_cam.y * delta_cam.y + delta_cam.z * delta_cam.z);
	theta = atan2f(r, delta_cam.x);
	phi = atan2f(delta_cam.z, delta_cam.y);

	/* Check that angle is within validity range of model
	 * TODO: fetch validity range of model instead of using
	 * a hardcoded value */
	if (theta > M_PI_2)
		return -ERANGE;

	/* Project result on camera */
	switch (session_meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE:
		ret = perspective_transform(session_meta,
					    &cam_principal_point,
					    theta,
					    phi,
					    hfov,
					    aspect_ratio,
					    &x,
					    &y);
		if (ret < 0)
			return ret;
		break;
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE:
		ret = fisheye_transform(session_meta,
					&cam_principal_point,
					theta,
					phi,
					aspect_ratio,
					&x,
					&y);
		if (ret < 0)
			return ret;
		break;
	default:
		ULOGE("%s: unknown camera model", __func__);
		return -EINVAL;
	}

	/* Check if the coordinates are in the image */
	if (x < 0. || x > 1. || y < 0. || y > 1.)
		return -ERANGE;

	if (screen_x)
		*screen_x = x;
	if (screen_y)
		*screen_y = y;
	if (horizontal_distance)
		*horizontal_distance = hdist;
	if (distance)
		*distance = dist;

	return 0;
}
