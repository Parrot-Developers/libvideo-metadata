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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_altitude_above_takeoff(struct vmeta_frame *meta,
					   double *alt)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(alt == NULL, EINVAL);
	*alt = 0.;

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
		if (!tm->drone || isnan(tm->drone->altitude_ato)) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*alt = tm->drone->altitude_ato;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_local_quat(struct vmeta_frame *meta,
				     struct vmeta_quaternion *local_quat)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(local_quat == NULL, EINVAL);
	memset(local_quat, 0, sizeof(*local_quat));

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
		if (!tm->camera || !tm->camera->local_quat) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		local_quat->w = tm->camera->local_quat->w;
		local_quat->x = tm->camera->local_quat->x;
		local_quat->y = tm->camera->local_quat->y;
		local_quat->z = tm->camera->local_quat->z;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_frame_utc_timestamp(struct vmeta_frame *meta,
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
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
	case VMETA_FRAME_TYPE_V3:
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
		*timestamp = tm->camera->utc_timestamp;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	if (res == 0 && *timestamp == 0)
		res = -ENOENT;
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_camera_zoom_level(struct vmeta_frame *meta,
				      float *zoom_level)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(zoom_level == NULL, EINVAL);
	*zoom_level = 0.;

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
		if (!tm->camera || isnan(tm->camera->zoom_level)) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*zoom_level = tm->camera->zoom_level;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_link_goodput(struct vmeta_frame *meta, uint32_t *goodput)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	const Vmeta__LinkMetadata *link;
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
		if (tm->n_links > 1) {
			/* Only take into account the first link (there
			 * should not be more than one) */
			res = -EPROTO;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		link = tm->links[0];
		if (link->protocol_case !=
		    VMETA__LINK_METADATA__PROTOCOL_WIFI) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*goodput = link->wifi->goodput;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_link_quality(struct vmeta_frame *meta, uint8_t *quality)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	const Vmeta__LinkMetadata *link;
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
		if (tm->n_links > 1) {
			/* Only take into account the first link (there
			 * should not be more than one) */
			res = -EPROTO;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		link = tm->links[0];
		switch (link->protocol_case) {
		case VMETA__LINK_METADATA__PROTOCOL_WIFI:
			*quality = link->wifi->quality;
			break;
		case VMETA__LINK_METADATA__PROTOCOL_STARFISH:
			*quality = link->starfish->quality;
			break;
		default:
			res = -ENOENT;
			break;
		}
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_wifi_rssi(struct vmeta_frame *meta, int8_t *rssi)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	const Vmeta__LinkMetadata *link;
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
		if (tm->n_links > 1) {
			/* Only take into account the first link (there
			 * should not be more than one) */
			res = -EPROTO;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		link = tm->links[0];
		if (link->protocol_case !=
		    VMETA__LINK_METADATA__PROTOCOL_WIFI) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*rssi = link->wifi->rssi;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
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
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_camera_spectrum(struct vmeta_frame *meta,
				    enum vmeta_camera_spectrum *spectrum)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(spectrum == NULL, EINVAL);
	*spectrum = VMETA_CAMERA_SPECTRUM_UNKNOWN;

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
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*spectrum = vmeta_frame_camera_spectrum_proto_to_vmeta(
			tm->camera->spectrum);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_thermal_mask(struct vmeta_frame *meta,
				 struct vmeta_rectf *mask)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mask == NULL, EINVAL);
	memset(mask, 0, sizeof(*mask));

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
		if (tm->thermal == NULL || tm->thermal->mask == NULL) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		mask->left = tm->thermal->mask->x;
		mask->top = tm->thermal->mask->y;
		mask->width = tm->thermal->mask->width;
		mask->height = tm->thermal->mask->height;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_lfic(struct vmeta_frame *meta,
			 struct vmeta_location *loc,
			 float *x,
			 float *y,
			 double *estimated_precision,
			 double *grid_precision)
{
	return vmeta_frame_get_lfic_by_type(meta,
					    VMETA_LFIC_TYPE_COT,
					    loc,
					    x,
					    y,
					    estimated_precision,
					    grid_precision);
}


int vmeta_frame_get_lfic_by_type(struct vmeta_frame *meta,
				 enum vmeta_lfic_type type,
				 struct vmeta_location *loc,
				 float *x,
				 float *y,
				 double *estimated_precision,
				 double *grid_precision)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	size_t index = 0;

	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(x == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(y == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(estimated_precision == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(grid_precision == NULL, EINVAL);
	memset(loc, 0, sizeof(*loc));
	*x = 0.f;
	*y = 0.f;
	*estimated_precision = 0.;
	*grid_precision = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		if (type != VMETA_LFIC_TYPE_COT)
			return -ENOENT;
		if (!meta->v3.has_lfic)
			return -ENOENT;
		*loc = meta->v3.lfic.target_location;
		*x = meta->v3.lfic.target_x;
		*y = meta->v3.lfic.target_y;
		*estimated_precision = meta->v3.lfic.estimated_precision;
		*grid_precision = meta->v3.lfic.grid_precision;
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;

		for (index = 0; index < tm->n_lfic; index++) {
			if (tm->lfic[index] == NULL) {
				index = tm->n_lfic;
				break;
			}
			if (tm->lfic[index]->type ==
			    vmeta_frame_lfic_type_vmeta_to_proto(type))
				break;
		}
		if ((tm->n_lfic == 0) || (index >= tm->n_lfic)) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}

		if (tm->lfic[index]->location != NULL) {
			loc->altitude_wgs84ellipsoid =
				tm->lfic[index]
					->location->altitude_wgs84ellipsoid;
			if (loc->altitude_wgs84ellipsoid == 0.)
				loc->altitude_wgs84ellipsoid = NAN;
			loc->altitude_egm96amsl =
				tm->lfic[index]->location->altitude_egm96amsl;
			if (loc->altitude_egm96amsl == 0.)
				loc->altitude_egm96amsl = NAN;
			loc->latitude = tm->lfic[index]->location->latitude;
			loc->longitude = tm->lfic[index]->location->longitude;
			loc->sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
			loc->valid = 1;
			*estimated_precision =
				tm->lfic[index]->location->horizontal_accuracy;
		} else {
			loc->valid = 0;
		}
		*x = tm->lfic[index]->x;
		*y = tm->lfic[index]->y;
		*grid_precision = tm->lfic[index]->grid_precision;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_lfic_count(struct vmeta_frame *meta, size_t *count)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;

	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(count == NULL, EINVAL);

	*count = 0;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		break;

	case VMETA_FRAME_TYPE_V3:
		if (meta->v3.has_lfic)
			*count = 1;
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		*count = tm->n_lfic;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_lfic_by_index(struct vmeta_frame *meta,
				  size_t index,
				  struct vmeta_location *loc,
				  float *x,
				  float *y,
				  double *estimated_precision,
				  double *grid_precision,
				  enum vmeta_lfic_type *type)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(x == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(y == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(estimated_precision == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(grid_precision == NULL, EINVAL);
	memset(loc, 0, sizeof(*loc));
	*x = 0.f;
	*y = 0.f;
	*estimated_precision = 0.;
	*grid_precision = 0.;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V2:
		res = -ENOENT;
		break;

	case VMETA_FRAME_TYPE_V3:
		if (index != 0 || !meta->v3.has_lfic)
			return -ENOENT;
		*loc = meta->v3.lfic.target_location;
		*x = meta->v3.lfic.target_x;
		*y = meta->v3.lfic.target_y;
		*estimated_precision = meta->v3.lfic.estimated_precision;
		*grid_precision = meta->v3.lfic.grid_precision;
		*type = VMETA_LFIC_TYPE_COT;
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_get_unpacked(meta, &tm);
		if (res < 0)
			break;
		if ((tm->n_lfic <= index) || (tm->lfic[index] == NULL)) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		if (tm->lfic[index]->location != NULL) {
			loc->altitude_wgs84ellipsoid =
				tm->lfic[index]
					->location->altitude_wgs84ellipsoid;
			if (loc->altitude_wgs84ellipsoid == 0.)
				loc->altitude_wgs84ellipsoid = NAN;
			loc->altitude_egm96amsl =
				tm->lfic[index]->location->altitude_egm96amsl;
			if (loc->altitude_egm96amsl == 0.)
				loc->altitude_egm96amsl = NAN;
			loc->latitude = tm->lfic[index]->location->latitude;
			loc->longitude = tm->lfic[index]->location->longitude;
			loc->sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
			loc->valid = 1;
			*estimated_precision =
				tm->lfic[index]->location->horizontal_accuracy;
		} else {
			loc->valid = 0;
		}
		*x = tm->lfic[index]->x;
		*y = tm->lfic[index]->y;
		*grid_precision = tm->lfic[index]->grid_precision;
		*type = vmeta_frame_lfic_type_proto_to_vmeta(
			tm->lfic[index]->type);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_tracking_box(struct vmeta_frame *meta,
				 struct vmeta_rectf *box)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;

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
		if (tm->tracking == NULL || tm->tracking->target == NULL) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		box->left = tm->tracking->target->x;
		box->top = tm->tracking->target->y;
		box->width = tm->tracking->target->width;
		box->height = tm->tracking->target->height;
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;
	default:
		res = -ENOSYS;
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
		ULOGE("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_get_camera_subtype(struct vmeta_frame *meta,
				   enum vmeta_camera_subtype *subtype)
{
	int res = 0;
	const Vmeta__TimedMetadata *tm;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(subtype == NULL, EINVAL);
	*subtype = VMETA_CAMERA_SUBTYPE_UNKNOWN;

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
		if (!tm->camera) {
			res = -ENOENT;
			vmeta_frame_proto_release_unpacked(meta, tm);
			break;
		}
		*subtype = vmeta_camera_subtype_proto_to_vmeta(
			tm->camera->subtype);
		vmeta_frame_proto_release_unpacked(meta, tm);
		break;

	default:
		ULOGE("unknown metadata type: %u", meta->type);
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
	} else if (strcasecmp(str, "front-stereo") == 0) {
		return VMETA_CAMERA_TYPE_FRONT_STEREO;
	} else if (strcasecmp(str, "front-stereo-left") == 0) {
		return VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT;
	} else if (strcasecmp(str, "front-stereo-right") == 0) {
		return VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT;
	} else if (strcasecmp(str, "vertical") == 0) {
		return VMETA_CAMERA_TYPE_VERTICAL;
	} else if (strcasecmp(str, "disparity") == 0) {
		return VMETA_CAMERA_TYPE_DISPARITY;
	} else if (strcasecmp(str, "horizontal-stereo") == 0) {
		return VMETA_CAMERA_TYPE_HORIZONTAL_STEREO;
	} else if (strcasecmp(str, "horizontal-stereo-left") == 0) {
		return VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT;
	} else if (strcasecmp(str, "horizontal-stereo-right") == 0) {
		return VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT;
	} else if (strcasecmp(str, "down-stereo") == 0) {
		return VMETA_CAMERA_TYPE_DOWN_STEREO;
	} else if (strcasecmp(str, "down-stereo-left") == 0) {
		return VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT;
	} else if (strcasecmp(str, "down-stereo-right") == 0) {
		return VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT;
	} else if (strcasecmp(str, "external") == 0) {
		return VMETA_CAMERA_TYPE_EXTERNAL;
	} else {
		ULOGE("%s: unknown camera type '%s'", __func__, str);
		return VMETA_CAMERA_TYPE_UNKNOWN;
	}
}


const char *vmeta_camera_type_to_str(enum vmeta_camera_type val)
{
	switch (val) {
	case VMETA_CAMERA_TYPE_FRONT:
		return "front";
	case VMETA_CAMERA_TYPE_FRONT_STEREO:
		return "front-stereo";
	case VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT:
		return "front-stereo-left";
	case VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT:
		return "front-stereo-right";
	case VMETA_CAMERA_TYPE_VERTICAL:
		return "vertical";
	case VMETA_CAMERA_TYPE_DISPARITY:
		return "disparity";
	case VMETA_CAMERA_TYPE_HORIZONTAL_STEREO:
		return "horizontal-stereo";
	case VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT:
		return "horizontal-stereo-left";
	case VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT:
		return "horizontal-stereo-right";
	case VMETA_CAMERA_TYPE_DOWN_STEREO:
		return "down-stereo";
	case VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT:
		return "down-stereo-left";
	case VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT:
		return "down-stereo-right";
	case VMETA_CAMERA_TYPE_EXTERNAL:
		return "external";
	default:
		return "unknown";
	}
}


enum vmeta_camera_subtype vmeta_camera_subtype_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_CAMERA_SUBTYPE_UNKNOWN;

	if (strcasecmp(str, "left") == 0) {
		return VMETA_CAMERA_SUBTYPE_LEFT;
	} else if (strcasecmp(str, "right") == 0) {
		return VMETA_CAMERA_SUBTYPE_RIGHT;
	} else if (strcasecmp(str, "wide") == 0) {
		return VMETA_CAMERA_SUBTYPE_WIDE;
	} else if (strcasecmp(str, "tele") == 0) {
		return VMETA_CAMERA_SUBTYPE_TELE;
	} else if (strcasecmp(str, "disparity") == 0) {
		return VMETA_CAMERA_SUBTYPE_DISPARITY;
	} else if (strcasecmp(str, "depth") == 0) {
		return VMETA_CAMERA_SUBTYPE_DEPTH;
	} else {
		ULOGE("%s: unknown camera subtype '%s'", __func__, str);
		return VMETA_CAMERA_SUBTYPE_UNKNOWN;
	}
}


const char *vmeta_camera_subtype_to_str(enum vmeta_camera_subtype val)
{
	switch (val) {
	case VMETA_CAMERA_SUBTYPE_LEFT:
		return "left";
	case VMETA_CAMERA_SUBTYPE_RIGHT:
		return "right";
	case VMETA_CAMERA_SUBTYPE_WIDE:
		return "wide";
	case VMETA_CAMERA_SUBTYPE_TELE:
		return "tele";
	case VMETA_CAMERA_SUBTYPE_DISPARITY:
		return "disparity";
	case VMETA_CAMERA_SUBTYPE_DEPTH:
		return "depth";
	default:
		return "unknown";
	}
}


enum vmeta_camera_spectrum vmeta_camera_spectrum_from_str(const char *str)
{
	if (str == NULL)
		return VMETA_CAMERA_SPECTRUM_UNKNOWN;

	if (strcasecmp(str, "visible") == 0) {
		return VMETA_CAMERA_SPECTRUM_VISIBLE;
	} else if (strcasecmp(str, "thermal") == 0) {
		return VMETA_CAMERA_SPECTRUM_THERMAL;
	} else if (strcasecmp(str, "blended") == 0) {
		return VMETA_CAMERA_SPECTRUM_BLENDED;
	} else {
		ULOGE("%s: unknown camera spectrum '%s'", __func__, str);
		return VMETA_CAMERA_SPECTRUM_UNKNOWN;
	}
}


const char *vmeta_camera_spectrum_to_str(enum vmeta_camera_spectrum val)
{
	switch (val) {
	case VMETA_CAMERA_SPECTRUM_VISIBLE:
		return "visible";
	case VMETA_CAMERA_SPECTRUM_THERMAL:
		return "thermal";
	case VMETA_CAMERA_SPECTRUM_BLENDED:
		return "blended";
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
		ULOGE("%s: unknown camera model type '%s'", __func__, str);
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


const char *vmeta_overlay_type_to_str(enum vmeta_overlay_type val)
{
	switch (val) {
	case VMETA_OVERLAY_TYPE_NONE:
		return "none";
	case VMETA_OVERLAY_TYPE_HEADER_FOOTER:
		return "header_footer";
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
	} else if (strcasecmp(str, "streamrec") == 0) {
		return VMETA_VIDEO_MODE_STREAMREC;
	} else {
		ULOGE("%s: unknown video mode '%s'", __func__, str);
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
	case VMETA_VIDEO_MODE_STREAMREC:
		return "streamrec";
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
	} else if (strcasecmp(str, "internal-error") == 0) {
		return VMETA_VIDEO_STOP_REASON_INTERNAL_ERROR;
	} else {
		ULOGE("%s: unknown stop reason '%s'", __func__, str);
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
	case VMETA_VIDEO_STOP_REASON_INTERNAL_ERROR:
		return "internal-error";
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
		ULOGE("%s: unknown dynamic range '%s'", __func__, str);
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
		ULOGE("%s: unknown tone mapping '%s'", __func__, str);
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
