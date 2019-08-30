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

struct vmeta_frame_proto {
	/* Encoded part */
	int packed;
	uint8_t *buf;
	size_t len;

	/* Decoded part */
	int unpacked;
	Vmeta__TimedMetadata *meta;

	/* lock */
	pthread_mutex_t lock;
	uint32_t rp_lock;
	uint32_t ru_lock;
	uint32_t w_lock;
};


static int vmeta_frame_proto_alloc(struct vmeta_frame_proto **meta)
{
	int res;

	*meta = calloc(1, sizeof(**meta));
	if (!*meta)
		return -ENOMEM;

	res = pthread_mutex_init(&(*meta)->lock, NULL);
	if (res != 0) {
		free(*meta);
		*meta = NULL;
		return -res;
	}

	return 0;
}


static int vmeta_frame_proto_pack(struct vmeta_frame *meta)
{
	size_t len;

	/* If the metadata is already packed, this is a no-op */
	if (meta->proto->packed)
		return 0;

	/* Do not pack if the metadata is packed-read or write locked */
	if (meta->proto->rp_lock || meta->proto->w_lock)
		return -EBUSY;

	/* If the metadata is neither packed nor unpacked, we have a problem */
	if (!meta->proto->unpacked)
		return -EINVAL;

	len = vmeta__timed_metadata__get_packed_size(meta->proto->meta);
	meta->proto->buf = malloc(len);
	if (!meta->proto->buf)
		return -ENOMEM;

	meta->proto->len = vmeta__timed_metadata__pack(meta->proto->meta,
						       meta->proto->buf);
	meta->proto->packed = 1;

	return 0;
}


static int vmeta_frame_proto_unpack(struct vmeta_frame *meta)
{
	/* If the metadata is already unpacked, this is a no-op */
	if (meta->proto->unpacked)
		return 0;

	/* Do not unpack if the metadata is unpacked-read or write locked */
	if (meta->proto->ru_lock || meta->proto->w_lock)
		return -EBUSY;

	/* If the metadata is neither packed nor unpacked, we have a problem */
	if (!meta->proto->packed)
		return -EINVAL;

	meta->proto->meta = vmeta__timed_metadata__unpack(
		NULL, meta->proto->len, meta->proto->buf);
	if (meta->proto->meta == NULL)
		return -EPROTO;
	meta->proto->unpacked = 1;

	return 0;
}


int vmeta_frame_proto_init(struct vmeta_frame_proto **meta)
{
	int res;
	struct vmeta_frame_proto *l_meta;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	res = vmeta_frame_proto_alloc(&l_meta);
	if (res != 0)
		return res;
	l_meta->meta = calloc(1, sizeof(*l_meta->meta));
	if (!l_meta->meta) {
		res = -ENOMEM;
		goto error;
	}
	vmeta__timed_metadata__init(l_meta->meta);
	l_meta->unpacked = 1;
	*meta = l_meta;

	return 0;

error:
	vmeta_frame_proto_destroy(l_meta);
	*meta = NULL;
	return res;
}


int vmeta_frame_proto_read(struct vmeta_buffer *buf,
			   struct vmeta_frame_proto **meta)
{
	int res;
	size_t alloc_len;
	struct vmeta_frame_proto *l_meta;

	ULOG_ERRNO_RETURN_ERR_IF(!buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	res = vmeta_frame_proto_alloc(&l_meta);
	if (res != 0)
		return res;
	l_meta->len = buf->len - buf->pos;

	/* We want to allow a "zero-length" input, but malloc(0) return value
	 * is implementation-defined, so we use malloc(1) in this case */
	alloc_len = l_meta->len == 0 ? 1 : l_meta->len;
	l_meta->buf = malloc(alloc_len);
	if (!l_meta->buf) {
		res = -ENOMEM;
		goto error;
	}
	memcpy(l_meta->buf, buf->data + buf->pos, l_meta->len);
	l_meta->packed = 1;
	*meta = l_meta;

	return 0;

error:
	vmeta_frame_proto_destroy(l_meta);
	*meta = NULL;
	return res;
}


int vmeta_frame_proto_write(struct vmeta_buffer *buf, struct vmeta_frame *meta)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	res = vmeta_frame_proto_pack(meta);
	if (res != 0)
		goto out;

	if (meta->proto->w_lock) {
		res = -EBUSY;
		goto out;
	}

	res = vmeta_buffer_write(buf, meta->proto->buf, meta->proto->len);

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return res;
}


int vmeta_frame_proto_to_json(struct vmeta_frame *meta,
			      struct json_object *jobj)
{
	int ret = 0;
	Vmeta__TimedMetadata *timed_meta;

	ret = vmeta_frame_proto_get_unpacked(
		meta, (const Vmeta__TimedMetadata **)&timed_meta);
	if (ret < 0) {
		ULOG_ERRNO("vmeta_frame_proto_get_unpacked", -ret);
		goto error;
	}
	ret = vmeta_json_proto_add_timed_metadata(jobj, timed_meta);
	if (ret < 0) {
		ULOG_ERRNO("vmeta_json_add_proto_timed_metadata", -ret);
		goto error;
	}
error:
	ret = vmeta_frame_proto_release_unpacked(meta, timed_meta);
	if (ret < 0)
		ULOG_ERRNO("vmeta_frame_proto_release_unpacked", -ret);
	return ret;
}


int vmeta_frame_proto_destroy(struct vmeta_frame_proto *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	if (meta->rp_lock)
		ULOGW("metadata destroyed with %" PRIu32
		      " packed-read-lock held",
		      meta->rp_lock);
	if (meta->ru_lock)
		ULOGW("metadata destroyed with %" PRIu32
		      " unpacked-read-lock held",
		      meta->ru_lock);
	if (meta->w_lock)
		ULOGW("metadata destroyed with write-lock held");

	if (meta->packed)
		free(meta->buf);

	if (meta->unpacked)
		vmeta__timed_metadata__free_unpacked(meta->meta, NULL);

	pthread_mutex_destroy(&meta->lock);
	free(meta);

	return 0;
}


int vmeta_frame_proto_get_unpacked(struct vmeta_frame *meta,
				   const Vmeta__TimedMetadata **proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	ret = vmeta_frame_proto_unpack(meta);
	if (ret < 0)
		goto out;

	if (meta->proto->w_lock) {
		ret = -EBUSY;
		goto out;
	}

	*proto_meta = meta->proto->meta;
	meta->proto->ru_lock++;

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


int vmeta_frame_proto_release_unpacked(struct vmeta_frame *meta,
				       const Vmeta__TimedMetadata *proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	if (!meta->proto->ru_lock) {
		ULOGW("%s called with no unpacked-read-lock held", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (!meta->proto->unpacked || proto_meta != meta->proto->meta) {
		ULOGW("%s called with a wrong proto_meta", __func__);
		ret = -EPROTO;
		goto out;
	}

	meta->proto->ru_lock--;

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


int vmeta_frame_proto_get_unpacked_rw(struct vmeta_frame *meta,
				      Vmeta__TimedMetadata **proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	ret = vmeta_frame_proto_unpack(meta);
	if (ret < 0)
		goto out;

	if (meta->proto->ru_lock || meta->proto->rp_lock ||
	    meta->proto->w_lock) {
		ret = -EBUSY;
		goto out;
	}

	*proto_meta = meta->proto->meta;
	meta->proto->w_lock = 1;

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


int vmeta_frame_proto_release_unpacked_rw(struct vmeta_frame *meta,
					  Vmeta__TimedMetadata *proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	if (!meta->proto->w_lock) {
		ULOGW("%s called with no write-lock held", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (!meta->proto->unpacked || proto_meta != meta->proto->meta) {
		ULOGW("%s called with a wrong proto_meta", __func__);
		ret = -EPROTO;
		goto out;
	}

	meta->proto->w_lock = 0;

	if (meta->proto->unpacked) {
		free(meta->proto->buf);
		meta->proto->buf = NULL;
		meta->proto->len = 0;
		meta->proto->packed = 0;
	}

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


int vmeta_frame_proto_get_buffer(struct vmeta_frame *meta,
				 const uint8_t **buf,
				 size_t *len)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!len, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	ret = vmeta_frame_proto_pack(meta);
	if (ret < 0)
		goto out;

	if (meta->proto->w_lock) {
		ret = -EBUSY;
		goto out;
	}

	*buf = meta->proto->buf;
	*len = meta->proto->len;
	meta->proto->rp_lock++;

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


int vmeta_frame_proto_release_buffer(struct vmeta_frame *meta,
				     const uint8_t *buf)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	if (!meta->proto->rp_lock) {
		ULOGW("%s called with no packed-read-lock held", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (!meta->proto->packed || buf != meta->proto->buf) {
		ULOGW("%s called with a wrong buffer", __func__);
		ret = -EPROTO;
		goto out;
	}

	meta->proto->rp_lock--;

out:
	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


Vmeta__CameraMetadata *vmeta_frame_proto_get_camera(Vmeta__TimedMetadata *meta)
{
	Vmeta__CameraMetadata *camera;

	ULOG_ERRNO_RETURN_VAL_IF(!meta, EINVAL, NULL);

	if (meta->camera)
		return meta->camera;

	camera = calloc(1, sizeof(*camera));
	if (!camera) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_metadata__init(camera);
	meta->camera = camera;
	return camera;
}


Vmeta__Quaternion *
vmeta_frame_proto_get_camera_base_quat(Vmeta__CameraMetadata *camera)
{
	Vmeta__Quaternion *quat;

	ULOG_ERRNO_RETURN_VAL_IF(!camera, EINVAL, NULL);

	if (camera->base_quat)
		return camera->base_quat;
	quat = calloc(1, sizeof(*quat));
	if (!quat) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__quaternion__init(quat);
	camera->base_quat = quat;
	return quat;
}


Vmeta__Quaternion *
vmeta_frame_proto_get_camera_quat(Vmeta__CameraMetadata *camera)
{
	Vmeta__Quaternion *quat;

	ULOG_ERRNO_RETURN_VAL_IF(!camera, EINVAL, NULL);

	if (camera->quat)
		return camera->quat;
	quat = calloc(1, sizeof(*quat));
	if (!quat) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__quaternion__init(quat);
	camera->quat = quat;
	return quat;
}


Vmeta__DroneMetadata *vmeta_frame_proto_get_drone(Vmeta__TimedMetadata *meta)
{
	Vmeta__DroneMetadata *drone;

	ULOG_ERRNO_RETURN_VAL_IF(!meta, EINVAL, NULL);

	if (meta->drone)
		return meta->drone;

	drone = calloc(1, sizeof(*drone));
	if (!drone) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__drone_metadata__init(drone);
	meta->drone = drone;
	return drone;
}


Vmeta__Location *
vmeta_frame_proto_get_drone_location(Vmeta__DroneMetadata *drone)
{
	Vmeta__Location *location;

	ULOG_ERRNO_RETURN_VAL_IF(!drone, EINVAL, NULL);

	if (drone->location)
		return drone->location;
	location = calloc(1, sizeof(*location));
	if (!location) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__location__init(location);
	drone->location = location;
	return location;
}


Vmeta__Quaternion *vmeta_frame_proto_get_drone_quat(Vmeta__DroneMetadata *drone)
{
	Vmeta__Quaternion *quat;

	ULOG_ERRNO_RETURN_VAL_IF(!drone, EINVAL, NULL);

	if (drone->quat)
		return drone->quat;
	quat = calloc(1, sizeof(*quat));
	if (!quat) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__quaternion__init(quat);
	drone->quat = quat;
	return quat;
}


Vmeta__NED *vmeta_frame_proto_get_drone_speed(Vmeta__DroneMetadata *drone)
{
	Vmeta__NED *speed;

	ULOG_ERRNO_RETURN_VAL_IF(!drone, EINVAL, NULL);

	if (drone->speed)
		return drone->speed;
	speed = calloc(1, sizeof(*speed));
	if (!speed) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__ned__init(speed);
	drone->speed = speed;
	return speed;
}


Vmeta__NED *vmeta_frame_proto_get_drone_position(Vmeta__DroneMetadata *drone)
{
	Vmeta__NED *position;

	ULOG_ERRNO_RETURN_VAL_IF(!drone, EINVAL, NULL);

	if (drone->position)
		return drone->position;
	position = calloc(1, sizeof(*position));
	if (!position) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__ned__init(position);
	drone->position = position;
	return position;
}


Vmeta__Vector3 *
vmeta_frame_proto_get_drone_local_position(Vmeta__DroneMetadata *drone)
{
	Vmeta__Vector3 *local_position;

	ULOG_ERRNO_RETURN_VAL_IF(!drone, EINVAL, NULL);

	if (drone->local_position)
		return drone->local_position;
	local_position = calloc(1, sizeof(*local_position));
	if (!local_position) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__vector3__init(local_position);
	drone->local_position = local_position;
	return local_position;
}


Vmeta__WifiLinkMetadata *
vmeta_frame_proto_add_wifi_link(Vmeta__TimedMetadata *meta)
{
	Vmeta__LinkMetadata *link, **tmp;
	Vmeta__WifiLinkMetadata *wifi;

	ULOG_ERRNO_RETURN_VAL_IF(!meta, EINVAL, NULL);

	link = calloc(1, sizeof(*link));
	if (!link) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__link_metadata__init(link);
	link->protocol_case = VMETA__LINK_METADATA__PROTOCOL_WIFI;

	wifi = calloc(1, sizeof(*wifi));
	if (!wifi) {
		ULOG_ERRNO("calloc", ENOMEM);
		vmeta__link_metadata__free_unpacked(link, NULL);
		return NULL;
	}
	vmeta__wifi_link_metadata__init(wifi);
	link->wifi = wifi;

	meta->n_links++;
	tmp = realloc(meta->links, meta->n_links * sizeof(link));
	if (!tmp) {
		meta->n_links--;
		vmeta__link_metadata__free_unpacked(link, NULL);
		return NULL;
	}
	meta->links = tmp;
	meta->links[meta->n_links - 1] = link;
	return wifi;
}


VMETA_API Vmeta__StarfishLinkInfo *
vmeta_frame_proto_add_starfish_link_info(Vmeta__StarfishLinkMetadata *starfish)
{
	Vmeta__StarfishLinkInfo *link, **tmp;

	ULOG_ERRNO_RETURN_VAL_IF(!starfish, EINVAL, NULL);

	link = calloc(1, sizeof(*link));
	if (!link) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__starfish_link_info__init(link);

	starfish->n_links++;
	tmp = realloc(starfish->links, starfish->n_links * sizeof(link));
	if (!tmp) {
		starfish->n_links--;
		vmeta__starfish_link_info__free_unpacked(link, NULL);
		return NULL;
	}
	starfish->links = tmp;
	starfish->links[starfish->n_links - 1] = link;
	return link;
}


Vmeta__StarfishLinkMetadata *
vmeta_frame_proto_add_starfish_link(Vmeta__TimedMetadata *meta)
{
	Vmeta__LinkMetadata *link, **tmp;
	Vmeta__StarfishLinkMetadata *starfish;

	ULOG_ERRNO_RETURN_VAL_IF(!meta, EINVAL, NULL);

	link = calloc(1, sizeof(*link));
	if (!link) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__link_metadata__init(link);
	link->protocol_case = VMETA__LINK_METADATA__PROTOCOL_STARFISH;

	starfish = calloc(1, sizeof(*starfish));
	if (!starfish) {
		ULOG_ERRNO("calloc", ENOMEM);
		vmeta__link_metadata__free_unpacked(link, NULL);
		return NULL;
	}
	vmeta__starfish_link_metadata__init(starfish);
	link->starfish = starfish;

	meta->n_links++;
	tmp = realloc(meta->links, meta->n_links * sizeof(link));
	if (!tmp) {
		meta->n_links--;
		vmeta__link_metadata__free_unpacked(link, NULL);
		return NULL;
	}
	meta->links = tmp;
	meta->links[meta->n_links - 1] = link;
	return starfish;
}


Vmeta__TrackingMetadata *
vmeta_frame_proto_get_tracking(Vmeta__TimedMetadata *meta)
{
	Vmeta__TrackingMetadata *tracking;

	ULOG_ERRNO_RETURN_VAL_IF(!meta, EINVAL, NULL);

	if (meta->tracking)
		return meta->tracking;

	tracking = calloc(1, sizeof(*tracking));
	if (!tracking) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__tracking_metadata__init(tracking);
	meta->tracking = tracking;
	return tracking;
}


Vmeta__BoundingBox *
vmeta_frame_proto_get_tracking_target(Vmeta__TrackingMetadata *tracking)
{
	Vmeta__BoundingBox *box;

	ULOG_ERRNO_RETURN_VAL_IF(!tracking, EINVAL, NULL);

	if (tracking->target)
		return tracking->target;
	box = calloc(1, sizeof(*box));
	if (!box) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__bounding_box__init(box);
	tracking->target = box;
	return box;
}


Vmeta__TrackingProposalMetadata *
vmeta_frame_proto_get_proposal(Vmeta__TimedMetadata *meta)
{
	Vmeta__TrackingProposalMetadata *proposal;

	ULOG_ERRNO_RETURN_VAL_IF(!meta, EINVAL, NULL);

	if (meta->proposal)
		return meta->proposal;

	proposal = calloc(1, sizeof(*proposal));
	if (!proposal) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__tracking_proposal_metadata__init(proposal);
	meta->proposal = proposal;
	return proposal;
}


Vmeta__BoundingBox *
vmeta_frame_proto_proposal_add_box(Vmeta__TrackingProposalMetadata *proposal)
{
	Vmeta__BoundingBox *box, **tmp;

	ULOG_ERRNO_RETURN_VAL_IF(!proposal, EINVAL, NULL);

	box = calloc(1, sizeof(*box));
	if (!box) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__bounding_box__init(box);
	proposal->n_proposals++;
	tmp = realloc(proposal->proposals, proposal->n_proposals * sizeof(box));
	if (!tmp) {
		proposal->n_proposals--;
		vmeta__bounding_box__free_unpacked(box, NULL);
		return NULL;
	}
	proposal->proposals = tmp;
	proposal->proposals[proposal->n_proposals - 1] = box;
	return box;
}


ssize_t vmeta_frame_proto_get_packed_size(struct vmeta_frame *meta)
{
	ssize_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta->type != VMETA_FRAME_TYPE_PROTO, EPROTO);
	ULOG_ERRNO_RETURN_ERR_IF(!meta->proto, EINVAL);

	pthread_mutex_lock(&meta->proto->lock);

	if (meta->proto->packed)
		ret = meta->proto->len;
	else if (meta->proto->unpacked)
		ret = vmeta__timed_metadata__get_packed_size(meta->proto->meta);
	else
		ret = -EINVAL;

	pthread_mutex_unlock(&meta->proto->lock);

	return ret;
}


enum vmeta_flying_state
vmeta_frame_flying_state_proto_to_vmeta(Vmeta__FlyingState state)
{
	enum vmeta_flying_state out = VMETA_FLYING_STATE_LANDED;
	switch (state) {
	case VMETA__FLYING_STATE__FS_LANDED:
		out = VMETA_FLYING_STATE_LANDED;
		break;
	case VMETA__FLYING_STATE__FS_TAKINGOFF:
		out = VMETA_FLYING_STATE_TAKINGOFF;
		break;
	case VMETA__FLYING_STATE__FS_HOVERING:
		out = VMETA_FLYING_STATE_HOVERING;
		break;
	case VMETA__FLYING_STATE__FS_FLYING:
		out = VMETA_FLYING_STATE_FLYING;
		break;
	case VMETA__FLYING_STATE__FS_LANDING:
		out = VMETA_FLYING_STATE_LANDING;
		break;
	case VMETA__FLYING_STATE__FS_EMERGENCY:
		out = VMETA_FLYING_STATE_EMERGENCY;
		break;
	default:
		break;
	}
	return out;
}
