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


int vmeta_json_proto_add_timed_metadata(struct json_object *jobj,
					Vmeta__TimedMetadata *timed)
{
	int res = 0;

	if (!timed) {
		res = ENODATA;
		ULOGD("No timed metadata info");
		goto out;
	}
	vmeta_json_proto_add_drone_metadata(jobj, "drone", timed->drone);
	vmeta_json_proto_add_camera_metadata(jobj, "camera", timed->camera);
	vmeta_json_add_array(jobj,
			     "links",
			     (union array_element_type *)timed->links,
			     timed->n_links,
			     vmeta_json_proto_add_link_metadata);
	vmeta_json_proto_add_tracking_metadata(
		jobj, "tracking", timed->tracking);
	vmeta_json_proto_add_tracking_proposal_metadata(
		jobj, "proposal", timed->proposal);

out:
	return res;
}


void vmeta_json_proto_add_quaternion(struct json_object *jobj,
				     const char *name,
				     const Vmeta__Quaternion *quaternion)
{
	struct json_object *jobj_quaternion;

	if (!quaternion) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_quaternion = json_object_new_object();
	vmeta_json_add_double(jobj_quaternion, "w", quaternion->w);
	vmeta_json_add_double(jobj_quaternion, "x", quaternion->x);
	vmeta_json_add_double(jobj_quaternion, "y", quaternion->y);
	vmeta_json_add_double(jobj_quaternion, "z", quaternion->z);

	json_object_object_add(jobj, name, jobj_quaternion);
}


void vmeta_json_proto_add_location(struct json_object *jobj,
				   const char *name,
				   const Vmeta__Location *location)
{
	struct json_object *jobj_location;

	if (!location) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_location = json_object_new_object();
	vmeta_json_add_double(jobj_location, "latitude", location->latitude);
	vmeta_json_add_double(jobj_location, "longitude", location->longitude);
	if (location->altitude_wgs84ellipsoid != 0.) {
		vmeta_json_add_double(jobj_location,
				      "altitude_wgs84ellipsoid",
				      location->altitude_wgs84ellipsoid);
	}
	if (location->altitude_egm96amsl != 0.) {
		vmeta_json_add_double(jobj_location,
				      "altitude_egm96amsl",
				      location->altitude_egm96amsl);
	}
	if (location->horizontal_accuracy != 0.) {
		vmeta_json_add_double(jobj_location,
				      "horizontal_accuracy",
				      location->horizontal_accuracy);
	}
	if (location->vertical_accuracy != 0.) {
		vmeta_json_add_double(jobj_location,
				      "vertical_accuracy",
				      location->vertical_accuracy);
	}
	if (location->sv_count != VMETA_LOCATION_INVALID_SV_COUNT)
		vmeta_json_add_int(
			jobj_location, "sv_count", location->sv_count);

	json_object_object_add(jobj, name, jobj_location);
}


void vmeta_json_proto_add_ned(struct json_object *jobj,
			      const char *name,
			      const Vmeta__NED *ned)
{
	struct json_object *jobj_ned;

	if (!ned) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_ned = json_object_new_object();
	vmeta_json_add_double(jobj_ned, "north", ned->north);
	vmeta_json_add_double(jobj_ned, "east", ned->east);
	vmeta_json_add_double(jobj_ned, "down", ned->down);

	json_object_object_add(jobj, name, jobj_ned);
}


void vmeta_json_proto_add_vec2(struct json_object *jobj,
			       const char *name,
			       const Vmeta__Vector2 *vec2)
{
	struct json_object *jobj_vec2;

	if (!vec2) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_vec2 = json_object_new_object();
	vmeta_json_add_double(jobj_vec2, "x", vec2->x);
	vmeta_json_add_double(jobj_vec2, "y", vec2->y);

	json_object_object_add(jobj, name, jobj_vec2);
}


void vmeta_json_proto_add_vec3(struct json_object *jobj,
			       const char *name,
			       const Vmeta__Vector3 *vec3)
{
	struct json_object *jobj_vec3;

	if (!vec3) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_vec3 = json_object_new_object();
	vmeta_json_add_double(jobj_vec3, "x", vec3->x);
	vmeta_json_add_double(jobj_vec3, "y", vec3->y);
	vmeta_json_add_double(jobj_vec3, "z", vec3->z);

	json_object_object_add(jobj, name, jobj_vec3);
}


void vmeta_json_proto_add_drone_metadata(struct json_object *jobj,
					 const char *name,
					 const Vmeta__DroneMetadata *drone)
{
	struct json_object *jobj_drone;
	const ProtobufCEnumValue *flying_state;

	if (!drone) {
		ULOGD("No %s info", name);
		return;
	}

	flying_state = protobuf_c_enum_descriptor_get_value(
		&vmeta__flying_state__descriptor, drone->flying_state);

	jobj_drone = json_object_new_object();

	vmeta_json_proto_add_quaternion(jobj_drone, "quat", drone->quat);
	vmeta_json_proto_add_location(jobj_drone, "location", drone->location);
	vmeta_json_add_double(
		jobj_drone, "ground_distance", drone->ground_distance);
	vmeta_json_proto_add_ned(jobj_drone, "position", drone->position);
	vmeta_json_proto_add_vec3(
		jobj_drone, "local_position", drone->local_position);
	vmeta_json_proto_add_ned(jobj_drone, "speed", drone->speed);
	vmeta_json_add_int(
		jobj_drone, "battery_percentage", drone->battery_percentage);
	if (flying_state != NULL) {
		vmeta_json_add_str(
			jobj_drone, "flying_state", flying_state->name);
	}
	json_object_object_add(jobj, name, jobj_drone);
}


void vmeta_json_proto_add_camera_metadata(struct json_object *jobj,
					  const char *name,
					  const Vmeta__CameraMetadata *camera)
{
	struct json_object *jobj_camera;

	if (!camera) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_camera = json_object_new_object();

	vmeta_json_add_int64(jobj_camera, "timestamp", camera->timestamp);
	if ((camera->utc_timestamp != 0) &&
	    (camera->utc_timestamp_accuracy != 0)) {
		vmeta_json_add_int64(
			jobj_camera, "utc_timestamp", camera->utc_timestamp);
		vmeta_json_add_int(jobj_camera,
				   "utc_timestamp_accuracy",
				   camera->utc_timestamp_accuracy);
	}
	vmeta_json_proto_add_quaternion(
		jobj_camera, "base_quat", camera->base_quat);
	vmeta_json_proto_add_quaternion(jobj_camera, "quat", camera->quat);
	vmeta_json_proto_add_location(
		jobj_camera, "location", camera->location);
	vmeta_json_proto_add_vec2(
		jobj_camera, "principal_point", camera->principal_point);
	vmeta_json_add_double(
		jobj_camera, "exposure_time", camera->exposure_time);
	vmeta_json_add_int(jobj_camera, "iso_gain", camera->iso_gain);
	vmeta_json_add_double(jobj_camera, "awb_r_gain", camera->awb_r_gain);
	vmeta_json_add_double(jobj_camera, "awb_b_gain", camera->awb_b_gain);
	vmeta_json_add_double(jobj_camera, "hfov", camera->hfov);
	vmeta_json_add_double(jobj_camera, "vfov", camera->vfov);

	json_object_object_add(jobj, name, jobj_camera);
}


void vmeta_json_proto_add_wifi_link_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__WifiLinkMetadata *wifi)
{
	struct json_object *jobj_wifi;

	if (!wifi) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_wifi = json_object_new_object();
	vmeta_json_add_int(jobj_wifi, "goodput", wifi->goodput);
	vmeta_json_add_int(jobj_wifi, "quality", wifi->quality);
	vmeta_json_add_int(jobj_wifi, "rssi", wifi->rssi);

	json_object_object_add(jobj, name, jobj_wifi);
}


void vmeta_json_proto_add_starfish_link_info(
	struct json_object *jobj,
	const char *name,
	const Vmeta__StarfishLinkInfo *starfish_info)
{
	struct json_object *jobj_starfish_info;

	if (!starfish_info) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_starfish_info = json_object_new_object();
	vmeta_json_add_str(jobj_starfish_info,
			   "type",
			   vmeta_link_type_to_str(starfish_info->type));
	vmeta_json_add_str(jobj_starfish_info,
			   "status",
			   vmeta_link_status_to_str(starfish_info->status));
	vmeta_json_add_int(
		jobj_starfish_info, "quality", starfish_info->quality);
	vmeta_json_add_bool(
		jobj_starfish_info, "active", starfish_info->active);

	if (json_object_get_type(jobj) == json_type_object)
		json_object_object_add(jobj, name, jobj_starfish_info);
	else if (json_object_get_type(jobj) == json_type_array)
		json_object_array_add(jobj, jobj_starfish_info);
	else
		free(jobj_starfish_info);
}


void vmeta_json_proto_add_starfish_link_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__StarfishLinkMetadata *starfish)
{
	struct json_object *jobj_starfish;

	if (!starfish) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_starfish = json_object_new_object();
	vmeta_json_add_array(jobj_starfish,
			     "links",
			     (union array_element_type *)starfish->links,
			     starfish->n_links,
			     vmeta_json_proto_add_starfish_link_info);
	vmeta_json_add_int(jobj_starfish, "quality", starfish->quality);

	json_object_object_add(jobj, name, jobj_starfish);
}


void vmeta_json_proto_add_link_metadata(struct json_object *jobj,
					const char *name,
					const Vmeta__LinkMetadata *link)
{
	struct json_object *jobj_link;

	if (!link) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_link = json_object_new_object();

	switch (link->protocol_case) {
	case VMETA__LINK_METADATA__PROTOCOL__NOT_SET:
		break;
	case VMETA__LINK_METADATA__PROTOCOL_WIFI:
		vmeta_json_proto_add_wifi_link_metadata(
			jobj_link, "wifi", link->wifi);
		break;
	case VMETA__LINK_METADATA__PROTOCOL_STARFISH:
		vmeta_json_proto_add_starfish_link_metadata(
			jobj_link, "starfish", link->starfish);
		break;
	default:
		break;
	}

	if (json_object_get_type(jobj) == json_type_object)
		json_object_object_add(jobj, name, jobj_link);
	else if (json_object_get_type(jobj) == json_type_array)
		json_object_array_add(jobj, jobj_link);
	else
		free(jobj_link);
}


void vmeta_json_proto_add_bounding_box(struct json_object *jobj,
				       const char *name,
				       const Vmeta__BoundingBox *bbox)
{
	struct json_object *jobj_bbox;
	const ProtobufCEnumValue *object_class;

	if (!bbox) {
		ULOGD("No %s info", name);
		return;
	}

	object_class = protobuf_c_enum_descriptor_get_value(
		&vmeta__tracking_class__descriptor, bbox->object_class);

	jobj_bbox = json_object_new_object();
	vmeta_json_add_double(jobj_bbox, "x", bbox->x);
	vmeta_json_add_double(jobj_bbox, "y", bbox->y);
	vmeta_json_add_double(jobj_bbox, "width", bbox->width);
	vmeta_json_add_double(jobj_bbox, "height", bbox->height);
	if (object_class != NULL) {
		vmeta_json_add_str(
			jobj_bbox, "object_class", object_class->name);
	}
	vmeta_json_add_double(jobj_bbox, "confidence", bbox->confidence);
	vmeta_json_add_int(jobj_bbox, "uid", bbox->uid);

	if (json_object_get_type(jobj) == json_type_object)
		json_object_object_add(jobj, name, jobj_bbox);
	else if (json_object_get_type(jobj) == json_type_array)
		json_object_array_add(jobj, jobj_bbox);
	else
		free(jobj_bbox);
}


void vmeta_json_proto_add_tracking_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__TrackingMetadata *tracking)
{
	struct json_object *jobj_tracking;
	const ProtobufCEnumValue *state;

	if (!tracking) {
		ULOGD("No %s info", name);
		return;
	}

	state = protobuf_c_enum_descriptor_get_value(
		&vmeta__tracking_state__descriptor, tracking->state);

	jobj_tracking = json_object_new_object();
	vmeta_json_proto_add_bounding_box(
		jobj_tracking, "target", tracking->target);
	vmeta_json_add_int64(jobj_tracking, "timestamp", tracking->timestamp);
	vmeta_json_add_int(jobj_tracking, "quality", tracking->quality);
	if (state != NULL)
		vmeta_json_add_str(jobj_tracking, "state", state->name);
	vmeta_json_add_int(jobj_tracking, "cookie", tracking->cookie);

	json_object_object_add(jobj, name, jobj_tracking);
}


void vmeta_json_proto_add_tracking_proposal_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__TrackingProposalMetadata *proposal)
{
	struct json_object *jobj_proposal;

	if (!proposal) {
		ULOGD("No %s info", name);
		return;
	}
	jobj_proposal = json_object_new_object();
	vmeta_json_add_array(jobj_proposal,
			     "proposals",
			     (union array_element_type *)proposal->proposals,
			     proposal->n_proposals,
			     vmeta_json_proto_add_bounding_box);
	vmeta_json_add_int64(jobj_proposal, "timestamp", proposal->timestamp);

	json_object_object_add(jobj, name, jobj_proposal);
}
