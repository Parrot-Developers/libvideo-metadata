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

#ifndef _VMETA_JSON_PROTO_H_
#define _VMETA_JSON_PROTO_H_


/* Enabling no cast (transparent union) is required when using the union
 * argument as other function arguments (used in vmeta_json_add_array for 3d arg
 * of object_nester) */
union array_element_type {
	const Vmeta__BoundingBox *bbox;
	const Vmeta__LinkMetadata *link;
	const Vmeta__StarfishLinkInfo *starfish_info;
} __attribute__((__transparent_union__));


static inline int
vmeta_json_add_array(struct json_object *jobj,
		     const char *name,
		     union array_element_type *val,
		     size_t size,
		     void (*object_nester)(struct json_object *,
					   const char *,
					   union array_element_type))
{
	struct json_object *jobj_array = json_object_new_array();
	char name_index[64];

	for (size_t i = 0; i < size; ++i) {
		snprintf(name_index, sizeof(name_index), "%s[%zu]", name, i);
		object_nester(jobj_array, name_index, val[i]);
	}

	json_object_object_add(jobj, name, jobj_array);
	return 0;
}


int vmeta_json_proto_add_timed_metadata(struct json_object *jobj,
					Vmeta__TimedMetadata *timed);


void vmeta_json_proto_add_quaternion(struct json_object *jobj,
				     const char *name,
				     const Vmeta__Quaternion *quaternion);


void vmeta_json_proto_add_location(struct json_object *jobj,
				   const char *name,
				   const Vmeta__Location *location);


void vmeta_json_proto_add_ned(struct json_object *jobj,
			      const char *name,
			      const Vmeta__NED *ned);


void vmeta_json_proto_add_vec2(struct json_object *jobj,
			       const char *name,
			       const Vmeta__Vector2 *vec2);


void vmeta_json_proto_add_vec3(struct json_object *jobj,
			       const char *name,
			       const Vmeta__Vector3 *vec3);


void vmeta_json_proto_add_drone_metadata(struct json_object *jobj,
					 const char *name,
					 const Vmeta__DroneMetadata *drone);


void vmeta_json_proto_add_camera_metadata(struct json_object *jobj,
					  const char *name,
					  const Vmeta__CameraMetadata *camera);


void vmeta_json_proto_add_wifi_link_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__WifiLinkMetadata *wifi);


void vmeta_json_proto_add_starfish_link_info(
	struct json_object *jobj,
	const char *name,
	const Vmeta__StarfishLinkInfo *starfish_info);


void vmeta_json_proto_add_starfish_link_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__StarfishLinkMetadata *starfish);


void vmeta_json_proto_add_link_metadata(struct json_object *jobj,
					const char *name,
					const Vmeta__LinkMetadata *link);


void vmeta_json_proto_add_bounding_box(struct json_object *jobj,
				       const char *name,
				       const Vmeta__BoundingBox *bbox);


void vmeta_json_proto_add_tracking_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__TrackingMetadata *tracking);


void vmeta_json_proto_add_tracking_proposal_metadata(
	struct json_object *jobj,
	const char *name,
	const Vmeta__TrackingProposalMetadata *proposal);


#endif /* !_VMETA_JSON_PROTO_H_ */
