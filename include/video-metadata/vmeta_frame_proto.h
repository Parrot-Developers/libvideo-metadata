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

#ifndef _VMETA_FRAME_PROTO_H_
#define _VMETA_FRAME_PROTO_H_

#include <vmeta.pb-c.h>

/* "Parrot Video Metadata" protobuf-based MIME type */
#define VMETA_FRAME_PROTO_MIME_TYPE                                            \
	"application/octet-stream;type=com.parrot.videometadataproto"

/* "Parrot Video Metadata" protobuf-based content encoding */
#define VMETA_FRAME_PROTO_CONTENT_ENCODING ""


/* "Parrot Video Metadata" protobuf-based structure definition
 * see Vmeta__TimedMetadata definition for more details */
struct vmeta_frame_proto;
struct vmeta_frame;


/**
 * Field getters
 */

/**
 * Get the Protobuf structure backed by this metadata in read-only mode.
 * This function only works with unpacked buffers (either created from
 * vmeta_frame_new() or unpacked via vmeta_frame_proto_unpack()).
 * The returned pointer is a direct pointer to the metadata contents, and
 * thus should not be modified.
 * The pointer is valid while the caller keeps a reference to the
 * metadata (see vmeta_frame_ref()/vmeta_frame_unref()), and until the caller
 * calls vmeta_frame_proto_release_unpacked().
 * @note If the caller unrefs the metadata while holding a read-only unpacked
 * pointer, the metadata will be locked in read-only mode until destroyed.
 * The function returns an error if the metadata is not initialized, of a
 * wrong type, not available in unpacked mode, or already locked in read-write
 * mode.
 * @param meta: the frame metadata
 * @param proto_meta: output pointer for the protobuf structure pointer
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_frame_proto_get_unpacked(struct vmeta_frame *meta,
			       const Vmeta__TimedMetadata **proto_meta);

/**
 * Release the read-only protobuf structure to the metadata.
 * The proto_meta pointer should not be used again after this call.
 * @param meta: the frame metadata
 * @param proto_meta: pointer to the protobuf structure
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_frame_proto_release_unpacked(struct vmeta_frame *meta,
				   const Vmeta__TimedMetadata *proto_meta);

/**
 * Get the Protobuf structure backed by this metadata in read-write mode.
 * This function only works with unpacked buffers (either created from
 * vmeta_frame_new() or unpacked via vmeta_frame_proto_unpack()).
 * The returned pointer is a direct pointer to the metadata contents, that can
 * be modified. This call is intended for metadata providers.
 * The pointer is valid while the caller keeps a reference to the
 * metadata (see vmeta_frame_ref()/vmeta_frame_unref()), and until the caller
 * calls vmeta_frame_proto_release_unpacked_rw().
 * @note If the caller unrefs the metadata while holding a read-write unpacked
 * pointer, the metadata will be locked until destroyed.
 * The function returns an error if the metadata is not initialized, of a
 * wrong type, not available in unpacked mode, or already locked in read-write
 * or read-only mode.
 * @param meta: the frame metadata
 * @param proto_meta: output pointer for the protobuf structure pointer
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_frame_proto_get_unpacked_rw(struct vmeta_frame *meta,
				  Vmeta__TimedMetadata **proto_meta);

/**
 * Release the read-write protobuf structure to the metadata.
 * The proto_meta pointer should not be used again after this call.
 * @note: Releasing the read-write view of the metadata will invalidate the
 * packed view of the metadata, a call to vmeta_frame_proto_pack() is needed
 * to recreate the packed view.
 * @param meta: the frame metadata
 * @param proto_meta: pointer to the protobuf structure
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_frame_proto_release_unpacked_rw(struct vmeta_frame *meta,
				      Vmeta__TimedMetadata *proto_meta);

/**
 * Get the packed protobuf data representing this metadata.
 * This function only works with packed buffers (either created from
 * vmeta_frame_read() or packed via vmeta_frame_proto_pack())
 * The returned pointer is a direct pointer to the metadata buffer, and
 * thus should not be modified.
 * The pointer is valid while the caller keeps a reference to the metadata (see
 * vmeta_frame_ref()/vmeta_frame_unref()), and until the caller calls
 * vmeta_frame_proto_release_buffer().
 * The function returns an error if the metadata is not initialized, of a
 * wrong type, not available in packed mode, or already locked in read-write
 * mode.
 * @param meta: the frame metadata
 * @param buf: output pointer for the protobuf buffer
 * @param len: output pointer for the protobuf buffer length
 * @return 0 on success, negative errno on error.
 */
VMETA_API int vmeta_frame_proto_get_buffer(struct vmeta_frame *meta,
					   const uint8_t **buf,
					   size_t *len);

/**
 * Release the read-only buffer to the metadata.
 * The buf pointer should not be used again after this call.
 * @param meta: the frame metadata
 * @param buf: pointer to the protobuf buffer
 * @return 0 on success, negative errno on error.
 */
VMETA_API int vmeta_frame_proto_release_buffer(struct vmeta_frame *meta,
					       const uint8_t *buf);


/**
 * Get the packed size of the metadata.
 * On an unpacked metadata, the result is computed at each call.
 * On a previously packed metadata, this function returns the size of the
 * current packed buffer.
 * @param meta: the metadata
 * @return packed size on success, negative errno on error.
 */
VMETA_API ssize_t vmeta_frame_proto_get_packed_size(struct vmeta_frame *meta);


/**
 * Writer API (to initialize subfields)
 * Those functions must operate on pointers from a previous
 * vmeta_frame_proto_get_unpacked_rw() call. These functions are intended as
 * helpers for metadata providers, and should not be used in metadata consumer
 * applications.
 */


/**
 * Get the CameraMetadata part of a TimedMetadata (root metadata), creating
 * it if required.
 * @param meta: the TimedMetadata
 * @return A pointer to the CameraMetadata, or NULL on error.
 */
VMETA_API Vmeta__CameraMetadata *
vmeta_frame_proto_get_camera(Vmeta__TimedMetadata *meta);

/**
 * Get the base Quaternion part of a CameraMetadata, creating it if required.
 * @param camera: the CameraMetadata
 * @return A pointer to the base Quaternion, or NULL on error.
 */
VMETA_API Vmeta__Quaternion *
vmeta_frame_proto_get_camera_base_quat(Vmeta__CameraMetadata *camera);

/**
 * Get the frame Quaternion part of a CameraMetadata, creating it if required.
 * @param camera: the CameraMetadata
 * @return A pointer to the frame Quaternion, or NULL on error.
 */
VMETA_API Vmeta__Quaternion *
vmeta_frame_proto_get_camera_quat(Vmeta__CameraMetadata *camera);


/**
 * Get the DroneMetadata part of a TimedMetadata (root metadata), creating
 * it if required.
 * @param meta: the TimedMetadata
 * @return A pointer to the DroneMetadata, or NULL on error.
 */
VMETA_API Vmeta__DroneMetadata *
vmeta_frame_proto_get_drone(Vmeta__TimedMetadata *meta);

/**
 * Get the Location part of a DroneMetadata, creating it if required.
 * @param drone: the DroneMetadata
 * @return A pointer to the Location, or NULL on error.
 */
VMETA_API Vmeta__Location *
vmeta_frame_proto_get_drone_location(Vmeta__DroneMetadata *drone);

/**
 * Get the Quaternion part of a DroneMetadata, creating it if required.
 * @param drone: the DroneMetadata
 * @return A pointer to the Quaternion, or NULL on error.
 */
VMETA_API Vmeta__Quaternion *
vmeta_frame_proto_get_drone_quat(Vmeta__DroneMetadata *drone);

/**
 * Get the NED speed part of a DroneMetadata, creating it if required.
 * @param drone: the DroneMetadata
 * @return A pointer to the NED speed, or NULL on error.
 */
VMETA_API Vmeta__NED *
vmeta_frame_proto_get_drone_speed(Vmeta__DroneMetadata *drone);

/**
 * Get the NED position part of a DroneMetadata, creating it if required.
 * @param drone: the DroneMetadata
 * @return A pointer to the NED position, or NULL on error.
 */
VMETA_API Vmeta__NED *
vmeta_frame_proto_get_drone_position(Vmeta__DroneMetadata *drone);

/**
 * Get the Vect3 local position part of a DroneMetadata,
 * creating it if required.
 * @param drone: the DroneMetadata
 * @return A pointer to the Vec3 local position, or NULL on error.
 */
VMETA_API Vmeta__Vector3 *
vmeta_frame_proto_get_drone_local_position(Vmeta__DroneMetadata *drone);


/**
 * Add a new WifiLinkMetadata to a TimedMetadata (root metadata).
 * @param meta: the TimedMetadata
 * @return A pointer to the new WifiLinkMetadata, or NULL on error.
 */
VMETA_API Vmeta__WifiLinkMetadata *
vmeta_frame_proto_add_wifi_link(Vmeta__TimedMetadata *meta);


/**
 * Add a new StarfishLinkInfo to a StarfishLinkMetadata.
 * @param meta: the StarfishLinkMetadata
 * @return A pointer to the new StarfishLinkInfo, or NULL on error.
 */
VMETA_API Vmeta__StarfishLinkInfo *
vmeta_frame_proto_add_starfish_link_info(Vmeta__StarfishLinkMetadata *starfish);


/**
 * Add a new StarfishLinkMetadata to a TimedMetadata (root metadata).
 * @param meta: the TimedMetadata
 * @return A pointer to the new StarfishLinkMetadata, or NULL on error.
 */
VMETA_API Vmeta__StarfishLinkMetadata *
vmeta_frame_proto_add_starfish_link(Vmeta__TimedMetadata *meta);


/**
 * Get the TrackingMetadata part of a TimedMetadata (root metadata), creating
 * it if required.
 * @param meta: the TimedMetadata
 * @return A pointer to the TrackingMetadata, or NULL on error.
 */
VMETA_API Vmeta__TrackingMetadata *
vmeta_frame_proto_get_tracking(Vmeta__TimedMetadata *meta);

/**
 * Get the target BoundingBox part of a TrackingMetadata, creating it if
 * required.
 * @param tracking: the TrackingMetadata
 * @return A pointer to the target BoundingBox, or NULL on error.
 */
VMETA_API Vmeta__BoundingBox *
vmeta_frame_proto_get_tracking_target(Vmeta__TrackingMetadata *tracking);


/**
 * Get the TrackingProposalMetadata part of a TimedMetadata (root metadata),
 * creating it if required.
 * @param meta: the TimedMetadata
 * @return A pointer to the TrackingProposalMetadata, or NULL on error.
 */
VMETA_API Vmeta__TrackingProposalMetadata *
vmeta_frame_proto_get_proposal(Vmeta__TimedMetadata *meta);

/**
 * Add a new proposal BoundingBox to a TrackingProposalMetadata.
 * @param proposal: the TrackingProposalMetadata
 * @return A pointer to the new BoundingBox, or NULL on error.
 */
VMETA_API Vmeta__BoundingBox *
vmeta_frame_proto_proposal_add_box(Vmeta__TrackingProposalMetadata *proposal);


/**
 * Enum converters
 */

/**
 * Convert a Vmeta__FlyingState enum into its vmeta_flying_state equivalent.
 *
 * @param state: state to convert
 * @return The converted flying state
 */
VMETA_API enum vmeta_flying_state
vmeta_frame_flying_state_proto_to_vmeta(Vmeta__FlyingState state);


#endif /* !_VMETA_FRAME_PROTO_H_ */
