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

#ifndef _VMETA_SESSION_PROTO_H_
#define _VMETA_SESSION_PROTO_H_

#include "vmeta_proto.h"
#include <vmeta.pb-c.h>


/* "Parrot Video Metadata" protobuf-based structure definition
 * see Vmeta__SessionMetadata definition for more details */
struct vmeta_session;
struct vmeta_session_proto;


/**
 * Convert a session metadata structure to a Protobuf structure.
 * The vmeta_session_proto structure can be accessed either unpacked (in
 * read-only mode via vmeta_session_proto_get_unpacked() or in read-write mode
 * via vmeta_session_proto_get_unpacked_rw()) or packed via
 * vmeta_session_proto_get_buffer().
 * @note The returned structure must be destroyed after usage using
 * vmeta_session_proto_destroy().
 * @param meta: the session metadata to convert
 * @param proto_meta: output pointer to the protobuf session metadata structure
 * @return 0 on success, negative errno on error.
 */
VMETA_API int vmeta_session_to_proto(const struct vmeta_session *meta,
				     struct vmeta_session_proto **proto_meta);


/**
 * Destroy the protobuf session metadata structure.
 * The caller shoud no longer use the meta structure after calling this.
 * @param meta: pointer to the protobuf session metadata structure
 * @return 0 on success, negative errno on error.
 */
VMETA_API int vmeta_session_proto_destroy(struct vmeta_session_proto *meta);


/**
 * Field getters
 */

/**
 * Get the Protobuf structure backed by this metadata in read-only mode.
 * This function only works with unpacked buffers (either created from
 * vmeta_session_to_proto() or unpacked via vmeta_session_proto_unpack()).
 * The returned pointer is a direct pointer to the metadata contents, and
 * thus should not be modified.
 * The pointer is valid while the vmeta_session_proto pointer is valid
 * (see vmeta_session_proto_destroy()), and until the caller
 * calls vmeta_session_proto_release_unpacked().
 * @note If the caller destroys the metadata while holding a read-only unpacked
 * pointer, the metadata will be locked in read-only mode until destroyed.
 * The function returns an error if the metadata is not initialized, of a
 * wrong type, not available in unpacked mode, or already locked in read-write
 * mode.
 * @param meta: the session metadata
 * @param proto_meta: output pointer for the protobuf structure pointer
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_session_proto_get_unpacked(struct vmeta_session_proto *meta,
				 const Vmeta__SessionMetadata **proto_meta);

/**
 * Release the read-only protobuf structure to the metadata.
 * The proto_meta pointer should not be used again after this call.
 * @param meta: the session metadata
 * @param proto_meta: pointer to the protobuf structure
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_session_proto_release_unpacked(struct vmeta_session_proto *meta,
				     const Vmeta__SessionMetadata *proto_meta);

/**
 * Get the Protobuf structure backed by this metadata in read-write mode.
 * This function only works with unpacked buffers (either created from
 * vmeta_session_to_proto() or unpacked via vmeta_session_proto_get_unpacked()).
 * The returned pointer is a direct pointer to the metadata contents, that can
 * be modified. This call is intended for metadata providers.
 * The pointer is valid while the the vmeta_session_proto pointer is valid
 * metadata (see vmeta_session_proto_destroy()), and until the caller
 * calls vmeta_session_proto_release_unpacked_rw().
 * @note If the caller destroys the metadata while holding a read-write unpacked
 * pointer, the metadata will be locked until destroyed.
 * The function returns an error if the metadata is not initialized, of a
 * wrong type, not available in unpacked mode, or already locked in read-write
 * or read-only mode.
 * @param meta: the session metadata
 * @param proto_meta: output pointer for the protobuf structure pointer
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_session_proto_get_unpacked_rw(struct vmeta_session_proto *meta,
				    Vmeta__SessionMetadata **proto_meta);

/**
 * Release the read-write protobuf structure to the metadata.
 * The proto_meta pointer should not be used again after this call.
 * @note: Releasing the read-write view of the metadata will invalidate the
 * packed view of the metadata, a call to vmeta_session_proto_pack() is needed
 * to recreate the packed view.
 * @param meta: the session metadata
 * @param proto_meta: pointer to the protobuf structure
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_session_proto_release_unpacked_rw(struct vmeta_session_proto *meta,
					Vmeta__SessionMetadata *proto_meta);

/**
 * Get the packed protobuf data representing this metadata.
 * This function only works with unpacked buffers (either created from
 * vmeta_session_to_proto() or unpacked via vmeta_session_proto_get_unpacked()).
 * The returned pointer is a direct pointer to the metadata buffer, and
 * thus should not be modified.
 * The pointer is valid while the the vmeta_session_proto pointer is valid
 * metadata (see vmeta_session_proto_destroy()), and until the caller
 * calls vmeta_session_proto_release_buffer().
 * The function returns an error if the metadata is not initialized, of a
 * wrong type, not available in packed mode, or already locked in read-write
 * mode.
 * @param meta: the session metadata
 * @param buf: output pointer for the protobuf buffer
 * @param len: output pointer for the protobuf buffer length
 * @return 0 on success, negative errno on error.
 */
VMETA_API int vmeta_session_proto_get_buffer(struct vmeta_session_proto *meta,
					     const uint8_t **buf,
					     size_t *len);

/**
 * Release the read-only buffer to the metadata.
 * The buf pointer should not be used again after this call.
 * @param meta: the session metadata
 * @param buf: pointer to the protobuf buffer
 * @return 0 on success, negative errno on error.
 */
VMETA_API int
vmeta_session_proto_release_buffer(struct vmeta_session_proto *meta,
				   const uint8_t *buf);

/**
 * Get the packed size of the metadata.
 * On an unpacked metadata, the result is computed at each call.
 * On a previously packed metadata, this function returns the size of the
 * current packed buffer.
 * @param meta: the session metadata
 * @return packed size on success, negative errno on error.
 */
VMETA_API ssize_t
vmeta_session_proto_get_packed_size(struct vmeta_session_proto *meta);


/**
 * Writer API (to initialize subfields)
 * Those functions must operate on pointers from a previous
 * vmeta_frame_proto_get_unpacked_rw() call. These functions are intended as
 * helpers for metadata providers, and should not be used in metadata consumer
 * applications.
 */

/**
 * Get the Location part of a SessionMetadata (root metadata), creating
 * it if required.
 * @param proto_meta: the SessionMetadata
 * @return A pointer to the Location, or NULL on error.
 */
VMETA_API Vmeta__Location *
vmeta_session_proto_get_takeoff_location(Vmeta__SessionMetadata *proto_meta);

/**
 * Get the picture fov (Vector2) part of a SessionMetadata (root metadata),
 * creating it if required.
 * @param proto_meta: the SessionMetadata
 * @return A pointer to the picture fov (Vector2), or NULL on error.
 */
VMETA_API Vmeta__Vector2 *
vmeta_session_proto_get_picture_fov(Vmeta__SessionMetadata *proto_meta);

/**
 * Get the ThermalSessionMetadata part of a SessionMetadata (root metadata),
 * creating it if required.
 * @param proto_meta: the SessionMetadata
 * @return A pointer to the ThermalSessionMetadata, or NULL on error.
 */
VMETA_API Vmeta__ThermalSessionMetadata *
vmeta_session_proto_get_thermal_session_metadata(
	Vmeta__SessionMetadata *proto_meta);

/**
 * Get the ThermalAlignment part of a ThermalSessionMetadata,
 * creating it if required.
 * @param thermal_meta: the ThermalSessionMetadata
 * @return A pointer to the ThermalAlignment, or NULL on error.
 */
VMETA_API Vmeta__ThermalAlignment *vmeta_session_proto_get_thermal_alignment(
	Vmeta__ThermalSessionMetadata *thermal_meta);

/**
 * Get the alignment rotation (Euler) part of a ThermalAlignment,
 * creating it if required.
 * @param thermal_alignment: the ThermalAlignment
 * @return A pointer to the alignment rotation (Euler), or NULL on error.
 */
VMETA_API Vmeta__Euler *vmeta_session_proto_get_thermal_alignment_rotation(
	Vmeta__ThermalAlignment *thermal_alignment);

/**
 * Get the ThermalConversion low part of a ThermalSessionMetadata,
 * creating it if required.
 * @param thermal_meta: the ThermalSessionMetadata
 * @return A pointer to the ThermalConversion low, or NULL on error.
 */
VMETA_API Vmeta__ThermalConversion *
vmeta_session_proto_get_thermal_conversion_low(
	Vmeta__ThermalSessionMetadata *thermal_meta);


/**
 * Get the ThermalConversion high part of a ThermalSessionMetadata,
 * creating it if required.
 * @param thermal_meta: the ThermalSessionMetadata
 * @return A pointer to the ThermalConversion high, or NULL on error.
 */
VMETA_API Vmeta__ThermalConversion *
vmeta_session_proto_get_thermal_conversion_high(
	Vmeta__ThermalSessionMetadata *thermal_meta);

/**
 * Get the CameraModel part of a SessionMetadata (root metadata),
 * creating it if required.
 * @param proto_meta: the SessionMetadata
 * @return A pointer to the CameraModel, or NULL on error.
 */
VMETA_API Vmeta__CameraModel *
vmeta_session_proto_get_camera_model(Vmeta__SessionMetadata *proto_meta);

/**
 * Get the PerspectiveCameraModel part of a CameraModel, creating it if
 * required.
 * @param camera_model: the CameraModel
 * @return A pointer to the PerspectiveCameraModel, or NULL on error.
 */
VMETA_API Vmeta__CameraModel__PerspectiveCameraModel *
vmeta_session_proto_get_perspective_camera_model(
	Vmeta__CameraModel *camera_model);

/**
 * Get the Distorsion part of a PerspectiveCameraModel, creating it if required.
 * @param perspective: the PerspectiveCameraModel
 * @return A pointer to the Distorsion, or NULL on error.
 */
VMETA_API Vmeta__CameraModel__PerspectiveCameraModel__Distorsion *
vmeta_session_proto_get_perspective_camera_model_distorsion(
	Vmeta__CameraModel__PerspectiveCameraModel *perspective);

/**
 * Get the FisheyeCameraModel part of a CameraModel, creating it if required.
 * @param camera_model: the CameraModel
 * @return A pointer to the FisheyeCameraModel, or NULL on error.
 */
VMETA_API Vmeta__CameraModel__FisheyeCameraModel *
vmeta_session_proto_get_fisheye_camera_model(Vmeta__CameraModel *camera_model);

/**
 * Get the AffineMatrix part of a FisheyeCameraModel, creating it if required.
 * @param fisheye: the FisheyeCameraModel
 * @return A pointer to the Distorsion, or NULL on error.
 */
VMETA_API Vmeta__CameraModel__FisheyeCameraModel__AffineMatrix *
vmeta_session_proto_get_fisheye_camera_model_affine_matrix(
	Vmeta__CameraModel__FisheyeCameraModel *fisheye);

/**
 * Get the Polynomial part of a FisheyeCameraModel, creating it if required.
 * @param fisheye: the FisheyeCameraModel
 * @return A pointer to the Polynomial, or NULL on error.
 */
VMETA_API Vmeta__CameraModel__FisheyeCameraModel__Polynomial *
vmeta_session_proto_get_fisheye_camera_model_polynomial(
	Vmeta__CameraModel__FisheyeCameraModel *fisheye);

/**
 * Get the Overlay part of a SessionMetadata (root metadata),
 * creating it if required.
 * @param proto_meta: the SessionMetadata
 * @return A pointer to the Overlay, or NULL on error.
 */
VMETA_API Vmeta__Overlay *
vmeta_session_proto_get_overlay(Vmeta__SessionMetadata *proto_meta);

/**
 * Get the HeaderFooter part of a Overlay, creating it if required.
 * @param overlay: the Overlay
 * @return A pointer to the HeaderFooter, or NULL on error.
 */
VMETA_API Vmeta__Overlay__HeaderFooter *
vmeta_session_proto_get_overlay_header_footer(Vmeta__Overlay *overlay);

/**
 * Get the principal point (Vector2) part of a SessionMetadata (root metadata),
 * creating it if required.
 * @param proto_meta: the SessionMetadata
 * @return A pointer to the principal point (Vector2), or NULL on error.
 */
VMETA_API Vmeta__Vector2 *
vmeta_session_proto_get_principal_point(Vmeta__SessionMetadata *proto_meta);


/**
 * Enum converters
 */

/**
 * Convert a vmeta_camera_type enum into its Vmeta__CameraType equivalent.
 *
 * @param type: camera type to convert
 * @return The converted camera type
 */
VMETA_API Vmeta__CameraType
vmeta_session_camera_type_vmeta_to_proto(enum vmeta_camera_type type);

/**
 * Convert a vmeta_camera_spectrum enum into its Vmeta__CameraSpectrum
 * equivalent.
 *
 * @param spectrum: camera spectrum to convert
 * @return The converted camera spectrum
 */
VMETA_API Vmeta__CameraSpectrum vmeta_session_camera_spectrum_vmeta_to_proto(
	enum vmeta_camera_spectrum spectrum);

/**
 * Convert a vmeta_video_mode enum into its Vmeta__VideoMode equivalent.
 *
 * @param mode: video mode to convert
 * @return The converted video mode
 */
VMETA_API Vmeta__VideoMode
vmeta_session_video_mode_vmeta_to_proto(enum vmeta_video_mode mode);

/**
 * Convert a vmeta_video_stop_reason enum into its Vmeta__VideoStopReason
 * equivalent.
 *
 * @param reason: video stop reason to convert
 * @return The converted video stop reason
 */
VMETA_API Vmeta__VideoStopReason vmeta_session_video_stop_reason_vmeta_to_proto(
	enum vmeta_video_stop_reason reason);

/**
 * Convert a vmeta_dynamic_range enum into its Vmeta__DynamicRange equivalent.
 *
 * @param range: dynamic range to convert
 * @return The converted dynamic range
 */
VMETA_API Vmeta__DynamicRange
vmeta_session_dynamic_range_vmeta_to_proto(enum vmeta_dynamic_range range);

/**
 * Convert a vmeta_tone_mapping enum into its Vmeta__ToneMapping equivalent.
 *
 * @param mapping: tone mapping to convert
 * @return The converted tone mapping
 */
VMETA_API Vmeta__ToneMapping
vmeta_session_tone_mapping_vmeta_to_proto(enum vmeta_tone_mapping mapping);


#endif /* !_VMETA_SESSION_PROTO_H_ */
