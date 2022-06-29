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

#ifndef _VMETA_FRAME_H_
#define _VMETA_FRAME_H_


/* Frame metadata maximum size in bytes (redefined later) */
#define VMETA_FRAME_MAX_SIZE 0


/* "Parrot Video Metadata" timestamp extension identifier */
#define VMETA_FRAME_EXT_TIMESTAMP_ID 0x4531


/* "Parrot Video Metadata" follow-me extension identifier */
#define VMETA_FRAME_EXT_FOLLOWME_ID 0x4532


/* "Parrot Video Metadata" automation extension identifier */
#define VMETA_FRAME_EXT_AUTOMATION_ID 0x4533


/* "Parrot Video Metadata" thermal extension identifier */
#define VMETA_FRAME_EXT_THERMAL_ID 0x4534


/* "Parrot Video Metadata" Location From Image Coordinates
 * extension identifier */
#define VMETA_FRAME_EXT_LFIC_ID 0x4536


/* Flying states */
enum vmeta_flying_state {
	/* Landed state */
	VMETA_FLYING_STATE_LANDED = 0,

	/* Taking off state */
	VMETA_FLYING_STATE_TAKINGOFF,

	/* Hovering state */
	VMETA_FLYING_STATE_HOVERING,

	/* Flying state */
	VMETA_FLYING_STATE_FLYING,

	/* Landing state */
	VMETA_FLYING_STATE_LANDING,

	/* Emergency state */
	VMETA_FLYING_STATE_EMERGENCY,

	/* User take off state */
	VMETA_FLYING_STATE_USER_TAKEOFF,

	/* Motor ramping state */
	VMETA_FLYING_STATE_MOTOR_RAMPING,

	/* Emergency landing state */
	VMETA_FLYING_STATE_EMERGENCY_LANDING,
};


/* Piloting modes */
enum vmeta_piloting_mode {
	/* Manual piloting by the user */
	VMETA_PILOTING_MODE_MANUAL = 0,

	/* Automatic return home in progress */
	VMETA_PILOTING_MODE_RETURN_HOME,

	/* Automatic flight plan in progress */
	VMETA_PILOTING_MODE_FLIGHT_PLAN,

	/* Automatic tracking in progress */
	VMETA_PILOTING_MODE_TRACKING,

	/* Follow-me in progress
	 * (deprecated, use VMETA_PILOTING_MODE_TRACKING instead) */
	VMETA_PILOTING_MODE_FOLLOW_ME = VMETA_PILOTING_MODE_TRACKING,

	/* Automatic "magic carpet" test in progress */
	VMETA_PILOTING_MODE_MAGIC_CARPET,

	/* Automatic "move to" in progress */
	VMETA_PILOTING_MODE_MOVE_TO,

	/* Unknown piloting mode */
	VMETA_PILOTING_MODE_UNKNOWN,
};


/* Follow-me animations */
enum vmeta_followme_anim {
	/* No animation in progress */
	VMETA_FOLLOWME_ANIM_NONE = 0,

	/* Orbit animation in progress */
	VMETA_FOLLOWME_ANIM_ORBIT,

	/* Boomerang animation in progress */
	VMETA_FOLLOWME_ANIM_BOOMERANG,

	/* Parabola animation in progress */
	VMETA_FOLLOWME_ANIM_PARABOLA,

	/* Zenith animation in progress */
	VMETA_FOLLOWME_ANIM_ZENITH,
};


/* Automation animations */
enum vmeta_automation_anim {
	/* No animation in progress */
	VMETA_AUTOMATION_ANIM_NONE = 0,

	/* Orbit animation in progress */
	VMETA_AUTOMATION_ANIM_ORBIT,

	/* Boomerang animation in progress */
	VMETA_AUTOMATION_ANIM_BOOMERANG,

	/* Parabola animation in progress */
	VMETA_AUTOMATION_ANIM_PARABOLA,

	/* Dolly slide animation in progress */
	VMETA_AUTOMATION_ANIM_DOLLY_SLIDE,

	/* Dolly zoom animation in progress */
	VMETA_AUTOMATION_ANIM_DOLLY_ZOOM,

	/* Vertical reveal animation in progress */
	VMETA_AUTOMATION_ANIM_REVEAL_VERT,

	/* Horizontal reveal animation in progress */
	VMETA_AUTOMATION_ANIM_REVEAL_HORZ,

	/* Horizontal panorama animation in progress */
	VMETA_AUTOMATION_ANIM_PANORAMA_HORZ,

	/* Candle animation in progress */
	VMETA_AUTOMATION_ANIM_CANDLE,

	/* Front flip animation in progress */
	VMETA_AUTOMATION_ANIM_FLIP_FRONT,

	/* Back flip animation in progress */
	VMETA_AUTOMATION_ANIM_FLIP_BACK,

	/* Left flip animation in progress */
	VMETA_AUTOMATION_ANIM_FLIP_LEFT,

	/* Right flip animation in progress */
	VMETA_AUTOMATION_ANIM_FLIP_RIGHT,

	/* Twist up animation in progress */
	VMETA_AUTOMATION_ANIM_TWISTUP,

	/* Position twist up animation in progress */
	VMETA_AUTOMATION_ANIM_POSITION_TWISTUP,
};


/* Thermal calibration state. */
enum vmeta_thermal_calib_state {
	/* Thermal calibration done. */
	VMETA_THERMAL_CALIB_STATE_DONE = 0,

	/* Thermal calibration requested. */
	VMETA_THERMAL_CALIB_STATE_REQUESTED,

	/* Thermal calibration in progress. */
	VMETA_THERMAL_CALIB_STATE_IN_PROGRESS,
};


/* Frame metadata type */
enum vmeta_frame_type {
	/* No metadata */
	VMETA_FRAME_TYPE_NONE = 0,

	/* "Parrot Video Recording Metadata" v1 */
	VMETA_FRAME_TYPE_V1_RECORDING,

	/* "Parrot Video Streaming Metadata" v1 basic */
	VMETA_FRAME_TYPE_V1_STREAMING_BASIC,

	/* "Parrot Video Streaming Metadata" v1 extended */
	VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED,

	/* "Parrot Video Metadata" v2 */
	VMETA_FRAME_TYPE_V2,

	/* "Parrot Video Metadata" v3 */
	VMETA_FRAME_TYPE_V3,

	/* "Parrot Video Metadata" protobuf-based */
	VMETA_FRAME_TYPE_PROTO,
};


/* "Parrot Video Metadata" timestamp extension definition */
struct vmeta_frame_ext_timestamp {
	/* Frame capture timestamp (us, monotonic) */
	uint64_t frame_timestamp;
};


/* "Parrot Video Metadata" follow-me extension definition */
/* clang-format off */
struct vmeta_frame_ext_followme {
	/* Target location (without valid sv_count) */
	struct vmeta_location target;

	/* 0: follow-me disabled, 1: follow-me enabled */
	uint32_t enabled:1;

	/* 0: look-at-me, 1: follow-me */
	uint32_t mode:1;

	/* 0: NED (North-East-Down) absolute angle mode, 1: constant angle
	 * relative to the target movement */
	uint32_t angle_locked:1;

	/* Follow-me animation */
	enum vmeta_followme_anim animation;
};
/* clang-format on */


/* "Parrot Video Metadata" automation extension definition */
/* clang-format off */
struct vmeta_frame_ext_automation {
	/* Framing target location (without valid sv_count) */
	struct vmeta_location framing_target;

	/* Flight destination location (without valid sv_count) */
	struct vmeta_location flight_destination;

	/* 0: follow-me disabled, 1: follow-me enabled */
	uint32_t followme_enabled:1;

	/* 0: look-at-me disabled, 1: look-at-me enabled */
	uint32_t lookatme_enabled:1;

	/* 0: NED (North-East-Down) absolute angle mode, 1: constant angle
	 * relative to the target movement */
	uint32_t angle_locked:1;

	/* Automated animation */
	enum vmeta_automation_anim animation;
};
/* clang-format on */


/* "Parrot Video Metadata" thermal extension definition */
struct vmeta_frame_ext_thermal {
	/* Calibration state */
	enum vmeta_thermal_calib_state calib_state;

	/* Minimum temperature spot */
	struct vmeta_thermal_spot min;

	/* Maximum temperature spot */
	struct vmeta_thermal_spot max;

	/* Probe temperature */
	struct vmeta_thermal_spot probe;
};


/* "Parrot Video Metadata" Location From Image Coordinates
 * extension definition */
struct vmeta_frame_ext_lfic {
	/* Normalized horizontal position of the target cursor
	 * in the image ([0..1]) */
	float target_x;

	/* Normalized vertical position of the target cursor
	 * in the image ([0..1]) */
	float target_y;

	/* Location of the target */
	struct vmeta_location target_location;

	/* Estimated precision of the position (m) */
	double estimated_precision;

	/* Grid precision used for calculation (m) */
	double grid_precision;
};


/**
 * ToString function for enum vmeta_flying_state.
 * @param val: flying state value to convert
 * @return a string description of the flying state
 */
VMETA_API
const char *vmeta_flying_state_str(enum vmeta_flying_state val);


/**
 * ToString function for enum vmeta_piloting_mode.
 * @param val: piloting mode value to convert
 * @return a string description of the piloting mode
 */
VMETA_API
const char *vmeta_piloting_mode_str(enum vmeta_piloting_mode val);


/**
 * ToString function for enum vmeta_followme_anim.
 * @param val: follow-me animation value to convert
 * @return a string description of the follow-me animation
 */
VMETA_API
const char *vmeta_followme_anim_str(enum vmeta_followme_anim val);


/**
 * ToString function for enum vmeta_automation_anim.
 * @param val: automation animation value to convert
 * @return a string description of the automation animation
 */
VMETA_API
const char *vmeta_automation_anim_str(enum vmeta_automation_anim val);


/**
 * ToString function for enum vmeta_thermal_calibration_state.
 * @param val: thermal calibration state value to convert
 * @return a string description of the thermal calibration state
 */
VMETA_API
const char *vmeta_thermal_calib_state_str(enum vmeta_thermal_calib_state val);


/**
 * ToString function for enum vmeta_frame_type.
 * @param val: frame metadata type value to convert
 * @return a string description of the frame metadata type
 */
VMETA_API
const char *vmeta_frame_type_str(enum vmeta_frame_type val);


#include "video-metadata/vmeta_frame_proto.h"
#include "video-metadata/vmeta_frame_v1.h"
#include "video-metadata/vmeta_frame_v2.h"
#include "video-metadata/vmeta_frame_v3.h"


/* Frame metadata maximum size in bytes */
#if VMETA_FRAME_V1_STREAMING_BASIC_SIZE > VMETA_FRAME_MAX_SIZE
#	undef VMETA_FRAME_MAX_SIZE
#	define VMETA_FRAME_MAX_SIZE VMETA_FRAME_V1_STREAMING_BASIC_SIZE
#endif
#if VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE > VMETA_FRAME_MAX_SIZE
#	undef VMETA_FRAME_MAX_SIZE
#	define VMETA_FRAME_MAX_SIZE VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE
#endif
#if VMETA_FRAME_V1_RECORDING_SIZE > VMETA_FRAME_MAX_SIZE
#	undef VMETA_FRAME_MAX_SIZE
#	define VMETA_FRAME_MAX_SIZE VMETA_FRAME_V1_RECORDING_SIZE
#endif
#if VMETA_FRAME_V2_MAX_SIZE > VMETA_FRAME_MAX_SIZE
#	undef VMETA_FRAME_MAX_SIZE
#	define VMETA_FRAME_MAX_SIZE VMETA_FRAME_V2_MAX_SIZE
#endif
#if VMETA_FRAME_V3_MAX_SIZE > VMETA_FRAME_MAX_SIZE
#	undef VMETA_FRAME_MAX_SIZE
#	define VMETA_FRAME_MAX_SIZE VMETA_FRAME_V3_MAX_SIZE
#endif


/* Frame metadata */
struct vmeta_frame {
	/* Frame metadata */
	enum vmeta_frame_type type;

	union {
		/* "Parrot Video Recording Metadata" v1 */
		struct vmeta_frame_v1_recording v1_rec;

		/* "Parrot Video Streaming Metadata" v1 basic */
		struct vmeta_frame_v1_streaming_basic v1_strm_basic;

		/* "Parrot Video Streaming Metadata" v1 extended */
		struct vmeta_frame_v1_streaming_extended v1_strm_ext;

		/* "Parrot Video Metadata" v2 */
		struct vmeta_frame_v2 v2;

		/* "Parrot Video Metadata" v3 */
		struct vmeta_frame_v3 v3;

		/* "Parrot Video Metadata" protobuf-based */
		struct vmeta_frame_proto *proto;
	};

	/* Frame metadata ref_count.
	 * DO NOT USE THIS FIELD!
	 * Use vmeta_frame_ref()/vmeta_frame_unref() to modify it, or
	 * vmeta_frame_get_ref_count() to read it. */
	unsigned int ref_count;
};


/**
 * Write frame metadata.
 * This function fills the supplied buffer with the serialized metadata,
 * according to the metadata type. The size of the data written is returned
 * through the pos field in the buf structure.
 * The buf structure must have been previously initialized using the
 * vmeta_buffer_set_data() function with a previously allocated buffer and
 * its size. The pos field in the buf structure does not need to be 0 prior
 * to calling the function (i.e. data can already be present in the buffer).
 * The ownership of the buffer stays with the caller.
 * @param buf: pointer to the buffer structure (output)
 * @param meta: pointer to the frame metadata structure
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_write(struct vmeta_buffer *buf, struct vmeta_frame *meta);


/**
 * Read frame metadata.
 * This function allocates a new metadata structure, deserializing data from
 * the provided buffer, and detecting the metadata type when possible.
 * Providing the MIME type is mandatory for protobuf-based metadata or metadata
 * from recording files (MP4).
 * The pos field in the buf structure must be set to the starting position for
 * reading (can be 0 if no previous data is present in the buffer). The size
 * of the buffer (len field) must be sufficient to allow reading the metadata
 * otherwise an error is returned. The ownership of the buffer stays with
 * the caller.
 * The allocated structure has a reference count of 1, and is converted to
 * protobuf-based metadata when possible.
 * @param buf: pointer to the buffer structure
 * @param mime_type: pointer to the metadata MIME type, if known;
 *                   if NULL, the type will be automatically detected when
 *                   possible.
 * @param ret_obj: pointer filled with the new vmeta_frame structure
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_read(struct vmeta_buffer *buf,
		     const char *mime_type,
		     struct vmeta_frame **ret_obj);

/**
 * Read frame metadata.
 * This function allocates a new metadata structure, deserializing data from
 * the provided buffer, and detecting the metadata type when possible.
 * Providing the MIME type is mandatory for protobuf-based metadata or metadata
 * from recording files (MP4).
 * The pos field in the buf structure must be set to the starting position for
 * reading (can be 0 if no previous data is present in the buffer). The size
 * of the buffer (len field) must be sufficient to allow reading the metadata
 * otherwise an error is returned. The ownership of the buffer stays with
 * the caller.
 * The allocated structure has a reference count of 1.
 * If convert is set to a non-zero value, this function has the same behavior as
 * `vmeta_frame_read(...)': the allocated structure is converted to
 * protobuf-based metadata when possible. If convert is zero, the raw
 * deserialized metadata is returned.
 * @param buf: pointer to the buffer structure
 * @param mime_type: pointer to the metadata MIME type, if known;
 *                   if NULL, the type will be automatically detected when
 *                   possible.
 * @param convert: if non-zero, ret_obj will be converted to protobuf-based
 *                 metadata, when possible.
 * @param ret_obj: pointer filled with the new vmeta_frame structure
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_read2(struct vmeta_buffer *buf,
		      const char *mime_type,
		      int convert,
		      struct vmeta_frame **ret_obj);


/**
 * Create a vmeta_frame structure for writing.
 * The returned structure has a reference count of 1.
 * @param type: vmeta_frame type to create.
 * @param ret_obj: pointer filled with the new vmeta_frame structure
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API int vmeta_frame_new(enum vmeta_frame_type type,
			      struct vmeta_frame **ret_obj);


/**
 * Increment the reference counter of a vmeta_frame structure.
 * @param meta: pointer to the frame metadata structure
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API int vmeta_frame_ref(struct vmeta_frame *meta);


/**
 * Decrement the reference counter of a vmeta_frame structure.
 * When the reference counter reaches zero, the vmeta_frame structure will be
 * released. The caller shoud no longer use the meta structure after calling
 * this.
 * @param meta: pointer to the frame metadata structure
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API int vmeta_frame_unref(struct vmeta_frame *meta);


/**
 * Read the reference counter of a vmeta_frame structure.
 * If the reference counter is greater than INT_MAX, INT_MAX will be returned.
 * @param meta: pointer to the frame metadata structure
 * @return the current reference count of the metadata on success
 *         (0 or greater), negative errno value in case of error
 */
VMETA_API int vmeta_frame_get_ref_count(struct vmeta_frame *meta);


/**
 * Write frame metadata to a JSON object.
 * The jobj JSON object must have been previously allocated.
 * The ownership of the JSON object stays with the caller.
 * @param meta: pointer to a frame metadata structure
 * @param jobj: pointer to the JSON object to write to (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_to_json(struct vmeta_frame *meta, struct json_object *jobj);


/**
 * Write frame metadata to a JSON string.
 * This function fills the str array with the null-terminated JSON string.
 * The string must have been previously allocated. The function writes
 * up to len characters.
 * @param meta: pointer to a frame metadata structure
 * @param output: pointer to the string to write to
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_to_json_str(struct vmeta_frame *meta,
			    char *output,
			    unsigned int len);


/**
 * Write frame metadata as a CSV string.
 * The str string must have been previously allocated. The function writes
 * up to maxlen characters. The CSV separator is a space character.
 * @param meta: pointer to a frame metadata structure
 * @param str: pointer to the string to write to (output)
 * @param maxlen: maximum length of the string
 * @return the number of characters written on success, negative errno value
 *         in case of error
 */
VMETA_API
ssize_t
vmeta_frame_to_csv(const struct vmeta_frame *meta, char *str, size_t maxlen);


/**
 * Write a frame metadata CSV file header string for the given type.
 * The str string must have been previously allocated. The function writes
 * up to maxlen characters. The CSV separator is a space character.
 * @param type: frame metadata type
 * @param str: pointer to the string to write to (output)
 * @param maxlen: maximum length of the string
 * @return the number of characters written on success, negative errno value
 *         in case of error
 */
VMETA_API
ssize_t
vmeta_frame_csv_header(enum vmeta_frame_type type, char *str, size_t maxlen);


/**
 * Get the recording metadata MIME type.
 * The function returns the MIME type string for a given recording frame
 * metadata type.
 * The library has ownership of the returned string and it must not be freed.
 * @param type: frame metadata type
 * @return the MIME type or NULL if not appliable or if an error occurred
 */
VMETA_API
const char *vmeta_frame_get_mime_type(enum vmeta_frame_type type);


/**
 * Get the drone location from a frame metadata structure.
 * The function fills the loc structure with the drone location if it is
 * available according to the metadata type. If the location is not available
 * for the given type, -ENOENT is returned. When a location is available it
 * can still be invalid (valid field in the loc structure).
 * @param meta: pointer to a frame metadata structure
 * @param loc: pointer to a drone location structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_location(struct vmeta_frame *meta,
			     struct vmeta_location *loc);


/**
 * Get the drone speed in NED (North-East-Down) from a frame metadata structure.
 * The function fills the speed structure with the drone speed if it is
 * available according to the metadata type. If the speed is not available
 * for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param speed: pointer to a drone speed structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_speed_ned(struct vmeta_frame *meta,
			      struct vmeta_ned *speed);


/**
 * Get the drone air speed from a frame metadata structure.
 * The function fills the speed value with the air speed if it is available
 * according to the metadata type. If the air speed is not available for the
 * given type, -ENOENT is returned. When an air speed is available it can
 * still be invalid (-1 value).
 * @param meta: pointer to a frame metadata structure
 * @param speed: pointer to an air speed value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_air_speed(struct vmeta_frame *meta, float *speed);


/**
 * Get the drone ground distance from a frame metadata structure.
 * The function fills the dist value with the ground distance if it is
 * available according to the metadata type. If the ground distance is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param dist: pointer to a ground distance value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_ground_distance(struct vmeta_frame *meta, double *dist);


/**
 * Get the drone attitude Euler angles from a frame metadata structure.
 * The function fills the euler structure with the drone attitude if it is
 * available according to the metadata type. If the attitude is not available
 * for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param euler: pointer to a euler structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_drone_euler(struct vmeta_frame *meta,
				struct vmeta_euler *euler);


/**
 * Get the drone attitude quaternion from a frame metadata structure.
 * The function fills the quat structure with the drone attitude if it is
 * available according to the metadata type. If the attitude is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param quat: pointer to a quaternion structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_drone_quat(struct vmeta_frame *meta,
			       struct vmeta_quaternion *quat);


/**
 * Get the frame orientation Euler angles from a frame metadata structure.
 * The function fills the euler structure with the frame orientation if it is
 * available according to the metadata type. If the orientation is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param euler: pointer to a euler structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_frame_euler(struct vmeta_frame *meta,
				struct vmeta_euler *euler);


/**
 * Get the frame orientation quaternion from a frame metadata structure.
 * The function fills the quat structure with the frame orientation if it is
 * available according to the metadata type. If the orientation is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param quat: pointer to a quaternion structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_frame_quat(struct vmeta_frame *meta,
			       struct vmeta_quaternion *quat);


/**
 * Get the frame base orientation Euler angles from a frame metadata structure.
 * The function fills the euler structure with the frame base orientation if
 * it is available according to the metadata type. If the base orientation is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param euler: pointer to a euler structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_frame_base_euler(struct vmeta_frame *meta,
				     struct vmeta_euler *euler);


/**
 * Get the frame base orientation quaternion from a frame metadata structure.
 * The function fills the quat structure with the frame base orientation if
 * it is available according to the metadata type. If the base orientation is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param quat: pointer to a quaternion structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_frame_base_quat(struct vmeta_frame *meta,
				    struct vmeta_quaternion *quat);


/**
 * Get the frame capture timestamp from a frame metadata structure.
 * The function fills the timestamp value with the frame timestamp if it
 * is available according to the metadata type. If the timestamp is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param timestamp: pointer to a timestamp value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_frame_timestamp(struct vmeta_frame *meta,
				    uint64_t *timestamp);


/**
 * Get the camera pan angle from a frame metadata structure.
 * The function fills the pan value with the camera pan angle if it is
 * available according to the metadata type. If the camera pan angle is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param pan: pointer to a camera pan value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_camera_pan(struct vmeta_frame *meta, float *pan);


/**
 * Get the camera tilt angle from a frame metadata structure.
 * The function fills the tilt value with the camera tilt angle if it is
 * available according to the metadata type. If the camera tilt angle is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param pan: pointer to a camera tilt value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_camera_tilt(struct vmeta_frame *meta, float *tilt);


/**
 * Get the frame exposure time in milliseconds from a frame metadata structure.
 * The function fills the exp value with the frame exposure time if it is
 * available according to the metadata type. If the frame exposure time is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param exp: pointer to a frame exposure time value in ms (output).
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_exposure_time(struct vmeta_frame *meta, float *exp);


/**
 * Get the frame gain from a frame metadata structure.
 * The function fills the gain value with the frame gain if it is available
 * according to the metadata type. If the frame gain is not available for
 * the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param gain: pointer to a frame gain value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_gain(struct vmeta_frame *meta, uint16_t *gain);


/**
 * Get the frame AWB (Auto White Balance) red gain from a frame metadata
 * structure.
 * The function fills the gain value with the frame AWB red gain if it is
 * available according to the metadata type. If the frame AWB red gain is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param gain: pointer to a frame AWB red gain value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_awb_r_gain(struct vmeta_frame *meta, float *gain);


/**
 * Get the frame AWB (Auto White Balance) blue gain from a frame metadata
 * structure.
 * The function fills the gain value with the frame AWB blue gain if it is
 * available according to the metadata type. If the frame AWB blue gain is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param gain: pointer to a frame AWB blue gain value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_awb_b_gain(struct vmeta_frame *meta, float *gain);


/**
 * Get the picture horizontal field of view from a frame metadata structure.
 * The function fills the fov value with the picture horizontal field of view
 * in degrees if it is available according to the metadata type. If the FOV is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param fov: pointer to a FOV value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_picture_h_fov(struct vmeta_frame *meta, float *fov);


/**
 * Get the picture vertical field of view from a frame metadata structure.
 * The function fills the fov value with the picture vertical field of view
 * in degrees if it is available according to the metadata type. If the FOV is
 * not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param fov: pointer to a FOV value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_picture_v_fov(struct vmeta_frame *meta, float *fov);


/**
 * Get the link goodput (throughput estimation) from a frame metadata structure.
 * The function fills the goodput value with the link goodput if it is
 * available according to the metadata type. If the link goodput is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param goodput: pointer to a link goodput value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_link_goodput(struct vmeta_frame *meta, uint32_t *goodput);


/**
 * Get the link quality (0 to 5, 5 is best) from a frame metadata structure.
 * The function fills the quality value with the link quality if it is
 * available according to the metadata type. If the link quality is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param quality: pointer to a link quality value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_link_quality(struct vmeta_frame *meta, uint8_t *quality);


/**
 * Get the wifi RSSI from a frame metadata structure.
 * The function fills the rssi value with the wifi RSSI if it is
 * available according to the metadata type. If the wifi RSSI is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param rssi: pointer to a wifi RSSI value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_wifi_rssi(struct vmeta_frame *meta, int8_t *rssi);


/**
 * Get the battery charge percentage from a frame metadata structure.
 * The function fills the bat value with the battery charge percentage if
 * it is available according to the metadata type. If the battery charge
 * percentage is not available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param bat: pointer to a battery charge percentage value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_battery_percentage(struct vmeta_frame *meta, uint8_t *bat);


/**
 * Get the flying state from a frame metadata structure.
 * The function fills the state value with the flying state if it is
 * available according to the metadata type. If the flying state is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param state: pointer to a flying state value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_flying_state(struct vmeta_frame *meta,
				 enum vmeta_flying_state *state);


/**
 * Get the piloting mode from a frame metadata structure.
 * The function fills the mode value with the piloting mode if it is
 * available according to the metadata type. If the piloting mode is not
 * available for the given type, -ENOENT is returned.
 * @param meta: pointer to a frame metadata structure
 * @param mode: pointer to a piloting mode value (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_get_piloting_mode(struct vmeta_frame *meta,
				  enum vmeta_piloting_mode *mode);


/**
 * Location to image coordinates.
 * This function computes the normalized image coordinates from a metadata
 * structure (camera location, angles and FOV) and a location. If the location
 * is not visible in the image, the functions returns -ERANGE. Otherwise the
 * function fills the values pointed by screen_x and screen_y with the
 * normalized coordinates (i.e. in the range 0.0 to 1.0, top-left to
 * bottom-right).
 * @param meta: pointer to a frame metadata structure
 * @param loc: pointer to a location structure
 * @param screen_x: optional pointer to a normalized horizontal coordinate in
 *           the image [0..1] (output)
 * @param screen_y: optional pointer to a normalized vertical coordinate in
 *           the image [0..1] (output)
 * @param horizontal_distance: optional pointer to the horizontal distance
 *           in meters (output)
 * @param distance: optional pointer to the distance in meters (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API int vmeta_frame_ltic(struct vmeta_frame *meta,
			       const struct vmeta_location *loc,
			       float *screen_x,
			       float *screen_y,
			       float *horizontal_distance,
			       float *distance);

#endif /* !_VMETA_FRAME_H_ */
