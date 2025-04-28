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

#ifndef _VMETA_H_
#define _VMETA_H_

#include <stdint.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef VMETA_API_EXPORTS
#	ifdef _WIN32
#		define VMETA_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define VMETA_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !VMETA_API_EXPORTS */
#	define VMETA_API
#endif /* !VMETA_API_EXPORTS */


#define VMETA_API_VERSION 2


/* Forward declaration */
struct json_object;


/* Camera type */
enum vmeta_camera_type {
	/* Unknown camera type */
	VMETA_CAMERA_TYPE_UNKNOWN = 0,

	/* Front camera */
	VMETA_CAMERA_TYPE_FRONT,

	/* Front stereo camera */
	VMETA_CAMERA_TYPE_FRONT_STEREO,

	/* Front stereo left camera */
	VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT,

	/* Front stereo right camera */
	VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT,

	/* Vertical camera */
	VMETA_CAMERA_TYPE_VERTICAL,

	/* Disparity map */
	VMETA_CAMERA_TYPE_DISPARITY,

	/* Horizontal stereo camera */
	VMETA_CAMERA_TYPE_HORIZONTAL_STEREO,

	/* Horizontal stereo left camera */
	VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT,

	/* Horizontal stereo right camera */
	VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT,

	/* Down stereo camera */
	VMETA_CAMERA_TYPE_DOWN_STEREO,

	/* Down stereo left camera */
	VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT,

	/* Down stereo right camera */
	VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT,

	/* External camera */
	VMETA_CAMERA_TYPE_EXTERNAL,
};


/* Camera subtype */
enum vmeta_camera_subtype {
	/* Unknown camera subtype */
	VMETA_CAMERA_SUBTYPE_UNKNOWN = 0,

	/* Left camera */
	VMETA_CAMERA_SUBTYPE_LEFT,

	/* Right camera */
	VMETA_CAMERA_SUBTYPE_RIGHT,

	/* Wide camera */
	VMETA_CAMERA_SUBTYPE_WIDE,

	/* Tele camera */
	VMETA_CAMERA_SUBTYPE_TELE,

	/* Disparity map camera */
	VMETA_CAMERA_SUBTYPE_DISPARITY,

	/* Depth map camera */
	VMETA_CAMERA_SUBTYPE_DEPTH,
};


/* Camera spectrum */
enum vmeta_camera_spectrum {
	/* Unknown spectrum */
	VMETA_CAMERA_SPECTRUM_UNKNOWN = 0,

	/* Visible spectrum */
	VMETA_CAMERA_SPECTRUM_VISIBLE,

	/* Thermal spectrum */
	VMETA_CAMERA_SPECTRUM_THERMAL,

	/* Blended spectrum */
	VMETA_CAMERA_SPECTRUM_BLENDED,
};


/* Camera model type */
enum vmeta_camera_model_type {
	/* Unknown camera model type */
	VMETA_CAMERA_MODEL_TYPE_UNKNOWN = 0,

	/* Perspective camera (rectilinear) */
	VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE,

	/* Fisheye camera */
	VMETA_CAMERA_MODEL_TYPE_FISHEYE,
};


/* Overlay type */
enum vmeta_overlay_type {
	/* No overlay */
	VMETA_OVERLAY_TYPE_NONE = 0,

	/* Header-Footer overlay type */
	VMETA_OVERLAY_TYPE_HEADER_FOOTER,
};


/* Video mode */
enum vmeta_video_mode {
	/* Unknown video mode */
	VMETA_VIDEO_MODE_UNKNOWN = 0,

	/* Standard video mode */
	VMETA_VIDEO_MODE_STANDARD,

	/* Hyperlapse video mode */
	VMETA_VIDEO_MODE_HYPERLAPSE,

	/* Slow motion video mode */
	VMETA_VIDEO_MODE_SLOWMOTION,

	/* Stream recording video mode */
	VMETA_VIDEO_MODE_STREAMREC,
};


/* Video stop reason */
enum vmeta_video_stop_reason {
	/* Unknown stop reason */
	VMETA_VIDEO_STOP_REASON_UNKNOWN = 0,

	/* User request stop reason */
	VMETA_VIDEO_STOP_REASON_USER,

	/* Reconfigure stop reason */
	VMETA_VIDEO_STOP_REASON_RECONFIGURATION,

	/* Poor storage performance stop reason */
	VMETA_VIDEO_STOP_REASON_POOR_STORAGE_PERF,

	/* Storage full stop reason */
	VMETA_VIDEO_STOP_REASON_STORAGE_FULL,

	/* Recovery stop reason */
	VMETA_VIDEO_STOP_REASON_RECOVERY,

	/* End-of-stream stop reason */
	VMETA_VIDEO_STOP_REASON_END_OF_STREAM,

	/* Shutdown stop reason */
	VMETA_VIDEO_STOP_REASON_SHUTDOWN,

	/* Internal error stop reason */
	VMETA_VIDEO_STOP_REASON_INTERNAL_ERROR,
};


/* Image dynamic range */
enum vmeta_dynamic_range {
	/* Unknown dynamic range */
	VMETA_DYNAMIC_RANGE_UNKNOWN = 0,

	/* Standard dynamic range */
	VMETA_DYNAMIC_RANGE_SDR,

	/* High dynamic range: Parrot 8bit HDR */
	VMETA_DYNAMIC_RANGE_HDR8,

	/* High dynamic range: standard 10bit HDR10
	 * (Rec. ITU-R BT.2020 color primaries,
	 * SMPTE ST 2084 perceptual quantization transfer function
	 * and SMPTE ST 2086 metadata) */
	VMETA_DYNAMIC_RANGE_HDR10,
};


/* Image tone mapping */
enum vmeta_tone_mapping {
	/* Unknown tone mapping */
	VMETA_TONE_MAPPING_UNKNOWN = 0,

	/* Standard tone mapping */
	VMETA_TONE_MAPPING_STANDARD,

	/* Parrot P-log tone mapping */
	VMETA_TONE_MAPPING_P_LOG,
};


/* Metadata buffer */
struct vmeta_buffer {
	union {
		/* Buffer data */
		uint8_t *data;

		/* Buffer data (const) */
		const uint8_t *cdata;
	};

	/* Buffer size in bytes */
	size_t len;

	/* Current position in the buffer in bytes */
	size_t pos;
};


/**
 * Fill a buffer structure.
 * @param buf: pointer to a metadata buffer structure (output)
 * @param data: pointer to the data
 * @param len: length of the data in bytes
 * @param pos: current position in bytes
 */
static inline void vmeta_buffer_set_data(struct vmeta_buffer *buf,
					 uint8_t *data,
					 size_t len,
					 size_t pos)
{
	if (buf == NULL)
		return;
	buf->data = data;
	buf->len = len;
	buf->pos = pos;
}


/**
 * Fill a buffer structure (const data).
 * @param buf: pointer to a metadata buffer structure (output)
 * @param data: pointer to the data
 * @param len: length of the data in bytes
 * @param pos: current position in bytes
 */
static inline void vmeta_buffer_set_cdata(struct vmeta_buffer *buf,
					  const uint8_t *data,
					  size_t len,
					  size_t pos)
{
	if (buf == NULL)
		return;
	buf->cdata = data;
	buf->len = len;
	buf->pos = pos;
}


/* Quaternion */
struct vmeta_quaternion {
	float w;
	float x;
	float y;
	float z;
};


/* Euler angles */
struct vmeta_euler {
	union {
		float yaw;
		float psi;
	};
	union {
		float pitch;
		float theta;
	};
	union {
		float roll;
		float phi;
	};
};


/* XY vector */
struct vmeta_xy {
	float x;
	float y;
};


/* XYZ vector */
struct vmeta_xyz {
	float x;
	float y;
	float z;
};


/* North-East-Down (NED) vector */
struct vmeta_ned {
	float north;
	float east;
	float down;
};


/* rectangle (float) */
struct vmeta_rectf {
	float left;
	float top;
	float width;
	float height;
};


/* Field of view */
/* clang-format off */
struct vmeta_fov {
	/* Horizontal field of view (deg) */
	float horz;

	/* Vertical field of view (deg) */
	float vert;

	/* 1 if horizontal field of view is valid, 0 otherwise */
	uint32_t has_horz:1;

	/* 1 if vertical field of view is valid, 0 otherwise */
	uint32_t has_vert:1;
};
/* clang-format on */


/* Thermal spot */
/* clang-format off */
struct vmeta_thermal_spot {
	/* Position x; Normalized relative to the frame width; [0:1]
	 * where 0 is the left frame border (-1 means unknown) */
	float x;

	/* Position y; Normalized relative to the frame height; [0:1]
	 * where 0 is the top frame border (-1 means unknown) */
	float y;

	/* Temperature in kelvin for radiometric sensor (-1 means unknown) */
	float temp;

	/* Temperature in raw sensor signal unit (unsigned 16-bit value;
	 * -1 means unknown) */
	int32_t value;

	/* Validity flag (1 if the structure contents are valid, 0 otherwise) */
	uint8_t valid;
};
/* clang-format on */


#define VMETA_LOCATION_INVALID_SV_COUNT ((uint8_t)-1)


/* Location on earth */
struct vmeta_location {
	/* Latitude (deg) */
	double latitude;

	/* Longitude (deg) */
	double longitude;

	/* Altitude above the WGS84 ellipsoid (m) (NaN means unknown) */
	double altitude_wgs84ellipsoid;

	/* Altitude above the EGM96 geoid (AMSL) (m) (NaN means unknown) */
	double altitude_egm96amsl;

	/* Horizontal location accuracy (m), zero means unknown */
	float horizontal_accuracy;

	/* Vertical location accuracy (m), zero means unknown */
	float vertical_accuracy;

	/* GPS satellite vehicle count; set to VMETA_LOCATION_INVALID_SV_COUNT
	 * if not available even if valid is set */
	uint8_t sv_count;

	/* Validity flag (1 if the structure contents are valid, 0 otherwise;
	 * when a location is valid, sv_count may still be invalid) */
	uint8_t valid;
};


/**
 * Euler angles to quaternion conversion.
 * @param euler: pointer to a Euler angles structure
 * @param quat: pointer to a quaternion structure (output)
 */
VMETA_API
void vmeta_euler_to_quat(const struct vmeta_euler *euler,
			 struct vmeta_quaternion *quat);


/**
 * Quaternion to Euler angles conversion.
 * @param quat: pointer to a quaternion structure
 * @param euler: pointer to a Euler angles structure (output)
 */
VMETA_API
void vmeta_quat_to_euler(const struct vmeta_quaternion *quat,
			 struct vmeta_euler *euler);


/**
 * Get an enum vmeta_camera_type value from a string.
 * Valid strings are only the suffix of the camera type (eg. 'FRONT').
 * The case is ignored.
 * @param str: camera type string to convert
 * @return the enum vmeta_camera_type value or
 * VMETA_CAMERA_TYPE_UNKNOWN if unknown
 */
VMETA_API enum vmeta_camera_type vmeta_camera_type_from_str(const char *str);


/**
 * Get a string from an enum vmeta_camera_type value.
 * @param val: camera type value to convert
 * @return a string description of the camera type
 */
VMETA_API const char *vmeta_camera_type_to_str(enum vmeta_camera_type val);


/**
 * Get an enum vmeta_camera_subtype value from a string.
 * Valid strings are only the suffix of the camera subtype (eg. 'LEFT').
 * The case is ignored.
 * @param str: camera subtype string to convert
 * @return the enum vmeta_camera_subtype value or
 * VMETA_CAMERA_SUBTYPE_UNKNOWN if unknown
 */
VMETA_API enum vmeta_camera_subtype
vmeta_camera_subtype_from_str(const char *str);


/**
 * Get a string from an enum vmeta_camera_subtype value.
 * @param val: camera subtype value to convert
 * @return a string description of the camera subtype
 */
VMETA_API const char *
vmeta_camera_subtype_to_str(enum vmeta_camera_subtype val);


/**
 * Get an enum vmeta_camera_spectrum value from a string.
 * Valid strings are only the suffix of the camera spectrum (eg. 'VISIBLE').
 * The case is ignored.
 * @param str: camera spectrum string to convert
 * @return the enum vmeta_camera_spectrum value or
 * VMETA_CAMERA_SPECTRUM_UNKNOWN if unknown
 */
VMETA_API enum vmeta_camera_spectrum
vmeta_camera_spectrum_from_str(const char *str);


/**
 * Get a string from an enum vmeta_camera_spectrum value.
 * @param val: camera spectrum value to convert
 * @return a string description of the camera spectrum
 */
VMETA_API const char *
vmeta_camera_spectrum_to_str(enum vmeta_camera_spectrum val);


/**
 * Get an enum vmeta_camera_model_type value from a string.
 * Valid strings are only the suffix of the camera model type (eg. 'FISHEYE').
 * The case is ignored.
 * @param str: camera model type string to convert
 * @return the enum vmeta_camera_model_type value or
 * VMETA_CAMERA_MODEL_TYPE_UNKNOWN if unknown
 */
VMETA_API enum vmeta_camera_model_type
vmeta_camera_model_type_from_str(const char *str);


/**
 * Get a string from an enum vmeta_camera_model_type value.
 * @param val: camera model type value to convert
 * @return a string description of the camera model type
 */
VMETA_API const char *
vmeta_camera_model_type_to_str(enum vmeta_camera_model_type val);


/**
 * Get a string from an enum vmeta_overlay_type value.
 * @param val: overlay type value to convert
 * @return a string description of the overlay type
 */
VMETA_API const char *vmeta_overlay_type_to_str(enum vmeta_overlay_type val);


/**
 * Get an enum vmeta_video_mode value from a string.
 * Valid strings are only the suffix of the video mode (eg. 'STANDARD').
 * The case is ignored.
 * @param str: video mode string to convert
 * @return the enum vmeta_video_mode value or
 * VMETA_VIDEO_MODE_UNKNOWN if unknown
 */
VMETA_API enum vmeta_video_mode vmeta_video_mode_from_str(const char *str);


/**
 * Get a string from an enum vmeta_video_mode value.
 * @param val: video mode value to convert
 * @return a string description of the video mode
 */
VMETA_API const char *vmeta_video_mode_to_str(enum vmeta_video_mode val);


/**
 * Get an enum vmeta_video_stop_reason value from a string
 * Valid strings are only the suffix of the stop reason (eg. 'USER').
 * The case is ignored.
 * @param str: stop reason string to convert
 * @return the enum vmeta_video_stop_reason value or
 * VMETA_VIDEO_STOP_REASON_UNKNOWN if unknown
 */
VMETA_API enum vmeta_video_stop_reason
vmeta_video_stop_reason_from_str(const char *str);


/**
 * Get a string from an enum vmeta_video_stop_reason value.
 * @param val: stop reason value to convert
 * @return a string description of the stop reason
 */
VMETA_API const char *
vmeta_video_stop_reason_to_str(enum vmeta_video_stop_reason val);


/**
 * Get an enum vmeta_dynamic_range value from a string
 * Valid strings are only the suffix of the dynamic range (eg. 'HDR10').
 * The case is ignored.
 * @param str: dynamic range string to convert
 * @return the enum vmeta_dynamic_range value or
 * VMETA_DYNAMIC_RANGE_UNKNOWN if unknown
 */
VMETA_API enum vmeta_dynamic_range
vmeta_dynamic_range_from_str(const char *str);


/**
 * Get a string from an enum vmeta_dynamic_range value.
 * @param val: dynamic range value to convert
 * @return a string description of the dynamic range
 */
VMETA_API const char *vmeta_dynamic_range_to_str(enum vmeta_dynamic_range val);


/**
 * Get an enum vmeta_tone_mapping value from a string
 * Valid strings are only the suffix of the tone mapping (eg. 'plog').
 * The case is ignored.
 * @param str: tone mapping string to convert
 * @return the enum vmeta_tone_mapping value or
 * VMETA_TONE_MAPPING_UNKNOWN if unknown
 */
VMETA_API enum vmeta_tone_mapping vmeta_tone_mapping_from_str(const char *str);


/**
 * Get a string from an enum vmeta_tone_mapping value.
 * @param val: tone mapping value to convert
 * @return a string description of the tone mapping
 */
VMETA_API const char *vmeta_tone_mapping_to_str(enum vmeta_tone_mapping val);


#include "video-metadata/vmeta_frame.h"
#include "video-metadata/vmeta_session.h"


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VMETA_H_ */
