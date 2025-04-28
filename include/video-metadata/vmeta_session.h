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

#ifndef _VMETA_SESSION_H_
#define _VMETA_SESSION_H_

#include <time.h>


/* Maximum length of a media date or a run date string */
#define VMETA_SESSION_DATE_MAX_LEN 26


/* Maximum length of a Parrot serial number */
#define VMETA_SESSION_PARROT_SERIAL_MAX_LEN 18

/* Maximum length of the camera serial number pattern */
#define VMETA_SESSION_CAMERA_SERIAL_PATTERN_MAX_LEN                            \
	(sizeof("wide:;tele:") + 2 * VMETA_SESSION_PARROT_SERIAL_MAX_LEN + 1)


/**
 * Location used for takeoff location; the main format is ISO 6709 Annex H
 * (eg. "+16.42850589-061.53569552+6.80/")
 */

/* Format for comma separated values (latitude,longitude,altitude)
 * (deprecated, use VMETA_SESSION_LOCATION_FORMAT_ISO6709) */
#define VMETA_SESSION_LOCATION_FORMAT_CSV "%.8f,%.8f,%.3f"

/* Format for an ISO 6709 Annex H string (used on streaming in SDES/RTCP and
 * SDP, and recording in a 'meta' box) */
#define VMETA_SESSION_LOCATION_FORMAT_ISO6709 "%+012.8f%+013.8f%+.2f/"

/* Format for an Android-compatible modified ISO 6709 Annex H string (used on
 * recording in a 'udta'/'.xyz' box) */
#define VMETA_SESSION_LOCATION_FORMAT_XYZ "%+08.4f%+09.4f/"

/* Maximum length of a location string */
#define VMETA_SESSION_LOCATION_MAX_LEN 40

/* Location format */
enum vmeta_session_location_format {
	/* Comma separated values (latitude,longitude,altitude)
	 * (deprecated, use VMETA_SESSION_LOCATION_ISO6709) */
	VMETA_SESSION_LOCATION_CSV = 0,

	/* ISO 6709 Annex H string (used on streaming in SDES/RTCP and SDP,
	 * and recording in a 'meta' box); the altitude is above the
	 * EGM96 geoid (AMSL) */
	VMETA_SESSION_LOCATION_ISO6709,

	/* Android-compatible modified ISO 6709 Annex H string
	 * (used on recording in a 'udta'/'.xyz' box) */
	VMETA_SESSION_LOCATION_XYZ,
};


/**
 * Field of view used for picture FOV; format is hfov,vfov
 * (eg. "78.00,49.00")
 */

/* Format for a field of view string */
#define VMETA_SESSION_FOV_FORMAT "%.2f,%.2f"

/* Maximum length of a field of view string */
#define VMETA_SESSION_FOV_MAX_LEN 14


/**
 * Perspective distortion parameters; format is R1,R2,R3,T1,T2 with R1, R2
 * and R3 the radial distortion coefficients, and T1 and T2 the tangential
 * distortion coefficients
 * (see https://support.pix4d.com/hc/en-us/articles/202559089)
 */

/* Format for a perspective distortion parameters string */
#define VMETA_SESSION_PERSPECTIVE_DISTORTION_FORMAT "%.8f,%.8f,%.8f,%.8f,%.8f"

/* Maximum length of a perspective distortion parameters string */
#define VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN 64


/**
 * Fisheye affine matrix coefficients; format is C,D,E,F
 * (see https://support.pix4d.com/hc/en-us/articles/202559089)
 */

/* Format for a fisheye affine matrix coefficients string */
#define VMETA_SESSION_FISHEYE_AFFINE_MATRIX_FORMAT "%.8f,%.8f,%.8f,%.8f"

/* Maximum length of a fisheye affine matrix coefficients string */
#define VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN 64


/**
 * Fisheye polynomial coefficients; format is p0,p1,p2,p3,p4
 * with p0 = 0 and p1 = 1
 * (see https://support.pix4d.com/hc/en-us/articles/202559089)
 */

/* Format for a fisheye polynomial coefficients string */
#define VMETA_SESSION_FISHEYE_POLYNOMIAL_FORMAT "0,1,%.8f,%.8f,%.8f"

/* Maximum length of a fisheye polynomial coefficientss string */
#define VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN 64


/* Format for a header-footer overlay string */
#define VMETA_SESSION_OVERLAY_HEADER_FOOTER_FORMAT "%.3f,%.3f"

/* Maximum length of a header-footer overlay string */
#define VMETA_SESSION_OVERLAY_HEADER_FOOTER_MAX_LEN 64

/**
 * Thermal camera alignment parameters; format is yaw,pitch,roll
 * (eg. "-1.355,0.609,89.730")
 */

/* Format for a thermal camera alignement parameters string */
#define VMETA_SESSION_THERMAL_ALIGNMENT_FORMAT "%.3f,%.3f,%.3f"

/* Maximum length of a thermal camera alignement parameters string */
#define VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN 25


/**
 * Thermal camera temperature conversion parameters; format is
 * R,B,F,O,TauWin,TWin,TBg,emissivity (see struct vmeta_thermal_conversion)
 * (eg. "1390082.947851,1449.5,1.0,1476.356,0.8,25.0,22.0,0.98")
 */

/* Format for a thermal camera temperature conversion parameters string */
#define VMETA_SESSION_THERMAL_CONVERSION_FORMAT                                \
	"%.6f,%.1f,%.1f,%.3f,%.1f,%.1f,%.1f,%.2f"

/* Maximum length of a thermal camera temperature conversion parameters
 * string */
#define VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN 100


/**
 * Thermal camera scale factor; format is scaleFactor (eg. "1.035156")
 */

/* Format for a thermal camera scale factor string */
#define VMETA_SESSION_THERMAL_SCALE_FACTOR_FORMAT "%.6lf"

/* Maximum length of a thermal camera scale factor string */
#define VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN 10


/**
 * Principal point; format is "x,y" with x and y both normalized to the picture
 * width ([0..1]; eg. "0.491170,0.395359")
 */

/* Format for a principal point string */
#define VMETA_SESSION_PRINCIPAL_POINT_FORMAT "%.6f,%.6f"

/* Maximum length of a principal point string */
#define VMETA_SESSION_PRINCIPAL_POINT_MAX_LEN 20


/* RTCP SDES packet types used on streaming */
enum vmeta_stream_sdes_type {
	/* END: unused for session metadata */
	VMETA_STRM_SDES_TYPE_END = 0,

	/* CNAME: used for serial_number */
	VMETA_STRM_SDES_TYPE_CNAME,

	/* NAME: used for friendly_name */
	VMETA_STRM_SDES_TYPE_NAME,

	/* EMAIL: unused for session metadata */
	VMETA_STRM_SDES_TYPE_EMAIL,

	/* PHONE: unused for session metadata */
	VMETA_STRM_SDES_TYPE_PHONE,

	/* LOC: used for location */
	VMETA_STRM_SDES_TYPE_LOC,

	/* TOOL: used for software_version */
	VMETA_STRM_SDES_TYPE_TOOL,

	/* NOTE: unused for session metadata */
	VMETA_STRM_SDES_TYPE_NOTE,

	/* PRIV: other values (see below) */
	VMETA_STRM_SDES_TYPE_PRIV,
};


/**
 * Keys used on streaming for RTCP SDES private items
 * NB: the following values do not use SDES private items:
 * - friendly_name uses SDES NAME type
 * - serial_number uses SDES CNAME type
 * - software_version uses SDES TOOL type
 * - takeoff_loc uses SDES LOC type
 */

/* Media date */
#define VMETA_STRM_SDES_KEY_MEDIA_DATE "media_date"

/* Run date */
#define VMETA_STRM_SDES_KEY_RUN_DATE "run_date"

/* Run UUID */
#define VMETA_STRM_SDES_KEY_RUN_ID "run_id"

/* Boot date */
#define VMETA_STRM_SDES_KEY_BOOT_DATE "boot_date"

/* Boot UUID */
#define VMETA_STRM_SDES_KEY_BOOT_ID "boot_id"

/* Flight date */
#define VMETA_STRM_SDES_KEY_FLIGHT_DATE "flight_date"

/* Flight UUID */
#define VMETA_STRM_SDES_KEY_FLIGHT_ID "flight_id"

/* Takeoff location */
#define VMETA_STRM_SDES_KEY_TAKEOFF_LOC "takeoff_location"

/* Custom ID */
#define VMETA_STRM_SDES_KEY_CUSTOM_ID "custom_id"

/* Product maker */
#define VMETA_STRM_SDES_KEY_MAKER "maker"

/* Product model */
#define VMETA_STRM_SDES_KEY_MODEL "model"

/* Model ID */
#define VMETA_STRM_SDES_KEY_MODEL_ID "model_id"

/* Software build ID */
#define VMETA_STRM_SDES_KEY_BUILD_ID "build_id"

/* Video title */
#define VMETA_STRM_SDES_KEY_TITLE "title"

/* Video comment */
#define VMETA_STRM_SDES_KEY_COMMENT "comment"

/* Video copyright */
#define VMETA_STRM_SDES_KEY_COPYRIGHT "copyright"

/* Picture horizontal field of view
 * (deprecated; use VMETA_STRM_SDES_KEY_PICTURE_FOV) */
#define VMETA_STRM_SDES_KEY_PICTURE_HORZ_FOV "picture_hfov"

/* Picture vertical field of view
 * (deprecated; use VMETA_STRM_SDES_KEY_PICTURE_FOV) */
#define VMETA_STRM_SDES_KEY_PICTURE_VERT_FOV "picture_vfov"

/* Picture field of view */
#define VMETA_STRM_SDES_KEY_PICTURE_FOV "picture_fov"

/* Thermal metadata version */
#define VMETA_STRM_SDES_KEY_THERMAL_METAVERSION "thermal_metaversion"

/* Thermal camera serial number */
#define VMETA_STRM_SDES_KEY_THERMAL_CAMSERIAL "thermal_camserial"

/* Thermal camera alignment parameters */
#define VMETA_STRM_SDES_KEY_THERMAL_ALIGNMENT "thermal_alignment"

/* Thermal camera temperature conversion parameters (low gain) */
#define VMETA_STRM_SDES_KEY_THERMAL_CONV_LOW "thermal_conv_low"

/* Thermal camera temperature conversion parameters (high gain) */
#define VMETA_STRM_SDES_KEY_THERMAL_CONV_HIGH "thermal_conv_high"

/* Thermal camera scale factor */
#define VMETA_STRM_SDES_KEY_THERMAL_SCALE_FACTOR "thermal_scale_factor"

/* Camera type */
#define VMETA_STRM_SDES_KEY_CAMERA_TYPE "camera_type"

/* Camera subtype */
#define VMETA_STRM_SDES_KEY_CAMERA_SUBTYPE "camera_subtype"

/* Camera spectrum */
#define VMETA_STRM_SDES_KEY_CAMERA_SPECTRUM "camera_spectrum"

/* Camera serial number */
#define VMETA_STRM_SDES_KEY_CAMERA_SERIAL_NUMBER "camera_serial_number"

/* Camera model type */
#define VMETA_STRM_SDES_KEY_CAMERA_MODEL_TYPE "camera_model_type"

/* Perspective distortion parameters */
#define VMETA_STRM_SDES_KEY_PERSPECTIVE_DISTORTION "perspective_distortion"

/* Fisheye affine matrix coefficients */
#define VMETA_STRM_SDES_KEY_FISHEYE_AFFINE_MATRIX "fisheye_affine_matrix"

/* Fisheye polynomial coefficients */
#define VMETA_STRM_SDES_KEY_FISHEYE_POLYNOMIAL "fisheye_polynomial"

/* Video mode */
#define VMETA_STRM_SDES_KEY_VIDEO_MODE "video_mode"

/* Video stop reason */
#define VMETA_STRM_SDES_KEY_VIDEO_STOP_REASON "video_stop_reason"

/* Image dynamic range */
#define VMETA_STRM_SDES_KEY_DYNAMIC_RANGE "dynamic_range"

/* Image tone mapping */
#define VMETA_STRM_SDES_KEY_TONE_MAPPING "tone_mapping"

/* Capture timestamp of the first frame */
#define VMETA_STRM_SDES_KEY_FIRST_FRAME_CAPTURE_TS "first_frame_capture_ts"

/* Sample index of the first frame */
#define VMETA_STRM_SDES_KEY_FIRST_FRAME_SAMPLE_INDEX "first_frame_sample_index"

/* Unique media identifier of the video */
#define VMETA_STRM_SDES_KEY_MEDIA_ID "media_id"

/* Resource index of the video in the media ID  */
#define VMETA_STRM_SDES_KEY_RESOURCE_INDEX "resource_index"

/* Principal point of the camera */
#define VMETA_STRM_SDES_KEY_PRINCIPAL_POINT "principal_point"


/* SDP types used on streaming */
enum vmeta_stream_sdp_type {
	/* Info ('i='): used for friendly_name */
	VMETA_STRM_SDP_TYPE_SESSION_INFO,

	/* Name ('s='): used for title */
	VMETA_STRM_SDP_TYPE_SESSION_NAME,

	/* Tool ('a=tool:'): used for software_version */
	VMETA_STRM_SDP_TYPE_SESSION_TOOL,

	/* Session-level attribute extensions ('a=X-*') */
	VMETA_STRM_SDP_TYPE_SESSION_ATTR,

	/* Media-level info ('i='): used for media description */
	VMETA_STRM_SDP_TYPE_MEDIA_INFO,

	/* Media-level attribute extensions ('a=X-*') */
	VMETA_STRM_SDP_TYPE_MEDIA_ATTR,
};


/**
 * Keys used on streaming for SDP session and media attribute extensions
 * NB: the following values do not use attribute extensions:
 * - friendly_name uses SDP session information ('i=')
 * - title uses SDP session name ('s=')
 * - software_version uses SDP session-level tool attribute ('a=tool:')
 */

/* Session-level only attributes */

/* Media date */
#define VMETA_STRM_SDP_KEY_MEDIA_DATE "X-com-parrot-media-date"

/* Run date */
#define VMETA_STRM_SDP_KEY_RUN_DATE "X-com-parrot-run-date"

/* Run UUID */
#define VMETA_STRM_SDP_KEY_RUN_ID "X-com-parrot-run-id"

/* Boot date */
#define VMETA_STRM_SDP_KEY_BOOT_DATE "X-com-parrot-boot-date"

/* Boot UUID */
#define VMETA_STRM_SDP_KEY_BOOT_ID "X-com-parrot-boot-id"

/* Flight date */
#define VMETA_STRM_SDP_KEY_FLIGHT_DATE "X-com-parrot-flight-date"

/* Flight UUID */
#define VMETA_STRM_SDP_KEY_FLIGHT_ID "X-com-parrot-flight-id"

/* Custom ID */
#define VMETA_STRM_SDP_KEY_CUSTOM_ID "X-com-parrot-custom-id"

/* Product maker */
#define VMETA_STRM_SDP_KEY_MAKER "X-com-parrot-maker"

/* Product model */
#define VMETA_STRM_SDP_KEY_MODEL "X-com-parrot-model"

/* Model ID */
#define VMETA_STRM_SDP_KEY_MODEL_ID "X-com-parrot-model-id"

/* Product serial number */
#define VMETA_STRM_SDP_KEY_SERIAL_NUMBER "X-com-parrot-serial"

/* Software build ID */
#define VMETA_STRM_SDP_KEY_BUILD_ID "X-com-parrot-build-id"

/* Video comment */
#define VMETA_STRM_SDP_KEY_COMMENT "X-com-parrot-comment"

/* Video copyright */
#define VMETA_STRM_SDP_KEY_COPYRIGHT "X-com-parrot-copyright"

/* Takeoff location */
#define VMETA_STRM_SDP_KEY_TAKEOFF_LOC "X-com-parrot-takeoff-loc"

/* Location */
#define VMETA_STRM_SDP_KEY_LOCATION "X-com-parrot-location"

/* Either session-level or media-level attributes */

/* Picture field of view */
#define VMETA_STRM_SDP_KEY_PICTURE_FOV "X-com-parrot-picture-fov"

/* Thermal metadata version */
#define VMETA_STRM_SDP_KEY_THERMAL_METAVERSION                                 \
	"X-com-parrot-thermal-metaversion"

/* Thermal camera serial number */
#define VMETA_STRM_SDP_KEY_THERMAL_CAMSERIAL "X-com-parrot-thermal-camserial"

/* Thermal camera alignment parameters */
#define VMETA_STRM_SDP_KEY_THERMAL_ALIGNMENT "X-com-parrot-thermal-alignment"

/* Thermal camera temperature conversion parameters (low gain) */
#define VMETA_STRM_SDP_KEY_THERMAL_CONV_LOW "X-com-parrot-thermal-conv-low"

/* Thermal camera temperature conversion parameters (high gain) */
#define VMETA_STRM_SDP_KEY_THERMAL_CONV_HIGH "X-com-parrot-thermal-conv-high"

/* Thermal camera scale factor */
#define VMETA_STRM_SDP_KEY_THERMAL_SCALE_FACTOR                                \
	"X-com-parrot-thermal-scale-factor"

/* Default media */
#define VMETA_STRM_SDP_KEY_DEFAULT_MEDIA "X-com-parrot-default-media"

/* Camera type */
#define VMETA_STRM_SDP_KEY_CAMERA_TYPE "X-com-parrot-camera-type"

/* Camera subtype */
#define VMETA_STRM_SDP_KEY_CAMERA_SUBTYPE "X-com-parrot-camera-subtype"

/* Camera spectrum */
#define VMETA_STRM_SDP_KEY_CAMERA_SPECTRUM "X-com-parrot-camera-spectrum"

/* Camera serial number */
#define VMETA_STRM_SDP_KEY_CAMERA_SERIAL_NUMBER "X-com-parrot-camera-serial"

/* Camera model type */
#define VMETA_STRM_SDP_KEY_CAMERA_MODEL_TYPE "X-com-parrot-camera-model-type"

/* Perspective distortion parameters */
#define VMETA_STRM_SDP_KEY_PERSPECTIVE_DISTORTION                              \
	"X-com-parrot-perspective-distortion"

/* Fisheye affine matrix coefficients */
#define VMETA_STRM_SDP_KEY_FISHEYE_AFFINE_MATRIX                               \
	"X-com-parrot-fisheye-affine-matrix"

/* Fisheye polynomial coefficients */
#define VMETA_STRM_SDP_KEY_FISHEYE_POLYNOMIAL "X-com-parrot-fisheye-polynomial"

/* Video mode */
#define VMETA_STRM_SDP_KEY_VIDEO_MODE "X-com-parrot-video-mode"

/* Video stop reason */
#define VMETA_STRM_SDP_KEY_VIDEO_STOP_REASON "X-com-parrot-video-stop-reason"

/* Image dynamic range */
#define VMETA_STRM_SDP_KEY_DYNAMIC_RANGE "X-com-parrot-dynamic-range"

/* Image tone mapping */
#define VMETA_STRM_SDP_KEY_TONE_MAPPING "X-com-parrot-tone-mapping"

/* Capture timestamp of the first frame */
#define VMETA_STRM_SDP_KEY_FIRST_FRAME_CAPTURE_TS                              \
	"X-com-parrot-first-frame-capture-ts"

/* Sample index of the first frame */
#define VMETA_STRM_SDP_KEY_FIRST_FRAME_SAMPLE_INDEX                            \
	"X-com-parrot-first-frame-sample-index"

/* Unique media identifier of the video */
#define VMETA_STRM_SDP_KEY_MEDIA_ID "X-com-parrot-media-id"

/* Resource index of the video in the media ID  */
#define VMETA_STRM_SDP_KEY_RESOURCE_INDEX "X-com-parrot-resource-index"

/* Principal point of the camera */
#define VMETA_STRM_SDP_KEY_PRINCIPAL_POINT "X-com-parrot-principal-point"


/* Recording metadata include method */
enum vmeta_record_type {
	/* MP4 file 'meta' item */
	VMETA_REC_META = 0,

	/* MP4 file 'udta' item */
	VMETA_REC_UDTA,

	/* Android-compatible MP4 file '.xyz' item */
	VMETA_REC_XYZ,
};


/**
 * Keys used on recording for MP4 file 'meta' items
 * NB: when both 'udta' and 'meta' items are available for the same
 * metadata type, use the 'meta' item value
 */

/* Friendly name */
#define VMETA_REC_META_KEY_FRIENDLY_NAME "com.apple.quicktime.artist"

/* Video title */
#define VMETA_REC_META_KEY_TITLE "com.apple.quicktime.title"

/* Video comment */
#define VMETA_REC_META_KEY_COMMENT "com.apple.quicktime.comment"

/* Video copyright */
#define VMETA_REC_META_KEY_COPYRIGHT "com.apple.quicktime.copyright"

/* Media date */
#define VMETA_REC_META_KEY_MEDIA_DATE "com.apple.quicktime.creationdate"

/* Takeoff location */
#define VMETA_REC_META_KEY_TAKEOFF_LOC "com.parrot.takeoff.loc"

/* Location */
#define VMETA_REC_META_KEY_LOCATION "com.apple.quicktime.location.ISO6709"

/* Product maker */
#define VMETA_REC_META_KEY_MAKER "com.apple.quicktime.make"

/* Product model */
#define VMETA_REC_META_KEY_MODEL "com.apple.quicktime.model"

/* Software version */
#define VMETA_REC_META_KEY_SOFTWARE_VERSION "com.apple.quicktime.software"

/* Product serial number */
#define VMETA_REC_META_KEY_SERIAL_NUMBER "com.parrot.serial"

/* Model ID */
#define VMETA_REC_META_KEY_MODEL_ID "com.parrot.model.id"

/* Software build ID */
#define VMETA_REC_META_KEY_BUILD_ID "com.parrot.build.id"

/* Run date */
#define VMETA_REC_META_KEY_RUN_DATE "com.parrot.run.date"

/* Run UUID */
#define VMETA_REC_META_KEY_RUN_ID "com.parrot.run.id"

/* Boot date */
#define VMETA_REC_META_KEY_BOOT_DATE "com.parrot.boot.date"

/* Boot UUID */
#define VMETA_REC_META_KEY_BOOT_ID "com.parrot.boot.id"

/* Flight date */
#define VMETA_REC_META_KEY_FLIGHT_DATE "com.parrot.flight.date"

/* Flight UUID */
#define VMETA_REC_META_KEY_FLIGHT_ID "com.parrot.flight.id"

/* Custom ID */
#define VMETA_REC_META_KEY_CUSTOM_ID "com.parrot.custom.id"

/* Picture horizontal field of view
 * (deprecated; use VMETA_REC_META_KEY_PICTURE_FOV) */
#define VMETA_REC_META_KEY_PICTURE_HORZ_FOV "com.parrot.picture.hfov"

/* Picture vertical field of view
 * (deprecated; use VMETA_REC_META_KEY_PICTURE_FOV) */
#define VMETA_REC_META_KEY_PICTURE_VERT_FOV "com.parrot.picture.vfov"

/* Picture field of view */
#define VMETA_REC_META_KEY_PICTURE_FOV "com.parrot.picture.fov"

/* Thermal metadata version */
#define VMETA_REC_META_KEY_THERMAL_METAVERSION "com.parrot.thermal.metaversion"

/* Thermal camera serial number */
#define VMETA_REC_META_KEY_THERMAL_CAMSERIAL "com.parrot.thermal.camserial"

/* Thermal camera alignment parameters */
#define VMETA_REC_META_KEY_THERMAL_ALIGNMENT "com.parrot.thermal.alignment"

/* Thermal camera temperature conversion parameters (low gain) */
#define VMETA_REC_META_KEY_THERMAL_CONV_LOW "com.parrot.thermal.conv.low"

/* Thermal camera temperature conversion parameters (high gain) */
#define VMETA_REC_META_KEY_THERMAL_CONV_HIGH "com.parrot.thermal.conv.high"

/* Thermal camera scale factor */
#define VMETA_REC_META_KEY_THERMAL_SCALE_FACTOR "com.parrot.thermal.scalefactor"

/* Principal point of the camera */
#define VMETA_REC_META_KEY_PRINCIPAL_POINT "com.parrot.principal.point"

/* Camera type */
#define VMETA_REC_META_KEY_CAMERA_TYPE "com.parrot.camera.type"

/* Camera subtype */
#define VMETA_REC_META_KEY_CAMERA_SUBTYPE "com.parrot.camera.subtype"

/* Camera spectrum */
#define VMETA_REC_META_KEY_CAMERA_SPECTRUM "com.parrot.camera.spectrum"

/* Camera serial number */
#define VMETA_REC_META_KEY_CAMERA_SERIAL_NUMBER "com.parrot.camera.serial"

/* Camera model type */
#define VMETA_REC_META_KEY_CAMERA_MODEL_TYPE "com.parrot.camera.model.type"

/* Perspective distortion parameters */
#define VMETA_REC_META_KEY_PERSPECTIVE_DISTORTION                              \
	"com.parrot.perspective.distortion"

/* Fisheye affine matrix coefficients */
#define VMETA_REC_META_KEY_FISHEYE_AFFINE_MATRIX                               \
	"com.parrot.fisheye.affine.matrix"

/* Fisheye polynomial coefficients */
#define VMETA_REC_META_KEY_FISHEYE_POLYNOMIAL "com.parrot.fisheye.polynomial"

/* Header-footer overlay */
#define VMETA_REC_META_KEY_HEADER_FOOTER "com.parrot.overlay.header.footer"

/* Video mode */
#define VMETA_REC_META_KEY_VIDEO_MODE "com.parrot.video.mode"

/* Video stop reason */
#define VMETA_REC_META_KEY_VIDEO_STOP_REASON "com.parrot.video.stop.reason"

/* Image dynamic range */
#define VMETA_REC_META_KEY_DYNAMIC_RANGE "com.parrot.dynamic.range"

/* Image tone mapping */
#define VMETA_REC_META_KEY_TONE_MAPPING "com.parrot.tone.mapping"

/* Capture timestamp of the first frame */
#define VMETA_REC_META_KEY_FIRST_FRAME_CAPTURE_TS                              \
	"com.parrot.first.frame.capture.ts"

/* Sample index of the first frame */
#define VMETA_REC_META_KEY_FIRST_FRAME_SAMPLE_INDEX                            \
	"com.parrot.first.frame.sample.index"

/* Unique media identifier of the video */
#define VMETA_REC_META_KEY_MEDIA_ID "com.parrot.media.id"

/* Resource index of the video in the media ID  */
#define VMETA_REC_META_KEY_RESOURCE_INDEX "com.parrot.resource.index"


/**
 * Keys used on recording for MP4 file 'udta' items
 */

/* Friendly name */
#define VMETA_REC_UDTA_KEY_FRIENDLY_NAME "\251ART"

/* Video title */
#define VMETA_REC_UDTA_KEY_TITLE "\251nam"

/* Video comment */
#define VMETA_REC_UDTA_KEY_COMMENT "\251cmt"

/* Video copyright */
#define VMETA_REC_UDTA_KEY_COPYRIGHT "\251cpy"

/* Media date */
#define VMETA_REC_UDTA_KEY_MEDIA_DATE "\251day"

/* Location */
#define VMETA_REC_UDTA_KEY_LOCATION "\251xyz"

/* Product maker */
#define VMETA_REC_UDTA_KEY_MAKER "\251mak"

/* Product model */
#define VMETA_REC_UDTA_KEY_MODEL "\251mod"

/* Software version */
#define VMETA_REC_UDTA_KEY_SOFTWARE_VERSION "\251swr"

/* Product serial number */
#define VMETA_REC_UDTA_KEY_SERIAL_NUMBER "\251too"


/**
 * Keys used on recording for a JSON comment string in a MP4 file 'udta' item
 * NB: this method is deprecated; use 'meta' items for metadata types that are
 * not available as 'udta' items
 */

/* Software version */
#define VMETA_REC_UDTA_JSON_KEY_SOFTWARE_VERSION "software_version"

/* Run UUID */
#define VMETA_REC_UDTA_JSON_KEY_RUN_ID "run_uuid"

/* Takeoff location */
#define VMETA_REC_UDTA_JSON_KEY_TAKEOFF_LOC "takeoff_position"

/* Location */
#define VMETA_REC_UDTA_JSON_KEY_LOCATION "location"

/* Media date */
#define VMETA_REC_UDTA_JSON_KEY_MEDIA_DATE "media_date"

/* Picture horizontal field of view */
#define VMETA_REC_UDTA_JSON_KEY_PICTURE_HORZ_FOV "picture_hfov"

/* Picture vertical field of view */
#define VMETA_REC_UDTA_JSON_KEY_PICTURE_VERT_FOV "picture_vfov"


/* Camera model parameters */
struct vmeta_camera_model {
	/* Camera model type */
	enum vmeta_camera_model_type type;

	union {
		/* Perspective camera parameters (only valid if
		 * camera_model_type is VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE */
		struct {
			/* Perspective distortion parameters */
			struct {
				float r1;
				float r2;
				float r3;
				float t1;
				float t2;
			} distortion;
		} perspective;

		/* Fisheye camera parameters (only valid if camera_model_type
		 * is VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE */
		struct {
			/* Fisheye affine matrix coefficients */
			struct {
				float c;
				float d;
				float e;
				float f;
			} affine_matrix;

			/* Fisheye polynomial coefficients
			 * (note: p0 = 0 and p1 = 1) */
			struct {
				float p2;
				float p3;
				float p4;
			} polynomial;
		} fisheye;
	};
};


/* Overlay parameters */
struct vmeta_overlay {
	/* Overlay type */
	enum vmeta_overlay_type type;

	union {
		/* Header-Footer overlay parameters (only valid if
		 * vmeta_overlay_type is VMETA_OVERLAY_TYPE_HEADER_FOOTER */
		struct {
			/* Header height in percentage of the image */
			float header_height;
			/* Footer height in percentage of the image */
			float footer_height;
		} header_footer;
	};
};


/* Thermal camera alignment parameters */
struct vmeta_thermal_alignment {
	/* Visible camera orientation relative to the thermal camera,
	 * order is yaw, pitch, roll */
	struct vmeta_euler rotation;

	/* Thermal camera alignment parameters validity flag
	 * (1 if the fields in the structure are valid, 0 otherwise) */
	uint8_t valid;
};


/* Thermal camera temperature conversion parameters */
struct vmeta_thermal_conversion {
	/* Camera temperature conversion responsivity parameter */
	float r;

	/* Camera temperature conversion B parameter */
	float b;

	/* Camera temperature conversion fit parameter */
	float f;

	/* Camera temperature conversion offset parameter */
	float o;

	/* Window transmission (0.0 .. 1.0) */
	float tau_win;

	/* Window temperature (in Celsius degrees) */
	float t_win;

	/* Scene background temperature (in Celsius degrees) */
	float t_bg;

	/* Scene emissivity (0.0 .. 1.0) */
	float emissivity;

	/* Thermal camera temperature conversion parameters validity flag
	 * (1 if the fields in the structure are valid, 0 otherwise) */
	uint8_t valid;
};


/* Thermal camera metadata */
struct vmeta_thermal {
	/* Thermal metadata version */
	int metaversion;

	/* Camera serial number */
	char camserial[50];

	/* Camera alignment parameters */
	struct vmeta_thermal_alignment alignment;

	/* Camera temperature conversion parameters (low gain) */
	struct vmeta_thermal_conversion conv_low;

	/* Camera temperature conversion parameters (high gain) */
	struct vmeta_thermal_conversion conv_high;

	/* Camera scale factor */
	double scale_factor;
};


struct vmeta_principal_point {
	/* Principal point horizontal and vertical coordinates, with x and y
	 * both normalized to the picture width ([0..1]) */
	struct vmeta_xy position;

	/* Principal point validity flag
	 * (1 if the fields in the structure are valid, 0 otherwise) */
	uint8_t valid;
};


#include "video-metadata/vmeta_session_proto.h"


/* Session metadata used on recording and streaming;
 * for streaming the same structure is used both on the drone and the
 * controller side, but only some of the fields are used on the
 * controller side */
/* clang-format off */
struct vmeta_session {
	/* Friendly name (generally the same as the wifi SSID) */
	char friendly_name[40];

	/* Product maker (brand name, eg. "Parrot") */
	char maker[40];

	/* Product model (commercial name, eg. "Bebop 2") */
	char model[40];

	/* Model ID (ARSDK 16-bit model ID in hex ASCII, eg. "090c")
	 * (unused on the controller side) */
	char model_id[5];

	/* Product serial number (18 chars string for Parrot products) */
	char serial_number[32];

	/* Software version (usually "<software_name> A.B.C"
	 * with A=major, B=minor, C=build) */
	char software_version[20];

	/* Software build ID (internal unique build identifier) */
	char build_id[80];

	/* Video title (unused on the controller side) */
	char title[80];

	/* Video comment (unused on the controller side) */
	char comment[100];

	/* Video copyright (unused on the controller side) */
	char copyright[80];

	/* Media date and time in seconds since the Epoch
	 * (record only, unused on live streaming) */
	uint64_t media_date;

	/* Media date GMT offset in seconds east (eg. GMT-6 is -21600)
	 * (record only, unused on live streaming) */
	long media_date_gmtoff;

	/* Run date and time in seconds since the Epoch
	 * (unused on the controller side) */
	uint64_t run_date;

	/* Run date GMT offset in seconds east (eg. GMT-6 is -21600)
	 * (unused on the controller side) */
	long run_date_gmtoff;

	/* Run UUID (32-chars hex string representing a 128bits value)
	 * (unused on the controller side) */
	char run_id[33];

	/* Boot date and time in seconds since the Epoch
	 * (unused on the controller side) */
	uint64_t boot_date;

	/* Boot date GMT offset in seconds east (eg. GMT-6 is -21600)
	 * (unused on the controller side) */
	long boot_date_gmtoff;

	/* Boot UUID (32-chars hex string representing a 128bits value)
	 * (unused on the controller side) */
	char boot_id[33];

	/* Flight date and time in seconds since the Epoch
	 * (unused on the controller side) */
	uint64_t flight_date;

	/* Flight date GMT offset in seconds east (eg. GMT-6 is -21600)
	 * (unused on the controller side) */
	long flight_date_gmtoff;

	/* Flight UUID (32-chars hex string representing a 128bits value)
	 * (unused on the controller side) */
	char flight_id[33];

	/* Application-defined custom ID (unused on the controller side) */
	char custom_id[80];

	/* Takeoff location (unused on the controller side) */
	struct vmeta_location takeoff_loc;

	/* Location (unused on the controller side) */
	struct vmeta_location location;

	/* Picture field of view in degrees (unused on the controller side) */
	struct vmeta_fov picture_fov;

	/* Thermal camera metadata (unused on the controller side) */
	struct vmeta_thermal thermal;

	/* Thermal camera metadata validity flag (1 if the thermal structure
	 * contents are valid, 0 otherwise) */
	uint32_t has_thermal:1;

	/* Default media flag (only used in SDP) */
	uint32_t default_media:1;

	/* Camera type */
	enum vmeta_camera_type camera_type;

	/* Camera subtype */
	enum vmeta_camera_subtype camera_subtype;

	/* Camera spectrum */
	enum vmeta_camera_spectrum camera_spectrum;

	/* Camera serial number (18 chars string for Parrot products
	 * or [cam1_name]:[PI_1];[cam2_name]:[PI_2] pattern
	 * in case of multiple cameras) */
	char camera_serial_number[VMETA_SESSION_CAMERA_SERIAL_PATTERN_MAX_LEN];

	/* Camera model */
	struct vmeta_camera_model camera_model;

	/* Overlay */
	struct vmeta_overlay overlay;

	/* Camera principal point */
	struct vmeta_principal_point principal_point;

	/* Video mode */
	enum vmeta_video_mode video_mode;

	/* Video stop reason */
	enum vmeta_video_stop_reason video_stop_reason;

	/* Image dynamic range */
	enum vmeta_dynamic_range dynamic_range;

	/* Image tone mapping */
	enum vmeta_tone_mapping tone_mapping;

	/* Capture timestamp of the first frame (optional, can be 0; can be used
	 * for example on raw video to compute the frame capture timestamps) */
	uint64_t first_frame_capture_ts;

	/* Sample index of the first frame (optional, can be 0; can be used
	 * for example on raw video to compute the frame capture timestamps) */
	uint32_t first_frame_sample_index;

	/* Unique media identifier of the video. A media is a single or a group
	 * of resources [1:N] (optional, can be 0) */
	uint32_t media_id;

	/* Resource index of the video in the media ID. A resource is a media
	 * file named using a pattern and a supported extension
	 * (optional, can be 0) */
	uint32_t resource_index;
};
/* clang-format on */


/**
 * Write a date string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param date: date in seconds since the Epoch
 * @param gmtoff: GMT offset in seconds increasing eastbound
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t
vmeta_session_date_write(char *str, size_t len, uint64_t date, long gmtoff);


/**
 * Read a date string.
 * The date is returned through the date and gmtoff parameters. date is
 * a value in seconds since the Epoch (1970-01-01 00:00:00 +0000 UTC);
 * gmtoff is the GMT offset in seconds east (eg. GMT-6 is -21600).
 * @param str: pointer to the string to read
 * @param date: pointer to the date in seconds since the Epoch (output)
 * @param gmtoff: pointer to the GMT offset in seconds increasing
 *                eastbound (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_date_read(const char *str, uint64_t *date, long *gmtoff);


/**
 * Write a location string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * The supported formats are those in enum vmeta_session_location_format.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param format: location format
 * @param loc: pointer to a location structure
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t vmeta_session_location_write(char *str,
				     size_t len,
				     enum vmeta_session_location_format format,
				     const struct vmeta_location *loc);


/**
 * Read a location string.
 * The location is returned in the loc structure parameter. The supported
 * formats are those in enum vmeta_session_location_format.
 * @param str: pointer to the string to read
 * @param loc: pointer to the location structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_location_read(const char *str, struct vmeta_location *loc);


/**
 * Write a field of view string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param fov: pointer to a field of view structure
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t
vmeta_session_fov_write(char *str, size_t len, const struct vmeta_fov *fov);


/**
 * Read a field of view string.
 * The field of view is returned in the fov structure parameter.
 * @param str: pointer to the string to read
 * @param fov: pointer to the field of view structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_fov_read(const char *str, struct vmeta_fov *fov);


/**
 * Write a perspective distortion parameters string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param r1: radial distortion parameter R1
 * @param r2: radial distortion parameter R2
 * @param r3: radial distortion parameter R3
 * @param t1: tangential distortion parameter T1
 * @param t2: tangential distortion parameter T2
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t vmeta_session_perspective_distortion_write(char *str,
						   size_t len,
						   float r1,
						   float r2,
						   float r3,
						   float t1,
						   float t2);


/**
 * Read a perspective distortion parameters string.
 * The perspective distortion parameters are returned through the 5 float
 * pointer parameters.
 * @param str: pointer to the string to read
 * @param r1: pointer to the radial distortion parameter R1 (output)
 * @param r2: pointer to the radial distortion parameter R2 (output)
 * @param r3: pointer to the radial distortion parameter R3 (output)
 * @param t1: pointer to the tangential distortion parameter T1 (output)
 * @param t2: pointer to the tangential distortion parameter T2 (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_perspective_distortion_read(const char *str,
					      float *r1,
					      float *r2,
					      float *r3,
					      float *t1,
					      float *t2);


/**
 * Write a fisheye affine matrix coefficients string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param c: fisheye affine matrix coefficient C
 * @param d: fisheye affine matrix coefficient D
 * @param e: fisheye affine matrix coefficient E
 * @param f: fisheye affine matrix coefficient F
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t vmeta_session_fisheye_affine_matrix_write(char *str,
						  size_t len,
						  float c,
						  float d,
						  float e,
						  float f);


/**
 * Read a fisheye affine matrix coefficients string.
 * The fisheye affine matrix coefficients are returned through the 4 float
 * pointer parameters.
 * @param str: pointer to the string to read
 * @param c: pointer to the fisheye affine matrix coefficient C (output)
 * @param d: pointer to the fisheye affine matrix coefficient D (output)
 * @param e: pointer to the fisheye affine matrix coefficient E (output)
 * @param f: pointer to the fisheye affine matrix coefficient F (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_fisheye_affine_matrix_read(const char *str,
					     float *c,
					     float *d,
					     float *e,
					     float *f);


/**
 * Write a fisheye polynomial coefficients string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param p2: fisheye polynomial coefficient p2
 * @param p3: fisheye polynomial coefficient p3
 * @param p4: fisheye polynomial coefficient p4
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t vmeta_session_fisheye_polynomial_write(char *str,
					       size_t len,
					       float p2,
					       float p3,
					       float p4);


/**
 * Read a fisheye polynomial coefficients string.
 * The fisheye polynomial coefficients are returned through the 3 float
 * pointer parameters.
 * @param str: pointer to the string to read
 * @param p2: pointer to the fisheye polynomial coefficient p2 (output)
 * @param p3: pointer to the fisheye polynomial coefficient p3 (output)
 * @param p4: pointer to the fisheye polynomial coefficient p4 (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_fisheye_polynomial_read(const char *str,
					  float *p2,
					  float *p3,
					  float *p4);


/**
 * Write a header-footer overlay string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param header_height: header height in percentage of the image
 * @param footer_height: footer height in percentage of the image
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API ssize_t
vmeta_session_overlay_header_footer_write(char *str,
					  size_t len,
					  float header_height,
					  float footer_height);


/**
 * Read a header-footer overlay string.
 * The header and footer height are returned through the 2 float
 * pointer parameters.
 * @param str: pointer to the string to read
 * @param header_height: header height in percentage of the image (output)
 * @param footer_height: footer height in percentage of the image (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API int vmeta_session_overlay_header_footer_read(const char *str,
						       float *header_height,
						       float *footer_height);


/**
 * Write a thermal camera alignment parameters string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param align: pointer to a thermal camera alignment parameters structure
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t vmeta_session_thermal_alignment_write(
	char *str,
	size_t len,
	const struct vmeta_thermal_alignment *align);


/**
 * Read a thermal camera alignment parameters string.
 * The alignment parameters are returned in the align structure parameter.
 * @param str: pointer to the string to read
 * @param loc: pointer to the thermal camera alignment parameters
 *             structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_thermal_alignment_read(const char *str,
					 struct vmeta_thermal_alignment *align);


/**
 * Write a thermal camera temperature conversion parameters string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param conv: pointer to a thermal camera temperature conversion parameters
 *             structure
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t vmeta_session_thermal_conversion_write(
	char *str,
	size_t len,
	const struct vmeta_thermal_conversion *conv);


/**
 * Read a thermal camera temperature conversion parameters string.
 * The conversion parameters are returned in the conv structure parameter.
 * @param str: pointer to the string to read
 * @param conv: pointer to the thermal camera temperature conversion parameters
 *             structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_thermal_conversion_read(
	const char *str,
	struct vmeta_thermal_conversion *conv);


/**
 * Write a thermal camera scale factor string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param value: thermal camera scale factor
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
VMETA_API
ssize_t
vmeta_session_thermal_scale_factor_write(char *str, size_t len, double value);


/**
 * Read a thermal camera scale factor string.
 * The scale factor is returned in the value parameter.
 * @param str: pointer to the string to read
 * @param value: pointer to the thermal camera scale factor (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_thermal_scale_factor_read(const char *str, double *value);


/**
 * Write a principal point string.
 * The str string must have been previously allocated.
 * The function writes up to len chars.
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @param principal_point: pointer to the principal point
 * @return the length of the string written on success,
 *         negative errno value in case of error
 */
ssize_t vmeta_session_principal_point_write(
	char *str,
	size_t len,
	const struct vmeta_principal_point *principal_point);


/**
 * Read a principal point string.
 * @param str: pointer to the string to read
 * @param principal_point: pointer to the principal_point (output)
 * @return 0 on success, negative errno value in case of error
 */
int vmeta_session_principal_point_read(
	const char *str,
	struct vmeta_principal_point *principal_point);


/**
 * RTCP SDES item writing callback function.
 * See the vmeta_session_streaming_sdes_write() function.
 * @param type: item type
 * @param value: item value
 * @param key: item key (only for PRIV SDES items, NULL otherwise)
 * @param userdata: user data pointer
 */
typedef void (*vmeta_session_streaming_sdes_write_cb_t)(
	enum vmeta_stream_sdes_type type,
	const char *value,
	const char *prefix,
	void *userdata);


/**
 * Write session metadata as RTCP SDES items.
 * The function is called for a whole session metadata structure and calls the
 * cb callback function for each RTCP SDES item that should be written.
 * For each call to the cb function, the RTCP SDES item type and value are
 * given, along with the prefix (only for PRIV SDES items). Both value and
 * prefix are null-terminated.
 * @param meta: pointer to the session metadata structure
 * @param cb: SDES item writing callback function
 * @param userdata: SDES item writing callback function user data pointer
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_streaming_sdes_write(
	const struct vmeta_session *meta,
	vmeta_session_streaming_sdes_write_cb_t cb,
	void *userdata);


/**
 * Read session metadata from RTCP SDES items.
 * The function should be called for each RTCP SDES item with its type and
 * value, along with the prefix (only for PRIV SDES items). Both value and
 * prefix must be null-terminated. The meta structure is filled with values
 * corresponding to the input SDES items when recognized.
 * @param type: SDES item type
 * @param value: pointer to a null-terminated SDES item value string
 * @param prefix: pointer to a null-terminated SDES PRIV item prefix string
 *                (NULL if the item is not PRIV)
 * @param meta: pointer to the session metadata structure to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_streaming_sdes_read(enum vmeta_stream_sdes_type type,
				      const char *value,
				      const char *prefix,
				      struct vmeta_session *meta);


/**
 * SDP item writing callback function.
 * See the vmeta_session_streaming_sdp_write() function.
 * @param type: item type
 * @param value: item value
 * @param key: item key (only for 'a=X-*:' attribute extensions)
 * @param userdata: user data pointer
 */
typedef void (*vmeta_session_streaming_sdp_write_cb_t)(
	enum vmeta_stream_sdp_type type,
	const char *value,
	const char *key,
	void *userdata);


/**
 * Write session metadata as SDP items.
 * The function is called for a whole session metadata structure and calls the
 * cb callback function for each SDP item that should be written.
 * For each call to the cb function, the SDP item type and value are
 * given, along with the key (only for 'a=X-*:' attribute extensions).
 * Both value and key are null-terminated. If media_level is 0, session-level
 * SDP items are written, otherwise media-level items are written.
 * @param meta: pointer to the session metadata structure
 * @param media_level: session-level items if 0, media-level otherwise
 * @param cb: SDP item writing callback function
 * @param userdata: SDP item writing callback function user data pointer
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_streaming_sdp_write(const struct vmeta_session *meta,
				      int media_level,
				      vmeta_session_streaming_sdp_write_cb_t cb,
				      void *userdata);


/**
 * Read session metadata from SDP items.
 * The function should be called for each SDP 's=', 'i=', 'a=tool:' and
 * 'a=X-*:' item with its type and value, along with the key (only for
 * 'a=X-*:' attribute extensions). Other SDP items do not apply to enum
 * vmeta_stream_sdp_type and must not be passed to this function.
 * Both value and key must be null-terminated. The value is the string without
 * the leading 's=', 'i=', 'a=tool:' or 'a=X-*:'. The key is the 'X-*' string
 * (only for 'a=X-*:' attribute extensions).
 * SDP items can be either session-level or media-level items (determined by
 * the type parameter).
 * The meta structure is filled with values corresponding to the input SDP
 * items when recognized.
 * @param type: SDP item type
 * @param value: pointer to a null-terminated SDP item value string
 * @param prefix: pointer to a null-terminated SDP attribute extension key
 *                string (NULL if the item is not an attribute extension)
 * @param meta: pointer to the session metadata structure to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_streaming_sdp_read(enum vmeta_stream_sdp_type type,
				     const char *value,
				     const char *key,
				     struct vmeta_session *meta);


/**
 * 'meta' or 'udta' item writing callback function.
 * See the vmeta_session_recording_write() function.
 * @param type: item type
 * @param key: item key
 * @param value: item value
 * @param userdata: user data pointer
 */
typedef void (*vmeta_session_recording_write_cb_t)(enum vmeta_record_type type,
						   const char *key,
						   const char *value,
						   void *userdata);


/**
 * Write session metadata as MP4 file 'meta' or 'udta' items.
 * The function is called for a whole session metadata structure and calls the
 * cb callback function for each 'meta' or 'udta' item that should be written.
 * For each call to the cb function, the item type, key and value are given.
 * Both key and value are null-terminated.
 * @param meta: pointer to the session metadata structure
 * @param cb: 'meta' or 'udta' item writing callback function
 * @param userdata: 'meta' or 'udta' item writing callback function user data
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_recording_write(const struct vmeta_session *meta,
				  vmeta_session_recording_write_cb_t cb,
				  void *userdata);


/**
 * Read session metadata from MP4 file 'meta' or 'udta' items.
 * The function should be called for each key/value pair. Both key and value
 * must be null-terminated. The meta structure is filled with values
 * corresponding to the input key/value pairs when recognized.
 * @param key: pointer to a null-terminated key string
 * @param value: pointer to a null-terminated value string
 * @param meta: pointer to the session metadata structure to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_recording_read(const char *key,
				 const char *value,
				 struct vmeta_session *meta);


/**
 * Write session metadata to a JSON object.
 * The jobj JSON object must have been previously allocated.
 * @param meta: pointer to a session metadata structure
 * @param jobj: pointer to the JSON object to write to (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_to_json(const struct vmeta_session *meta,
			  struct json_object *jobj);


/**
 * Write session metadata as a string.
 * The str string must have been previously allocated. The function writes
 * up to maxlen characters.
 * @param meta: pointer to a session metadata structure
 * @param str: pointer to the string to write to (output)
 * @param maxlen: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_to_str(const struct vmeta_session *meta,
			 char *str,
			 size_t maxlen);


/**
 * Merge common session metadata fields from an array of session metadata into a
 * single one. Erase the duplicated fields from the array.
 * @param meta_list: session metadata array to merge
 * @param meta_list_count: size of the meta_list array
 * @param merged_meta: pointer to the merged session metadata (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_merge_metadata(struct vmeta_session **meta_list,
				 size_t meta_list_count,
				 struct vmeta_session *merged_meta);


/**
 * Compare two session metadata structures.
 * @param meta_1: vmeta_session to compare with meta_2
 * @param meta_2: vmeta_session to compare with meta_1
 * @return 1 if the two are equal, 0 otherwise
 */
VMETA_API
int vmeta_session_cmp(const struct vmeta_session *meta1,
		      const struct vmeta_session *meta2);


/**
 * Check if a session metadata structure is valid.
 * Valid means the friendly name and the model field are not empty and the maker
 * is 'Parrot'.
 * @param meta: vmeta_session to check
 * @return 1 if the vmeta_session is valid, 0 otherwise
 */
VMETA_API
int vmeta_session_is_valid(const struct vmeta_session *meta);


#endif /* !_VMETA_SESSION_H_ */
