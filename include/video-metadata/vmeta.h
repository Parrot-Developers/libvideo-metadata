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


/* Forward declaration */
struct json_object;


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


#define VMETA_LOCATION_INVALID_SV_COUNT ((uint8_t)-1)


/* Geographic coordinates location */
struct vmeta_location {
	/* Latitude (deg) */
	double latitude;

	/* Longitude (deg) */
	double longitude;

	/* Altitude ASL (m) */
	double altitude;

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


#include "video-metadata/vmeta_frame.h"
#include "video-metadata/vmeta_session.h"


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VMETA_H_ */
