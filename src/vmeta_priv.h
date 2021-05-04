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

#ifndef _VMETA_PRIV_H_
#define _VMETA_PRIV_H_

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#ifdef BUILD_JSON
#	include <json-c/json.h>
#endif /* BUILD_JSON */

#define ULOG_TAG vmeta
#include <ulog.h>

#include "video-metadata/vmeta.h"

#include "vmeta_csv.h"
#include "vmeta_json.h"


#define VMETA_STR_PRINT(_str, _len, _max, _fmt, ...)                           \
	(_len += snprintf(_str, _max, _fmt, ##__VA_ARGS__))

#define VMETA_STR_SPACE(_str, _len, _max) (_len += snprintf(_str, _max, " "))

#define VMETA_STR_LF(_str, _len, _max) (_len += snprintf(_str, _max, "\n"))


static inline void vmeta_location_adjust_read(const struct vmeta_location *in,
					      struct vmeta_location *out)
{
	*out = *in;
	out->valid = in->latitude != 500.0 && in->longitude != 500.0;
	if (!out->valid) {
		out->latitude = NAN;
		out->longitude = NAN;
		out->altitude = NAN;
		out->sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
	}
}


static inline void vmeta_location_adjust_write(const struct vmeta_location *in,
					       struct vmeta_location *out)
{
	*out = *in;
	if (!in->valid) {
		out->latitude = 500.0;
		out->longitude = 500.0;
		out->altitude = 500.0;
		out->sv_count = 0;
	}
}


static inline int vmeta_buffer_read(struct vmeta_buffer *buf, void *p, size_t n)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(p == NULL, EINVAL);

	/* Make sure there is enough room in data buffer */
	ULOG_ERRNO_RETURN_ERR_IF(buf->pos + n > buf->len, ENOBUFS);

	/* Copy data */
	memcpy(p, buf->cdata + buf->pos, n);
	buf->pos += n;
	return 0;
}


static inline int
vmeta_buffer_write(struct vmeta_buffer *buf, const void *p, size_t n)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(p == NULL, EINVAL);

	/* Make sure there is enough room in data buffer */
	ULOG_ERRNO_RETURN_ERR_IF(buf->pos + n > buf->len, ENOBUFS);

	/* Copy data, adjust used length */
	memcpy(buf->data + buf->pos, p, n);
	buf->pos += n;
	return 0;
}


static inline int vmeta_write_u8(struct vmeta_buffer *buf, uint8_t v)
{
	return vmeta_buffer_write(buf, &v, sizeof(v));
}


static inline int vmeta_write_i8(struct vmeta_buffer *buf, int8_t v)
{
	return vmeta_buffer_write(buf, &v, sizeof(v));
}


static inline int vmeta_write_u16(struct vmeta_buffer *buf, uint16_t v)
{
	v = htons(v);
	return vmeta_buffer_write(buf, &v, sizeof(v));
}


static inline int vmeta_write_i16(struct vmeta_buffer *buf, int16_t v)
{
	v = htons(v);
	return vmeta_buffer_write(buf, &v, sizeof(v));
}


static inline int vmeta_write_u32(struct vmeta_buffer *buf, uint32_t v)
{
	v = htonl(v);
	return vmeta_buffer_write(buf, &v, sizeof(v));
}


static inline int vmeta_write_i32(struct vmeta_buffer *buf, int32_t v)
{
	v = htonl(v);
	return vmeta_buffer_write(buf, &v, sizeof(v));
}


static inline int vmeta_write_u64(struct vmeta_buffer *buf, uint64_t v)
{
	uint32_t _v[2];
	_v[0] = htonl((v >> 32) & 0xffffffff);
	_v[1] = htonl(v & 0xffffffff);
	return vmeta_buffer_write(buf, _v, sizeof(_v));
}


static inline int vmeta_write_i64(struct vmeta_buffer *buf, int64_t v)
{
	uint32_t _v[2];
	_v[0] = htonl((v >> 32) & 0xffffffff);
	_v[1] = htonl(v & 0xffffffff);
	return vmeta_buffer_write(buf, _v, sizeof(_v));
}


static inline int
vmeta_write_f32_u16(struct vmeta_buffer *buf, float v, uint32_t shift)
{
	uint16_t u16 = (uint16_t)(v * (1 << shift));
	return vmeta_write_u16(buf, u16);
}


static inline int
vmeta_write_f32_i16(struct vmeta_buffer *buf, float v, uint32_t shift)
{
	int16_t i16 = (int16_t)(v * (1 << shift));
	return vmeta_write_i16(buf, i16);
}


static inline int
vmeta_write_f64_u32(struct vmeta_buffer *buf, double v, uint32_t shift)
{
	uint32_t u32 = (uint32_t)(v * (1 << shift));
	return vmeta_write_u32(buf, u32);
}


static inline int
vmeta_write_f64_i32(struct vmeta_buffer *buf, double v, uint32_t shift)
{
	int32_t i32 = (int32_t)(v * (1 << shift));
	return vmeta_write_i32(buf, i32);
}


static inline int vmeta_read_u8(struct vmeta_buffer *buf, uint8_t *v)
{
	return vmeta_buffer_read(buf, v, sizeof(*v));
}


static inline int vmeta_read_i8(struct vmeta_buffer *buf, int8_t *v)
{
	return vmeta_buffer_read(buf, v, sizeof(*v));
}


static inline int vmeta_read_u16(struct vmeta_buffer *buf, uint16_t *v)
{
	int res = 0;
	res = vmeta_buffer_read(buf, v, sizeof(*v));
	if (res == 0)
		*v = ntohs(*v);
	return res;
}


static inline int vmeta_read_i16(struct vmeta_buffer *buf, int16_t *v)
{
	int res = 0;
	res = vmeta_buffer_read(buf, v, sizeof(*v));
	if (res == 0)
		*v = ntohs(*v);
	return res;
}


static inline int vmeta_read_u32(struct vmeta_buffer *buf, uint32_t *v)
{
	int res = 0;
	res = vmeta_buffer_read(buf, v, sizeof(*v));
	if (res == 0)
		*v = ntohl(*v);
	return res;
}


static inline int vmeta_read_i32(struct vmeta_buffer *buf, int32_t *v)
{
	int res = 0;
	res = vmeta_buffer_read(buf, v, sizeof(*v));
	if (res == 0)
		*v = ntohl(*v);
	return res;
}


static inline int vmeta_read_u64(struct vmeta_buffer *buf, uint64_t *v)
{
	int res = 0;
	uint32_t _v[2];
	res = vmeta_buffer_read(buf, _v, sizeof(_v));
	if (res == 0)
		*v = ((uint64_t)ntohl(_v[0]) << 32) | ntohl(_v[1]);
	return res;
}


static inline int vmeta_read_i64(struct vmeta_buffer *buf, int64_t *v)
{
	int res = 0;
	uint32_t _v[2];
	res = vmeta_buffer_read(buf, _v, sizeof(_v));
	if (res == 0)
		*v = ((int64_t)ntohl(_v[0]) << 32) | ntohl(_v[1]);
	return res;
}


static inline int
vmeta_read_f32_u16(struct vmeta_buffer *buf, float *v, uint32_t shift)
{
	int res = 0;
	uint16_t u16 = 0;
	res = vmeta_read_u16(buf, &u16);
	if (res == 0)
		*v = (float)u16 / (1 << shift);
	return res;
}


static inline int
vmeta_read_f32_i16(struct vmeta_buffer *buf, float *v, uint32_t shift)
{
	int res = 0;
	int16_t i16 = 0;
	res = vmeta_read_i16(buf, &i16);
	if (res == 0)
		*v = (float)i16 / (1 << shift);
	return res;
}


static inline int
vmeta_read_f64_i32(struct vmeta_buffer *buf, double *v, uint32_t shift)
{
	int res = 0;
	int32_t i32 = 0;
	res = vmeta_read_i32(buf, &i32);
	if (res == 0)
		*v = (double)i32 / (1 << shift);
	return res;
}


static inline int
vmeta_read_f64_u32(struct vmeta_buffer *buf, double *v, uint32_t shift)
{
	int res = 0;
	uint32_t u32 = 0;
	res = vmeta_read_u32(buf, &u32);
	if (res == 0)
		*v = (double)u32 / (1 << shift);
	return res;
}


int vmeta_frame_ext_timestamp_write(
	struct vmeta_buffer *buf,
	const struct vmeta_frame_ext_timestamp *meta);


int vmeta_frame_ext_timestamp_read(struct vmeta_buffer *buf,
				   struct vmeta_frame_ext_timestamp *meta);


int vmeta_frame_ext_followme_write(struct vmeta_buffer *buf,
				   const struct vmeta_frame_ext_followme *meta);


int vmeta_frame_ext_followme_read(struct vmeta_buffer *buf,
				  struct vmeta_frame_ext_followme *meta);


int vmeta_frame_ext_automation_write(
	struct vmeta_buffer *buf,
	const struct vmeta_frame_ext_automation *meta);


int vmeta_frame_ext_automation_read(struct vmeta_buffer *buf,
				    struct vmeta_frame_ext_automation *meta);


int vmeta_frame_ext_thermal_write(struct vmeta_buffer *buf,
				  const struct vmeta_frame_ext_thermal *meta);


int vmeta_frame_ext_thermal_read(struct vmeta_buffer *buf,
				 struct vmeta_frame_ext_thermal *meta);


#endif /* !_VMETA_PRIV_H_ */
