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

/* clang-format off */
#define CHECK(_x) do { if ((res = (_x)) < 0) goto out; } while (0)
/* clang-format on */


const char *vmeta_flying_state_str(enum vmeta_flying_state val)
{
	switch (val) {
	case VMETA_FLYING_STATE_LANDED:
		return "LANDED";
	case VMETA_FLYING_STATE_TAKINGOFF:
		return "TAKINGOFF";
	case VMETA_FLYING_STATE_HOVERING:
		return "HOVERING";
	case VMETA_FLYING_STATE_FLYING:
		return "FLYING";
	case VMETA_FLYING_STATE_LANDING:
		return "LANDING";
	case VMETA_FLYING_STATE_EMERGENCY:
		return "EMERGENCY";
	case VMETA_FLYING_STATE_USER_TAKEOFF:
		return "USER_TAKEOFF";
	case VMETA_FLYING_STATE_MOTOR_RAMPING:
		return "MOTOR_RAMPING";
	case VMETA_FLYING_STATE_EMERGENCY_LANDING:
		return "EMERGENCY_LANDING";
	default:
		return "UNKNOWN";
	}
}


const char *vmeta_piloting_mode_str(enum vmeta_piloting_mode val)
{
	switch (val) {
	case VMETA_PILOTING_MODE_MANUAL:
		return "MANUAL";
	case VMETA_PILOTING_MODE_RETURN_HOME:
		return "RETURN_HOME";
	case VMETA_PILOTING_MODE_FLIGHT_PLAN:
		return "FLIGHT_PLAN";
	case VMETA_PILOTING_MODE_TRACKING:
		return "TRACKING";
	case VMETA_PILOTING_MODE_MAGIC_CARPET:
		return "MAGIC_CARPET";
	case VMETA_PILOTING_MODE_MOVE_TO:
		return "MOVE_TO";
	case VMETA_PILOTING_MODE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}


const char *vmeta_followme_anim_str(enum vmeta_followme_anim val)
{
	switch (val) {
	case VMETA_FOLLOWME_ANIM_NONE:
		return "NONE";
	case VMETA_FOLLOWME_ANIM_ORBIT:
		return "ORBIT";
	case VMETA_FOLLOWME_ANIM_BOOMERANG:
		return "BOOMERANG";
	case VMETA_FOLLOWME_ANIM_PARABOLA:
		return "PARABOLA";
	case VMETA_FOLLOWME_ANIM_ZENITH:
		return "ZENITH";
	default:
		return "UNKNOWN";
	}
}


const char *vmeta_automation_anim_str(enum vmeta_automation_anim val)
{
	switch (val) {
	case VMETA_AUTOMATION_ANIM_NONE:
		return "NONE";
	case VMETA_AUTOMATION_ANIM_ORBIT:
		return "ORBIT";
	case VMETA_AUTOMATION_ANIM_BOOMERANG:
		return "BOOMERANG";
	case VMETA_AUTOMATION_ANIM_PARABOLA:
		return "PARABOLA";
	case VMETA_AUTOMATION_ANIM_DOLLY_SLIDE:
		return "DOLLY_SLIDE";
	case VMETA_AUTOMATION_ANIM_DOLLY_ZOOM:
		return "DOLLY_ZOOM";
	case VMETA_AUTOMATION_ANIM_REVEAL_VERT:
		return "REVEAL_VERT";
	case VMETA_AUTOMATION_ANIM_REVEAL_HORZ:
		return "REVEAL_HORZ";
	case VMETA_AUTOMATION_ANIM_PANORAMA_HORZ:
		return "PANORAMA_HORZ";
	case VMETA_AUTOMATION_ANIM_CANDLE:
		return "CANDLE";
	case VMETA_AUTOMATION_ANIM_FLIP_FRONT:
		return "FLIP_FRONT";
	case VMETA_AUTOMATION_ANIM_FLIP_BACK:
		return "FLIP_BACK";
	case VMETA_AUTOMATION_ANIM_FLIP_LEFT:
		return "FLIP_LEFT";
	case VMETA_AUTOMATION_ANIM_FLIP_RIGHT:
		return "FLIP_RIGHT";
	case VMETA_AUTOMATION_ANIM_TWISTUP:
		return "TWISTUP";
	case VMETA_AUTOMATION_ANIM_POSITION_TWISTUP:
		return "POSITION_TWISTUP";
	default:
		return "UNKNOWN";
	}
}


const char *vmeta_thermal_calib_state_str(enum vmeta_thermal_calib_state val)
{
	switch (val) {
	case VMETA_THERMAL_CALIB_STATE_DONE:
		return "DONE";
	case VMETA_THERMAL_CALIB_STATE_REQUESTED:
		return "REQUESTED";
	case VMETA_THERMAL_CALIB_STATE_IN_PROGRESS:
		return "IN_PROGRESS";
	default:
		return "UNKNOWN";
	}
}


const char *vmeta_frame_type_str(enum vmeta_frame_type val)
{
	switch (val) {
	case VMETA_FRAME_TYPE_NONE:
		return "NONE";
	case VMETA_FRAME_TYPE_V1_RECORDING:
		return "V1_REC";
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		return "V1_STRM_BASIC";
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		return "V1_STRM_EXTENDED";
	case VMETA_FRAME_TYPE_V2:
		return "V2";
	case VMETA_FRAME_TYPE_V3:
		return "V3";
	case VMETA_FRAME_TYPE_PROTO:
		return "PROTO";
	default:
		return "UNKNOWN";
	}
}


int vmeta_frame_write(struct vmeta_buffer *buf, struct vmeta_frame *meta)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		res = vmeta_frame_v1_recording_write(buf, &meta->v1_rec);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = vmeta_frame_v1_streaming_basic_write(
			buf, &meta->v1_strm_basic);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		res = vmeta_frame_v1_streaming_extended_write(
			buf, &meta->v1_strm_ext);
		break;

	case VMETA_FRAME_TYPE_V2:
		res = vmeta_frame_v2_write(buf, &meta->v2);
		break;

	case VMETA_FRAME_TYPE_V3:
		res = vmeta_frame_v3_write(buf, &meta->v3);
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_write(buf, meta);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_read(struct vmeta_buffer *buf,
		     const char *mime_type,
		     struct vmeta_frame **ret_obj)
{
	return vmeta_frame_read2(buf, mime_type, 1, ret_obj);
}


int vmeta_frame_read2(struct vmeta_buffer *buf,
		      const char *mime_type,
		      int convert,
		      struct vmeta_frame **ret_obj)
{
	int res = 0;
	size_t start = 0, len = 0;
	uint16_t id = 0;
	struct vmeta_frame *meta = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	meta = calloc(1, sizeof(*meta));
	if (!meta) {
		res = -ENOMEM;
		goto out;
	}
	res = vmeta_frame_ref(meta);
	if (res != 0) {
		free(meta);
		meta = NULL;
		goto out;
	}

	if (mime_type) {
		/* MIME type is provided. Use it to get metadata type */

		/* Determine type */
		if (strcmp(mime_type, VMETA_FRAME_V1_RECORDING_MIME_TYPE) ==
		    0) {
			meta->type = VMETA_FRAME_TYPE_V1_RECORDING;
		} else if (strcmp(mime_type, VMETA_FRAME_V2_MIME_TYPE) == 0) {
			meta->type = VMETA_FRAME_TYPE_V2;
		} else if (strcmp(mime_type, VMETA_FRAME_V3_MIME_TYPE) == 0) {
			meta->type = VMETA_FRAME_TYPE_V3;
		} else if (strcmp(mime_type, VMETA_FRAME_PROTO_MIME_TYPE) ==
			   0) {
			meta->type = VMETA_FRAME_TYPE_PROTO;
		} else {
			ULOGW("unknown metadata MIME type: '%s'", mime_type);
			res = -ENOSYS;
			goto out;
		}
	} else {
		/* MIME type is not provided. Guess type from first 2 bytes */

		/* Compute buffer size, read Id then rewind */
		start = buf->pos;
		len = buf->len - start;
		CHECK(vmeta_read_u16(buf, &id));
		buf->pos = start;

		/* Determine type */
		switch (id) {
		case VMETA_FRAME_V1_STREAMING_ID:
			if (len >= VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE) {
				meta->type =
					VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED;
			} else if (len >= VMETA_FRAME_V1_STREAMING_BASIC_SIZE) {
				meta->type =
					VMETA_FRAME_TYPE_V1_STREAMING_BASIC;
			} else {
				ULOGW("bad metadata streaming v1 length: %zu",
				      len);
				res = -EPROTO;
				goto out;
			}
			break;

		case VMETA_FRAME_V2_BASE_ID:
			meta->type = VMETA_FRAME_TYPE_V2;
			break;

		case VMETA_FRAME_V3_BASE_ID:
			meta->type = VMETA_FRAME_TYPE_V3;
			break;

		default:
			ULOGW("unknown metadata id: 0x%04x", id);
			res = -EPROTO;
			goto out;
		}
	}

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		res = vmeta_frame_v1_recording_read(buf, &meta->v1_rec);
		break;


	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = vmeta_frame_v1_streaming_basic_read(buf,
							  &meta->v1_strm_basic);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		res = vmeta_frame_v1_streaming_extended_read(
			buf, &meta->v1_strm_ext);
		break;

	case VMETA_FRAME_TYPE_V2:
		res = vmeta_frame_v2_read(buf, &meta->v2);
		break;

	case VMETA_FRAME_TYPE_V3:
		res = vmeta_frame_v3_read(buf, &meta->v3);
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_read(buf, &meta->proto);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	if (res != 0 || convert == 0)
		goto out;
	/* Convert metadata to PROTO */
	if (meta->type != VMETA_FRAME_TYPE_PROTO) {
		struct vmeta_frame *proto = NULL;
		res = vmeta_frame_convert(meta, &proto, VMETA_FRAME_TYPE_PROTO);
		if (res != 0) {
			/* If the conversion failed, return the non-converted
			 * metadata */
			res = 0;
			goto out;
		}
		vmeta_frame_unref(meta);
		meta = proto;
	}

out:
	if (res != 0 && meta) {
		vmeta_frame_unref(meta);
		meta = NULL;
	}
	*ret_obj = meta;
	return res;
}


int vmeta_frame_new(enum vmeta_frame_type type, struct vmeta_frame **ret_obj)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	struct vmeta_frame *meta = calloc(1, sizeof(*meta));
	if (!meta) {
		res = -ENOMEM;
		goto out;
	}
	res = vmeta_frame_ref(meta);
	if (res != 0) {
		free(meta);
		meta = NULL;
		goto out;
	}

	meta->type = type;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V2:
	case VMETA_FRAME_TYPE_V3:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_init(&meta->proto);
		break;

	default:
		ULOGW("unknown metadata streaming type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

out:
	if (res != 0 && meta) {
		vmeta_frame_unref(meta);
		meta = NULL;
	}
	*ret_obj = meta;
	return res;
}


int vmeta_frame_ref(struct vmeta_frame *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

#if defined(__GNUC__)
	__atomic_add_fetch(&meta->ref_count, 1, __ATOMIC_SEQ_CST);
#else
#	error no atomic increment function found on this platform
#endif

	return 0;
}


int vmeta_frame_unref(struct vmeta_frame *meta)
{
	unsigned int ref;
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

#if defined(__GNUC__)
	/* Yes, this can be racy, but calling unref on an already
	 * unrefed buffer is ugly too. We just try to catch it here for
	 * debugging purposes */
	ref = __atomic_load_n(&meta->ref_count, __ATOMIC_ACQUIRE);
	if (ref < 1)
		return -ENOENT;
	ref = __atomic_sub_fetch(&meta->ref_count, 1, __ATOMIC_SEQ_CST);
#else
#	error no atomic increment function found on this platform
#endif
	/* Still ref, nothing to do */
	if (ref > 0)
		goto out;

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_RECORDING:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
	case VMETA_FRAME_TYPE_V2:
	case VMETA_FRAME_TYPE_V3:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_destroy(meta->proto);
		meta->proto = NULL;
		break;

	default:
		/* Just log and return 0 properly since we will free the
		 * metadata anyway */
		ULOGW("unknown metadata streaming type: %u", meta->type);
		break;
	}

	free(meta);
out:
	return res;
}


int vmeta_frame_get_ref_count(struct vmeta_frame *meta)
{
	unsigned int ref;
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

#if defined(__GNUC__)
	ref = __atomic_load_n(&meta->ref_count, __ATOMIC_ACQUIRE);
	if (ref > INT_MAX)
		ref = INT_MAX;
#else
#	error no atomic increment function found on this platform
#endif
	return (int)ref;
}


int vmeta_frame_to_json(struct vmeta_frame *meta, struct json_object *jobj)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(jobj == NULL, EINVAL);

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		res = vmeta_frame_v1_recording_to_json(&meta->v1_rec, jobj);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		res = vmeta_frame_v1_streaming_basic_to_json(
			&meta->v1_strm_basic, jobj);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		res = vmeta_frame_v1_streaming_extended_to_json(
			&meta->v1_strm_ext, jobj);
		break;

	case VMETA_FRAME_TYPE_V2:
		res = vmeta_frame_v2_to_json(&meta->v2, jobj);
		break;

	case VMETA_FRAME_TYPE_V3:
		res = vmeta_frame_v3_to_json(&meta->v3, jobj);
		break;

	case VMETA_FRAME_TYPE_PROTO:
		res = vmeta_frame_proto_to_json(meta, jobj);
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		res = -ENOSYS;
		break;
	}

	return res;
}


int vmeta_frame_to_json_str(struct vmeta_frame *meta,
			    char *output,
			    unsigned int len)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(output == NULL, EINVAL);

	const char *jstr;
	struct json_object *jobj = json_object_new_object();
	if (jobj == NULL)
		return -ENOMEM;
	int ret = vmeta_frame_to_json(meta, jobj);
	if (ret < 0)
		goto out;

	jstr = json_object_to_json_string(jobj);
	if (strlen(jstr) + 1 > len) {
		ret = -ENOBUFS;
		goto out;
	}
	strcpy(output, jstr);

out:
	json_object_put(jobj);
	return ret;
}


ssize_t
vmeta_frame_to_csv(const struct vmeta_frame *meta, char *str, size_t maxlen)
{
	size_t len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	switch (meta->type) {
	case VMETA_FRAME_TYPE_NONE:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		len = vmeta_frame_v1_recording_to_csv(
			&meta->v1_rec, str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		len = vmeta_frame_v1_streaming_basic_to_csv(
			&meta->v1_strm_basic, str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		len = vmeta_frame_v1_streaming_extended_to_csv(
			&meta->v1_strm_ext, str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V2:
		len = vmeta_frame_v2_to_csv(&meta->v2, str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V3:
		len = vmeta_frame_v3_to_csv(&meta->v3, str, maxlen);
		break;

	case VMETA_FRAME_TYPE_PROTO:
		len = (size_t)-ENOSYS;
		break;

	default:
		ULOGW("unknown metadata type: %u", meta->type);
		return -ENOSYS;
	}

	return (ssize_t)len;
}


ssize_t
vmeta_frame_csv_header(enum vmeta_frame_type type, char *str, size_t maxlen)
{
	size_t len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	switch (type) {
	case VMETA_FRAME_TYPE_NONE:
		/* Nothing to do */
		break;

	case VMETA_FRAME_TYPE_V1_RECORDING:
		len = vmeta_frame_v1_recording_csv_header(str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
		len = vmeta_frame_v1_streaming_basic_csv_header(str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		len = vmeta_frame_v1_streaming_extended_csv_header(str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V2:
		len = vmeta_frame_v2_csv_header(str, maxlen);
		break;

	case VMETA_FRAME_TYPE_V3:
		len = vmeta_frame_v3_csv_header(str, maxlen);
		break;

	case VMETA_FRAME_TYPE_PROTO:
		len = (size_t)-ENOSYS;
		break;

	default:
		ULOGW("unknown metadata type: %u", type);
		return -ENOSYS;
	}

	return (ssize_t)len;
}


const char *vmeta_frame_get_mime_type(enum vmeta_frame_type type)
{
	switch (type) {
	case VMETA_FRAME_TYPE_NONE:
	case VMETA_FRAME_TYPE_V1_STREAMING_BASIC:
	case VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED:
		return NULL;
	case VMETA_FRAME_TYPE_V1_RECORDING:
		return VMETA_FRAME_V1_RECORDING_MIME_TYPE;
	case VMETA_FRAME_TYPE_V2:
		return VMETA_FRAME_V2_MIME_TYPE;
	case VMETA_FRAME_TYPE_V3:
		return VMETA_FRAME_V3_MIME_TYPE;
	case VMETA_FRAME_TYPE_PROTO:
		return VMETA_FRAME_PROTO_MIME_TYPE;
	default:
		ULOGW("unknown metadata type: %u", type);
		return NULL;
	}
}


int vmeta_frame_ext_timestamp_write(
	struct vmeta_buffer *buf,
	const struct vmeta_frame_ext_timestamp *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u64(buf, meta->frame_timestamp));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_EXT_TIMESTAMP_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_ext_timestamp_read(struct vmeta_buffer *buf,
				   struct vmeta_frame_ext_timestamp *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_EXT_TIMESTAMP_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_EXT_TIMESTAMP_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	CHECK(vmeta_read_u64(buf, &meta->frame_timestamp));

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_ext_followme_write(struct vmeta_buffer *buf,
				   const struct vmeta_frame_ext_followme *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	struct vmeta_location target;
	uint8_t mode = 0, anim = 0;
	uint8_t reserved1 = 0, reserved2 = 0;
	uint32_t reserved3 = 0, reserved4 = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Adjust target if needed */
	vmeta_location_adjust_write(&meta->target, &target);

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f64_i32(buf, target.latitude, 22));
	CHECK(vmeta_write_f64_i32(buf, target.longitude, 22));
	CHECK(vmeta_write_f64_i32(buf, target.altitude_egm96amsl, 16));

	mode = meta->enabled | (meta->mode << 1) | (meta->angle_locked << 2);
	anim = meta->animation;
	CHECK(vmeta_write_u8(buf, mode));
	CHECK(vmeta_write_u8(buf, anim));
	CHECK(vmeta_write_u8(buf, reserved1));
	CHECK(vmeta_write_u8(buf, reserved2));
	CHECK(vmeta_write_u32(buf, reserved3));
	CHECK(vmeta_write_u32(buf, reserved4));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_EXT_FOLLOWME_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_ext_followme_read(struct vmeta_buffer *buf,
				  struct vmeta_frame_ext_followme *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	struct vmeta_location target;
	uint8_t mode = 0, anim = 0;
	uint8_t reserved1 = 0, reserved2 = 0;
	uint32_t reserved3 = 0, reserved4 = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_EXT_FOLLOWME_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_EXT_FOLLOWME_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	memset(&target, 0, sizeof(target));
	CHECK(vmeta_read_f64_i32(buf, &target.latitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &target.longitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &target.altitude_egm96amsl, 16));
	CHECK(vmeta_read_u8(buf, &mode));
	CHECK(vmeta_read_u8(buf, &anim));
	meta->enabled = mode & 0x1;
	meta->mode = (mode >> 1) & 0x1;
	meta->angle_locked = (mode >> 2) & 0x1;
	meta->animation = anim;
	CHECK(vmeta_read_u8(buf, &reserved1));
	CHECK(vmeta_read_u8(buf, &reserved2));
	CHECK(vmeta_read_u32(buf, &reserved3));
	CHECK(vmeta_read_u32(buf, &reserved4));

	/* Adjust target if needed */
	target.altitude_wgs84ellipsoid = NAN;
	vmeta_location_adjust_read(&target, &meta->target);
	meta->target.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_ext_automation_write(
	struct vmeta_buffer *buf,
	const struct vmeta_frame_ext_automation *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	struct vmeta_location framing_target, flight_destination;
	uint8_t flags = 0, anim = 0;
	uint16_t reserved = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Adjust targets if needed */
	vmeta_location_adjust_write(&meta->framing_target, &framing_target);
	vmeta_location_adjust_write(&meta->flight_destination,
				    &flight_destination);

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f64_i32(buf, framing_target.latitude, 22));
	CHECK(vmeta_write_f64_i32(buf, framing_target.longitude, 22));
	CHECK(vmeta_write_f64_i32(buf, framing_target.altitude_egm96amsl, 16));
	CHECK(vmeta_write_f64_i32(buf, flight_destination.latitude, 22));
	CHECK(vmeta_write_f64_i32(buf, flight_destination.longitude, 22));
	CHECK(vmeta_write_f64_i32(
		buf, flight_destination.altitude_egm96amsl, 16));

	anim = meta->animation;
	flags = meta->followme_enabled | (meta->lookatme_enabled << 1) |
		(meta->angle_locked << 2);
	CHECK(vmeta_write_u8(buf, anim));
	CHECK(vmeta_write_u8(buf, flags));
	CHECK(vmeta_write_u16(buf, reserved));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_EXT_AUTOMATION_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_ext_automation_read(struct vmeta_buffer *buf,
				    struct vmeta_frame_ext_automation *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	struct vmeta_location framing_target, flight_destination;
	uint8_t flags = 0, anim = 0;
	uint16_t reserved = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_EXT_AUTOMATION_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_EXT_AUTOMATION_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	memset(&framing_target, 0, sizeof(framing_target));
	memset(&flight_destination, 0, sizeof(flight_destination));
	CHECK(vmeta_read_f64_i32(buf, &framing_target.latitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &framing_target.longitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &framing_target.altitude_egm96amsl, 16));
	CHECK(vmeta_read_f64_i32(buf, &flight_destination.latitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &flight_destination.longitude, 22));
	CHECK(vmeta_read_f64_i32(
		buf, &flight_destination.altitude_egm96amsl, 16));
	CHECK(vmeta_read_u8(buf, &anim));
	CHECK(vmeta_read_u8(buf, &flags));
	meta->followme_enabled = flags & 0x1;
	meta->lookatme_enabled = (flags >> 1) & 0x1;
	meta->angle_locked = (flags >> 2) & 0x1;
	meta->animation = anim;
	CHECK(vmeta_read_u16(buf, &reserved));

	/* Adjust targets if needed */
	framing_target.altitude_wgs84ellipsoid = NAN;
	vmeta_location_adjust_read(&framing_target, &meta->framing_target);
	meta->framing_target.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;
	flight_destination.altitude_wgs84ellipsoid = NAN;
	vmeta_location_adjust_read(&flight_destination,
				   &meta->flight_destination);
	meta->flight_destination.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_ext_thermal_write(struct vmeta_buffer *buf,
				  const struct vmeta_frame_ext_thermal *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	uint8_t flags = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f32_i16(buf, meta->min.x, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->min.y, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->min.temp, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->max.x, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->max.y, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->max.temp, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->probe.x, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->probe.y, 5));
	CHECK(vmeta_write_f32_i16(buf, meta->probe.temp, 5));
	CHECK(vmeta_write_u8(buf, (uint8_t)meta->calib_state));

	flags = meta->min.valid | (meta->max.valid << 1) |
		(meta->probe.valid << 2);
	CHECK(vmeta_write_u8(buf, flags));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_EXT_THERMAL_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_ext_thermal_read(struct vmeta_buffer *buf,
				 struct vmeta_frame_ext_thermal *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	uint8_t flags = 0;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_EXT_THERMAL_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_EXT_THERMAL_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	CHECK(vmeta_read_f32_i16(buf, &meta->min.x, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->min.y, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->min.temp, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->max.x, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->max.y, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->max.temp, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->probe.x, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->probe.y, 5));
	CHECK(vmeta_read_f32_i16(buf, &meta->probe.temp, 5));
	CHECK(vmeta_read_u8(buf, (uint8_t *)&meta->calib_state));
	CHECK(vmeta_read_u8(buf, &flags));
	meta->min.valid = flags & 0x1;
	meta->max.valid = (flags >> 1) & 0x1;
	meta->probe.valid = (flags >> 2) & 0x1;

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_ext_lfic_write(struct vmeta_buffer *buf,
			       const struct vmeta_frame_ext_lfic *meta)
{
	int res = 0;
	size_t start = 0, end = 0;
	uint16_t len = 0;
	struct vmeta_location target;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Remember start position */
	start = buf->pos;

	/* Adjust target if needed */
	vmeta_location_adjust_write(&meta->target_location, &target);

	/* Write fields */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_u16(buf, 0)); /* temp value, updated later */
	CHECK(vmeta_write_f32_u16(buf, meta->target_x, 14));
	CHECK(vmeta_write_f32_u16(buf, meta->target_y, 14));
	CHECK(vmeta_write_f64_i32(buf, target.latitude, 22));
	CHECK(vmeta_write_f64_i32(buf, target.longitude, 22));
	CHECK(vmeta_write_f64_i32(buf, target.altitude_egm96amsl, 16));
	CHECK(vmeta_write_f64_u32(buf, meta->estimated_precision, 16));
	CHECK(vmeta_write_f64_u32(buf, meta->grid_precision, 16));

	/* Check for correct alignment */
	if ((buf->pos - start) % 4 != 0) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: buffer not aligned: %zu",
		      buf->pos - start);
		goto out;
	}

	/* Write id and length */
	end = buf->pos;
	len = (buf->pos - start - 4) / 4;
	buf->pos = start;
	CHECK(vmeta_write_u16(buf, VMETA_FRAME_EXT_LFIC_ID));
	CHECK(vmeta_write_u16(buf, len));
	buf->pos = end;

out:
	return res;
}


int vmeta_frame_ext_lfic_read(struct vmeta_buffer *buf,
			      struct vmeta_frame_ext_lfic *meta)
{
	int res = 0;
	size_t start = 0;
	uint16_t id = 0, len = 0;
	struct vmeta_location target;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	/* Get buffer start */
	start = buf->pos;

	/* Read Id */
	CHECK(vmeta_read_u16(buf, &id));
	if (id != VMETA_FRAME_EXT_LFIC_ID) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad id: 0x%04x (0x%04x)",
		      id,
		      VMETA_FRAME_EXT_LFIC_ID);
		goto out;
	}

	/* Read len */
	CHECK(vmeta_read_u16(buf, &len));
	if (buf->len - start < (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->len - start,
		      len * 4 + 4);
		goto out;
	}

	/* Read fields */
	memset(meta, 0, sizeof(*meta));
	memset(&target, 0, sizeof(target));
	CHECK(vmeta_read_f32_u16(buf, &meta->target_x, 14));
	CHECK(vmeta_read_f32_u16(buf, &meta->target_y, 14));
	CHECK(vmeta_read_f64_i32(buf, &target.latitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &target.longitude, 22));
	CHECK(vmeta_read_f64_i32(buf, &target.altitude_egm96amsl, 16));
	CHECK(vmeta_read_f64_u32(buf, &meta->estimated_precision, 16));
	CHECK(vmeta_read_f64_u32(buf, &meta->grid_precision, 16));

	/* Adjust target if needed */
	target.altitude_wgs84ellipsoid = NAN;
	vmeta_location_adjust_read(&target, &meta->target_location);
	meta->target_location.sv_count = VMETA_LOCATION_INVALID_SV_COUNT;

	/* Make sure we read the correct number of bytes */
	if (buf->pos - start != (size_t)len * 4 + 4) {
		res = -EPROTO;
		ULOGE("vmeta_frame_ext: bad length: %zu (%u)",
		      buf->pos - start,
		      len * 4 + 4);
		goto out;
	}

out:
	return res;
}


int vmeta_frame_convert(struct vmeta_frame *in_frame,
			struct vmeta_frame **out_frame,
			enum vmeta_frame_type out_type)
{
	int res;
	struct vmeta_frame *new = NULL;
	Vmeta__TimedMetadata *proto = NULL;
	Vmeta__DroneMetadata *drone;
	Vmeta__CameraMetadata *camera;
	Vmeta__WifiLinkMetadata *wifi;
	Vmeta__AutomationMetadata *automation;
	Vmeta__ThermalMetadata *thermal;
	Vmeta__LFICMetadata *lfic;
	Vmeta__Quaternion *quat;
	Vmeta__Location *loc;
	Vmeta__NED *ned;
	Vmeta__ThermalSpot *spot;
	ULOG_ERRNO_RETURN_ERR_IF(in_frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(out_frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_frame->type != VMETA_FRAME_TYPE_V3, ENOSYS);
	ULOG_ERRNO_RETURN_ERR_IF(out_type != VMETA_FRAME_TYPE_PROTO, ENOSYS);

	/* For now, this function only converts from v3 to proto, so no need to
	 * test in_frame->type & out_type here since they are checked in the
	 * ULOG_ERRNO_RETURN_ERR_IF(...) blocks */

	res = vmeta_frame_new(out_type, &new);
	if (res != 0)
		goto out;

	/* Get the output metadata */
	res = vmeta_frame_proto_get_unpacked_rw(new, &proto);
	if (res != 0)
		goto out;

	drone = vmeta_frame_proto_get_drone(proto);
	if (drone == NULL)
		goto out;

	camera = vmeta_frame_proto_get_camera(proto);
	if (camera == NULL)
		goto out;

	wifi = vmeta_frame_proto_add_wifi_link(proto);
	if (wifi == NULL)
		goto out;

	/* Convert v3.base */
	{
		struct vmeta_frame_v3_base *base = &in_frame->v3.base;
		/* drone quat */
		quat = vmeta_frame_proto_get_drone_quat(drone);
		if (quat == NULL)
			goto out;
		quat->w = base->drone_quat.w;
		quat->x = base->drone_quat.x;
		quat->y = base->drone_quat.y;
		quat->z = base->drone_quat.z;
		/* location */
		if (base->location.valid) {
			loc = vmeta_frame_proto_get_drone_location(drone);
			if (loc == NULL)
				goto out;
			loc->altitude_wgs84ellipsoid =
				base->location.altitude_wgs84ellipsoid;
			if (isnan(loc->altitude_wgs84ellipsoid))
				loc->altitude_wgs84ellipsoid = 0.;
			else if (loc->altitude_wgs84ellipsoid == 0.)
				loc->altitude_wgs84ellipsoid = DBL_MIN;
			loc->altitude_egm96amsl =
				base->location.altitude_egm96amsl;
			if (isnan(loc->altitude_egm96amsl))
				loc->altitude_egm96amsl = 0.;
			else if (loc->altitude_egm96amsl == 0.)
				loc->altitude_egm96amsl = DBL_MIN;
			loc->latitude = base->location.latitude;
			loc->longitude = base->location.longitude;
			loc->horizontal_accuracy =
				base->location.horizontal_accuracy;
			loc->vertical_accuracy =
				base->location.vertical_accuracy;
			loc->sv_count = base->location.sv_count;
		}
		/* ground_distance */
		drone->ground_distance = base->ground_distance;
		/* speed */
		ned = vmeta_frame_proto_get_drone_speed(drone);
		if (ned == NULL)
			goto out;
		ned->north = base->speed.north;
		ned->east = base->speed.east;
		ned->down = base->speed.down;
		/* air_speed omitted in proto metadata */
		/* frame_base_quat */
		quat = vmeta_frame_proto_get_camera_base_quat(camera);
		if (quat == NULL)
			goto out;
		quat->w = base->frame_base_quat.w;
		quat->x = base->frame_base_quat.x;
		quat->y = base->frame_base_quat.y;
		quat->z = base->frame_base_quat.z;
		/* frame_quat */
		quat = vmeta_frame_proto_get_camera_quat(camera);
		if (quat == NULL)
			goto out;
		quat->w = base->frame_quat.w;
		quat->x = base->frame_quat.x;
		quat->y = base->frame_quat.y;
		quat->z = base->frame_quat.z;
		/* exposure_time */
		camera->exposure_time = base->exposure_time;
		/* gain */
		camera->iso_gain = base->gain;
		/* awb_r_gain */
		camera->awb_r_gain = base->awb_r_gain;
		/* awb_b_gain */
		camera->awb_b_gain = base->awb_b_gain;
		/* picture_hfov, deg->rad */
		camera->hfov = base->picture_hfov * M_PI / 180.;
		/* picture_vfov, deg->rad */
		camera->vfov = base->picture_vfov * M_PI / 180.;
		/* link_goodput */
		wifi->goodput = base->link_goodput;
		/* link_quality */
		wifi->quality = base->link_quality;
		/* wifi_rssi */
		wifi->rssi = base->wifi_rssi;
		/* battery_percentage */
		drone->battery_percentage = base->battery_percentage;
		/* animation */
		drone->animation_in_progress = base->animation;
		/* state */
		drone->flying_state =
			vmeta_frame_flying_state_vmeta_to_proto(base->state);
		/* mode */
		drone->piloting_mode =
			vmeta_frame_piloting_mode_vmeta_to_proto(base->mode);
	}

	/* Convert v3.timestamp */
	if (in_frame->v3.has_timestamp)
		camera->timestamp = in_frame->v3.timestamp.frame_timestamp;

	/* Convert v3.automation */
	if (in_frame->v3.has_automation) {
		struct vmeta_frame_ext_automation *automationv3 =
			&in_frame->v3.automation;
		automation = vmeta_frame_proto_get_automation(proto);
		if (automation == NULL)
			goto out;
		/* framing_target */
		if (automationv3->framing_target.valid) {
			loc = vmeta_frame_proto_get_automation_target_location(
				automation);
			if (loc == NULL)
				goto out;
			loc->altitude_wgs84ellipsoid =
				automationv3->framing_target
					.altitude_wgs84ellipsoid;
			if (isnan(loc->altitude_wgs84ellipsoid))
				loc->altitude_wgs84ellipsoid = 0.;
			else if (loc->altitude_wgs84ellipsoid == 0.)
				loc->altitude_wgs84ellipsoid = DBL_MIN;
			loc->altitude_egm96amsl =
				automationv3->framing_target.altitude_egm96amsl;
			if (isnan(loc->altitude_egm96amsl))
				loc->altitude_egm96amsl = 0.;
			else if (loc->altitude_egm96amsl == 0.)
				loc->altitude_egm96amsl = DBL_MIN;
			loc->latitude = automationv3->framing_target.latitude;
			loc->longitude = automationv3->framing_target.longitude;
			loc->horizontal_accuracy = automationv3->framing_target
							   .horizontal_accuracy;
			loc->vertical_accuracy =
				automationv3->framing_target.vertical_accuracy;
			loc->sv_count = automationv3->framing_target.sv_count;
		}
		/* flight_destination */
		if (automationv3->flight_destination.valid) {
			loc = vmeta_frame_proto_get_automation_destination(
				automation);
			if (loc == NULL)
				goto out;
			loc->altitude_wgs84ellipsoid =
				automationv3->flight_destination
					.altitude_wgs84ellipsoid;
			if (isnan(loc->altitude_wgs84ellipsoid))
				loc->altitude_wgs84ellipsoid = 0.;
			else if (loc->altitude_wgs84ellipsoid == 0.)
				loc->altitude_wgs84ellipsoid = DBL_MIN;
			loc->altitude_egm96amsl =
				automationv3->flight_destination
					.altitude_egm96amsl;
			if (isnan(loc->altitude_egm96amsl))
				loc->altitude_egm96amsl = 0.;
			else if (loc->altitude_egm96amsl == 0.)
				loc->altitude_egm96amsl = DBL_MIN;
			loc->latitude =
				automationv3->flight_destination.latitude;
			loc->longitude =
				automationv3->flight_destination.longitude;
			loc->horizontal_accuracy =
				automationv3->flight_destination
					.horizontal_accuracy;
			loc->vertical_accuracy =
				automationv3->flight_destination
					.vertical_accuracy;
			loc->sv_count =
				automationv3->flight_destination.sv_count;
		}
		/* followme_enabled */
		automation->follow_me = automationv3->followme_enabled;
		/* lookatme_enabled */
		automation->lookat_me = automationv3->lookatme_enabled;
		/* angle_locked */
		automation->angle_locked = automationv3->angle_locked;
		/* animation */
		automation->animation =
			vmeta_frame_automation_anim_vmeta_to_proto(
				automationv3->animation);
	}

	/* Convert v3.thermal */
	if (in_frame->v3.has_thermal) {
		struct vmeta_frame_ext_thermal *thermalv3 =
			&in_frame->v3.thermal;
		thermal = vmeta_frame_proto_get_thermal(proto);
		if (thermal == NULL)
			goto out;
		/* calib_state */
		thermal->calibration_state =
			vmeta_frame_thermal_calib_state_vmeta_to_proto(
				thermalv3->calib_state);
		/* min */
		if (thermalv3->min.valid) {
			spot = vmeta_frame_proto_get_thermal_min(thermal);
			if (spot == NULL)
				goto out;
			spot->x = thermalv3->min.x;
			spot->y = thermalv3->min.y;
			spot->temp = thermalv3->min.temp;
		}
		/* max */
		if (thermalv3->max.valid) {
			spot = vmeta_frame_proto_get_thermal_max(thermal);
			if (spot == NULL)
				goto out;
			spot->x = thermalv3->max.x;
			spot->y = thermalv3->max.y;
			spot->temp = thermalv3->max.temp;
		}
		/* probe */
		if (thermalv3->probe.valid) {
			spot = vmeta_frame_proto_get_thermal_probe(thermal);
			if (spot == NULL)
				goto out;
			spot->x = thermalv3->probe.x;
			spot->y = thermalv3->probe.y;
			spot->temp = thermalv3->probe.temp;
		}
	}

	/* Convert v3.lfic */
	if (in_frame->v3.has_lfic) {
		struct vmeta_frame_ext_lfic *lficv3 = &in_frame->v3.lfic;
		lfic = vmeta_frame_proto_get_lfic(proto);
		if (lfic == NULL)
			goto out;
		/* target_x */
		lfic->x = lficv3->target_x;
		/* target_y */
		lfic->y = lficv3->target_y;
		/* target_location + estimated_precision */
		if (lficv3->target_location.valid) {
			loc = vmeta_frame_proto_get_lfic_location(lfic);
			if (loc == NULL)
				goto out;
			loc->altitude_wgs84ellipsoid =
				lficv3->target_location.altitude_wgs84ellipsoid;
			if (isnan(loc->altitude_wgs84ellipsoid))
				loc->altitude_wgs84ellipsoid = 0.;
			else if (loc->altitude_wgs84ellipsoid == 0.)
				loc->altitude_wgs84ellipsoid = DBL_MIN;
			loc->altitude_egm96amsl =
				lficv3->target_location.altitude_egm96amsl;
			if (isnan(loc->altitude_egm96amsl))
				loc->altitude_egm96amsl = 0.;
			else if (loc->altitude_egm96amsl == 0.)
				loc->altitude_egm96amsl = DBL_MIN;
			loc->latitude = lficv3->target_location.latitude;
			loc->longitude = lficv3->target_location.longitude;
			loc->horizontal_accuracy = lficv3->estimated_precision;
			loc->vertical_accuracy = lficv3->estimated_precision;
			loc->sv_count = lficv3->target_location.sv_count;
		}
		/* grid_precision */
		lfic->grid_precision = lficv3->grid_precision;
	}


out:
	if (proto != NULL)
		vmeta_frame_proto_release_unpacked_rw(new, proto);
	if (res == 0)
		*out_frame = new;
	else
		vmeta_frame_unref(new);
	return res;
}
