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

#ifndef _VMETA_FRAME_V2_H_
#define _VMETA_FRAME_V2_H_


/* "Parrot Video Metadata" v2 maximum data size */
#define VMETA_FRAME_V2_MAX_SIZE 96

/* "Parrot Video Metadata" v2 identifier */
#define VMETA_FRAME_V2_BASE_ID 0x5032

/* "Parrot Video Metadata" v2 MIME type */
#define VMETA_FRAME_V2_MIME_TYPE                                               \
	"application/octet-stream;type=com.parrot.videometadata2"

/* "Parrot Video Metadata" v2 content encoding */
#define VMETA_FRAME_V2_CONTENT_ENCODING ""


/* "Parrot Video Metadata" v2 base structure definition */
/* clang-format off */
struct vmeta_frame_v2_base {
	/* Drone quaternion */
	struct vmeta_quaternion drone_quat;

	/* Drone location */
	struct vmeta_location location;

	/* Best ground distance estimation (m) */
	double ground_distance;

	/* Speed vector in NED (North-East-Down) (m/s) */
	struct vmeta_ned speed;

	/* Speed relative to air (m/s), negative means no data */
	float air_speed;

	/* Frame view quaternion */
	struct vmeta_quaternion frame_quat;

	/* Camera pan (rad) */
	float camera_pan;

	/* Camera tilt (rad) */
	float camera_tilt;

	/* Frame exposure time (ms) */
	float exposure_time;

	/* Frame ISO gain */
	uint16_t gain;

	/* Wifi RSSI (dBm) */
	int8_t wifi_rssi;

	/* Battery charge percentage */
	uint8_t battery_percentage;

	/* Sensor binning (0: disabled, 1: enabled) */
	uint32_t binning:1;

	/* Animation (0: no animation, 1: animation in progress) */
	uint32_t animation:1;

	/* Flying state */
	enum vmeta_flying_state state;

	/* Piloting mode */
	enum vmeta_piloting_mode mode;
};
/* clang-format on */


/* "Parrot Video Metadata" v2 definition */
/* clang-format off */
struct vmeta_frame_v2 {
	/* Base structure */
	struct vmeta_frame_v2_base base;

	/* Timestamp extension validity flag (1 if the timestamp structure
	 * contents are valid, 0 otherwise) */
	uint32_t has_timestamp:1;

	/* Follow-me extension validity flag (1 if the followme structure
	 * contents are valid, 0 otherwise) */
	uint32_t has_followme:1;

	/* Timestamp extension */
	struct vmeta_frame_ext_timestamp timestamp;

	/* Follow-me extension */
	struct vmeta_frame_ext_followme followme;
};
/* clang-format on */


/**
 * Write "Parrot Video Metadata" v2 frame metadata.
 * This function fills the supplied buffer with the serialized metadata.
 * The size of the data written is returned through the pos field in the buf
 * structure.
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
int vmeta_frame_v2_write(struct vmeta_buffer *buf,
			 const struct vmeta_frame_v2 *meta);


/**
 * Read "Parrot Video Metadata" v2 frame metadata.
 * This function fills the supplied structure with the deserialized metadata.
 * The pos field in the buf structure must be set to the starting position for
 * reading (can be 0 if no previous data is present in the buffer). The size
 * of the buffer (len field) must be sufficient to allow reading the metadata
 * otherwise an error is returned. The ownership of the buffer stays with
 * the caller.
 * @param buf: pointer to the buffer structure
 * @param meta: pointer to the frame metadata structure (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_v2_read(struct vmeta_buffer *buf, struct vmeta_frame_v2 *meta);


/**
 * Write "Parrot Video Metadata" v2 frame metadata to a JSON object.
 * The jobj JSON object must have been previously allocated.
 * The ownership of the JSON object stays with the caller.
 * @param meta: pointer to a frame metadata structure
 * @param jobj: pointer to the JSON object to write to (output)
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_v2_to_json(const struct vmeta_frame_v2 *meta,
			   struct json_object *jobj);


/**
 * Write "Parrot Video Metadata" v2 frame metadata as a CSV string.
 * The str string must have been previously allocated. The function writes
 * up to maxlen characters. The CSV separator is a space character.
 * @param meta: pointer to a frame metadata structure
 * @param str: pointer to the string to write to (output)
 * @param maxlen: maximum length of the string
 * @return the number of characters written
 */
VMETA_API
size_t vmeta_frame_v2_to_csv(const struct vmeta_frame_v2 *meta,
			     char *str,
			     size_t maxlen);


/**
 * Write a "Parrot Video Metadata" v2 frame metadata CSV file header string.
 * The str string must have been previously allocated. The function writes
 * up to maxlen characters. The CSV separator is a space character.
 * @param str: pointer to the string to write to (output)
 * @param maxlen: maximum length of the string
 * @return the number of characters written
 */
VMETA_API
size_t vmeta_frame_v2_csv_header(char *str, size_t maxlen);


#endif /* !_VMETA_FRAME_V2_H_ */
