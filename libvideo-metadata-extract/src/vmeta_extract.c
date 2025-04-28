/**
 * Copyright (c) 2023 Parrot Drones SAS
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

#include <video-metadata/vmeta.h>
#include <video-metadata/vmeta_extract.h>

#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libmp4.h>

#define ULOG_TAG vmeta_extract
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


static int mp4_extract(struct vmeta_session *meta, struct mp4_demux *demux)
{
	int ret = 0;
	unsigned int session_meta_count, k;
	char **keys = NULL;
	char **values = NULL;

	/* Get the session metadata */
	memset(meta, 0, sizeof(*meta));
	ret = mp4_demux_get_metadata_strings(
		demux, &session_meta_count, &keys, &values);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_metadata_strings", -ret);
		goto out;
	}
	for (k = 0; k < session_meta_count; k++) {
		char *key = keys[k];
		char *value = values[k];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, meta);
			if (ret < 0) {
				ULOG_ERRNO("vmeta_session_recording_read",
					   -ret);
				continue;
			}
		}
	}

out:
	return ret;
}


int vmeta_extract(const char *path,
		  struct vmeta_session *meta,
		  uint64_t *duration_us,
		  struct mp4_demux *demux)
{
	int ret = 0;
	int create_new_demuxer = !!(demux == NULL);
	struct mp4_demux *demuxer = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(path == NULL && create_new_demuxer, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	if (!create_new_demuxer) {
		demuxer = demux;
		goto skip_mux_creation;
	}

	if (strncasecmp(path + strlen(path) - 4, ".mp4", 4)) {
		ULOGE("invalid file %s", path);
		ret = -EINVAL;
		goto out;
	}

	ret = mp4_demux_open(path, &demuxer);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_open '%s'", -ret, path);
		goto out;
	}

skip_mux_creation:
	/* Read the MP4 file */
	ret = mp4_extract(meta, demuxer);
	if (ret < 0) {
		ULOG_ERRNO("mp4_extract", -ret);
		goto out;
	}

	if (duration_us != NULL) {
		struct mp4_media_info media_info = {};
		/* Read the MP4 duration */
		ret = mp4_demux_get_media_info(demuxer, &media_info);
		if (ret < 0) {
			ULOG_ERRNO("mp4_demux_get_media_info", -ret);
			goto out;
		}
		*duration_us = media_info.duration;
	}

out:
	if (demuxer != NULL && create_new_demuxer)
		mp4_demux_close(demuxer);

	return ret;
}


int vmeta_extract_get_track_count(const char *path, struct mp4_demux *demux)
{
	int ret = 0;
	int create_new_demuxer = !!(demux == NULL);
	struct mp4_demux *demuxer = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(path == NULL && create_new_demuxer, EINVAL);

	if (!create_new_demuxer) {
		demuxer = demux;
		goto skip_mux_creation;
	}

	if (strncasecmp(path + strlen(path) - 4, ".mp4", 4)) {
		ULOGE("invalid file %s", path);
		ret = -EINVAL;
		goto out;
	}

	ret = mp4_demux_open(path, &demuxer);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_open '%s'", -ret, path);
		goto out;
	}

skip_mux_creation:
	ret = mp4_demux_get_track_count(demuxer);

out:
	if (demuxer != NULL && create_new_demuxer)
		mp4_demux_close(demuxer);

	return ret;
}


static int track_extract(struct vmeta_session *meta,
			 uint32_t track_index,
			 uint32_t *track_id,
			 uint64_t *track_duration_us,
			 struct mp4_demux *demux)
{
	int ret = 0;
	unsigned int session_meta_count, k;
	char **keys = NULL;
	char **values = NULL;
	struct mp4_track_info track_info;

	ret = mp4_demux_get_track_info(demux, track_index, &track_info);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_track_info", -ret);
		goto out;
	}

	/* Get the session metadata */
	memset(meta, 0, sizeof(*meta));
	ret = mp4_demux_get_track_metadata_strings(
		demux, track_info.id, &session_meta_count, &keys, &values);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_metadata_strings", -ret);
		goto out;
	}
	for (k = 0; k < session_meta_count; k++) {
		char *key = keys[k];
		char *value = values[k];
		if ((key) && (value)) {
			ret = vmeta_session_recording_read(key, value, meta);
			if (ret < 0) {
				ULOG_ERRNO("vmeta_session_recording_read",
					   -ret);
				continue;
			}
		}
	}

	if (track_id != NULL)
		*track_id = track_info.id;

	if (track_duration_us != NULL)
		*track_duration_us = track_info.duration;
out:
	return ret;
}


int vmeta_extract_track(const char *path,
			uint32_t track_index,
			uint32_t *track_id,
			struct vmeta_session *meta,
			uint64_t *track_duration_us,
			struct mp4_demux *demux)
{
	int ret = 0;
	int create_new_demuxer = !!(demux == NULL);
	struct mp4_demux *demuxer = NULL;
	uint32_t track_count;

	ULOG_ERRNO_RETURN_ERR_IF(path == NULL && create_new_demuxer, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	if (!create_new_demuxer) {
		demuxer = demux;
		goto skip_mux_creation;
	}

	if (strncasecmp(path + strlen(path) - 4, ".mp4", 4)) {
		ULOGE("invalid file %s", path);
		ret = -EINVAL;
		goto out;
	}

	ret = mp4_demux_open(path, &demuxer);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_open '%s'", -ret, path);
		goto out;
	}

skip_mux_creation:
	ret = mp4_demux_get_track_count(demuxer);
	if (ret < 0) {
		ULOG_ERRNO("mp4_demux_get_track_count", -ret);
		goto out;
	}
	track_count = (uint32_t)ret;
	if (track_index >= track_count) {
		ULOGE("invalid track index (%" PRIu32 ")", track_index);
		ret = -EINVAL;
		goto out;
	}

	ret = track_extract(
		meta, track_index, track_id, track_duration_us, demuxer);
	if (ret < 0) {
		ULOG_ERRNO("track_extract", -ret);
		goto out;
	}

out:
	if (demuxer != NULL && create_new_demuxer)
		mp4_demux_close(demuxer);

	return ret;
}
