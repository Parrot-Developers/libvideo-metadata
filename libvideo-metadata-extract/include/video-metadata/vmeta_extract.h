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

#ifndef _VMETA_EXTRACT_H_
#define _VMETA_EXTRACT_H_

#include <libmp4.h>
#include <video-metadata/vmeta.h>

/* To be used for all public API */
#ifdef VMETA_EXTRACT_API_EXPORTS
#	ifdef _WIN32
#		define VMETA_EXTRACT_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define VMETA_EXTRACT_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !VMETA_EXTRACT_EXPORTS */
#	define VMETA_EXTRACT_API
#endif /* !VMETA_EXTRACT_EXPORTS */


/**
 * Fill a vmeta_session structure from an MP4 file.
 * The vmeta_session structure must be previously allocated by the caller.
 * @param path: path to the MP4 file (unused if demux is given)
 * @param meta: vmeta_session structure to fill (output)
 * @param duration_us: MP4 duration in microseconds to fill (output, optional)
 * @param demux: demuxer to use (optional)
 * @return: 0 on success, negative errno on failure
 */
VMETA_EXTRACT_API int vmeta_extract(const char *path,
				    struct vmeta_session *meta,
				    uint64_t *duration_us,
				    struct mp4_demux *demux);


/**
 * Get the track count from an MP4 file.
 * @param path: path to the MP4 file (unused if demux is given)
 * @param demux: demuxer to use (optional)
 * @return: the track count on success, negative errno on failure
 */
VMETA_EXTRACT_API int vmeta_extract_get_track_count(const char *path,
						    struct mp4_demux *demux);


/**
 * Fill a vmeta_session structure from track of an MP4 file.
 * The vmeta_session structure must be previously allocated by the caller.
 * @param path: path to the MP4 file (unused if demux is given)
 * @param track_index: index of the track
 * @param track_id: track ID to fill (output, optional)
 * @param meta: track vmeta_session structure to fill (output)
 * @param track_duration_us: track duration in microseconds to fill (output,
 * optional)
 * @param demux: demuxer to use (optional)
 * @return: 0 on success, negative errno on failure
 */
VMETA_EXTRACT_API int vmeta_extract_track(const char *path,
					  uint32_t track_index,
					  uint32_t *track_id,
					  struct vmeta_session *meta,
					  uint64_t *track_duration_us,
					  struct mp4_demux *demux);


#endif /*_VMETA_EXTRACT_H_*/
