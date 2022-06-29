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

#ifndef _FILE_OFFSET_BITS
#	define _FILE_OFFSET_BITS 64
#endif

#include <errno.h>
#include <getopt.h>
#include <json-c/json.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <video-metadata/vmeta.h>
#ifdef BUILD_LIBMP4
#	include <libmp4.h>
#endif /* BUILD_LIBMP4 */
#ifdef BUILD_LIBPCAP
#	include <pcap/pcap.h>
#endif /* BUILD_LIBPCAP */

#define ULOG_TAG vmeta_extract
#include <ulog.h>
ULOG_DECLARE_TAG(vmeta_extract);


#define BUF_SIZE 400
#define STR_SIZE 2000
#define RTP_DEFAULT_PORT 55004
#define RTCP_DEFAULT_PORT 55005


enum args_id {
	ARGS_ID_CSV = 256,
	ARGS_ID_KML,
	ARGS_ID_JSON,
	ARGS_ID_JSON_PRETTY,
	ARGS_ID_RTP_PORT,
	ARGS_ID_RTCP_PORT,
};


struct vmeta_extract {
	char *input_file_name;
	int is_first;
	int rtp_port;
	int rtcp_port;
	uint64_t highest_ext_rtp_ts;
	uint64_t prev_ext_rtp_ts;
	struct vmeta_session session_meta;

	char *csv_file_name;
	FILE *csv_file;
	enum vmeta_frame_type type_for_csv;

	char *kml_file_name;
	FILE *kml_file;
	enum vmeta_frame_type type_for_kml;

	char *json_file_name;
	FILE *json_file;
	int json_pretty;
	json_object *json_data;
};


static void kml_header(struct vmeta_extract *self, int is_absolute_altitude)
{
	fprintf(self->kml_file, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	fprintf(self->kml_file,
		"<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
	fprintf(self->kml_file, "  <Document>\n");
	fprintf(self->kml_file, "    <Placemark>\n");
	/* TODO: get name and description from the session metadata */
	fprintf(self->kml_file, "      <name>Flight</name>\n");
	fprintf(self->kml_file, "      <description>...</description>\n");
	fprintf(self->kml_file, "      <LineString>\n");
	fprintf(self->kml_file,
		"        <altitudeMode>%s</altitudeMode>\n",
		(is_absolute_altitude) ? "absolute" : "relativeToGround");
	fprintf(self->kml_file, "        <coordinates>\n");
}


static void kml_coord(struct vmeta_extract *self, struct vmeta_location *loc)
{
	if (loc->valid) {
		fprintf(self->kml_file,
			"          %.8f,%.8f,%.3f\n",
			loc->longitude,
			loc->latitude,
			loc->altitude);
	}
}


static void kml_footer(struct vmeta_extract *self)
{
	fprintf(self->kml_file, "        </coordinates>\n");
	fprintf(self->kml_file, "      </LineString>\n");
	fprintf(self->kml_file, "      <Style>\n");
	fprintf(self->kml_file, "        <LineStyle>\n");
	fprintf(self->kml_file, "          <color>#ff0000ff</color>\n");
	fprintf(self->kml_file, "          <width>3</width>\n");
	fprintf(self->kml_file, "        </LineStyle>\n");
	fprintf(self->kml_file, "      </Style>\n");
	fprintf(self->kml_file, "    </Placemark>\n");
	fprintf(self->kml_file, "  </Document>\n");
	fprintf(self->kml_file, "</kml>\n");
}


static int frame_extract(struct vmeta_extract *self,
			 struct vmeta_buffer *buf,
			 uint64_t ts,
			 const char *mime_format)
{
	int ret = 0;
	struct vmeta_frame *meta;

	ret = vmeta_frame_read2(buf, mime_format, 0, &meta);
	if (ret < 0) {
		ULOG_ERRNO("vmeta_frame_read2", -ret);
		return ret;
	}

	/* CSV output */
	if (self->csv_file) {
		ssize_t err;
		size_t len = 0;
		char *str = malloc(STR_SIZE);
		if (str == NULL) {
			ret = -ENOMEM;
			ULOG_ERRNO("malloc(%d)", -ret, STR_SIZE);
			goto out;
		}
		if (self->type_for_csv == VMETA_FRAME_TYPE_NONE) {
			self->type_for_csv =
				(meta->type ==
				 VMETA_FRAME_TYPE_V1_STREAMING_BASIC)
					? VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED
					: meta->type;
			vmeta_frame_csv_header(
				self->type_for_csv, str, STR_SIZE);
			fprintf(self->csv_file, "time %s\n", str);
			str[0] = '\0';
		} else if ((self->type_for_csv != meta->type) &&
			   (meta->type !=
			    VMETA_FRAME_TYPE_V1_STREAMING_BASIC)) {
			ULOGE("unsupported change of metadata "
			      "type (found:%s expected:%s)\n",
			      vmeta_frame_type_str(meta->type),
			      vmeta_frame_type_str(self->type_for_csv));
			free(str);
			ret = -EPROTO;
			goto out;
		}

		err = vmeta_frame_to_csv(meta, str, STR_SIZE);
		if (err < 0) {
			ULOG_ERRNO("vmeta_frame_to_csv", (int)-err);
			free(str);
			ret = err;
			goto out;
		}
		len += err;
		fprintf(self->csv_file, "%" PRIu64 " %s\n", ts, str);
		free(str);
	}

	/* KML output */
	if (self->kml_file) {
		struct vmeta_location loc;

		if (self->type_for_kml == VMETA_FRAME_TYPE_NONE) {
			self->type_for_kml =
				(meta->type ==
				 VMETA_FRAME_TYPE_V1_STREAMING_BASIC)
					? VMETA_FRAME_TYPE_V1_STREAMING_EXTENDED
					: meta->type;
			kml_header(
				self,
				((self->type_for_kml == VMETA_FRAME_TYPE_V2) ||
				 (self->type_for_kml == VMETA_FRAME_TYPE_V3))
					? 1
					: 0);
		} else if ((self->type_for_kml != meta->type) &&
			   (meta->type !=
			    VMETA_FRAME_TYPE_V1_STREAMING_BASIC)) {
			ULOGE("unsupported change of metadata "
			      "type (found:%s expected:%s)\n",
			      vmeta_frame_type_str(meta->type),
			      vmeta_frame_type_str(self->type_for_kml));
			ret = -ENOSYS;
			goto out;
		}

		ret = vmeta_frame_get_location(meta, &loc);
		if (ret == 0)
			kml_coord(self, &loc);
	}

	/* JSON output */
	if (self->json_file_name) {
		json_object *jobj = json_object_new_object();
		json_object *jobj_meta = json_object_new_object();

		ret = vmeta_frame_to_json(meta, jobj_meta);
		if (ret < 0) {
			ULOG_ERRNO("vmeta_frame_to_json", -ret);
			json_object_put(jobj);
			json_object_put(jobj_meta);
			goto out;
		}

		json_object_object_add(jobj, "time", json_object_new_int64(ts));
		json_object_object_add(jobj, "metadata", jobj_meta);

		json_object *jarray;
		if (json_object_object_get_ex(
			    self->json_data, "frame", &jarray))
			json_object_array_add(jarray, jobj);
		else
			json_object_put(jobj);
	}

	self->is_first = 0;

out:
	vmeta_frame_unref(meta);
	return ret;
}


static int session_output(struct vmeta_extract *self)
{
	int ret;
	char session_meta_str[1000];

	ret = vmeta_session_to_str(&self->session_meta,
				   session_meta_str,
				   sizeof(session_meta_str));
	if (ret < 0) {
		ULOG_ERRNO("vmeta_session_to_str", -ret);
		return ret;
	}
	printf("%s\n", session_meta_str);

	/* JSON output */
	if (self->json_file_name) {
		json_object *jobj = json_object_new_object();

		ret = vmeta_session_to_json(&self->session_meta, jobj);
		if (ret < 0) {
			ULOG_ERRNO("vmeta_session_to_json", -ret);
			json_object_put(jobj);
			return ret;
		}
		json_object_object_add(self->json_data, "session", jobj);
	}

	return 0;
}


static int raw_extract(struct vmeta_extract *self)
{
	int ret = 0, err;
	FILE *in_file = NULL;
	uint8_t data[BUF_SIZE];
	size_t size = 0;
	struct vmeta_buffer buf;
	uint64_t ts = 0, ts_inc = 33333;
	char *mime_format = NULL;

	/* JSON output */
	if (self->json_file_name) {
		json_object *jarray = json_object_new_array();
		json_object_object_add(self->json_data, "frame", jarray);
	}

	/* Open the input file */
	in_file = fopen(self->input_file_name, "r");
	if (in_file == NULL) {
		fprintf(stderr,
			"failed to open input file '%s'\n",
			self->input_file_name);
		ret = -errno;
		goto cleanup;
	}

	do {
		/* Get the metadata type and size */
		uint32_t val32 = 0;
		err = fread(&val32, 4, 1, in_file);
		if (err != 1) {
			if (!feof(in_file)) {
				ret = -EIO;
				ULOG_ERRNO("fread", -ret);
			}
			break;
		}
		val32 = ntohl(val32);
		uint16_t id = val32 >> 16;
		uint16_t sz = val32 & 0xFFFF;
		if ((id == VMETA_FRAME_V1_STREAMING_ID) &&
		    (sz == (VMETA_FRAME_V1_STREAMING_BASIC_SIZE - 4) / 4)) {
			size = VMETA_FRAME_V1_STREAMING_BASIC_SIZE;
		} else if ((id == VMETA_FRAME_V1_STREAMING_ID) &&
			   (sz ==
			    (VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE - 4) / 4)) {
			size = VMETA_FRAME_V1_STREAMING_EXTENDED_SIZE;
		} else if (id == VMETA_FRAME_V2_BASE_ID) {
			size = sz * 4 + 4;
		} else if (id == VMETA_FRAME_V3_BASE_ID) {
			size = sz * 4 + 4;
		} else {
			size = VMETA_FRAME_V1_RECORDING_SIZE;
		}

		/* Read the data */
		err = fseek(in_file, -4, SEEK_CUR);
		if (err != 0) {
			ret = -errno;
			ULOG_ERRNO("fseek", -ret);
			break;
		}
		err = fread(data, size, 1, in_file);
		if (err != 1) {
			ret = -EIO;
			ULOG_ERRNO("fread", -ret);
			break;
		}

		/* Process the data */
		vmeta_buffer_set_cdata(&buf, data, size, 0);
		err = frame_extract(self, &buf, ts, mime_format);
		if (err != 0) {
			ret = err;
			break;
		}
		ts += ts_inc;
	} while (!feof(in_file));

cleanup:
	if (in_file)
		fclose(in_file);

	return ret;
}


#ifdef BUILD_LIBMP4

static int mp4_extract(struct vmeta_extract *self)
{
	struct mp4_demux *demux = NULL;
	struct mp4_track_info tk;
	struct mp4_track_sample sample;
	int ret = 0, i, count, err, found = 0, meta_found = 0;
	unsigned int id, session_meta_count, k;
	uint8_t data[BUF_SIZE];
	struct vmeta_buffer buf;
	char *mime_format = NULL;
	char **keys = NULL;
	char **values = NULL;

	/* Read the MP4 file */
	ret = mp4_demux_open(self->input_file_name, &demux);
	if (ret < 0) {
		fprintf(stderr,
			"failed to read MP4 file '%s'\n",
			self->input_file_name);
		goto cleanup;
	}

	/* Get the session metadata */
	memset(&self->session_meta, 0, sizeof(self->session_meta));
	err = mp4_demux_get_metadata_strings(
		demux, &session_meta_count, &keys, &values);
	if (err < 0) {
		ULOG_ERRNO("mp4_demux_get_metadata_strings", -err);
		goto cleanup;
	}
	for (k = 0; k < session_meta_count; k++) {
		char *key = keys[k];
		char *value = values[k];
		if ((key) && (value)) {
			err = vmeta_session_recording_read(
				key, value, &self->session_meta);
			if (err < 0) {
				ULOG_ERRNO("vmeta_session_recording_read",
					   -err);
				continue;
			}
		}
	}
	err = session_output(self);
	if (err < 0)
		goto cleanup;

	/* Find the video+metadata track */
	count = mp4_demux_get_track_count(demux);

	for (i = 0; i < count; i++) {
		err = mp4_demux_get_track_info(demux, i, &tk);
		if ((err == 0) && (tk.type == MP4_TRACK_TYPE_VIDEO)) {
			if ((tk.has_metadata) && (tk.metadata_mime_format)) {
				mime_format = strdup(tk.metadata_mime_format);
				meta_found = 1;
			}
			id = tk.id;
			found = 1;
			break;
		}
	}

	if (!found) {
		fprintf(stderr,
			"unable to find a video track in file '%s'\n",
			self->input_file_name);
		ret = -ENOENT;
		goto cleanup;
	}

	if (!meta_found)
		goto cleanup;

	/* JSON output */
	if (self->json_file_name) {
		json_object *jarray = json_object_new_array();
		json_object_object_add(self->json_data, "frame", jarray);
	}

	/* Get the samples and process them */
	i = 0;
	do {
		err = mp4_demux_get_track_sample(
			demux, id, 1, NULL, 0, data, BUF_SIZE, &sample);
		if (err < 0) {
			ULOG_ERRNO("mp4_demux_get_track_sample", -err);
			continue;
		}
		if (sample.metadata_size == 0)
			continue;
		vmeta_buffer_set_cdata(&buf, data, sample.metadata_size, 0);
		err = frame_extract(
			self,
			&buf,
			mp4_sample_time_to_usec(sample.dts, tk.timescale),
			mime_format);
		if (err < 0) {
			ret = err;
			break;
		}
		i++;
	} while (sample.size);

cleanup:
	if (demux)
		mp4_demux_close(demux);
	free(mime_format);

	return ret;
}

#endif /* BUILD_LIBMP4 */


#ifdef BUILD_LIBPCAP

/* Ethernet header */
#	define ETHER_ADDR_LEN 6
struct vmeta_ethernet {
	/* Destination host address */
	uint8_t dhost[ETHER_ADDR_LEN];

	/* Source host address */
	uint8_t shost[ETHER_ADDR_LEN];

	/* IP, ARP, RARP, etc. */
	uint16_t type;
};
#	define ETHER_HEADER_LEN 14

/* IP header */
struct vmeta_ip {
	/* Version << 4 | header length >> 2 */
	uint8_t vhl;

	/* Type of service */
	uint8_t tos;

	/* Total length */
	uint16_t len;

	/* Identification */
	uint16_t id;

	/* Fragment offset field */
	uint16_t off;

	/* Time to live */
	uint8_t ttl;

	/* Protocol */
	uint8_t proto;

	/* Checksum */
	uint16_t sum;

	/* Source address */
	uint32_t src;

	/* Destination address */
	uint32_t dst;
};
#	define IP_HEADER_LEN 20
#	define IP_HEADER_LENGTH_SHIFT 0
#	define IP_HEADER_LENGTH_MASK 0x0f

/* UDP header */
struct vmeta_udp {
	/* Source port */
	uint16_t sport;

	/* Destination port */
	uint16_t dport;

	/* Total length */
	uint16_t len;

	/* Checksum */
	uint16_t sum;
};
#	define UDP_HEADER_LEN 8

/* RTP header */
struct vmeta_rtp {
	/* Version, padding, extension, CSRC count, marker, payload type */
	uint16_t flags;

	/* Sequence number */
	uint16_t seqnum;

	/* RTP timestamp */
	uint32_t timestamp;

	/* Synchronization source (SSRC) identifier */
	uint32_t ssrc;
};
#	define RTP_HEADER_LEN 12
#	define RTP_HEADER_FLAGS_EXTENSION_SHIFT 12
#	define RTP_HEADER_FLAGS_EXTENSION_MASK 0x0001
#	define RTP_HEADER_FLAGS_CSRC_COUNT_SHIFT 8
#	define RTP_HEADER_FLAGS_CSRC_COUNT_MASK 0x000f

struct vmeta_rtcp {
	uint8_t flags;
	uint8_t type;
	uint16_t len;
};
#	define RTCP_HEADER_LEN 4
#	define RTCP_HEADER_FLAGS_COUNT_SHIFT 0
#	define RTCP_HEADER_FLAGS_COUNT_MASK 0x1f
#	define RTCP_TYPE_SDES 202
#	define RTCP_SDES_TYPE_END 0
#	define RTCP_SDES_TYPE_PRIV 8


static int read_sdes_item(const uint8_t *buf,
			  size_t len,
			  size_t *pos,
			  uint8_t type,
			  struct vmeta_session *meta)
{
	int res = 0;

	/* Read item type */
	if (*pos + 1 >= len)
		return -EIO;
	size_t item_len = *(buf + *pos);
	*pos += 1;

	if (item_len > len - *pos)
		return -EIO;

	if (item_len != 0) {
		char value[200];
		char prefix[50];
		value[0] = '\0';
		prefix[0] = '\0';
		if (type == VMETA_STRM_SDES_TYPE_PRIV) {
			size_t prefix_len = *(buf + *pos);
			*pos += 1;
			item_len--;
			if (prefix_len > item_len)
				return -EIO;
			if (prefix_len < sizeof(prefix)) {
				memcpy(prefix, buf + *pos, prefix_len);
				*pos += prefix_len;
				prefix[prefix_len] = '\0';
				memcpy(value,
				       buf + *pos,
				       item_len - prefix_len);
				*pos += item_len - prefix_len;
				value[item_len - prefix_len] = '\0';
			}
		} else if (item_len < sizeof(value)) {
			memcpy(value, buf + *pos, item_len);
			*pos += item_len;
			value[item_len] = '\0';
		}

		if (strlen(value) > 0) {
			res = vmeta_session_streaming_sdes_read(
				(enum vmeta_stream_sdes_type)type,
				value,
				prefix,
				meta);
			if (res < 0) {
				ULOG_ERRNO("vmeta_session_streaming_sdes_read",
					   -res);
			}
		}
	}

	return res;
}


static int read_sdes_chunk(const uint8_t *buf,
			   size_t len,
			   size_t *pos,
			   struct vmeta_session *meta)
{
	int res = 0;
	uint8_t type = 0;

	/* SSRC */
	if (*pos + 4 >= len)
		return -EIO;
	*pos += 4;

	while (*pos < len) {
		/* Read item type */
		if (*pos + 1 > len)
			return -EIO;

		type = *(buf + *pos);
		*pos += 1;

		if (type == VMETA_STRM_SDES_TYPE_END)
			break;

		/* Rewind and read item */
		res = read_sdes_item(buf, len, pos, type, meta);
		if (res != 0)
			break;
	}

	/* Align */
	while (*pos < len && *pos % 4 != 0)
		*pos += 1;

	return res;
}

static int read_sdes(const uint8_t *buf,
		     size_t len,
		     size_t *pos,
		     const struct vmeta_rtcp *rtcp,
		     struct vmeta_session *meta)
{
	int res = 0;
	uint8_t chunk_count = 0, i;

	chunk_count = (rtcp->flags >> RTCP_HEADER_FLAGS_COUNT_SHIFT) &
		      RTCP_HEADER_FLAGS_COUNT_MASK;
	*pos += RTCP_HEADER_LEN;

	for (i = 0; i < chunk_count; i++) {
		res = read_sdes_chunk(buf, len, pos, meta);
		if (res != 0)
			break;
	}

	return res;
}


static void pcap_packet_cb(u_char *args,
			   const struct pcap_pkthdr *header,
			   const u_char *packet)
{
	struct vmeta_extract *self = (struct vmeta_extract *)args;
	const struct vmeta_ip *ip =
		(const struct vmeta_ip *)(packet + ETHER_HEADER_LEN);
	const struct vmeta_udp *udp =
		(const struct vmeta_udp *)(packet + ETHER_HEADER_LEN +
					   IP_HEADER_LEN);
	const struct vmeta_rtp *rtp =
		(const struct vmeta_rtp *)(packet + ETHER_HEADER_LEN +
					   IP_HEADER_LEN + UDP_HEADER_LEN);
	const uint8_t *payload =
		(const uint8_t *)(packet + ETHER_HEADER_LEN + IP_HEADER_LEN +
				  UDP_HEADER_LEN + RTP_HEADER_LEN);
	const struct vmeta_rtcp *rtcp =
		(const struct vmeta_rtcp *)(packet + ETHER_HEADER_LEN +
					    IP_HEADER_LEN + UDP_HEADER_LEN);
	struct vmeta_buffer buf;
	int ret;

	/* Check the packet validity */
	if (header->caplen < header->len) {
		ULOGE("truncated packet (%d vs. %d bytes)",
		      header->caplen,
		      header->len);
		return;
	}
	if (header->len < ETHER_HEADER_LEN + IP_HEADER_LEN + UDP_HEADER_LEN) {
		ULOGE("invalid packet size: %d", header->len);
		return;
	}
	int ip_header_len =
		((ip->vhl >> IP_HEADER_LENGTH_SHIFT) & IP_HEADER_LENGTH_MASK) *
		4;
	if (ip_header_len < IP_HEADER_LEN) {
		ULOGE("invalid IP header length: %d", ip_header_len);
		return;
	}
	if (ntohs(udp->len) < UDP_HEADER_LEN) {
		ULOGE("invalid UDP length: %d", ntohs(udp->len));
		return;
	}

	uint16_t dport = ntohs(udp->dport);
	if (dport == self->rtp_port) {
		if (header->len < ETHER_HEADER_LEN + IP_HEADER_LEN +
					  UDP_HEADER_LEN + RTP_HEADER_LEN) {
			ULOGE("invalid RTP packet size: %d", header->len);
			return;
		}
		if (((ntohs(rtp->flags) >> RTP_HEADER_FLAGS_EXTENSION_SHIFT) &
		     RTP_HEADER_FLAGS_EXTENSION_MASK) != 1)
			return;
		/* Find metadata in RTP header extensions */
		int cc = ((ntohs(rtp->flags) >>
			   RTP_HEADER_FLAGS_CSRC_COUNT_SHIFT) &
			  RTP_HEADER_FLAGS_CSRC_COUNT_MASK);
		payload += cc * 4;
		size_t ext_len = ntohs(*((const uint16_t *)payload + 1));
		ext_len = ext_len * 4 + 4;

		/* Compute a monotonic timestamp in microseconds
		 * from the RTP timestamp (90000kHz 32 bits) */
		uint32_t rtp_ts = ntohl(rtp->timestamp);
		uint64_t ext_rtp_ts =
			(self->highest_ext_rtp_ts & 0xFFFFFFFF00000000ULL) |
			((uint64_t)rtp_ts & 0xFFFFFFFFULL);
		if ((int64_t)ext_rtp_ts - (int64_t)self->prev_ext_rtp_ts <
		    -2147483648LL)
			ext_rtp_ts += 0x100000000ULL;
		else if ((int64_t)ext_rtp_ts - (int64_t)self->prev_ext_rtp_ts >
			 2147483648LL)
			ext_rtp_ts -= 0x100000000ULL;
		if (ext_rtp_ts > self->highest_ext_rtp_ts)
			self->highest_ext_rtp_ts = ext_rtp_ts;
		uint64_t ts = (ext_rtp_ts + 90000 / 2) * 1000000 / 90000;

		/* Process the data */
		vmeta_buffer_set_cdata(&buf, payload, ext_len, 0);
		ret = frame_extract(self, &buf, ts, NULL);
		if (ret < 0)
			return;
	} else if (dport == self->rtcp_port) {
		if (header->len < ETHER_HEADER_LEN + IP_HEADER_LEN +
					  UDP_HEADER_LEN + RTCP_HEADER_LEN) {
			ULOGE("invalid RTCP packet size: %d", header->len);
			return;
		}
		const uint8_t *p = (const uint8_t *)rtcp;
		size_t offset = 0, len;
		while (offset + 4 <= header->len - ETHER_HEADER_LEN -
					     IP_HEADER_LEN - UDP_HEADER_LEN) {
			rtcp = (const struct vmeta_rtcp *)p;
			len = ntohs(rtcp->len) * 4 + 4;
			if (rtcp->type == RTCP_TYPE_SDES) {
				size_t pos = 0;
				ret = read_sdes(p,
						len,
						&pos,
						rtcp,
						&self->session_meta);
				if (ret < 0) {
					ULOG_ERRNO("read_sdes", -ret);
					return;
				}
			}
			p += len;
			offset += len;
		}
	}
}


static int pcap_extract(struct vmeta_extract *self)
{
	int ret = 0, err;
	char errbuf[PCAP_ERRBUF_SIZE];
	pcap_t *pcap;
	int datalink;
	struct bpf_program fp;
	char filter_exp[50];

	/* Open the capture file */
	pcap = pcap_open_offline(self->input_file_name, errbuf);
	if (pcap == NULL) {
		fprintf(stderr,
			"failed to open PCAP file '%s' (error: %s)\n",
			self->input_file_name,
			errbuf);
		ret = -ENOENT;
		goto cleanup;
	}

	/* Check that the link-layer header type is Ethernet */
	datalink = pcap_datalink(pcap);
	if (datalink != DLT_EN10MB) {
		fprintf(stderr,
			"unsupported network type in PCAP file '%s'\n",
			self->input_file_name);
		ret = -ENOSYS;
		goto cleanup;
	}

	/* Set the capture filter */
	snprintf(filter_exp,
		 sizeof(filter_exp),
		 "udp dst port %d or udp dst port %d",
		 self->rtcp_port,
		 self->rtp_port);
	err = pcap_compile(pcap, &fp, filter_exp, 0, PCAP_NETMASK_UNKNOWN);
	if (err == -1) {
		ULOGE("failed to compile filter for PCAP file (error: %s)",
		      pcap_geterr(pcap));
		ret = -EPROTO;
		goto cleanup;
	}
	err = pcap_setfilter(pcap, &fp);
	if (err == -1) {
		ULOGE("failed to install filter for PCAP file (error: %s)",
		      pcap_geterr(pcap));
		ret = -EPROTO;
		goto cleanup;
	}

	memset(&self->session_meta, 0, sizeof(self->session_meta));

	/* JSON output */
	if (self->json_file_name) {
		json_object *jarray = json_object_new_array();
		json_object_object_add(self->json_data, "frame", jarray);
	}

	/* Filter and process packets */
	err = pcap_loop(pcap, 0, pcap_packet_cb, (u_char *)self);
	if (err == -1) {
		ULOGE("failed to process packets in PCAP file (error: %s)",
		      pcap_geterr(pcap));
		ret = -ENOENT;
		goto cleanup;
	}

	err = session_output(self);
	if (err < 0)
		goto cleanup;

cleanup:
	if (pcap)
		pcap_close(pcap);

	return ret;
}

#endif /* BUILD_LIBPCAP */


static const char short_options[] = "h";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"csv", required_argument, NULL, ARGS_ID_CSV},
	{"kml", required_argument, NULL, ARGS_ID_KML},
	{"json", required_argument, NULL, ARGS_ID_JSON},
	{"pretty", no_argument, NULL, ARGS_ID_JSON_PRETTY},
	{"rtp-port", required_argument, NULL, ARGS_ID_RTP_PORT},
	{"rtcp-port", required_argument, NULL, ARGS_ID_RTCP_PORT},
	{0, 0, 0, 0},
};


static void welcome(char *prog_name)
{
	printf("\n%s - Parrot Drones video metadata extractor tool\n"
	       "Copyright (c) 2016 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	printf("Usage: %s [options] <input file>\n"
	       "\n"
	       "Options:\n"
	       "-h | --help                        Print this message\n"
	       "     --csv  <file>                 Output to CSV file\n"
	       "     --kml  <file>                 Output to KML file\n"
	       "     --json <file>                 Output to JSON file\n"
	       "     --pretty                      Pretty output for "
	       "JSON file\n"
#ifdef BUILD_LIBPCAP
	       "     --rtp-port <port>             RTP destination port "
	       "(.pcap files only, default is 55004)\n"
	       "     --rctp-port <port>            RTCP destination port "
	       "(.pcap files only, default is 55005)\n"
#endif /* BUILD_LIBPCAP */
	       "\n"
	       "Supported input files: "
#ifdef BUILD_LIBMP4
	       ".mp4, "
#endif /* BUILD_LIBMP4 */
#ifdef BUILD_LIBPCAP
	       ".pcap, "
#endif /* BUILD_LIBPCAP */
	       "raw metadata\n"
	       "\n",
	       prog_name);
}


int main(int argc, char *argv[])
{
	int status = EXIT_SUCCESS;
	int idx, c, ret;
	struct vmeta_extract *self;

	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		exit(EXIT_FAILURE);
	}
	self->type_for_csv = VMETA_FRAME_TYPE_NONE;
	self->type_for_kml = VMETA_FRAME_TYPE_NONE;
	self->is_first = 1;
	self->rtp_port = RTP_DEFAULT_PORT;
	self->rtcp_port = RTCP_DEFAULT_PORT;

	welcome(argv[0]);

	if (argc < 2) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
			break;

		case ARGS_ID_CSV:
			self->csv_file_name = optarg;
			break;

		case ARGS_ID_KML:
			self->kml_file_name = optarg;
			break;

		case ARGS_ID_JSON:
			self->json_file_name = optarg;
			self->json_data = json_object_new_object();
			break;

		case ARGS_ID_JSON_PRETTY:
			self->json_pretty = 1;
			break;

		case ARGS_ID_RTP_PORT:
			self->rtp_port = atoi(optarg);
			break;

		case ARGS_ID_RTCP_PORT:
			self->rtcp_port = atoi(optarg);
			break;

		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
			break;
		}
	}

	if (argc - optind < 1) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	self->input_file_name = argv[optind];

	if (self->input_file_name == NULL) {
		fprintf(stderr, "no input file provided\n");
		status = EXIT_FAILURE;
		goto cleanup;
	}

	if (self->csv_file_name) {
		self->csv_file = fopen(self->csv_file_name, "w");
		if (self->csv_file == NULL) {
			fprintf(stderr,
				"failed to open CSV file '%s'\n",
				self->csv_file_name);
			status = EXIT_FAILURE;
			goto cleanup;
		}
	}

	if (self->kml_file_name) {
		self->kml_file = fopen(self->kml_file_name, "w");
		if (self->kml_file == NULL) {
			fprintf(stderr,
				"failed to open KML file '%s'\n",
				self->kml_file_name);
			status = EXIT_FAILURE;
			goto cleanup;
		}
	}

	if (!strncasecmp(self->input_file_name + strlen(self->input_file_name) -
				 4,
			 ".mp4",
			 4)) {
#ifdef BUILD_LIBMP4
		/* .mp4 file input */
		ret = mp4_extract(self);
		if (ret != 0) {
			status = EXIT_FAILURE;
			goto cleanup;
		}
#else /* BUILD_LIBMP4 */
		fprintf(stderr,
			"MP4 files are not supported; "
			"please rebuild with libmp4 enabled\n");
		status = EXIT_FAILURE;
		goto cleanup;
#endif /* BUILD_LIBMP4 */
	} else if (!strncasecmp(self->input_file_name +
					strlen(self->input_file_name) - 5,
				".pcap",
				5)) {
#ifdef BUILD_LIBPCAP
		/* .pcap file input */
		ret = pcap_extract(self);
		if (ret != 0) {
			status = EXIT_FAILURE;
			goto cleanup;
		}
#else /* BUILD_LIBPCAP */
		fprintf(stderr,
			"PCAP files are not supported; "
			"please rebuild with libpcap enabled\n");
		status = EXIT_FAILURE;
		goto cleanup;
#endif /* BUILD_LIBPCAP */
	} else {
		/* Raw metadata file input */
		ret = raw_extract(self);
		if (ret != 0) {
			status = EXIT_FAILURE;
			goto cleanup;
		}
	}
	if (self->json_file_name) {
		ret = json_object_to_file_ext(
			self->json_file_name,
			self->json_data,
			(self->json_pretty) ? JSON_C_TO_STRING_PRETTY : 0);
		if (ret != 0) {
			status = EXIT_FAILURE;
			ULOG_ERRNO("json_object_to_file_ext", EPROTO);
			goto cleanup;
		}
	}

cleanup:
	if (self->csv_file)
		fclose(self->csv_file);
	if (self->kml_file) {
		if (!self->is_first)
			kml_footer(self);
		fclose(self->kml_file);
	}
	if (self->json_file_name)
		json_object_put(self->json_data);

	printf("%s\n", (status == EXIT_SUCCESS) ? "Done!" : "Failed!");
	exit(status);
}
