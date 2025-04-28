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


#define COPY_VALUE(_dst, _src)                                                 \
	{                                                                      \
		strncpy(_dst, _src, sizeof(_dst));                             \
		_dst[sizeof(_dst) - 1] = '\0';                                 \
	}


#define MERGE_META_STR(_dst, _src, _field)                                     \
	{                                                                      \
		if (strlen(_dst->_field) != strlen(_src->_field)) {            \
			_dst->_field[0] = '\0';                                \
		}                                                              \
		if (strncmp(_dst->_field,                                      \
			    _src->_field,                                      \
			    strlen(_src->_field)) != 0) {                      \
			_dst->_field[0] = '\0';                                \
		}                                                              \
	}


#define REMOVE_DUPLICATE_META_STR(_dst, _src, _field)                          \
	{                                                                      \
		if (_src->_field[0] != '\0') {                                 \
			_dst->_field[0] = '\0';                                \
		}                                                              \
	}


#define MERGE_META_VAL(_dst, _src, _field, _v)                                 \
	{                                                                      \
		if (_dst->_field != _src->_field) {                            \
			_dst->_field = _v;                                     \
		}                                                              \
	}


#define REMOVE_DUPLICATE_META_VAL(_dst, _src, _field, _v)                      \
	{                                                                      \
		if (_src->_field != _v) {                                      \
			_dst->_field = _v;                                     \
		}                                                              \
	}


#define MERGE_META_PTR(_dst, _src, _field, _size)                              \
	{                                                                      \
		if (memcmp(_dst->_field, _src->_field, _size) != 0) {          \
			memset(_dst->_field, 0, _size);                        \
		}                                                              \
	}


#define REMOVE_DUPLICATE_META_PTR(_dst, _src, _field, _size)                   \
	{                                                                      \
		if (memcmp(_dst->_field, _src->_field, _size) == 0) {          \
			memset(_dst->_field, 0, _size);                        \
		}                                                              \
	}


#define CMP_FIELD_VAL(_dst, _src, _field)                                      \
	{                                                                      \
		if (_dst->_field != _src->_field)                              \
			return 0;                                              \
	}


#define CMP_FIELD_VAL_NAN_ALLOWED(_dst, _src, _field)                          \
	{                                                                      \
		if (isnan(_dst->_field) != isnan(_src->_field))                \
			return 0;                                              \
		if (!isnan(_dst->_field) && _dst->_field != _src->_field)      \
			return 0;                                              \
	}


#define CMP_FIELD_PTR(_dst, _src, _field)                                      \
	{                                                                      \
		if (memcmp(&_dst->_field,                                      \
			   &_src->_field,                                      \
			   sizeof(_dst->_field)) != 0) {                       \
			return 0;                                              \
		}                                                              \
	}


#define CMP_FIELD_STR(_dst, _src, _field)                                      \
	{                                                                      \
		if (strlen(_dst->_field) != strlen(_src->_field)) {            \
			return 0;                                              \
		}                                                              \
		if (strncmp(_dst->_field,                                      \
			    _src->_field,                                      \
			    strlen(_dst->_field)) != 0) {                      \
			return 0;                                              \
		}                                                              \
	}


ssize_t
vmeta_session_date_write(char *str, size_t len, uint64_t date, long gmtoff)
{
	int ret;
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	ret = time_local_format(date, gmtoff, TIME_FMT_LONG, str, len);
	if (ret < 0)
		return ret;

	return (ssize_t)strlen(str);
}


int vmeta_session_date_read(const char *str, uint64_t *date, long *gmtoff)
{
	int ret;
	uint64_t epoch_sec;
	int32_t utc_off_sec;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(date == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(gmtoff == NULL, EINVAL);

	ret = time_local_parse(str, &epoch_sec, &utc_off_sec);
	if (ret < 0)
		return ret;

	*date = epoch_sec;
	*gmtoff = utc_off_sec;
	return 0;
}


ssize_t vmeta_session_location_write(char *str,
				     size_t len,
				     enum vmeta_session_location_format format,
				     const struct vmeta_location *loc)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);

	if (!loc->valid)
		return 0;

	switch (format) {
	default:
	case VMETA_SESSION_LOCATION_CSV:
		ret = snprintf(str,
			       len,
			       VMETA_SESSION_LOCATION_FORMAT_CSV,
			       loc->latitude,
			       loc->longitude,
			       loc->altitude_egm96amsl);
		break;
	case VMETA_SESSION_LOCATION_ISO6709:
		ret = snprintf(str,
			       len,
			       VMETA_SESSION_LOCATION_FORMAT_ISO6709,
			       loc->latitude,
			       loc->longitude,
			       loc->altitude_egm96amsl);
		break;
	case VMETA_SESSION_LOCATION_XYZ:
		ret = snprintf(str,
			       len,
			       VMETA_SESSION_LOCATION_FORMAT_XYZ,
			       loc->latitude,
			       loc->longitude);
		break;
	}

	return (ssize_t)ret;
}


int vmeta_session_location_read(const char *str, struct vmeta_location *loc)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(loc == NULL, EINVAL);

	memset(loc, 0, sizeof(*loc));
	loc->latitude = 500.;
	loc->longitude = 500.;
	loc->altitude_wgs84ellipsoid = NAN;
	loc->altitude_egm96amsl = NAN;
	loc->sv_count = VMETA_LOCATION_INVALID_SV_COUNT;

	if (strchr(str, ',')) {
		/* CSV format */
		ret = sscanf(str,
			     "%lf,%lf,%lf",
			     &loc->latitude,
			     &loc->longitude,
			     &loc->altitude_egm96amsl);
		if (ret == 3)
			loc->valid = 1;
	} else {
		/* ISO 6709 Annex H string or Android-compatible modified
		 * ISO 6709 Annex H string format */
		const char *p1;
		char *p2;
		p1 = str;
		loc->latitude = strtod(p1, &p2);
		if (p2) {
			p1 = p2;
			loc->longitude = strtod(p1, &p2);
		}
		if (p2) {
			p1 = p2;
			loc->altitude_egm96amsl = strtod(p1, &p2);
		}
	}

	vmeta_location_adjust_read(loc, loc);

	return 0;
}


ssize_t
vmeta_session_fov_write(char *str, size_t len, const struct vmeta_fov *fov)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fov == NULL, EINVAL);

	if ((!fov->has_horz) || (!fov->has_vert))
		return 0;

	ret = snprintf(
		str, len, VMETA_SESSION_FOV_FORMAT, fov->horz, fov->vert);

	return (ssize_t)ret;
}


int vmeta_session_fov_read(const char *str, struct vmeta_fov *fov)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(fov == NULL, EINVAL);

	fov->has_horz = 0;
	fov->has_vert = 0;

	ret = sscanf(str, "%f,%f", &fov->horz, &fov->vert);
	if (ret == 2) {
		fov->has_horz = 1;
		fov->has_vert = 1;
	}

	return 0;
}


ssize_t vmeta_session_perspective_distortion_write(char *str,
						   size_t len,
						   float r1,
						   float r2,
						   float r3,
						   float t1,
						   float t2)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	ret = snprintf(str,
		       len,
		       VMETA_SESSION_PERSPECTIVE_DISTORTION_FORMAT,
		       r1,
		       r2,
		       r3,
		       t1,
		       t2);

	return (ssize_t)ret;
}


int vmeta_session_perspective_distortion_read(const char *str,
					      float *r1,
					      float *r2,
					      float *r3,
					      float *t1,
					      float *t2)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(r1 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(r2 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(r3 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(t1 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(t2 == NULL, EINVAL);

	ret = sscanf(str, "%f,%f,%f,%f,%f", r1, r2, r3, t1, t2);

	if (ret != 5)
		return -EPROTO;

	return 0;
}


ssize_t vmeta_session_fisheye_affine_matrix_write(char *str,
						  size_t len,
						  float c,
						  float d,
						  float e,
						  float f)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	ret = snprintf(str,
		       len,
		       VMETA_SESSION_FISHEYE_AFFINE_MATRIX_FORMAT,
		       c,
		       d,
		       e,
		       f);

	return (ssize_t)ret;
}


int vmeta_session_fisheye_affine_matrix_read(const char *str,
					     float *c,
					     float *d,
					     float *e,
					     float *f)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(c == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(d == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(e == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(f == NULL, EINVAL);

	ret = sscanf(str, "%f,%f,%f,%f", c, d, e, f);

	if (ret != 4)
		return -EPROTO;

	return 0;
}


ssize_t vmeta_session_fisheye_polynomial_write(char *str,
					       size_t len,
					       float p2,
					       float p3,
					       float p4)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	ret = snprintf(
		str, len, VMETA_SESSION_FISHEYE_POLYNOMIAL_FORMAT, p2, p3, p4);

	return (ssize_t)ret;
}


int vmeta_session_fisheye_polynomial_read(const char *str,
					  float *p2,
					  float *p3,
					  float *p4)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(p2 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(p3 == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(p4 == NULL, EINVAL);

	ret = sscanf(str, "0,1,%f,%f,%f", p2, p3, p4);

	if (ret != 3)
		return -EPROTO;

	return 0;
}


ssize_t vmeta_session_overlay_header_footer_write(char *str,
						  size_t len,
						  float header_height,
						  float footer_height)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	ret = snprintf(str,
		       len,
		       VMETA_SESSION_OVERLAY_HEADER_FOOTER_FORMAT,
		       header_height,
		       footer_height);

	return (ssize_t)ret;
}


int vmeta_session_overlay_header_footer_read(const char *str,
					     float *header_height,
					     float *footer_height)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(header_height == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(footer_height == NULL, EINVAL);

	ret = sscanf(str, "%f,%f", header_height, footer_height);

	if (ret != 2)
		return -EPROTO;

	return 0;
}


ssize_t vmeta_session_thermal_alignment_write(
	char *str,
	size_t len,
	const struct vmeta_thermal_alignment *align)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(align == NULL, EINVAL);

	ret = snprintf(str,
		       len,
		       VMETA_SESSION_THERMAL_ALIGNMENT_FORMAT,
		       align->rotation.yaw,
		       align->rotation.pitch,
		       align->rotation.roll);

	return (ssize_t)ret;
}


int vmeta_session_thermal_alignment_read(const char *str,
					 struct vmeta_thermal_alignment *align)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(align == NULL, EINVAL);

	align->valid = 0;

	ret = sscanf(str,
		     "%f,%f,%f",
		     &align->rotation.yaw,
		     &align->rotation.pitch,
		     &align->rotation.roll);
	if (ret == 3)
		align->valid = 1;

	return 0;
}


ssize_t vmeta_session_thermal_conversion_write(
	char *str,
	size_t len,
	const struct vmeta_thermal_conversion *conv)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(conv == NULL, EINVAL);

	ret = snprintf(str,
		       len,
		       VMETA_SESSION_THERMAL_CONVERSION_FORMAT,
		       conv->r,
		       conv->b,
		       conv->f,
		       conv->o,
		       conv->tau_win,
		       conv->t_win,
		       conv->t_bg,
		       conv->emissivity);

	return (ssize_t)ret;
}


int vmeta_session_thermal_conversion_read(const char *str,
					  struct vmeta_thermal_conversion *conv)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(conv == NULL, EINVAL);

	conv->valid = 0;

	ret = sscanf(str,
		     "%f,%f,%f,%f,%f,%f,%f,%f",
		     &conv->r,
		     &conv->b,
		     &conv->f,
		     &conv->o,
		     &conv->tau_win,
		     &conv->t_win,
		     &conv->t_bg,
		     &conv->emissivity);
	if (ret == 8)
		conv->valid = 1;

	return 0;
}


ssize_t
vmeta_session_thermal_scale_factor_write(char *str, size_t len, double value)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	ret = snprintf(
		str, len, VMETA_SESSION_THERMAL_SCALE_FACTOR_FORMAT, value);

	return (ssize_t)ret;
}


int vmeta_session_thermal_scale_factor_read(const char *str, double *value)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(value == NULL, EINVAL);

	ret = sscanf(str, "%lf", value);
	if (ret != 1)
		*value = 0.;

	return 0;
}


ssize_t vmeta_session_principal_point_write(
	char *str,
	size_t len,
	const struct vmeta_principal_point *principal_point)
{
	size_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(principal_point == NULL, EINVAL);

	if (!principal_point->valid)
		return 0;

	ret = snprintf(str,
		       len,
		       VMETA_SESSION_PRINCIPAL_POINT_FORMAT,
		       principal_point->position.x,
		       principal_point->position.y);

	return (ssize_t)ret;
}


int vmeta_session_principal_point_read(
	const char *str,
	struct vmeta_principal_point *principal_point)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(principal_point == NULL, EINVAL);

	principal_point->valid = 1;
	ret = sscanf(str,
		     "%f,%f",
		     &principal_point->position.x,
		     &principal_point->position.y);
	if (ret != 2) {
		principal_point->position.x = 0.;
		principal_point->position.y = 0.;
		principal_point->valid = 0;
	}

	return 0;
}


int vmeta_session_streaming_sdes_write(
	const struct vmeta_session *meta,
	vmeta_session_streaming_sdes_write_cb_t cb,
	void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	if (meta->serial_number[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_CNAME,
		      meta->serial_number,
		      NULL,
		      userdata);
	}

	if (meta->friendly_name[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_NAME,
		      meta->friendly_name,
		      NULL,
		      userdata);
	}

	if (meta->software_version[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_TOOL,
		      meta->software_version,
		      NULL,
		      userdata);
	}

	if (meta->takeoff_loc.valid) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->takeoff_loc);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      loc,
			      VMETA_STRM_SDES_KEY_TAKEOFF_LOC,
			      userdata);
	}

	if (meta->location.valid) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->location);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_LOC, loc, NULL, userdata);
	}

	if (meta->maker[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->maker,
		      VMETA_STRM_SDES_KEY_MAKER,
		      userdata);
	}

	if (meta->model[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->model,
		      VMETA_STRM_SDES_KEY_MODEL,
		      userdata);
	}

	if (meta->model_id[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->model_id,
		      VMETA_STRM_SDES_KEY_MODEL_ID,
		      userdata);
	}

	if (meta->build_id[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->build_id,
		      VMETA_STRM_SDES_KEY_BUILD_ID,
		      userdata);
	}

	if (meta->title[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->title,
		      VMETA_STRM_SDES_KEY_TITLE,
		      userdata);
	}

	if (meta->comment[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->comment,
		      VMETA_STRM_SDES_KEY_COMMENT,
		      userdata);
	}

	if (meta->copyright[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->copyright,
		      VMETA_STRM_SDES_KEY_COPYRIGHT,
		      userdata);
	}

	if (meta->media_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->media_date,
						       meta->media_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      date,
			      VMETA_STRM_SDES_KEY_MEDIA_DATE,
			      userdata);
	}

	if (meta->run_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->run_date,
						       meta->run_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      date,
			      VMETA_STRM_SDES_KEY_RUN_DATE,
			      userdata);
	}

	if (meta->run_id[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->run_id,
		      VMETA_STRM_SDES_KEY_RUN_ID,
		      userdata);
	}

	if (meta->boot_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->boot_date,
						       meta->boot_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      date,
			      VMETA_STRM_SDES_KEY_BOOT_DATE,
			      userdata);
	}

	if (meta->boot_id[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->boot_id,
		      VMETA_STRM_SDES_KEY_BOOT_ID,
		      userdata);
	}

	if (meta->flight_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret =
			vmeta_session_date_write(date,
						 sizeof(date),
						 meta->flight_date,
						 meta->flight_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      date,
			      VMETA_STRM_SDES_KEY_FLIGHT_DATE,
			      userdata);
	}

	if (meta->flight_id[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->flight_id,
		      VMETA_STRM_SDES_KEY_FLIGHT_ID,
		      userdata);
	}

	if (meta->custom_id[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->custom_id,
		      VMETA_STRM_SDES_KEY_CUSTOM_ID,
		      userdata);
	}

	if ((meta->picture_fov.has_horz) && (meta->picture_fov.has_vert)) {
		char fov[VMETA_SESSION_FOV_MAX_LEN];
		ssize_t ret = vmeta_session_fov_write(
			fov, sizeof(fov), &meta->picture_fov);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      fov,
			      VMETA_STRM_SDES_KEY_PICTURE_FOV,
			      userdata);
	}

	if (meta->camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_camera_type_to_str(meta->camera_type),
		      VMETA_STRM_SDES_KEY_CAMERA_TYPE,
		      userdata);
	}

	if (meta->camera_subtype != VMETA_CAMERA_SUBTYPE_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_camera_subtype_to_str(meta->camera_subtype),
		      VMETA_STRM_SDES_KEY_CAMERA_SUBTYPE,
		      userdata);
	}

	if (meta->camera_spectrum != VMETA_CAMERA_SPECTRUM_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_camera_spectrum_to_str(meta->camera_spectrum),
		      VMETA_STRM_SDES_KEY_CAMERA_SPECTRUM,
		      userdata);
	}

	if (meta->camera_serial_number[0] != '\0') {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      meta->camera_serial_number,
		      VMETA_STRM_SDES_KEY_CAMERA_SERIAL_NUMBER,
		      userdata);
	}

	switch (meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
		char dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_camera_model_type_to_str(meta->camera_model.type),
		      VMETA_STRM_SDES_KEY_CAMERA_MODEL_TYPE,
		      userdata);
		ssize_t ret = vmeta_session_perspective_distortion_write(
			dist,
			sizeof(dist),
			meta->camera_model.perspective.distortion.r1,
			meta->camera_model.perspective.distortion.r2,
			meta->camera_model.perspective.distortion.r3,
			meta->camera_model.perspective.distortion.t1,
			meta->camera_model.perspective.distortion.t2);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      dist,
			      VMETA_STRM_SDES_KEY_PERSPECTIVE_DISTORTION,
			      userdata);
		break;
	}
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
		char matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
		char coef[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_camera_model_type_to_str(meta->camera_model.type),
		      VMETA_STRM_SDES_KEY_CAMERA_MODEL_TYPE,
		      userdata);
		ssize_t ret = vmeta_session_fisheye_affine_matrix_write(
			matrix,
			sizeof(matrix),
			meta->camera_model.fisheye.affine_matrix.c,
			meta->camera_model.fisheye.affine_matrix.d,
			meta->camera_model.fisheye.affine_matrix.e,
			meta->camera_model.fisheye.affine_matrix.f);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      matrix,
			      VMETA_STRM_SDES_KEY_FISHEYE_AFFINE_MATRIX,
			      userdata);
		ret = vmeta_session_fisheye_polynomial_write(
			coef,
			sizeof(coef),
			meta->camera_model.fisheye.polynomial.p2,
			meta->camera_model.fisheye.polynomial.p3,
			meta->camera_model.fisheye.polynomial.p4);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      coef,
			      VMETA_STRM_SDES_KEY_FISHEYE_POLYNOMIAL,
			      userdata);
		break;
	}
	default:
		break;
	}

	if (meta->video_mode != VMETA_VIDEO_MODE_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_video_mode_to_str(meta->video_mode),
		      VMETA_STRM_SDES_KEY_VIDEO_MODE,
		      userdata);
	}

	if (meta->video_stop_reason != VMETA_VIDEO_STOP_REASON_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_video_stop_reason_to_str(meta->video_stop_reason),
		      VMETA_STRM_SDES_KEY_VIDEO_STOP_REASON,
		      userdata);
	}

	if (meta->dynamic_range != VMETA_DYNAMIC_RANGE_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_dynamic_range_to_str(meta->dynamic_range),
		      VMETA_STRM_SDES_KEY_DYNAMIC_RANGE,
		      userdata);
	}

	if (meta->tone_mapping != VMETA_TONE_MAPPING_UNKNOWN) {
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      vmeta_tone_mapping_to_str(meta->tone_mapping),
		      VMETA_STRM_SDES_KEY_TONE_MAPPING,
		      userdata);
	}

	if (meta->has_thermal) {
		if (meta->thermal.metaversion != 0) {
			char metaversion[10];
			snprintf(metaversion,
				 sizeof(metaversion),
				 "%d",
				 meta->thermal.metaversion);
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      metaversion,
			      VMETA_STRM_SDES_KEY_THERMAL_METAVERSION,
			      userdata);
		}
		if (meta->thermal.camserial[0] != '\0') {
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
			      meta->thermal.camserial,
			      VMETA_STRM_SDES_KEY_THERMAL_CAMSERIAL,
			      userdata);
		}
		if (meta->thermal.alignment.valid) {
			char align[VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_alignment_write(
				align, sizeof(align), &meta->thermal.alignment);
			if (ret > 0)
				(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
				      align,
				      VMETA_STRM_SDES_KEY_THERMAL_ALIGNMENT,
				      userdata);
		}
		if (meta->thermal.conv_low.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_low);
			if (ret > 0)
				(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
				      conv,
				      VMETA_STRM_SDES_KEY_THERMAL_CONV_LOW,
				      userdata);
		}
		if (meta->thermal.conv_high.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_high);
			if (ret > 0)
				(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
				      conv,
				      VMETA_STRM_SDES_KEY_THERMAL_CONV_HIGH,
				      userdata);
		}
		if (meta->thermal.scale_factor != 0.) {
			const char *key =
				VMETA_STRM_SDES_KEY_THERMAL_SCALE_FACTOR;
			char conv[VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_scale_factor_write(
				conv, sizeof(conv), meta->thermal.scale_factor);
			if (ret > 0)
				(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
				      conv,
				      key,
				      userdata);
		}
	}

	if (meta->first_frame_capture_ts != 0) {
		char conv[21];
		snprintf(conv,
			 sizeof(conv),
			 "%" PRIu64,
			 meta->first_frame_capture_ts);
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      conv,
		      VMETA_STRM_SDES_KEY_FIRST_FRAME_CAPTURE_TS,
		      userdata);
	}

	if (meta->first_frame_sample_index != 0) {
		char conv[11];
		snprintf(conv,
			 sizeof(conv),
			 "%" PRIu32,
			 meta->first_frame_sample_index);
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      conv,
		      VMETA_STRM_SDES_KEY_FIRST_FRAME_SAMPLE_INDEX,
		      userdata);
	}

	if (meta->media_id != 0) {
		char conv[11];
		snprintf(conv, sizeof(conv), "%" PRIu32, meta->media_id);
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      conv,
		      VMETA_STRM_SDES_KEY_MEDIA_ID,
		      userdata);
	}

	if (meta->resource_index != 0) {
		char conv[11];
		snprintf(conv, sizeof(conv), "%" PRIu32, meta->resource_index);
		(*cb)(VMETA_STRM_SDES_TYPE_PRIV,
		      conv,
		      VMETA_STRM_SDES_KEY_RESOURCE_INDEX,
		      userdata);
	}

	if (meta->principal_point.valid) {
		const char *key = VMETA_STRM_SDES_KEY_PRINCIPAL_POINT;
		char conv[VMETA_SESSION_PRINCIPAL_POINT_MAX_LEN];
		ssize_t ret = vmeta_session_principal_point_write(
			conv, sizeof(conv), &meta->principal_point);
		if (ret > 0)
			(*cb)(VMETA_STRM_SDES_TYPE_PRIV, conv, key, userdata);
	}

	return 0;
}


int vmeta_session_streaming_sdes_read(enum vmeta_stream_sdes_type type,
				      const char *value,
				      const char *prefix,
				      struct vmeta_session *meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(value == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((type == VMETA_STRM_SDES_TYPE_PRIV) &&
					 (prefix == NULL),
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	switch (type) {
	case VMETA_STRM_SDES_TYPE_CNAME:
		COPY_VALUE(meta->serial_number, value);
		break;

	case VMETA_STRM_SDES_TYPE_NAME:
		COPY_VALUE(meta->friendly_name, value);
		break;

	case VMETA_STRM_SDES_TYPE_TOOL:
		COPY_VALUE(meta->software_version, value);
		break;

	case VMETA_STRM_SDES_TYPE_LOC:
		ret = vmeta_session_location_read(value, &meta->location);
		break;

	case VMETA_STRM_SDES_TYPE_PRIV:
		if (strcmp(prefix, VMETA_STRM_SDES_KEY_MEDIA_DATE) == 0) {
			ret = vmeta_session_date_read(value,
						      &meta->media_date,
						      &meta->media_date_gmtoff);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_RUN_DATE) == 0) {
			ret = vmeta_session_date_read(
				value, &meta->run_date, &meta->run_date_gmtoff);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_TAKEOFF_LOC) ==
			   0) {
			ret = vmeta_session_location_read(value,
							  &meta->takeoff_loc);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_RUN_ID) == 0) {
			COPY_VALUE(meta->run_id, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_BOOT_DATE) == 0) {
			ret = vmeta_session_date_read(value,
						      &meta->boot_date,
						      &meta->boot_date_gmtoff);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_BOOT_ID) == 0) {
			COPY_VALUE(meta->boot_id, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_FLIGHT_DATE) ==
			   0) {
			ret = vmeta_session_date_read(
				value,
				&meta->flight_date,
				&meta->flight_date_gmtoff);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_FLIGHT_ID) == 0) {
			COPY_VALUE(meta->flight_id, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_CUSTOM_ID) == 0) {
			COPY_VALUE(meta->custom_id, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_MAKER) == 0) {
			COPY_VALUE(meta->maker, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_MODEL) == 0) {
			COPY_VALUE(meta->model, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_MODEL_ID) == 0) {
			COPY_VALUE(meta->model_id, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_BUILD_ID) == 0) {
			COPY_VALUE(meta->build_id, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_TITLE) == 0) {
			COPY_VALUE(meta->title, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_COMMENT) == 0) {
			COPY_VALUE(meta->comment, value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_COPYRIGHT) == 0) {
			COPY_VALUE(meta->copyright, value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_PICTURE_HORZ_FOV) == 0) {
			meta->picture_fov.horz = atof(value);
			meta->picture_fov.has_horz = 1;

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_PICTURE_VERT_FOV) == 0) {
			meta->picture_fov.vert = atof(value);
			meta->picture_fov.has_vert = 1;

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_PICTURE_FOV) ==
			   0) {
			ret = vmeta_session_fov_read(value, &meta->picture_fov);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_THERMAL_METAVERSION) ==
			   0) {
			char metaversion[10];
			COPY_VALUE(metaversion, value);
			meta->thermal.metaversion = atoi(metaversion);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_THERMAL_CAMSERIAL) == 0) {
			COPY_VALUE(meta->thermal.camserial, value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_THERMAL_ALIGNMENT) == 0) {
			ret = vmeta_session_thermal_alignment_read(
				value, &meta->thermal.alignment);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_THERMAL_CONV_LOW) == 0) {
			ret = vmeta_session_thermal_conversion_read(
				value, &meta->thermal.conv_low);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_THERMAL_CONV_HIGH) == 0) {
			ret = vmeta_session_thermal_conversion_read(
				value, &meta->thermal.conv_high);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_THERMAL_SCALE_FACTOR) ==
			   0) {
			ret = vmeta_session_thermal_scale_factor_read(
				value, &meta->thermal.scale_factor);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_CAMERA_TYPE) ==
			   0) {
			meta->camera_type = vmeta_camera_type_from_str(value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_CAMERA_SUBTYPE) ==
			   0) {
			meta->camera_subtype =
				vmeta_camera_subtype_from_str(value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_CAMERA_SPECTRUM) == 0) {
			meta->camera_spectrum =
				vmeta_camera_spectrum_from_str(value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_CAMERA_SERIAL_NUMBER) ==
			   0) {
			COPY_VALUE(meta->camera_serial_number, value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_CAMERA_MODEL_TYPE) == 0) {
			meta->camera_model.type =
				vmeta_camera_model_type_from_str(value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_PERSPECTIVE_DISTORTION) ==
			   0) {
			ret = vmeta_session_perspective_distortion_read(
				value,
				&meta->camera_model.perspective.distortion.r1,
				&meta->camera_model.perspective.distortion.r2,
				&meta->camera_model.perspective.distortion.r3,
				&meta->camera_model.perspective.distortion.t1,
				&meta->camera_model.perspective.distortion.t2);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_FISHEYE_AFFINE_MATRIX) ==
			   0) {
			ret = vmeta_session_fisheye_affine_matrix_read(
				value,
				&meta->camera_model.fisheye.affine_matrix.c,
				&meta->camera_model.fisheye.affine_matrix.d,
				&meta->camera_model.fisheye.affine_matrix.e,
				&meta->camera_model.fisheye.affine_matrix.f);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_FISHEYE_POLYNOMIAL) ==
			   0) {
			ret = vmeta_session_fisheye_polynomial_read(
				value,
				&meta->camera_model.fisheye.polynomial.p2,
				&meta->camera_model.fisheye.polynomial.p3,
				&meta->camera_model.fisheye.polynomial.p4);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_VIDEO_MODE) ==
			   0) {
			meta->video_mode = vmeta_video_mode_from_str(value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_VIDEO_STOP_REASON) == 0) {
			meta->video_stop_reason =
				vmeta_video_stop_reason_from_str(value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_DYNAMIC_RANGE) ==
			   0) {
			meta->dynamic_range =
				vmeta_dynamic_range_from_str(value);

		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_TONE_MAPPING) ==
			   0) {
			meta->tone_mapping = vmeta_tone_mapping_from_str(value);

		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_FIRST_FRAME_CAPTURE_TS) ==
			   0) {
			meta->first_frame_capture_ts = strtoull(value, NULL, 0);
		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_MEDIA_ID) == 0) {
			meta->media_id = strtoul(value, NULL, 0);
		} else if (strcmp(prefix, VMETA_STRM_SDES_KEY_RESOURCE_INDEX) ==
			   0) {
			meta->resource_index = strtoul(value, NULL, 0);
		} else if (strcmp(prefix,
				  VMETA_STRM_SDES_KEY_PRINCIPAL_POINT) == 0) {
			ret = vmeta_session_principal_point_read(
				value, &meta->principal_point);
		}
		meta->has_thermal = (meta->thermal.metaversion > 0) ||
				    (meta->thermal.alignment.valid) ||
				    (meta->thermal.camserial[0] != '\0');
		break;

	default:
		break;
	}

	return ret;
}


int vmeta_session_streaming_sdp_write(const struct vmeta_session *meta,
				      int media_level,
				      vmeta_session_streaming_sdp_write_cb_t cb,
				      void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	/* Below: session-level only metadata */

	if ((!media_level) && (meta->friendly_name[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_INFO,
		      meta->friendly_name,
		      NULL,
		      userdata);
	}

	if ((!media_level) && (meta->title[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_NAME,
		      meta->title,
		      NULL,
		      userdata);
	}

	if ((!media_level) && (meta->software_version[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_TOOL,
		      meta->software_version,
		      NULL,
		      userdata);
	}

	if ((!media_level) && (meta->maker[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->maker,
		      VMETA_STRM_SDP_KEY_MAKER,
		      userdata);
	}

	if ((!media_level) && (meta->model[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->model,
		      VMETA_STRM_SDP_KEY_MODEL,
		      userdata);
	}

	if ((!media_level) && (meta->model_id[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->model_id,
		      VMETA_STRM_SDP_KEY_MODEL_ID,
		      userdata);
	}

	if ((!media_level) && (meta->serial_number[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->serial_number,
		      VMETA_STRM_SDP_KEY_SERIAL_NUMBER,
		      userdata);
	}

	if ((!media_level) && (meta->build_id[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->build_id,
		      VMETA_STRM_SDP_KEY_BUILD_ID,
		      userdata);
	}

	if ((!media_level) && (meta->comment[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->comment,
		      VMETA_STRM_SDP_KEY_COMMENT,
		      userdata);
	}

	if ((!media_level) && (meta->copyright[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->copyright,
		      VMETA_STRM_SDP_KEY_COPYRIGHT,
		      userdata);
	}

	if ((!media_level) && (meta->media_date != 0)) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->media_date,
						       meta->media_date_gmtoff);
		if (ret > 0) {
			(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			      date,
			      VMETA_STRM_SDP_KEY_MEDIA_DATE,
			      userdata);
		}
	}

	if ((!media_level) && (meta->run_date != 0)) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->run_date,
						       meta->run_date_gmtoff);
		if (ret > 0) {
			(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			      date,
			      VMETA_STRM_SDP_KEY_RUN_DATE,
			      userdata);
		}
	}

	if ((!media_level) && (meta->run_id[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->run_id,
		      VMETA_STRM_SDP_KEY_RUN_ID,
		      userdata);
	}

	if ((!media_level) && (meta->boot_date != 0)) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->boot_date,
						       meta->boot_date_gmtoff);
		if (ret > 0) {
			(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			      date,
			      VMETA_STRM_SDP_KEY_BOOT_DATE,
			      userdata);
		}
	}

	if ((!media_level) && (meta->boot_id[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->boot_id,
		      VMETA_STRM_SDP_KEY_BOOT_ID,
		      userdata);
	}

	if ((!media_level) && (meta->flight_date != 0)) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret =
			vmeta_session_date_write(date,
						 sizeof(date),
						 meta->flight_date,
						 meta->flight_date_gmtoff);
		if (ret > 0) {
			(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			      date,
			      VMETA_STRM_SDP_KEY_FLIGHT_DATE,
			      userdata);
		}
	}

	if ((!media_level) && (meta->flight_id[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->flight_id,
		      VMETA_STRM_SDP_KEY_FLIGHT_ID,
		      userdata);
	}

	if ((!media_level) && (meta->custom_id[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      meta->custom_id,
		      VMETA_STRM_SDP_KEY_CUSTOM_ID,
		      userdata);
	}

	if ((!media_level) && (meta->takeoff_loc.valid)) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->takeoff_loc);
		if (ret > 0) {
			(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			      loc,
			      VMETA_STRM_SDP_KEY_TAKEOFF_LOC,
			      userdata);
		}
	}

	if ((!media_level) && (meta->location.valid)) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->location);
		if (ret > 0) {
			(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
			      loc,
			      VMETA_STRM_SDP_KEY_LOCATION,
			      userdata);
		}
	}

	if ((!media_level) && (meta->video_mode != VMETA_VIDEO_MODE_UNKNOWN)) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      vmeta_video_mode_to_str(meta->video_mode),
		      VMETA_STRM_SDP_KEY_VIDEO_MODE,
		      userdata);
	}

	if ((!media_level) &&
	    (meta->video_stop_reason != VMETA_VIDEO_STOP_REASON_UNKNOWN)) {
		(*cb)(VMETA_STRM_SDP_TYPE_SESSION_ATTR,
		      vmeta_video_stop_reason_to_str(meta->video_stop_reason),
		      VMETA_STRM_SDP_KEY_VIDEO_STOP_REASON,
		      userdata);
	}

	/* Below: either session-level or media-level metadata.
	 * Note: session-level metadata apply to all media and can be
	 * overriden by media-level metadata, which only apply to the
	 * corresponding media. */

	if ((media_level) && (meta->title[0] != '\0')) {
		(*cb)(VMETA_STRM_SDP_TYPE_MEDIA_INFO,
		      meta->title,
		      NULL,
		      userdata);
	}

	enum vmeta_stream_sdp_type type =
		media_level ? VMETA_STRM_SDP_TYPE_MEDIA_ATTR
			    : VMETA_STRM_SDP_TYPE_SESSION_ATTR;

	if ((meta->picture_fov.has_horz) && (meta->picture_fov.has_vert)) {
		char fov[VMETA_SESSION_FOV_MAX_LEN];
		ssize_t ret = vmeta_session_fov_write(
			fov, sizeof(fov), &meta->picture_fov);
		if (ret > 0) {
			(*cb)(type,
			      fov,
			      VMETA_STRM_SDP_KEY_PICTURE_FOV,
			      userdata);
		}
	}

	if (meta->camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
		(*cb)(type,
		      vmeta_camera_type_to_str(meta->camera_type),
		      VMETA_STRM_SDP_KEY_CAMERA_TYPE,
		      userdata);
	}

	if (meta->camera_subtype != VMETA_CAMERA_SUBTYPE_UNKNOWN) {
		(*cb)(type,
		      vmeta_camera_subtype_to_str(meta->camera_subtype),
		      VMETA_STRM_SDP_KEY_CAMERA_SUBTYPE,
		      userdata);
	}

	if (meta->camera_spectrum != VMETA_CAMERA_SPECTRUM_UNKNOWN) {
		(*cb)(type,
		      vmeta_camera_spectrum_to_str(meta->camera_spectrum),
		      VMETA_STRM_SDP_KEY_CAMERA_SPECTRUM,
		      userdata);
	}

	if (meta->camera_serial_number[0] != '\0') {
		(*cb)(type,
		      meta->camera_serial_number,
		      VMETA_STRM_SDP_KEY_CAMERA_SERIAL_NUMBER,
		      userdata);
	}

	switch (meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
		char dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
		(*cb)(type,
		      vmeta_camera_model_type_to_str(meta->camera_model.type),
		      VMETA_STRM_SDP_KEY_CAMERA_MODEL_TYPE,
		      userdata);
		ssize_t ret = vmeta_session_perspective_distortion_write(
			dist,
			sizeof(dist),
			meta->camera_model.perspective.distortion.r1,
			meta->camera_model.perspective.distortion.r2,
			meta->camera_model.perspective.distortion.r3,
			meta->camera_model.perspective.distortion.t1,
			meta->camera_model.perspective.distortion.t2);
		if (ret > 0)
			(*cb)(type,
			      dist,
			      VMETA_STRM_SDP_KEY_PERSPECTIVE_DISTORTION,
			      userdata);
		break;
	}
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
		char matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
		char coef[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
		(*cb)(type,
		      vmeta_camera_model_type_to_str(meta->camera_model.type),
		      VMETA_STRM_SDP_KEY_CAMERA_MODEL_TYPE,
		      userdata);
		ssize_t ret = vmeta_session_fisheye_affine_matrix_write(
			matrix,
			sizeof(matrix),
			meta->camera_model.fisheye.affine_matrix.c,
			meta->camera_model.fisheye.affine_matrix.d,
			meta->camera_model.fisheye.affine_matrix.e,
			meta->camera_model.fisheye.affine_matrix.f);
		if (ret > 0)
			(*cb)(type,
			      matrix,
			      VMETA_STRM_SDP_KEY_FISHEYE_AFFINE_MATRIX,
			      userdata);
		ret = vmeta_session_fisheye_polynomial_write(
			coef,
			sizeof(coef),
			meta->camera_model.fisheye.polynomial.p2,
			meta->camera_model.fisheye.polynomial.p3,
			meta->camera_model.fisheye.polynomial.p4);
		if (ret > 0)
			(*cb)(type,
			      coef,
			      VMETA_STRM_SDP_KEY_FISHEYE_POLYNOMIAL,
			      userdata);
		break;
	}
	default:
		break;
	}

	if (meta->dynamic_range != VMETA_DYNAMIC_RANGE_UNKNOWN) {
		(*cb)(type,
		      vmeta_dynamic_range_to_str(meta->dynamic_range),
		      VMETA_STRM_SDP_KEY_DYNAMIC_RANGE,
		      userdata);
	}

	if (meta->tone_mapping != VMETA_TONE_MAPPING_UNKNOWN) {
		(*cb)(type,
		      vmeta_tone_mapping_to_str(meta->tone_mapping),
		      VMETA_STRM_SDP_KEY_TONE_MAPPING,
		      userdata);
	}

	if (meta->has_thermal) {
		if (meta->thermal.metaversion != 0) {
			char metaversion[10];
			snprintf(metaversion,
				 sizeof(metaversion),
				 "%d",
				 meta->thermal.metaversion);
			(*cb)(type,
			      metaversion,
			      VMETA_STRM_SDP_KEY_THERMAL_METAVERSION,
			      userdata);
		}
		if (meta->thermal.camserial[0] != '\0') {
			(*cb)(type,
			      meta->thermal.camserial,
			      VMETA_STRM_SDP_KEY_THERMAL_CAMSERIAL,
			      userdata);
		}
		if (meta->thermal.alignment.valid) {
			char align[VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_alignment_write(
				align, sizeof(align), &meta->thermal.alignment);
			if (ret > 0) {
				(*cb)(type,
				      align,
				      VMETA_STRM_SDP_KEY_THERMAL_ALIGNMENT,
				      userdata);
			}
		}
		if (meta->thermal.conv_low.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_low);
			if (ret > 0) {
				(*cb)(type,
				      conv,
				      VMETA_STRM_SDP_KEY_THERMAL_CONV_LOW,
				      userdata);
			}
		}
		if (meta->thermal.conv_high.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_high);
			if (ret > 0) {
				(*cb)(type,
				      conv,
				      VMETA_STRM_SDP_KEY_THERMAL_CONV_HIGH,
				      userdata);
			}
		}
		if (meta->thermal.scale_factor != 0.) {
			const char *key =
				VMETA_STRM_SDP_KEY_THERMAL_SCALE_FACTOR;
			char conv[VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_scale_factor_write(
				conv, sizeof(conv), meta->thermal.scale_factor);
			if (ret > 0)
				(*cb)(type, conv, key, userdata);
		}
	}

	if (meta->default_media) {
		const char *key = VMETA_STRM_SDP_KEY_DEFAULT_MEDIA;
		(*cb)(type, "1", key, userdata);
	}

	if (meta->first_frame_capture_ts != 0) {
		char conv[21];
		snprintf(conv,
			 sizeof(conv),
			 "%" PRIu64,
			 meta->first_frame_capture_ts);
		(*cb)(type,
		      conv,
		      VMETA_STRM_SDP_KEY_FIRST_FRAME_CAPTURE_TS,
		      userdata);
	}

	if (meta->first_frame_sample_index != 0) {
		char conv[11];
		snprintf(conv,
			 sizeof(conv),
			 "%" PRIu32,
			 meta->first_frame_sample_index);
		(*cb)(type,
		      conv,
		      VMETA_STRM_SDP_KEY_FIRST_FRAME_SAMPLE_INDEX,
		      userdata);
	}

	if (meta->media_id != 0) {
		char conv[11];
		snprintf(conv, sizeof(conv), "%" PRIu32, meta->media_id);
		(*cb)(type, conv, VMETA_STRM_SDP_KEY_MEDIA_ID, userdata);
	}

	if (meta->resource_index != 0) {
		char conv[11];
		snprintf(conv, sizeof(conv), "%" PRIu32, meta->resource_index);
		(*cb)(type, conv, VMETA_STRM_SDP_KEY_RESOURCE_INDEX, userdata);
	}

	if (meta->principal_point.valid) {
		const char *key = VMETA_STRM_SDP_KEY_PRINCIPAL_POINT;
		char conv[VMETA_SESSION_PRINCIPAL_POINT_MAX_LEN];
		ssize_t ret = vmeta_session_principal_point_write(
			conv, sizeof(conv), &meta->principal_point);
		if (ret > 0)
			(*cb)(type, conv, key, userdata);
	}

	return 0;
}


int vmeta_session_streaming_sdp_read(enum vmeta_stream_sdp_type type,
				     const char *value,
				     const char *key,
				     struct vmeta_session *meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(value == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(((type == VMETA_STRM_SDP_TYPE_SESSION_ATTR) ||
				  (type == VMETA_STRM_SDP_TYPE_MEDIA_ATTR)) &&
					 (key == NULL),
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	switch (type) {
	case VMETA_STRM_SDP_TYPE_SESSION_INFO:
		COPY_VALUE(meta->friendly_name, value);
		break;

	case VMETA_STRM_SDP_TYPE_SESSION_NAME:
	case VMETA_STRM_SDP_TYPE_MEDIA_INFO:
		COPY_VALUE(meta->title, value);
		break;

	case VMETA_STRM_SDP_TYPE_SESSION_TOOL:
		COPY_VALUE(meta->software_version, value);
		break;

	case VMETA_STRM_SDP_TYPE_SESSION_ATTR:
		if (strcmp(key, VMETA_STRM_SDP_KEY_MEDIA_DATE) == 0) {
			ret = vmeta_session_date_read(value,
						      &meta->media_date,
						      &meta->media_date_gmtoff);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_RUN_DATE) == 0) {
			ret = vmeta_session_date_read(
				value, &meta->run_date, &meta->run_date_gmtoff);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_RUN_ID) == 0) {
			COPY_VALUE(meta->run_id, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_BOOT_DATE) == 0) {
			ret = vmeta_session_date_read(value,
						      &meta->boot_date,
						      &meta->boot_date_gmtoff);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_BOOT_ID) == 0) {
			COPY_VALUE(meta->boot_id, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_FLIGHT_DATE) == 0) {
			ret = vmeta_session_date_read(
				value,
				&meta->flight_date,
				&meta->flight_date_gmtoff);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_FLIGHT_ID) == 0) {
			COPY_VALUE(meta->flight_id, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_CUSTOM_ID) == 0) {
			COPY_VALUE(meta->custom_id, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_MAKER) == 0) {
			COPY_VALUE(meta->maker, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_MODEL) == 0) {
			COPY_VALUE(meta->model, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_MODEL_ID) == 0) {
			COPY_VALUE(meta->model_id, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_SERIAL_NUMBER) == 0) {
			COPY_VALUE(meta->serial_number, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_BUILD_ID) == 0) {
			COPY_VALUE(meta->build_id, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_COMMENT) == 0) {
			COPY_VALUE(meta->comment, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_COPYRIGHT) == 0) {
			COPY_VALUE(meta->copyright, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_TAKEOFF_LOC) == 0) {
			ret = vmeta_session_location_read(value,
							  &meta->takeoff_loc);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_LOCATION) == 0) {
			ret = vmeta_session_location_read(value,
							  &meta->location);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_VIDEO_MODE) == 0) {
			meta->video_mode = vmeta_video_mode_from_str(value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_VIDEO_STOP_REASON) ==
			   0) {
			meta->video_stop_reason =
				vmeta_video_stop_reason_from_str(value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_DYNAMIC_RANGE) == 0) {
			meta->dynamic_range =
				vmeta_dynamic_range_from_str(value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_TONE_MAPPING) == 0) {
			meta->tone_mapping = vmeta_tone_mapping_from_str(value);

		} else if (strcmp(key,
				  VMETA_STRM_SDP_KEY_FIRST_FRAME_CAPTURE_TS) ==
			   0) {
			meta->first_frame_capture_ts = strtoull(value, NULL, 0);
		} else if (
			strcmp(key,
			       VMETA_STRM_SDP_KEY_FIRST_FRAME_SAMPLE_INDEX) ==
			0) {
			meta->first_frame_sample_index =
				strtoul(value, NULL, 0);
		} else if (strcmp(key, VMETA_STRM_SDP_KEY_MEDIA_ID) == 0) {
			meta->media_id = strtoul(value, NULL, 0);
		} else if (strcmp(key, VMETA_STRM_SDP_KEY_RESOURCE_INDEX) ==
			   0) {
			meta->resource_index = strtoul(value, NULL, 0);
		}
		/* fall through */
	case VMETA_STRM_SDP_TYPE_MEDIA_ATTR:
		if (strcmp(key, VMETA_STRM_SDP_KEY_PICTURE_FOV) == 0) {
			ret = vmeta_session_fov_read(value, &meta->picture_fov);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_CAMERA_TYPE) == 0) {
			meta->camera_type = vmeta_camera_type_from_str(value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_CAMERA_SUBTYPE) ==
			   0) {
			meta->camera_subtype =
				vmeta_camera_subtype_from_str(value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_CAMERA_SPECTRUM) ==
			   0) {
			meta->camera_spectrum =
				vmeta_camera_spectrum_from_str(value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_CAMERA_MODEL_TYPE) ==
			   0) {
			meta->camera_model.type =
				vmeta_camera_model_type_from_str(value);

		} else if (strcmp(key,
				  VMETA_STRM_SDP_KEY_PERSPECTIVE_DISTORTION) ==
			   0) {
			ret = vmeta_session_perspective_distortion_read(
				value,
				&meta->camera_model.perspective.distortion.r1,
				&meta->camera_model.perspective.distortion.r2,
				&meta->camera_model.perspective.distortion.r3,
				&meta->camera_model.perspective.distortion.t1,
				&meta->camera_model.perspective.distortion.t2);

		} else if (strcmp(key,
				  VMETA_STRM_SDP_KEY_FISHEYE_AFFINE_MATRIX) ==
			   0) {
			ret = vmeta_session_fisheye_affine_matrix_read(
				value,
				&meta->camera_model.fisheye.affine_matrix.c,
				&meta->camera_model.fisheye.affine_matrix.d,
				&meta->camera_model.fisheye.affine_matrix.e,
				&meta->camera_model.fisheye.affine_matrix.f);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_FISHEYE_POLYNOMIAL) ==
			   0) {
			ret = vmeta_session_fisheye_polynomial_read(
				value,
				&meta->camera_model.fisheye.polynomial.p2,
				&meta->camera_model.fisheye.polynomial.p3,
				&meta->camera_model.fisheye.polynomial.p4);

		} else if (strcmp(key,
				  VMETA_STRM_SDP_KEY_CAMERA_SERIAL_NUMBER) ==
			   0) {
			COPY_VALUE(meta->camera_serial_number, value);

		} else if (strcmp(key,
				  VMETA_STRM_SDP_KEY_THERMAL_METAVERSION) ==
			   0) {
			char metaversion[10];
			COPY_VALUE(metaversion, value);
			meta->thermal.metaversion = atoi(metaversion);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_THERMAL_CAMSERIAL) ==
			   0) {
			COPY_VALUE(meta->thermal.camserial, value);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_THERMAL_ALIGNMENT) ==
			   0) {
			ret = vmeta_session_thermal_alignment_read(
				value, &meta->thermal.alignment);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_THERMAL_CONV_LOW) ==
			   0) {
			ret = vmeta_session_thermal_conversion_read(
				value, &meta->thermal.conv_low);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_THERMAL_CONV_HIGH) ==
			   0) {
			ret = vmeta_session_thermal_conversion_read(
				value, &meta->thermal.conv_high);

		} else if (strcmp(key,
				  VMETA_STRM_SDP_KEY_THERMAL_SCALE_FACTOR) ==
			   0) {
			ret = vmeta_session_thermal_scale_factor_read(
				value, &meta->thermal.scale_factor);

		} else if (strcmp(key, VMETA_STRM_SDP_KEY_DEFAULT_MEDIA) == 0) {
			meta->default_media = 1;
		} else if (strcmp(key, VMETA_STRM_SDP_KEY_PRINCIPAL_POINT) ==
			   0) {
			ret = vmeta_session_principal_point_read(
				value, &meta->principal_point);
		}
		meta->has_thermal = (meta->thermal.metaversion > 0) ||
				    (meta->thermal.alignment.valid) ||
				    (meta->thermal.camserial[0] != '\0');
		break;

	default:
		break;
	}

	return ret;
}


int vmeta_session_recording_write(const struct vmeta_session *meta,
				  vmeta_session_recording_write_cb_t cb,
				  void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cb == NULL, EINVAL);

	if (meta->friendly_name[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_FRIENDLY_NAME,
		      meta->friendly_name,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_FRIENDLY_NAME,
		      meta->friendly_name,
		      userdata);
	}

	if (meta->title[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_TITLE,
		      meta->title,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_TITLE,
		      meta->title,
		      userdata);
	}

	if (meta->comment[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_COMMENT,
		      meta->comment,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_COMMENT,
		      meta->comment,
		      userdata);
	}

	if (meta->copyright[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_COPYRIGHT,
		      meta->copyright,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_COPYRIGHT,
		      meta->copyright,
		      userdata);
	}

	if (meta->media_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->media_date,
						       meta->media_date_gmtoff);
		if (ret > 0) {
			(*cb)(VMETA_REC_UDTA,
			      VMETA_REC_UDTA_KEY_MEDIA_DATE,
			      date,
			      userdata);
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_MEDIA_DATE,
			      date,
			      userdata);
		}
	}

	if (meta->takeoff_loc.valid) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->takeoff_loc);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_TAKEOFF_LOC,
			      loc,
			      userdata);
	}

	if (meta->location.valid) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret =
			vmeta_session_location_write(loc,
						     sizeof(loc),
						     VMETA_SESSION_LOCATION_XYZ,
						     &meta->location);
		if (ret > 0)
			(*cb)(VMETA_REC_XYZ,
			      VMETA_REC_UDTA_KEY_LOCATION,
			      loc,
			      userdata);
		ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->location);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_LOCATION,
			      loc,
			      userdata);
	}

	if (meta->maker[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_MAKER,
		      meta->maker,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_MAKER,
		      meta->maker,
		      userdata);
	}

	if (meta->model[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_MODEL,
		      meta->model,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_MODEL,
		      meta->model,
		      userdata);
	}

	if (meta->software_version[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_SOFTWARE_VERSION,
		      meta->software_version,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_SOFTWARE_VERSION,
		      meta->software_version,
		      userdata);
	}

	if (meta->serial_number[0] != '\0') {
		(*cb)(VMETA_REC_UDTA,
		      VMETA_REC_UDTA_KEY_SERIAL_NUMBER,
		      meta->serial_number,
		      userdata);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_SERIAL_NUMBER,
		      meta->serial_number,
		      userdata);
	}

	if (meta->model_id[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_MODEL_ID,
		      meta->model_id,
		      userdata);
	}

	if (meta->build_id[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_BUILD_ID,
		      meta->build_id,
		      userdata);
	}

	if (meta->run_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->run_date,
						       meta->run_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_RUN_DATE,
			      date,
			      userdata);
	}

	if (meta->run_id[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_RUN_ID,
		      meta->run_id,
		      userdata);
	}

	if (meta->boot_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->boot_date,
						       meta->boot_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_BOOT_DATE,
			      date,
			      userdata);
	}

	if (meta->boot_id[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_BOOT_ID,
		      meta->boot_id,
		      userdata);
	}

	if (meta->flight_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret =
			vmeta_session_date_write(date,
						 sizeof(date),
						 meta->flight_date,
						 meta->flight_date_gmtoff);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_FLIGHT_DATE,
			      date,
			      userdata);
	}

	if (meta->flight_id[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_FLIGHT_ID,
		      meta->flight_id,
		      userdata);
	}

	if (meta->custom_id[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CUSTOM_ID,
		      meta->custom_id,
		      userdata);
	}

	if ((meta->picture_fov.has_horz) && (meta->picture_fov.has_vert)) {
		char fov[VMETA_SESSION_FOV_MAX_LEN];
		ssize_t ret = vmeta_session_fov_write(
			fov, sizeof(fov), &meta->picture_fov);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_PICTURE_FOV,
			      fov,
			      userdata);
	}

	if (meta->camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CAMERA_TYPE,
		      vmeta_camera_type_to_str(meta->camera_type),
		      userdata);
	}

	if (meta->camera_subtype != VMETA_CAMERA_SUBTYPE_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CAMERA_SUBTYPE,
		      vmeta_camera_subtype_to_str(meta->camera_subtype),
		      userdata);
	}

	if (meta->camera_spectrum != VMETA_CAMERA_SPECTRUM_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CAMERA_SPECTRUM,
		      vmeta_camera_spectrum_to_str(meta->camera_spectrum),
		      userdata);
	}

	if (meta->camera_serial_number[0] != '\0') {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CAMERA_SERIAL_NUMBER,
		      meta->camera_serial_number,
		      userdata);
	}

	switch (meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
		char dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CAMERA_MODEL_TYPE,
		      vmeta_camera_model_type_to_str(meta->camera_model.type),
		      userdata);
		ssize_t ret = vmeta_session_perspective_distortion_write(
			dist,
			sizeof(dist),
			meta->camera_model.perspective.distortion.r1,
			meta->camera_model.perspective.distortion.r2,
			meta->camera_model.perspective.distortion.r3,
			meta->camera_model.perspective.distortion.t1,
			meta->camera_model.perspective.distortion.t2);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_PERSPECTIVE_DISTORTION,
			      dist,
			      userdata);
		break;
	}
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
		char matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
		char coef[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_CAMERA_MODEL_TYPE,
		      vmeta_camera_model_type_to_str(meta->camera_model.type),
		      userdata);
		ssize_t ret = vmeta_session_fisheye_affine_matrix_write(
			matrix,
			sizeof(matrix),
			meta->camera_model.fisheye.affine_matrix.c,
			meta->camera_model.fisheye.affine_matrix.d,
			meta->camera_model.fisheye.affine_matrix.e,
			meta->camera_model.fisheye.affine_matrix.f);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_FISHEYE_AFFINE_MATRIX,
			      matrix,
			      userdata);
		ret = vmeta_session_fisheye_polynomial_write(
			coef,
			sizeof(coef),
			meta->camera_model.fisheye.polynomial.p2,
			meta->camera_model.fisheye.polynomial.p3,
			meta->camera_model.fisheye.polynomial.p4);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_FISHEYE_POLYNOMIAL,
			      coef,
			      userdata);
		break;
	}
	default:
		break;
	}

	switch (meta->overlay.type) {
	case VMETA_OVERLAY_TYPE_HEADER_FOOTER: {
		char header_footer[VMETA_SESSION_OVERLAY_HEADER_FOOTER_MAX_LEN];
		ssize_t ret = vmeta_session_overlay_header_footer_write(
			header_footer,
			sizeof(header_footer),
			meta->overlay.header_footer.header_height,
			meta->overlay.header_footer.footer_height);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_HEADER_FOOTER,
			      header_footer,
			      userdata);
		break;
	}
	default:
		break;
	}


	if (meta->video_mode != VMETA_VIDEO_MODE_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_VIDEO_MODE,
		      vmeta_video_mode_to_str(meta->video_mode),
		      userdata);
	}

	if (meta->video_stop_reason != VMETA_VIDEO_STOP_REASON_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_VIDEO_STOP_REASON,
		      vmeta_video_stop_reason_to_str(meta->video_stop_reason),
		      userdata);
	}

	if (meta->dynamic_range != VMETA_DYNAMIC_RANGE_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_DYNAMIC_RANGE,
		      vmeta_dynamic_range_to_str(meta->dynamic_range),
		      userdata);
	}

	if (meta->tone_mapping != VMETA_TONE_MAPPING_UNKNOWN) {
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_TONE_MAPPING,
		      vmeta_tone_mapping_to_str(meta->tone_mapping),
		      userdata);
	}

	if (meta->has_thermal) {
		if (meta->thermal.metaversion != 0) {
			char metaversion[10];
			snprintf(metaversion,
				 sizeof(metaversion),
				 "%d",
				 meta->thermal.metaversion);
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_THERMAL_METAVERSION,
			      metaversion,
			      userdata);
		}
		if (meta->thermal.camserial[0] != '\0') {
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_THERMAL_CAMSERIAL,
			      meta->thermal.camserial,
			      userdata);
		}
		if (meta->thermal.alignment.valid) {
			char align[VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_alignment_write(
				align, sizeof(align), &meta->thermal.alignment);
			if (ret > 0)
				(*cb)(VMETA_REC_META,
				      VMETA_REC_META_KEY_THERMAL_ALIGNMENT,
				      align,
				      userdata);
		}
		if (meta->thermal.conv_low.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_low);
			if (ret > 0)
				(*cb)(VMETA_REC_META,
				      VMETA_REC_META_KEY_THERMAL_CONV_LOW,
				      conv,
				      userdata);
		}
		if (meta->thermal.conv_high.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_high);
			if (ret > 0)
				(*cb)(VMETA_REC_META,
				      VMETA_REC_META_KEY_THERMAL_CONV_HIGH,
				      conv,
				      userdata);
		}
		if (meta->thermal.scale_factor != 0.) {
			char conv[VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_scale_factor_write(
				conv, sizeof(conv), meta->thermal.scale_factor);
			if (ret > 0)
				(*cb)(VMETA_REC_META,
				      VMETA_REC_META_KEY_THERMAL_SCALE_FACTOR,
				      conv,
				      userdata);
		}
	}

	if (meta->first_frame_capture_ts != 0) {
		char conv[21];
		snprintf(conv,
			 sizeof(conv),
			 "%" PRIu64,
			 meta->first_frame_capture_ts);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_FIRST_FRAME_CAPTURE_TS,
		      conv,
		      userdata);
	}

	if (meta->first_frame_sample_index != 0) {
		char conv[11];
		snprintf(conv,
			 sizeof(conv),
			 "%" PRIu32,
			 meta->first_frame_sample_index);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_FIRST_FRAME_SAMPLE_INDEX,
		      conv,
		      userdata);
	}

	if (meta->media_id != 0) {
		char conv[11];
		snprintf(conv, sizeof(conv), "%" PRIu32, meta->media_id);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_MEDIA_ID,
		      conv,
		      userdata);
	}

	if (meta->resource_index != 0) {
		char conv[11];
		snprintf(conv, sizeof(conv), "%" PRIu32, meta->resource_index);
		(*cb)(VMETA_REC_META,
		      VMETA_REC_META_KEY_RESOURCE_INDEX,
		      conv,
		      userdata);
	}

	if (meta->principal_point.valid) {
		char conv[VMETA_SESSION_PRINCIPAL_POINT_MAX_LEN];
		ssize_t ret = vmeta_session_principal_point_write(
			conv, sizeof(conv), &meta->principal_point);
		if (ret > 0)
			(*cb)(VMETA_REC_META,
			      VMETA_REC_META_KEY_PRINCIPAL_POINT,
			      conv,
			      userdata);
	}

	return 0;
}


static int vmeta_session_recording_json_comment_read(const char *value,
						     struct vmeta_session *meta)
{
	int ret = 0;
	json_object *jobj;
	json_object *jitem;
	json_bool jret;

	jobj = json_tokener_parse(value);
	if (jobj == NULL) {
		ret = -1;
		goto out;
	}

	/* software_version */
	if (meta->software_version[0] == '\0') {
		jret = json_object_object_get_ex(
			jobj, VMETA_REC_UDTA_JSON_KEY_SOFTWARE_VERSION, &jitem);
		if ((jret) && (jitem != NULL)) {
			COPY_VALUE(meta->software_version,
				   json_object_get_string(jitem));
		}
	}

	/* run_id */
	if (meta->run_id[0] == '\0') {
		jret = json_object_object_get_ex(
			jobj, VMETA_REC_UDTA_JSON_KEY_RUN_ID, &jitem);
		if ((jret) && (jitem != NULL))
			COPY_VALUE(meta->run_id, json_object_get_string(jitem));
	}

	/* takeoff_loc */
	if (!meta->takeoff_loc.valid) {
		jret = json_object_object_get_ex(
			jobj, VMETA_REC_UDTA_JSON_KEY_TAKEOFF_LOC, &jitem);
		if ((jret) && (jitem != NULL)) {
			ret = vmeta_session_location_read(
				json_object_get_string(jitem),
				&meta->takeoff_loc);
			if (ret != 0)
				goto out;
		}
	}

	/* media_date */
	if (meta->media_date == 0) {
		jret = json_object_object_get_ex(
			jobj, VMETA_REC_UDTA_JSON_KEY_MEDIA_DATE, &jitem);
		if ((jret) && (jitem != NULL)) {
			ret = vmeta_session_date_read(
				json_object_get_string(jitem),
				&meta->media_date,
				&meta->media_date_gmtoff);
			if (ret != 0)
				goto out;
		}
	}

	/* picture_fov.horz */
	if (!meta->picture_fov.has_horz) {
		jret = json_object_object_get_ex(
			jobj, VMETA_REC_UDTA_JSON_KEY_PICTURE_HORZ_FOV, &jitem);
		if ((jret) && (jitem != NULL)) {
			meta->picture_fov.horz = json_object_get_double(jitem);
			meta->picture_fov.has_horz = 1;
		}
	}

	/* picture_fov.vert */
	if (!meta->picture_fov.has_vert) {
		jret = json_object_object_get_ex(
			jobj, VMETA_REC_UDTA_JSON_KEY_PICTURE_VERT_FOV, &jitem);
		if ((jret) && (jitem != NULL)) {
			meta->picture_fov.vert = json_object_get_double(jitem);
			meta->picture_fov.has_vert = 1;
		}
	}

out:
	return ret;
}


int vmeta_session_recording_read(const char *key,
				 const char *value,
				 struct vmeta_session *meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(key == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(value == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	if (strcmp(key, VMETA_REC_META_KEY_FRIENDLY_NAME) == 0) {
		COPY_VALUE(meta->friendly_name, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_TITLE) == 0) {
		COPY_VALUE(meta->title, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_COMMENT) == 0) {
		COPY_VALUE(meta->comment, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_COPYRIGHT) == 0) {
		COPY_VALUE(meta->copyright, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_MEDIA_DATE) == 0) {
		ret = vmeta_session_date_read(
			value, &meta->media_date, &meta->media_date_gmtoff);

	} else if (strcmp(key, VMETA_REC_META_KEY_TAKEOFF_LOC) == 0) {
		ret = vmeta_session_location_read(value, &meta->takeoff_loc);

	} else if (strcmp(key, VMETA_REC_META_KEY_LOCATION) == 0) {
		ret = vmeta_session_location_read(value, &meta->location);

	} else if (strcmp(key, VMETA_REC_META_KEY_MAKER) == 0) {
		COPY_VALUE(meta->maker, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_MODEL) == 0) {
		COPY_VALUE(meta->model, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_SOFTWARE_VERSION) == 0) {
		COPY_VALUE(meta->software_version, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_SERIAL_NUMBER) == 0) {
		COPY_VALUE(meta->serial_number, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_MODEL_ID) == 0) {
		COPY_VALUE(meta->model_id, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_BUILD_ID) == 0) {
		COPY_VALUE(meta->build_id, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_RUN_DATE) == 0) {
		ret = vmeta_session_date_read(
			value, &meta->run_date, &meta->run_date_gmtoff);

	} else if (strcmp(key, VMETA_REC_META_KEY_RUN_ID) == 0) {
		COPY_VALUE(meta->run_id, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_BOOT_DATE) == 0) {
		ret = vmeta_session_date_read(
			value, &meta->boot_date, &meta->boot_date_gmtoff);

	} else if (strcmp(key, VMETA_REC_META_KEY_BOOT_ID) == 0) {
		COPY_VALUE(meta->boot_id, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_FLIGHT_DATE) == 0) {
		ret = vmeta_session_date_read(
			value, &meta->flight_date, &meta->flight_date_gmtoff);

	} else if (strcmp(key, VMETA_REC_META_KEY_FLIGHT_ID) == 0) {
		COPY_VALUE(meta->flight_id, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_CUSTOM_ID) == 0) {
		COPY_VALUE(meta->custom_id, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_PICTURE_HORZ_FOV) == 0) {
		meta->picture_fov.horz = atof(value);
		meta->picture_fov.has_horz = 1;

	} else if (strcmp(key, VMETA_REC_META_KEY_PICTURE_VERT_FOV) == 0) {
		meta->picture_fov.vert = atof(value);
		meta->picture_fov.has_vert = 1;

	} else if (strcmp(key, VMETA_REC_META_KEY_PICTURE_FOV) == 0) {
		ret = vmeta_session_fov_read(value, &meta->picture_fov);

	} else if (strcmp(key, VMETA_REC_UDTA_KEY_FRIENDLY_NAME) == 0) {
		if (meta->friendly_name[0] == '\0')
			COPY_VALUE(meta->friendly_name, value);
		if ((strncmp(value, "Parrot", 6) == 0) && (strlen(value) > 7)) {
			/* Friendly name is "<maker> <model>" */
			if (meta->maker[0] == '\0') {
				strncpy(meta->maker,
					value,
					(sizeof(meta->maker) < 6)
						? sizeof(meta->maker)
						: 6);
				meta->maker[sizeof(meta->maker) - 1] = '\0';
			}
			if (meta->model[0] == '\0')
				COPY_VALUE(meta->model, value + 7);
		}

	} else if (strcmp(key, VMETA_REC_UDTA_KEY_TITLE) == 0) {
		if (meta->title[0] == '\0')
			COPY_VALUE(meta->title, value);
		if (meta->run_date == 0) {
			/* The title might be an ISO 8601 date, but it can
			 * also be a custom title set by the application. */
			vmeta_session_date_read(
				value, &meta->run_date, &meta->run_date_gmtoff);
		}

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_COMMENT) == 0) &&
		   (meta->comment[0] == '\0')) {
		if ((value[0] == '{') && (value[strlen(value) - 1] == '}')) {
			/* The comment is a JSON string */
			ret = vmeta_session_recording_json_comment_read(value,
									meta);
		} else {
			/* This is a real comment */
			COPY_VALUE(meta->comment, value);
		}

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_COPYRIGHT) == 0) &&
		   (meta->copyright[0] == '\0')) {
		COPY_VALUE(meta->copyright, value);

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_MEDIA_DATE) == 0) &&
		   (meta->media_date == 0)) {
		ret = vmeta_session_date_read(
			value, &meta->media_date, &meta->media_date_gmtoff);

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_LOCATION) == 0) &&
		   (meta->location.valid == 0)) {
		ret = vmeta_session_location_read(value, &meta->takeoff_loc);

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_MAKER) == 0) &&
		   (meta->maker[0] == '\0')) {
		COPY_VALUE(meta->maker, value);

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_MODEL) == 0) &&
		   (meta->model[0] == '\0')) {
		COPY_VALUE(meta->model, value);

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_SOFTWARE_VERSION) == 0) &&
		   (meta->software_version[0] == '\0')) {
		COPY_VALUE(meta->software_version, value);

	} else if ((strcmp(key, VMETA_REC_UDTA_KEY_SERIAL_NUMBER) == 0) &&
		   (meta->serial_number[0] == '\0')) {
		COPY_VALUE(meta->serial_number, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_THERMAL_METAVERSION) == 0) {
		char metaversion[10];
		COPY_VALUE(metaversion, value);
		meta->thermal.metaversion = atoi(metaversion);

	} else if (strcmp(key, VMETA_REC_META_KEY_THERMAL_CAMSERIAL) == 0) {
		COPY_VALUE(meta->thermal.camserial, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_THERMAL_ALIGNMENT) == 0) {
		ret = vmeta_session_thermal_alignment_read(
			value, &meta->thermal.alignment);

	} else if (strcmp(key, VMETA_REC_META_KEY_THERMAL_CONV_LOW) == 0) {
		ret = vmeta_session_thermal_conversion_read(
			value, &meta->thermal.conv_low);

	} else if (strcmp(key, VMETA_REC_META_KEY_THERMAL_CONV_HIGH) == 0) {
		ret = vmeta_session_thermal_conversion_read(
			value, &meta->thermal.conv_high);

	} else if (strcmp(key, VMETA_REC_META_KEY_THERMAL_SCALE_FACTOR) == 0) {
		ret = vmeta_session_thermal_scale_factor_read(
			value, &meta->thermal.scale_factor);

	} else if (strcmp(key, VMETA_REC_META_KEY_CAMERA_TYPE) == 0) {
		meta->camera_type = vmeta_camera_type_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_CAMERA_SUBTYPE) == 0) {
		meta->camera_subtype = vmeta_camera_subtype_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_CAMERA_SPECTRUM) == 0) {
		meta->camera_spectrum = vmeta_camera_spectrum_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_CAMERA_SERIAL_NUMBER) == 0) {
		COPY_VALUE(meta->camera_serial_number, value);

	} else if (strcmp(key, VMETA_REC_META_KEY_CAMERA_MODEL_TYPE) == 0) {
		meta->camera_model.type =
			vmeta_camera_model_type_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_PERSPECTIVE_DISTORTION) ==
		   0) {
		ret = vmeta_session_perspective_distortion_read(
			value,
			&meta->camera_model.perspective.distortion.r1,
			&meta->camera_model.perspective.distortion.r2,
			&meta->camera_model.perspective.distortion.r3,
			&meta->camera_model.perspective.distortion.t1,
			&meta->camera_model.perspective.distortion.t2);

	} else if (strcmp(key, VMETA_REC_META_KEY_FISHEYE_AFFINE_MATRIX) == 0) {
		ret = vmeta_session_fisheye_affine_matrix_read(
			value,
			&meta->camera_model.fisheye.affine_matrix.c,
			&meta->camera_model.fisheye.affine_matrix.d,
			&meta->camera_model.fisheye.affine_matrix.e,
			&meta->camera_model.fisheye.affine_matrix.f);

	} else if (strcmp(key, VMETA_REC_META_KEY_FISHEYE_POLYNOMIAL) == 0) {
		ret = vmeta_session_fisheye_polynomial_read(
			value,
			&meta->camera_model.fisheye.polynomial.p2,
			&meta->camera_model.fisheye.polynomial.p3,
			&meta->camera_model.fisheye.polynomial.p4);

	} else if (strcmp(key, VMETA_REC_META_KEY_HEADER_FOOTER) == 0) {
		meta->overlay.type = VMETA_OVERLAY_TYPE_HEADER_FOOTER;
		ret = vmeta_session_overlay_header_footer_read(
			value,
			&meta->overlay.header_footer.header_height,
			&meta->overlay.header_footer.footer_height);

	} else if (strcmp(key, VMETA_REC_META_KEY_VIDEO_MODE) == 0) {
		meta->video_mode = vmeta_video_mode_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_VIDEO_STOP_REASON) == 0) {
		meta->video_stop_reason =
			vmeta_video_stop_reason_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_DYNAMIC_RANGE) == 0) {
		meta->dynamic_range = vmeta_dynamic_range_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_TONE_MAPPING) == 0) {
		meta->tone_mapping = vmeta_tone_mapping_from_str(value);

	} else if (strcmp(key, VMETA_REC_META_KEY_FIRST_FRAME_CAPTURE_TS) ==
		   0) {
		meta->first_frame_capture_ts = strtoull(value, NULL, 0);
	} else if (strcmp(key, VMETA_REC_META_KEY_FIRST_FRAME_SAMPLE_INDEX) ==
		   0) {
		meta->first_frame_sample_index = strtoul(value, NULL, 0);
	} else if (strcmp(key, VMETA_REC_META_KEY_MEDIA_ID) == 0) {
		meta->media_id = strtoul(value, NULL, 0);
	} else if (strcmp(key, VMETA_REC_META_KEY_RESOURCE_INDEX) == 0) {
		meta->resource_index = strtoul(value, NULL, 0);
	} else if (strcmp(key, VMETA_REC_META_KEY_PRINCIPAL_POINT) == 0) {
		ret = vmeta_session_principal_point_read(
			value, &meta->principal_point);
	}
	meta->has_thermal = (meta->thermal.metaversion > 0) ||
			    (meta->thermal.alignment.valid) ||
			    (meta->thermal.camserial[0] != '\0');

	return ret;
}


int vmeta_session_to_json(const struct vmeta_session *meta,
			  struct json_object *jobj)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(jobj == NULL, EINVAL);

	if (meta->friendly_name[0] != '\0')
		vmeta_json_add_str(jobj, "friendly_name", meta->friendly_name);

	if (meta->maker[0] != '\0')
		vmeta_json_add_str(jobj, "maker", meta->maker);

	if (meta->model[0] != '\0')
		vmeta_json_add_str(jobj, "model", meta->model);

	if (meta->model_id[0] != '\0')
		vmeta_json_add_str(jobj, "model_id", meta->model_id);

	if (meta->serial_number[0] != '\0')
		vmeta_json_add_str(jobj, "serial_number", meta->serial_number);

	if (meta->software_version[0] != '\0') {
		vmeta_json_add_str(
			jobj, "software_version", meta->software_version);
	}

	if (meta->build_id[0] != '\0')
		vmeta_json_add_str(jobj, "build_id", meta->build_id);

	if (meta->title[0] != '\0')
		vmeta_json_add_str(jobj, "title", meta->title);

	if (meta->comment[0] != '\0')
		vmeta_json_add_str(jobj, "comment", meta->comment);

	if (meta->copyright[0] != '\0')
		vmeta_json_add_str(jobj, "copyright", meta->copyright);

	if (meta->media_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->media_date,
						       meta->media_date_gmtoff);
		if (ret > 0)
			vmeta_json_add_str(jobj, "media_date", date);
	}

	if (meta->run_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->run_date,
						       meta->run_date_gmtoff);
		if (ret > 0)
			vmeta_json_add_str(jobj, "run_date", date);
	}

	if (meta->run_id[0] != '\0')
		vmeta_json_add_str(jobj, "run_id", meta->run_id);

	if (meta->boot_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->boot_date,
						       meta->boot_date_gmtoff);
		if (ret > 0)
			vmeta_json_add_str(jobj, "boot_date", date);
	}

	if (meta->boot_id[0] != '\0')
		vmeta_json_add_str(jobj, "boot_id", meta->boot_id);

	if (meta->flight_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret =
			vmeta_session_date_write(date,
						 sizeof(date),
						 meta->flight_date,
						 meta->flight_date_gmtoff);
		if (ret > 0)
			vmeta_json_add_str(jobj, "flight_date", date);
	}

	if (meta->flight_id[0] != '\0')
		vmeta_json_add_str(jobj, "flight_id", meta->flight_id);

	if (meta->custom_id[0] != '\0')
		vmeta_json_add_str(jobj, "custom_id", meta->custom_id);

	if (meta->takeoff_loc.valid) {
		vmeta_json_add_location(
			jobj, "takeoff_loc", &meta->takeoff_loc);
	}

	if (meta->location.valid)
		vmeta_json_add_location(jobj, "location", &meta->location);

	if ((meta->picture_fov.has_horz) || (meta->picture_fov.has_vert))
		vmeta_json_add_fov(jobj, "picture_fov", &meta->picture_fov);

	/* Note: the default_media field is deliberately ommited here */

	if (meta->camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
		vmeta_json_add_str(jobj,
				   "camera_type",
				   vmeta_camera_type_to_str(meta->camera_type));
	}

	if (meta->camera_subtype != VMETA_CAMERA_SUBTYPE_UNKNOWN) {
		vmeta_json_add_str(
			jobj,
			"camera_subtype",
			vmeta_camera_subtype_to_str(meta->camera_subtype));
	}

	if (meta->camera_spectrum != VMETA_CAMERA_SPECTRUM_UNKNOWN) {
		vmeta_json_add_str(
			jobj,
			"camera_spectrum",
			vmeta_camera_spectrum_to_str(meta->camera_spectrum));
	}

	if (meta->camera_serial_number[0] != '\0') {
		vmeta_json_add_str(jobj,
				   "camera_serial_number",
				   meta->camera_serial_number);
	}

	switch (meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
		struct json_object *jobj_model = json_object_new_object();
		struct json_object *jpd = json_object_new_object();
		vmeta_json_add_str(jobj_model,
				   "type",
				   vmeta_camera_model_type_to_str(
					   meta->camera_model.type));
		vmeta_json_add_double(
			jpd,
			"r1",
			meta->camera_model.perspective.distortion.r1);
		vmeta_json_add_double(
			jpd,
			"r2",
			meta->camera_model.perspective.distortion.r2);
		vmeta_json_add_double(
			jpd,
			"r3",
			meta->camera_model.perspective.distortion.r3);
		vmeta_json_add_double(
			jpd,
			"t1",
			meta->camera_model.perspective.distortion.t1);
		vmeta_json_add_double(
			jpd,
			"t2",
			meta->camera_model.perspective.distortion.t2);
		json_object_object_add(
			jobj_model, "perspective_distortion", jpd);
		json_object_object_add(jobj, "camera_model", jobj_model);
		break;
	}
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
		struct json_object *jobj_model = json_object_new_object();
		struct json_object *jfam = json_object_new_object();
		struct json_object *jfp = json_object_new_object();
		vmeta_json_add_str(jobj_model,
				   "type",
				   vmeta_camera_model_type_to_str(
					   meta->camera_model.type));
		vmeta_json_add_double(
			jfam, "c", meta->camera_model.fisheye.affine_matrix.c);
		vmeta_json_add_double(
			jfam, "d", meta->camera_model.fisheye.affine_matrix.d);
		vmeta_json_add_double(
			jfam, "e", meta->camera_model.fisheye.affine_matrix.e);
		vmeta_json_add_double(
			jfam, "f", meta->camera_model.fisheye.affine_matrix.f);
		json_object_object_add(
			jobj_model, "fisheye_affine_matrix", jfam);
		vmeta_json_add_double(jfp, "p0", 0.);
		vmeta_json_add_double(jfp, "p1", 1.);
		vmeta_json_add_double(
			jfp, "p2", meta->camera_model.fisheye.polynomial.p2);
		vmeta_json_add_double(
			jfp, "p3", meta->camera_model.fisheye.polynomial.p3);
		vmeta_json_add_double(
			jfp, "p4", meta->camera_model.fisheye.polynomial.p4);
		json_object_object_add(jobj_model, "fisheye_polynomial", jfp);
		json_object_object_add(jobj, "camera_model", jobj_model);
		break;
	}
	default:
		break;
	}

	switch (meta->overlay.type) {
	case VMETA_OVERLAY_TYPE_HEADER_FOOTER: {
		struct json_object *jobj_overlay = json_object_new_object();
		struct json_object *jobj_header_footer =
			json_object_new_object();
		vmeta_json_add_str(
			jobj_overlay,
			"type",
			vmeta_overlay_type_to_str(meta->overlay.type));
		vmeta_json_add_double(
			jobj_header_footer,
			"header_height",
			meta->overlay.header_footer.header_height);
		vmeta_json_add_double(
			jobj_header_footer,
			"footer_height",
			meta->overlay.header_footer.footer_height);
		json_object_object_add(
			jobj_overlay, "header_footer", jobj_header_footer);
		json_object_object_add(jobj, "overlay", jobj_overlay);
		break;
	}
	default:
		break;
	}

	if (meta->principal_point.valid) {
		vmeta_json_add_xy(jobj,
				  "principal_point",
				  &meta->principal_point.position);
	}

	if (meta->video_mode != VMETA_VIDEO_MODE_UNKNOWN)
		vmeta_json_add_str(jobj,
				   "video_mode",
				   vmeta_video_mode_to_str(meta->video_mode));

	if (meta->video_stop_reason != VMETA_VIDEO_STOP_REASON_UNKNOWN)
		vmeta_json_add_str(jobj,
				   "video_stop_reason",
				   vmeta_video_stop_reason_to_str(
					   meta->video_stop_reason));

	if (meta->dynamic_range != VMETA_DYNAMIC_RANGE_UNKNOWN)
		vmeta_json_add_str(
			jobj,
			"dynamic_range",
			vmeta_dynamic_range_to_str(meta->dynamic_range));

	if (meta->tone_mapping != VMETA_TONE_MAPPING_UNKNOWN)
		vmeta_json_add_str(
			jobj,
			"tone_mapping",
			vmeta_tone_mapping_to_str(meta->tone_mapping));

	if (meta->has_thermal) {
		struct json_object *jobj_thermal = json_object_new_object();
		vmeta_json_add_int(
			jobj_thermal, "metaversion", meta->thermal.metaversion);
		if (meta->thermal.camserial[0] != '\0') {
			vmeta_json_add_str(jobj_thermal,
					   "camserial",
					   meta->thermal.camserial);
		}
		if (meta->thermal.alignment.valid) {
			struct json_object *jobj_align =
				json_object_new_object();
			vmeta_json_add_euler(jobj_align,
					     "rotation",
					     &meta->thermal.alignment.rotation);
			json_object_object_add(
				jobj_thermal, "alignment", jobj_align);
		}
		if (meta->thermal.conv_low.valid) {
			vmeta_json_add_thermal_conversion(
				jobj_thermal,
				"conv_low",
				&meta->thermal.conv_low);
		}
		if (meta->thermal.conv_high.valid) {
			vmeta_json_add_thermal_conversion(
				jobj_thermal,
				"conv_high",
				&meta->thermal.conv_high);
		}
		if (meta->thermal.scale_factor != 0.) {
			vmeta_json_add_double(jobj_thermal,
					      "scale_factor",
					      meta->thermal.scale_factor);
		}
		json_object_object_add(jobj, "thermal", jobj_thermal);
	}

	if (meta->first_frame_capture_ts != 0) {
		vmeta_json_add_int64(jobj,
				     "first_frame_capture_ts",
				     meta->first_frame_capture_ts);
	}

	if (meta->first_frame_sample_index != 0) {
		vmeta_json_add_int(jobj,
				   "first_frame_sample_index",
				   meta->first_frame_sample_index);
	}

	if (meta->media_id != 0)
		vmeta_json_add_int(jobj, "media_id", meta->media_id);

	if (meta->resource_index != 0) {
		vmeta_json_add_int(
			jobj, "resource_index", meta->resource_index);
	}

	return 0;
}


int vmeta_session_to_str(const struct vmeta_session *meta,
			 char *str,
			 size_t maxlen)
{
	size_t len = 0;

	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(str == NULL, EINVAL);

	if (meta->friendly_name[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"friendly_name: %s\n",
				meta->friendly_name);
	}

	if (meta->maker[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"maker: %s\n",
				meta->maker);
	}

	if (meta->model[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"model: %s\n",
				meta->model);
	}

	if (meta->model_id[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"model_id: %s\n",
				meta->model_id);
	}

	if (meta->serial_number[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"serial_number: %s\n",
				meta->serial_number);
	}

	if (meta->software_version[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"software_version: %s\n",
				meta->software_version);
	}

	if (meta->build_id[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"build_id: %s\n",
				meta->build_id);
	}

	if (meta->title[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"title: %s\n",
				meta->title);
	}

	if (meta->comment[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"comment: %s\n",
				meta->comment);
	}

	if (meta->copyright[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"copyright: %s\n",
				meta->copyright);
	}

	if (meta->media_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->media_date,
						       meta->media_date_gmtoff);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"media_date: %s\n",
					date);
		}
	}

	if (meta->run_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->run_date,
						       meta->run_date_gmtoff);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"run_date: %s\n",
					date);
		}
	}

	if (meta->run_id[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"run_id: %s\n",
				meta->run_id);
	}

	if (meta->boot_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret = vmeta_session_date_write(date,
						       sizeof(date),
						       meta->boot_date,
						       meta->boot_date_gmtoff);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"boot_date: %s\n",
					date);
		}
	}

	if (meta->boot_id[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"boot_id: %s\n",
				meta->boot_id);
	}

	if (meta->flight_date != 0) {
		char date[VMETA_SESSION_DATE_MAX_LEN];
		ssize_t ret =
			vmeta_session_date_write(date,
						 sizeof(date),
						 meta->flight_date,
						 meta->flight_date_gmtoff);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"flight_date: %s\n",
					date);
		}
	}

	if (meta->flight_id[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"flight_id: %s\n",
				meta->flight_id);
	}

	if (meta->custom_id[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"custom_id: %s\n",
				meta->custom_id);
	}

	if (meta->takeoff_loc.valid) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->takeoff_loc);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"takeoff_loc: %s\n",
					loc);
		}
	}

	if (meta->location.valid) {
		char loc[VMETA_SESSION_LOCATION_MAX_LEN];
		ssize_t ret = vmeta_session_location_write(
			loc,
			sizeof(loc),
			VMETA_SESSION_LOCATION_ISO6709,
			&meta->location);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"location: %s\n",
					loc);
		}
	}

	if ((meta->picture_fov.has_horz) && (meta->picture_fov.has_vert)) {
		char fov[VMETA_SESSION_FOV_MAX_LEN];
		ssize_t ret = vmeta_session_fov_write(
			fov, sizeof(fov), &meta->picture_fov);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"picture_fov: %s\n",
					fov);
		}
	}

	/* Note: the default_media field is deliberately ommited here */

	if (meta->camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"camera_type: %s\n",
				vmeta_camera_type_to_str(meta->camera_type));
	}

	if (meta->camera_subtype != VMETA_CAMERA_SUBTYPE_UNKNOWN) {
		VMETA_STR_PRINT(
			str + len,
			len,
			maxlen - len,
			"camera_subtype: %s\n",
			vmeta_camera_subtype_to_str(meta->camera_subtype));
	}

	if (meta->camera_spectrum != VMETA_CAMERA_SPECTRUM_UNKNOWN) {
		VMETA_STR_PRINT(
			str + len,
			len,
			maxlen - len,
			"camera_spectrum: %s\n",
			vmeta_camera_spectrum_to_str(meta->camera_spectrum));
	}

	if (meta->camera_serial_number[0] != '\0') {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"camera_serial_number: %s\n",
				meta->camera_serial_number);
	}

	switch (meta->camera_model.type) {
	case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
		char dist[VMETA_SESSION_PERSPECTIVE_DISTORTION_MAX_LEN];
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"camera_model_type: %s\n",
				vmeta_camera_model_type_to_str(
					meta->camera_model.type));
		ssize_t ret = vmeta_session_perspective_distortion_write(
			dist,
			sizeof(dist),
			meta->camera_model.perspective.distortion.r1,
			meta->camera_model.perspective.distortion.r2,
			meta->camera_model.perspective.distortion.r3,
			meta->camera_model.perspective.distortion.t1,
			meta->camera_model.perspective.distortion.t2);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"camera_model_"
					"perspective_distortion: %s\n",
					dist);
		}
		break;
	}
	case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
		char matrix[VMETA_SESSION_FISHEYE_AFFINE_MATRIX_MAX_LEN];
		char coef[VMETA_SESSION_FISHEYE_POLYNOMIAL_MAX_LEN];
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"camera_model_type: %s\n",
				vmeta_camera_model_type_to_str(
					meta->camera_model.type));
		ssize_t ret = vmeta_session_fisheye_affine_matrix_write(
			matrix,
			sizeof(matrix),
			meta->camera_model.fisheye.affine_matrix.c,
			meta->camera_model.fisheye.affine_matrix.d,
			meta->camera_model.fisheye.affine_matrix.e,
			meta->camera_model.fisheye.affine_matrix.f);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"camera_model_"
					"fisheye_affine_matrix: %s\n",
					matrix);
		}
		ret = vmeta_session_fisheye_polynomial_write(
			coef,
			sizeof(coef),
			meta->camera_model.fisheye.polynomial.p2,
			meta->camera_model.fisheye.polynomial.p3,
			meta->camera_model.fisheye.polynomial.p4);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"camera_model_"
					"fisheye_polynomial: %s\n",
					coef);
		}
		break;
	}
	default:
		break;
	}

	switch (meta->overlay.type) {
	case VMETA_OVERLAY_TYPE_HEADER_FOOTER: {
		char header_footer[VMETA_SESSION_OVERLAY_HEADER_FOOTER_MAX_LEN];
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"overlay_type: %s\n",
				vmeta_overlay_type_to_str(meta->overlay.type));
		ssize_t ret = vmeta_session_overlay_header_footer_write(
			header_footer,
			sizeof(header_footer),
			meta->overlay.header_footer.header_height,
			meta->overlay.header_footer.footer_height);
		if (ret > 0) {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"overlay_"
					"header_footer: %s\n",
					header_footer);
		}
		break;
	}
	default:
		break;
	}

	if (meta->video_mode != VMETA_VIDEO_MODE_UNKNOWN) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"video_mode: %s\n",
				vmeta_video_mode_to_str(meta->video_mode));
	}

	if (meta->video_stop_reason != VMETA_VIDEO_STOP_REASON_UNKNOWN) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"video_stop_reason: %s\n",
				vmeta_video_stop_reason_to_str(
					meta->video_stop_reason));
	}

	if (meta->dynamic_range != VMETA_DYNAMIC_RANGE_UNKNOWN) {
		VMETA_STR_PRINT(
			str + len,
			len,
			maxlen - len,
			"dynamic_range: %s\n",
			vmeta_dynamic_range_to_str(meta->dynamic_range));
	}

	if (meta->tone_mapping != VMETA_TONE_MAPPING_UNKNOWN) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"tone_mapping: %s\n",
				vmeta_tone_mapping_to_str(meta->tone_mapping));
	}

	if (meta->has_thermal) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"thermal_metaversion: %d\n",
				meta->thermal.metaversion);
		if (meta->thermal.camserial[0] != '\0') {
			VMETA_STR_PRINT(str + len,
					len,
					maxlen - len,
					"thermal_camserial: %s\n",
					meta->thermal.camserial);
		}
		if (meta->thermal.alignment.valid) {
			char align[VMETA_SESSION_THERMAL_ALIGNMENT_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_alignment_write(
				align, sizeof(align), &meta->thermal.alignment);
			if (ret > 0) {
				VMETA_STR_PRINT(
					str + len,
					len,
					maxlen - len,
					"thermal_alignment_rotation: %s\n",
					align);
			}
		}
		if (meta->thermal.conv_low.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_low);
			if (ret > 0) {
				VMETA_STR_PRINT(str + len,
						len,
						maxlen - len,
						"thermal_conv_low: %s\n",
						conv);
			}
		}
		if (meta->thermal.conv_high.valid) {
			char conv[VMETA_SESSION_THERMAL_CONVERSION_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_conversion_write(
				conv, sizeof(conv), &meta->thermal.conv_high);
			if (ret > 0) {
				VMETA_STR_PRINT(str + len,
						len,
						maxlen - len,
						"thermal_conv_high: %s\n",
						conv);
			}
		}
		if (meta->thermal.scale_factor != 0.) {
			char conv[VMETA_SESSION_THERMAL_SCALE_FACTOR_MAX_LEN];
			ssize_t ret = vmeta_session_thermal_scale_factor_write(
				conv, sizeof(conv), meta->thermal.scale_factor);
			if (ret > 0) {
				VMETA_STR_PRINT(str + len,
						len,
						maxlen - len,
						"thermal_scale_factor: %s\n",
						conv);
			}
		}
	}

	if (meta->first_frame_capture_ts != 0) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"first_frame_capture_ts: %" PRIu64 "\n",
				meta->first_frame_capture_ts);
	}

	if (meta->first_frame_sample_index != 0) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"first_frame_sample_index: %" PRIu32 "\n",
				meta->first_frame_sample_index);
	}

	if (meta->media_id != 0) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"media_id: %" PRIu32 "\n",
				meta->media_id);
	}

	if (meta->resource_index != 0) {
		VMETA_STR_PRINT(str + len,
				len,
				maxlen - len,
				"resource_index: %" PRIu32 "\n",
				meta->resource_index);
	}

	if (meta->principal_point.valid) {
		VMETA_STR_PRINT(
			str + len,
			len,
			maxlen - len,
			"principal_point: " VMETA_SESSION_PRINCIPAL_POINT_FORMAT
			"\n",
			meta->principal_point.position.x,
			meta->principal_point.position.y);
	}

	return 0;
}


static void
fill_session_meta_with_identical_values(const struct vmeta_session *ref,
					struct vmeta_session *ret)
{
	MERGE_META_STR(ret, ref, friendly_name);
	MERGE_META_STR(ret, ref, maker);
	MERGE_META_STR(ret, ref, model);
	MERGE_META_STR(ret, ref, model_id);
	MERGE_META_STR(ret, ref, serial_number);
	MERGE_META_STR(ret, ref, software_version);
	MERGE_META_STR(ret, ref, build_id);
	MERGE_META_STR(ret, ref, title);
	MERGE_META_STR(ret, ref, comment);
	MERGE_META_STR(ret, ref, copyright);

	MERGE_META_VAL(ret, ref, media_date, 0);
	MERGE_META_VAL(ret, ref, media_date_gmtoff, 0);
	MERGE_META_VAL(ret, ref, run_date, 0);
	MERGE_META_VAL(ret, ref, run_date_gmtoff, 0);

	MERGE_META_STR(ret, ref, run_id);

	MERGE_META_VAL(ret, ref, boot_date, 0);
	MERGE_META_VAL(ret, ref, boot_date_gmtoff, 0);

	MERGE_META_STR(ret, ref, boot_id);

	MERGE_META_VAL(ret, ref, flight_date, 0);
	MERGE_META_VAL(ret, ref, flight_date_gmtoff, 0);

	MERGE_META_STR(ret, ref, flight_id);
	MERGE_META_STR(ret, ref, custom_id);

	MERGE_META_PTR(&ret, &ref, takeoff_loc, sizeof(struct vmeta_location));
	MERGE_META_PTR(&ret, &ref, location, sizeof(struct vmeta_location));
	MERGE_META_PTR(&ret, &ref, picture_fov, sizeof(struct vmeta_fov));
	MERGE_META_PTR(&ret, &ref, thermal, sizeof(struct vmeta_thermal));

	if ((ret->has_thermal != ref->has_thermal) ||
	    (memcmp(&ret->thermal,
		    &ref->thermal,
		    sizeof(struct vmeta_thermal)) != 0))
		ret->has_thermal = 0;

	MERGE_META_VAL(ret, ref, default_media, 0);

	MERGE_META_VAL(ret, ref, camera_type, VMETA_CAMERA_TYPE_UNKNOWN);
	MERGE_META_VAL(ret, ref, camera_subtype, VMETA_CAMERA_SUBTYPE_UNKNOWN);
	MERGE_META_VAL(
		ret, ref, camera_spectrum, VMETA_CAMERA_SPECTRUM_UNKNOWN);

	MERGE_META_STR(ret, ref, camera_serial_number);
	MERGE_META_PTR(
		&ret, &ref, camera_model, sizeof(struct vmeta_camera_model));
	MERGE_META_PTR(&ret, &ref, overlay, sizeof(struct vmeta_overlay));
	MERGE_META_VAL(ret, ref, video_mode, VMETA_VIDEO_MODE_UNKNOWN);
	MERGE_META_VAL(
		ret, ref, video_stop_reason, VMETA_VIDEO_STOP_REASON_UNKNOWN);
	MERGE_META_VAL(ret, ref, dynamic_range, VMETA_DYNAMIC_RANGE_UNKNOWN);
	MERGE_META_VAL(ret, ref, tone_mapping, VMETA_TONE_MAPPING_UNKNOWN);
	MERGE_META_VAL(ret, ref, first_frame_capture_ts, 0);
	MERGE_META_VAL(ret, ref, first_frame_sample_index, 0);
	MERGE_META_VAL(ret, ref, media_id, 0);
	MERGE_META_VAL(ret, ref, resource_index, 0);

	if ((ret->principal_point.valid != ref->principal_point.valid) ||
	    (memcmp(&ret->principal_point.position,
		    &ref->principal_point.position,
		    sizeof(struct vmeta_xy)) != 0))
		ret->principal_point.valid = 0;
}


static void
erase_session_meta_with_identical_values(const struct vmeta_session *ref,
					 struct vmeta_session *ret)
{
	REMOVE_DUPLICATE_META_STR(ret, ref, friendly_name);
	REMOVE_DUPLICATE_META_STR(ret, ref, maker);
	REMOVE_DUPLICATE_META_STR(ret, ref, model);
	REMOVE_DUPLICATE_META_STR(ret, ref, model_id);
	REMOVE_DUPLICATE_META_STR(ret, ref, serial_number);
	REMOVE_DUPLICATE_META_STR(ret, ref, software_version);
	REMOVE_DUPLICATE_META_STR(ret, ref, build_id);
	REMOVE_DUPLICATE_META_STR(ret, ref, title);
	REMOVE_DUPLICATE_META_STR(ret, ref, comment);
	REMOVE_DUPLICATE_META_STR(ret, ref, copyright);

	REMOVE_DUPLICATE_META_VAL(ret, ref, media_date, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, media_date_gmtoff, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, run_date, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, run_date_gmtoff, 0);

	REMOVE_DUPLICATE_META_STR(ret, ref, run_id);

	REMOVE_DUPLICATE_META_VAL(ret, ref, boot_date, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, boot_date_gmtoff, 0);

	REMOVE_DUPLICATE_META_STR(ret, ref, boot_id);

	REMOVE_DUPLICATE_META_VAL(ret, ref, flight_date, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, flight_date_gmtoff, 0);

	REMOVE_DUPLICATE_META_STR(ret, ref, flight_id);
	REMOVE_DUPLICATE_META_STR(ret, ref, custom_id);

	REMOVE_DUPLICATE_META_VAL(ret, ref, takeoff_loc.valid, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, location.valid, 0);

	if (ref->picture_fov.has_horz || ref->picture_fov.has_vert) {
		ret->picture_fov.has_horz = 0;
		ret->picture_fov.has_vert = 0;
	}

	if (ref->has_thermal)
		ret->has_thermal = 0;

	REMOVE_DUPLICATE_META_VAL(ret, ref, default_media, 0);

	REMOVE_DUPLICATE_META_VAL(
		ret, ref, camera_type, VMETA_CAMERA_TYPE_UNKNOWN);
	REMOVE_DUPLICATE_META_VAL(
		ret, ref, camera_subtype, VMETA_CAMERA_SUBTYPE_UNKNOWN);
	REMOVE_DUPLICATE_META_VAL(
		ret, ref, camera_spectrum, VMETA_CAMERA_SPECTRUM_UNKNOWN);

	REMOVE_DUPLICATE_META_STR(ret, ref, camera_serial_number);
	REMOVE_DUPLICATE_META_PTR(
		&ret, &ref, camera_model, sizeof(struct vmeta_camera_model));
	REMOVE_DUPLICATE_META_PTR(
		&ret, &ref, overlay, sizeof(struct vmeta_overlay));
	REMOVE_DUPLICATE_META_VAL(
		ret, ref, video_stop_reason, VMETA_VIDEO_STOP_REASON_UNKNOWN);
	REMOVE_DUPLICATE_META_VAL(
		ret, ref, dynamic_range, VMETA_DYNAMIC_RANGE_UNKNOWN);
	REMOVE_DUPLICATE_META_VAL(
		ret, ref, tone_mapping, VMETA_TONE_MAPPING_UNKNOWN);

	REMOVE_DUPLICATE_META_VAL(
		ret, ref, video_mode, VMETA_VIDEO_MODE_UNKNOWN);

	REMOVE_DUPLICATE_META_VAL(ret, ref, first_frame_capture_ts, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, first_frame_sample_index, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, media_id, 0);
	REMOVE_DUPLICATE_META_VAL(ret, ref, resource_index, 0);

	if (ref->principal_point.valid)
		ret->principal_point.valid = 0;
}


int vmeta_session_merge_metadata(struct vmeta_session **meta_list,
				 size_t meta_list_count,
				 struct vmeta_session *merged_meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(merged_meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta_list == NULL, EINVAL);

	if (meta_list_count == 0)
		return 0;

	/* Copy first vmeta into merged_meta */
	memcpy(merged_meta, meta_list[0], sizeof(*merged_meta));

	/* First pass: remove different values from merged_meta */
	for (size_t i = 1; i < meta_list_count; i++) {
		if (meta_list[i] == NULL)
			continue;
		fill_session_meta_with_identical_values(meta_list[i],
							merged_meta);
	}

	/* Second pass: remove identical values from meta_list */
	for (size_t i = 0; i < meta_list_count; i++) {
		if (meta_list[i] == NULL)
			continue;
		erase_session_meta_with_identical_values(merged_meta,
							 meta_list[i]);
	}

	return 0;
}


static int vmeta_location_cmp(const struct vmeta_location *meta1,
			      const struct vmeta_location *meta2)
{
	CMP_FIELD_VAL(meta1, meta2, valid);

	if (meta1->valid == 0)
		return 1;

	CMP_FIELD_VAL(meta1, meta2, latitude);
	CMP_FIELD_VAL(meta1, meta2, longitude);

	CMP_FIELD_VAL_NAN_ALLOWED(meta1, meta2, altitude_wgs84ellipsoid);
	CMP_FIELD_VAL_NAN_ALLOWED(meta1, meta2, altitude_egm96amsl);

	CMP_FIELD_VAL(meta1, meta2, horizontal_accuracy);
	CMP_FIELD_VAL(meta1, meta2, vertical_accuracy);
	CMP_FIELD_VAL(meta1, meta2, sv_count);

	return 1;
}


int vmeta_session_cmp(const struct vmeta_session *meta1,
		      const struct vmeta_session *meta2)
{
	ULOG_ERRNO_RETURN_VAL_IF(meta1 == NULL && meta2 == NULL, EINVAL, 1);
	ULOG_ERRNO_RETURN_VAL_IF(meta1 == NULL || meta2 == NULL, EINVAL, 0);

	CMP_FIELD_STR(meta1, meta2, friendly_name);
	CMP_FIELD_STR(meta1, meta2, maker);
	CMP_FIELD_STR(meta1, meta2, model);
	CMP_FIELD_STR(meta1, meta2, model_id);
	CMP_FIELD_STR(meta1, meta2, serial_number);
	CMP_FIELD_STR(meta1, meta2, software_version);
	CMP_FIELD_STR(meta1, meta2, build_id);
	CMP_FIELD_STR(meta1, meta2, title);
	CMP_FIELD_STR(meta1, meta2, comment);
	CMP_FIELD_STR(meta1, meta2, copyright);

	CMP_FIELD_VAL(meta1, meta2, media_date);
	CMP_FIELD_VAL(meta1, meta2, media_date_gmtoff);
	CMP_FIELD_VAL(meta1, meta2, run_date);
	CMP_FIELD_VAL(meta1, meta2, run_date_gmtoff);

	CMP_FIELD_STR(meta1, meta2, run_id);

	CMP_FIELD_VAL(meta1, meta2, boot_date);
	CMP_FIELD_VAL(meta1, meta2, boot_date_gmtoff);

	CMP_FIELD_STR(meta1, meta2, boot_id);

	CMP_FIELD_VAL(meta1, meta2, flight_date);
	CMP_FIELD_VAL(meta1, meta2, flight_date_gmtoff);

	CMP_FIELD_STR(meta1, meta2, flight_id);
	CMP_FIELD_STR(meta1, meta2, custom_id);

	if (!vmeta_location_cmp(&meta1->takeoff_loc, &meta2->takeoff_loc))
		return 0;

	if (!vmeta_location_cmp(&meta1->location, &meta2->location))
		return 0;

	if (meta1->picture_fov.has_horz != meta2->picture_fov.has_horz)
		return 0;
	else if (meta1->picture_fov.has_horz &&
		 meta1->picture_fov.horz != meta2->picture_fov.horz)
		return 0;

	if (meta1->picture_fov.has_vert != meta2->picture_fov.has_vert)
		return 0;
	else if (meta1->picture_fov.has_vert &&
		 meta1->picture_fov.vert != meta2->picture_fov.vert)
		return 0;

	if (meta1->has_thermal != meta2->has_thermal)
		return 0;
	else if (meta1->has_thermal)
		CMP_FIELD_PTR(meta1, meta2, thermal);

	if (meta1->default_media != meta2->default_media)
		return 0;
	CMP_FIELD_VAL(meta1, meta2, camera_type);
	CMP_FIELD_VAL(meta1, meta2, camera_subtype);
	CMP_FIELD_VAL(meta1, meta2, camera_spectrum);

	CMP_FIELD_STR(meta1, meta2, camera_serial_number);

	CMP_FIELD_PTR(meta1, meta2, camera_model);
	CMP_FIELD_PTR(meta1, meta2, overlay);
	CMP_FIELD_VAL(meta1, meta2, video_mode);
	CMP_FIELD_VAL(meta1, meta2, video_stop_reason);
	CMP_FIELD_VAL(meta1, meta2, dynamic_range);
	CMP_FIELD_VAL(meta1, meta2, tone_mapping);
	CMP_FIELD_VAL(meta1, meta2, first_frame_capture_ts);
	CMP_FIELD_VAL(meta1, meta2, first_frame_sample_index);
	CMP_FIELD_VAL(meta1, meta2, media_id);
	CMP_FIELD_VAL(meta1, meta2, resource_index);
	if (meta1->principal_point.valid != meta2->principal_point.valid)
		return 0;
	else if (meta1->principal_point.valid)
		CMP_FIELD_PTR(meta1, meta2, principal_point.position);
	return 1;
}


int vmeta_session_is_valid(const struct vmeta_session *meta)
{
	ULOG_ERRNO_RETURN_VAL_IF(meta == NULL, EINVAL, 0);

	if (meta->friendly_name[0] == '\0')
		return 0;

	if (strcmp(meta->maker, "Parrot") != 0)
		return 0;

	if (meta->model[0] == '\0')
		return 0;

	return 1;
}
