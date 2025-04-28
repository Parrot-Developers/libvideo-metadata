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


struct vmeta_session_proto {
	/* Encoded part */
	int packed;
	uint8_t *buf;
	size_t len;

	/* Decoded part */
	int unpacked;
	Vmeta__SessionMetadata *meta;

	/* lock */
	pthread_mutex_t lock;
	uint32_t rp_lock;
	uint32_t ru_lock;
	uint32_t w_lock;
};


static int vmeta_session_proto_alloc(struct vmeta_session_proto **meta)
{
	int res;

	*meta = calloc(1, sizeof(**meta));
	if (!*meta)
		return -ENOMEM;

	res = pthread_mutex_init(&(*meta)->lock, NULL);
	if (res != 0) {
		free(*meta);
		*meta = NULL;
		return -res;
	}

	return 0;
}


static int vmeta_session_proto_pack(struct vmeta_session_proto *meta)
{
	size_t len;

	/* If the metadata is already packed, this is a no-op */
	if (meta->packed)
		return 0;

	/* Do not pack if the metadata is packed-read or write locked */
	if (meta->rp_lock || meta->w_lock)
		return -EBUSY;

	/* If the metadata is neither packed nor unpacked, we have a problem */
	if (!meta->unpacked)
		return -EINVAL;

	len = vmeta__session_metadata__get_packed_size(meta->meta);
	meta->buf = malloc(len);
	if (!meta->buf)
		return -ENOMEM;

	meta->len = vmeta__session_metadata__pack(meta->meta, meta->buf);
	meta->packed = 1;

	return 0;
}


static int vmeta_session_proto_unpack(struct vmeta_session_proto *meta)
{
	/* If the metadata is already unpacked, this is a no-op */
	if (meta->unpacked)
		return 0;

	/* Do not unpack if the metadata is unpacked-read or write locked */
	if (meta->ru_lock || meta->w_lock)
		return -EBUSY;

	/* If the metadata is neither packed nor unpacked, we have a problem */
	if (!meta->packed)
		return -EINVAL;

	meta->meta =
		vmeta__session_metadata__unpack(NULL, meta->len, meta->buf);
	if (meta->meta == NULL)
		return -EPROTO;
	meta->unpacked = 1;

	return 0;
}


static int vmeta_session_proto_init(struct vmeta_session_proto **meta)
{
	int res;
	struct vmeta_session_proto *l_meta;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	res = vmeta_session_proto_alloc(&l_meta);
	if (res != 0)
		return res;
	l_meta->meta = calloc(1, sizeof(*l_meta->meta));
	if (!l_meta->meta) {
		res = -ENOMEM;
		goto error;
	}
	vmeta__session_metadata__init(l_meta->meta);
	l_meta->unpacked = 1;
	*meta = l_meta;

	return 0;

error:
	vmeta_session_proto_destroy(l_meta);
	*meta = NULL;
	return res;
}


int vmeta_session_proto_destroy(struct vmeta_session_proto *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	if (meta->rp_lock)
		ULOGW("metadata destroyed with %" PRIu32
		      " packed-read-lock held",
		      meta->rp_lock);
	if (meta->ru_lock)
		ULOGW("metadata destroyed with %" PRIu32
		      " unpacked-read-lock held",
		      meta->ru_lock);
	if (meta->w_lock)
		ULOGW("metadata destroyed with write-lock held");

	if (meta->packed)
		free(meta->buf);

	if (meta->unpacked)
		vmeta__session_metadata__free_unpacked(meta->meta, NULL);

	pthread_mutex_destroy(&meta->lock);
	free(meta);

	return 0;
}


Vmeta__Location *
vmeta_session_proto_get_takeoff_location(Vmeta__SessionMetadata *proto_meta)
{
	Vmeta__Location *location;

	ULOG_ERRNO_RETURN_VAL_IF(!proto_meta, EINVAL, NULL);

	if (proto_meta->takeoff_location)
		return proto_meta->takeoff_location;
	location = calloc(1, sizeof(*location));
	if (!location) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__location__init(location);
	proto_meta->takeoff_location = location;
	return location;
}


Vmeta__Vector2 *
vmeta_session_proto_get_picture_fov(Vmeta__SessionMetadata *proto_meta)
{
	Vmeta__Vector2 *picture_fov;

	ULOG_ERRNO_RETURN_VAL_IF(!proto_meta, EINVAL, NULL);

	if (proto_meta->picture_fov)
		return proto_meta->picture_fov;
	picture_fov = calloc(1, sizeof(*picture_fov));
	if (!picture_fov) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__vector2__init(picture_fov);
	proto_meta->picture_fov = picture_fov;
	return picture_fov;
}


Vmeta__ThermalSessionMetadata *vmeta_session_proto_get_thermal_session_metadata(
	Vmeta__SessionMetadata *proto_meta)
{
	Vmeta__ThermalSessionMetadata *thermal;

	ULOG_ERRNO_RETURN_VAL_IF(!proto_meta, EINVAL, NULL);

	if (proto_meta->thermal)
		return proto_meta->thermal;
	thermal = calloc(1, sizeof(*thermal));
	if (!thermal) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__thermal_session_metadata__init(thermal);
	proto_meta->thermal = thermal;
	return thermal;
}


Vmeta__ThermalAlignment *vmeta_session_proto_get_thermal_alignment(
	Vmeta__ThermalSessionMetadata *thermal_meta)
{
	Vmeta__ThermalAlignment *alignment;

	ULOG_ERRNO_RETURN_VAL_IF(!thermal_meta, EINVAL, NULL);

	if (thermal_meta->alignment)
		return thermal_meta->alignment;
	alignment = calloc(1, sizeof(*alignment));
	if (!alignment) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__thermal_alignment__init(alignment);
	thermal_meta->alignment = alignment;
	return alignment;
}


Vmeta__Euler *vmeta_session_proto_get_thermal_alignment_rotation(
	Vmeta__ThermalAlignment *thermal_alignment)
{
	Vmeta__Euler *rotation;

	ULOG_ERRNO_RETURN_VAL_IF(!thermal_alignment, EINVAL, NULL);

	if (thermal_alignment->rotation)
		return thermal_alignment->rotation;
	rotation = calloc(1, sizeof(*rotation));
	if (!rotation) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__euler__init(rotation);
	thermal_alignment->rotation = rotation;
	return rotation;
}


Vmeta__ThermalConversion *vmeta_session_proto_get_thermal_conversion_low(
	Vmeta__ThermalSessionMetadata *thermal_meta)
{
	Vmeta__ThermalConversion *conv_low;

	ULOG_ERRNO_RETURN_VAL_IF(!thermal_meta, EINVAL, NULL);

	if (thermal_meta->conv_low)
		return thermal_meta->conv_low;
	conv_low = calloc(1, sizeof(*conv_low));
	if (!conv_low) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__thermal_conversion__init(conv_low);
	thermal_meta->conv_low = conv_low;
	return conv_low;
}


Vmeta__ThermalConversion *vmeta_session_proto_get_thermal_conversion_high(
	Vmeta__ThermalSessionMetadata *thermal_meta)
{
	Vmeta__ThermalConversion *conv_high;

	ULOG_ERRNO_RETURN_VAL_IF(!thermal_meta, EINVAL, NULL);

	if (thermal_meta->conv_high)
		return thermal_meta->conv_high;
	conv_high = calloc(1, sizeof(*conv_high));
	if (!conv_high) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__thermal_conversion__init(conv_high);
	thermal_meta->conv_high = conv_high;
	return conv_high;
}


Vmeta__CameraModel *
vmeta_session_proto_get_camera_model(Vmeta__SessionMetadata *proto_meta)
{
	Vmeta__CameraModel *camera_model;

	ULOG_ERRNO_RETURN_VAL_IF(!proto_meta, EINVAL, NULL);

	if (proto_meta->camera_model)
		return proto_meta->camera_model;
	camera_model = calloc(1, sizeof(*camera_model));
	if (!camera_model) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_model__init(camera_model);
	proto_meta->camera_model = camera_model;
	return camera_model;
}


Vmeta__CameraModel__PerspectiveCameraModel *
vmeta_session_proto_get_perspective_camera_model(
	Vmeta__CameraModel *camera_model)
{
	Vmeta__CameraModel__PerspectiveCameraModel *perspective;

	ULOG_ERRNO_RETURN_VAL_IF(!camera_model, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(
		camera_model->id_case != VMETA__CAMERA_MODEL__ID__NOT_SET &&
			camera_model->id_case !=
				VMETA__CAMERA_MODEL__ID_PERSPECTIVE,
		EINVAL,
		NULL);

	if (camera_model->perspective)
		return camera_model->perspective;
	perspective = calloc(1, sizeof(*perspective));
	if (!perspective) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_model__perspective_camera_model__init(perspective);
	camera_model->id_case = VMETA__CAMERA_MODEL__ID_PERSPECTIVE;
	camera_model->perspective = perspective;
	return perspective;
}


Vmeta__CameraModel__PerspectiveCameraModel__Distorsion *
vmeta_session_proto_get_perspective_camera_model_distorsion(
	Vmeta__CameraModel__PerspectiveCameraModel *perspective)
{
	Vmeta__CameraModel__PerspectiveCameraModel__Distorsion *distorsion;

	ULOG_ERRNO_RETURN_VAL_IF(!perspective, EINVAL, NULL);

	if (perspective->distorsion)
		return perspective->distorsion;
	distorsion = calloc(1, sizeof(*distorsion));
	if (!distorsion) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_model__perspective_camera_model__distorsion__init(
		distorsion);
	perspective->distorsion = distorsion;
	return distorsion;
}


Vmeta__CameraModel__FisheyeCameraModel *
vmeta_session_proto_get_fisheye_camera_model(Vmeta__CameraModel *camera_model)
{
	Vmeta__CameraModel__FisheyeCameraModel *fisheye;

	ULOG_ERRNO_RETURN_VAL_IF(!camera_model, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(
		camera_model->id_case != VMETA__CAMERA_MODEL__ID__NOT_SET &&
			camera_model->id_case !=
				VMETA__CAMERA_MODEL__ID_FISHEYE,
		EINVAL,
		NULL);

	if (camera_model->fisheye)
		return camera_model->fisheye;
	fisheye = calloc(1, sizeof(*fisheye));
	if (!fisheye) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_model__fisheye_camera_model__init(fisheye);
	camera_model->id_case = VMETA__CAMERA_MODEL__ID_FISHEYE;
	camera_model->fisheye = fisheye;
	return fisheye;
}


Vmeta__CameraModel__FisheyeCameraModel__AffineMatrix *
vmeta_session_proto_get_fisheye_camera_model_affine_matrix(
	Vmeta__CameraModel__FisheyeCameraModel *fisheye)
{
	Vmeta__CameraModel__FisheyeCameraModel__AffineMatrix *affine_matrix;

	ULOG_ERRNO_RETURN_VAL_IF(!fisheye, EINVAL, NULL);

	if (fisheye->affine_matrix)
		return fisheye->affine_matrix;
	affine_matrix = calloc(1, sizeof(*affine_matrix));
	if (!affine_matrix) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_model__fisheye_camera_model__affine_matrix__init(
		affine_matrix);
	fisheye->affine_matrix = affine_matrix;
	return affine_matrix;
}


Vmeta__CameraModel__FisheyeCameraModel__Polynomial *
vmeta_session_proto_get_fisheye_camera_model_polynomial(
	Vmeta__CameraModel__FisheyeCameraModel *fisheye)
{
	Vmeta__CameraModel__FisheyeCameraModel__Polynomial *polynomial;

	ULOG_ERRNO_RETURN_VAL_IF(!fisheye, EINVAL, NULL);

	if (fisheye->polynomial)
		return fisheye->polynomial;
	polynomial = calloc(1, sizeof(*polynomial));
	if (!polynomial) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__camera_model__fisheye_camera_model__polynomial__init(polynomial);
	fisheye->polynomial = polynomial;
	return polynomial;
}


Vmeta__Overlay *
vmeta_session_proto_get_overlay(Vmeta__SessionMetadata *proto_meta)
{
	Vmeta__Overlay *overlay;

	ULOG_ERRNO_RETURN_VAL_IF(!proto_meta, EINVAL, NULL);

	if (proto_meta->overlay)
		return proto_meta->overlay;
	overlay = calloc(1, sizeof(*overlay));
	if (!overlay) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__overlay__init(overlay);
	proto_meta->overlay = overlay;
	return overlay;
}


Vmeta__Overlay__HeaderFooter *
vmeta_session_proto_get_overlay_header_footer(Vmeta__Overlay *overlay)
{
	Vmeta__Overlay__HeaderFooter *header_footer;

	ULOG_ERRNO_RETURN_VAL_IF(!overlay, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(
		overlay->id_case != VMETA__OVERLAY__ID__NOT_SET &&
			overlay->id_case != VMETA__OVERLAY__ID_HEADER_FOOTER,
		EINVAL,
		NULL);

	if (overlay->header_footer)
		return overlay->header_footer;
	header_footer = calloc(1, sizeof(*header_footer));
	if (!header_footer) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__overlay__header_footer__init(header_footer);
	overlay->id_case = VMETA__OVERLAY__ID_HEADER_FOOTER;
	overlay->header_footer = header_footer;
	return header_footer;
}


Vmeta__Vector2 *
vmeta_session_proto_get_principal_point(Vmeta__SessionMetadata *proto_meta)
{
	Vmeta__Vector2 *principal_point;

	ULOG_ERRNO_RETURN_VAL_IF(!proto_meta, EINVAL, NULL);

	if (proto_meta->principal_point)
		return proto_meta->principal_point;
	principal_point = calloc(1, sizeof(*principal_point));
	if (!principal_point) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}
	vmeta__vector2__init(principal_point);
	proto_meta->principal_point = principal_point;
	return principal_point;
}


int vmeta_session_to_proto(const struct vmeta_session *meta,
			   struct vmeta_session_proto **proto_meta)
{
	int res;
	struct vmeta_session_proto *new = NULL;
	Vmeta__SessionMetadata *proto = NULL;
	Vmeta__Location *loc = NULL;
	Vmeta__Vector2 *fov = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(proto_meta == NULL, EINVAL);

	res = vmeta_session_proto_init(&new);
	if (res < 0) {
		ULOG_ERRNO("vmeta_session_proto_init", -res);
		goto out;
	}

	/* Get the output metadata */
	res = vmeta_session_proto_get_unpacked_rw(new, &proto);
	if (res < 0) {
		ULOG_ERRNO("vmeta_session_proto_get_unpacked_rw", -res);
		goto out;
	}

	proto->friendly_name = strdup(meta->friendly_name);
	proto->maker = strdup(meta->maker);
	proto->model = strdup(meta->model);
	proto->model_id = strdup(meta->model_id);
	proto->serial_number = strdup(meta->serial_number);
	proto->software_version = strdup(meta->software_version);
	proto->build_id = strdup(meta->build_id);
	proto->title = strdup(meta->title);
	proto->comment = strdup(meta->comment);
	proto->copyright = strdup(meta->copyright);
	proto->media_date = meta->media_date;
	proto->media_date_gmtoff = meta->media_date_gmtoff;
	proto->boot_date = meta->boot_date;
	proto->boot_date_gmtoff = meta->boot_date_gmtoff;
	proto->boot_id = strdup(meta->boot_id);
	proto->flight_date = meta->flight_date;
	proto->flight_date_gmtoff = meta->flight_date_gmtoff;
	proto->flight_id = strdup(meta->flight_id);
	proto->custom_id = strdup(meta->custom_id);

	/* takeoff_loc */
	if (meta->takeoff_loc.valid) {
		loc = vmeta_session_proto_get_takeoff_location(proto);
		if (loc == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("vmeta_session_proto_get_takeoff_location",
				   -res);
			goto out;
		}
		loc->altitude_wgs84ellipsoid =
			meta->takeoff_loc.altitude_wgs84ellipsoid;
		if (isnan(loc->altitude_wgs84ellipsoid))
			loc->altitude_wgs84ellipsoid = 0.;
		else if (loc->altitude_wgs84ellipsoid == 0.)
			loc->altitude_wgs84ellipsoid = DBL_MIN;
		loc->altitude_egm96amsl = meta->takeoff_loc.altitude_egm96amsl;
		if (isnan(loc->altitude_egm96amsl))
			loc->altitude_egm96amsl = 0.;
		else if (loc->altitude_egm96amsl == 0.)
			loc->altitude_egm96amsl = DBL_MIN;
		loc->latitude = meta->takeoff_loc.latitude;
		loc->longitude = meta->takeoff_loc.longitude;
		loc->horizontal_accuracy =
			meta->takeoff_loc.horizontal_accuracy;
		loc->vertical_accuracy = meta->takeoff_loc.vertical_accuracy;
		loc->sv_count = meta->takeoff_loc.sv_count;
	}

	if (meta->picture_fov.has_horz && meta->picture_fov.has_vert) {
		fov = vmeta_session_proto_get_picture_fov(proto);
		if (fov == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("vmeta_session_proto_get_picture_fov", -res);
			goto out;
		}
		/* deg to rad */
		fov->x = meta->picture_fov.horz * M_PI / 180.;
		fov->y = meta->picture_fov.vert * M_PI / 180.;
	}

	if (meta->has_thermal) {
		Vmeta__ThermalSessionMetadata *thermal = NULL;
		Vmeta__ThermalAlignment *alignment = NULL;
		Vmeta__Euler *rotation = NULL;
		Vmeta__ThermalConversion *conv_low = NULL;
		Vmeta__ThermalConversion *conv_high = NULL;

		thermal =
			vmeta_session_proto_get_thermal_session_metadata(proto);
		if (thermal == NULL) {
			res = -EPROTO;
			ULOG_ERRNO(
				"vmeta_session_proto_get_"
				"thermal_session_metadata",
				-res);
			goto out;
		}

		thermal->metaversion = meta->thermal.metaversion;
		thermal->camera_serial_number = strdup(meta->thermal.camserial);

		alignment = vmeta_session_proto_get_thermal_alignment(thermal);
		if (alignment == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("vmeta_session_proto_get_thermal_alignment",
				   -res);
			goto out;
		}

		rotation = vmeta_session_proto_get_thermal_alignment_rotation(
			alignment);
		if (rotation == NULL) {
			res = -EPROTO;
			ULOG_ERRNO(
				"vmeta_session_proto_get_"
				"thermal_alignment_rotation",
				-res);
			goto out;
		}
		rotation->yaw = meta->thermal.alignment.rotation.yaw;
		rotation->pitch = meta->thermal.alignment.rotation.pitch;
		rotation->roll = meta->thermal.alignment.rotation.roll;

		conv_low =
			vmeta_session_proto_get_thermal_conversion_low(thermal);
		if (conv_low == NULL) {
			res = -EPROTO;
			ULOG_ERRNO(
				"vmeta_session_proto_get_thermal_conversion_low",
				-res);
			goto out;
		}
		conv_low->r = meta->thermal.conv_low.r;
		conv_low->b = meta->thermal.conv_low.b;
		conv_low->f = meta->thermal.conv_low.f;
		conv_low->o = meta->thermal.conv_low.o;
		conv_low->tau_win = meta->thermal.conv_low.tau_win;
		conv_low->t_win = meta->thermal.conv_low.t_win;
		conv_low->t_bg = meta->thermal.conv_low.t_bg;
		conv_low->emissivity = meta->thermal.conv_low.emissivity;

		conv_high = vmeta_session_proto_get_thermal_conversion_high(
			thermal);
		if (conv_high == NULL) {
			res = -EPROTO;
			ULOG_ERRNO(
				"vmeta_session_proto_get_"
				"thermal_conversion_high",
				-res);
			goto out;
		}
		conv_high->r = meta->thermal.conv_high.r;
		conv_high->b = meta->thermal.conv_high.b;
		conv_high->f = meta->thermal.conv_high.f;
		conv_high->o = meta->thermal.conv_high.o;
		conv_high->tau_win = meta->thermal.conv_high.tau_win;
		conv_high->t_win = meta->thermal.conv_high.t_win;
		conv_high->t_bg = meta->thermal.conv_high.t_bg;
		conv_high->emissivity = meta->thermal.conv_high.emissivity;

		thermal->scale_factor = meta->thermal.scale_factor;
	}

	proto->default_media = meta->default_media;
	proto->camera_type =
		vmeta_session_camera_type_vmeta_to_proto(meta->camera_type);
	proto->camera_subtype =
		vmeta_camera_subtype_vmeta_to_proto(meta->camera_subtype);
	proto->camera_spectrum = vmeta_session_camera_spectrum_vmeta_to_proto(
		meta->camera_spectrum);
	proto->camera_serial_number = strdup(meta->camera_serial_number);

	if (meta->camera_model.type != VMETA_CAMERA_MODEL_TYPE_UNKNOWN) {
		Vmeta__CameraModel *model = NULL;

		model = vmeta_session_proto_get_camera_model(proto);
		if (model == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("vmeta_session_proto_get_camera_model",
				   -res);
			goto out;
		}

		switch (meta->camera_model.type) {
		case VMETA_CAMERA_MODEL_TYPE_PERSPECTIVE: {
			Vmeta__CameraModel__PerspectiveCameraModel
				*perspective = NULL;
			Vmeta__CameraModel__PerspectiveCameraModel__Distorsion
				*distorsion = NULL;

			perspective =
				/* codecheck_ignore[LONG_LINE] */
				vmeta_session_proto_get_perspective_camera_model(
					model);
			if (perspective == NULL) {
				res = -EPROTO;
				ULOG_ERRNO(
					"vmeta_session_proto_get_"
					"perspective_camera_model",
					-res);
				goto out;
			}
			distorsion =
				/* codecheck_ignore[LONG_LINE] */
				vmeta_session_proto_get_perspective_camera_model_distorsion(
					perspective);
			if (distorsion == NULL) {
				res = -EPROTO;
				ULOG_ERRNO(
					"vmeta_session_proto_get_"
					"perspective_camera_model_distorsion",
					-res);
				goto out;
			}
			distorsion->r1 =
				meta->camera_model.perspective.distortion.r1;
			distorsion->r2 =
				meta->camera_model.perspective.distortion.r2;
			distorsion->r3 =
				meta->camera_model.perspective.distortion.r3;
			distorsion->t1 =
				meta->camera_model.perspective.distortion.t1;
			distorsion->t2 =
				meta->camera_model.perspective.distortion.t2;
			break;
		}
		case VMETA_CAMERA_MODEL_TYPE_FISHEYE: {
			Vmeta__CameraModel__FisheyeCameraModel *fisheye = NULL;
			Vmeta__CameraModel__FisheyeCameraModel__AffineMatrix
				*affine_matrix = NULL;
			Vmeta__CameraModel__FisheyeCameraModel__Polynomial
				*polynomial = NULL;

			fisheye = vmeta_session_proto_get_fisheye_camera_model(
				model);
			if (fisheye == NULL) {
				res = -EPROTO;
				ULOG_ERRNO(
					"vmeta_session_proto_get_"
					"fisheye_camera_model",
					-res);
				goto out;
			}
			affine_matrix =
				/* codecheck_ignore[LONG_LINE] */
				vmeta_session_proto_get_fisheye_camera_model_affine_matrix(
					fisheye);
			if (affine_matrix == NULL) {
				res = -EPROTO;
				ULOG_ERRNO(
					"vmeta_session_proto_get_"
					"fisheye_camera_model_affine_matrix",
					-res);
				goto out;
			}
			affine_matrix->c =
				meta->camera_model.fisheye.affine_matrix.c;
			affine_matrix->d =
				meta->camera_model.fisheye.affine_matrix.d;
			affine_matrix->e =
				meta->camera_model.fisheye.affine_matrix.e;
			affine_matrix->f =
				meta->camera_model.fisheye.affine_matrix.f;
			polynomial =
				/* codecheck_ignore[LONG_LINE] */
				vmeta_session_proto_get_fisheye_camera_model_polynomial(
					fisheye);
			if (polynomial == NULL) {
				res = -EPROTO;
				ULOG_ERRNO(
					"vmeta_session_proto_get_"
					"fisheye_camera_model_polynomial",
					-res);
				goto out;
			}
			polynomial->p2 =
				meta->camera_model.fisheye.polynomial.p2;
			polynomial->p3 =
				meta->camera_model.fisheye.polynomial.p3;
			polynomial->p4 =
				meta->camera_model.fisheye.polynomial.p4;
			break;
		}
		default:
			res = -ENOSYS;
			ULOGE("unknown camera_model type: %d",
			      meta->camera_model.type);
			goto out;
		}
	}

	if (meta->overlay.type != VMETA_OVERLAY_TYPE_NONE) {
		Vmeta__Overlay *overlay = NULL;

		overlay = vmeta_session_proto_get_overlay(proto);
		if (overlay == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("vmeta_session_proto_get_overlay", -res);
			goto out;
		}

		switch (meta->overlay.type) {
		case VMETA_OVERLAY_TYPE_HEADER_FOOTER: {
			Vmeta__Overlay__HeaderFooter *header_footer = NULL;

			header_footer =
				vmeta_session_proto_get_overlay_header_footer(
					overlay);
			if (header_footer == NULL) {
				res = -EPROTO;
				ULOG_ERRNO(
					"vmeta_session_proto_get_"
					"overlay_header_footer",
					-res);
				goto out;
			}
			header_footer->header_height =
				meta->overlay.header_footer.header_height;
			header_footer->footer_height =
				meta->overlay.header_footer.footer_height;
			break;
		}
		default:
			res = -ENOSYS;
			ULOGE("unknown overlay type: %d", meta->overlay.type);
			goto out;
		}
	}

	if (meta->principal_point.valid) {
		Vmeta__Vector2 *principal_point = NULL;

		principal_point =
			vmeta_session_proto_get_principal_point(proto);
		if (principal_point == NULL) {
			res = -EPROTO;
			ULOG_ERRNO("vmeta_session_proto_get_principal_point",
				   -res);
			goto out;
		}
		principal_point->x = meta->principal_point.position.x;
		principal_point->y = meta->principal_point.position.y;
	}

	proto->video_mode =
		vmeta_session_video_mode_vmeta_to_proto(meta->video_mode);
	proto->video_stop_reason =
		vmeta_session_video_stop_reason_vmeta_to_proto(
			meta->video_stop_reason);
	proto->dynamic_range =
		vmeta_session_dynamic_range_vmeta_to_proto(meta->dynamic_range);
	proto->tone_mapping =
		vmeta_session_tone_mapping_vmeta_to_proto(meta->tone_mapping);
	proto->first_frame_capture_ts = meta->first_frame_capture_ts;
	proto->first_frame_sample_index = meta->first_frame_sample_index;
	proto->media_id = meta->media_id;
	proto->resource_index = meta->resource_index;

out:
	if (proto != NULL)
		vmeta_session_proto_release_unpacked_rw(new, proto);
	if (res == 0)
		*proto_meta = new;
	else
		vmeta_session_proto_destroy(new);
	return res;
}


int vmeta_session_proto_get_unpacked(struct vmeta_session_proto *meta,
				     const Vmeta__SessionMetadata **proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);

	pthread_mutex_lock(&meta->lock);

	ret = vmeta_session_proto_unpack(meta);
	if (ret < 0)
		goto out;

	if (meta->w_lock) {
		ret = -EBUSY;
		goto out;
	}

	*proto_meta = meta->meta;
	meta->ru_lock++;

out:
	pthread_mutex_unlock(&meta->lock);

	return ret;
}


int vmeta_session_proto_release_unpacked(
	struct vmeta_session_proto *meta,
	const Vmeta__SessionMetadata *proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);

	pthread_mutex_lock(&meta->lock);

	if (!meta->ru_lock) {
		ULOGE("%s called with no unpacked-read-lock held", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (!meta->unpacked || proto_meta != meta->meta) {
		ULOGE("%s called with a wrong proto_meta", __func__);
		ret = -EPROTO;
		goto out;
	}

	meta->ru_lock--;

out:
	pthread_mutex_unlock(&meta->lock);

	return ret;
}


int vmeta_session_proto_get_unpacked_rw(struct vmeta_session_proto *meta,
					Vmeta__SessionMetadata **proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);
	;

	pthread_mutex_lock(&meta->lock);

	ret = vmeta_session_proto_unpack(meta);
	if (ret < 0)
		goto out;

	if (meta->ru_lock || meta->rp_lock || meta->w_lock) {
		ret = -EBUSY;
		goto out;
	}

	*proto_meta = meta->meta;
	meta->w_lock = 1;

out:
	pthread_mutex_unlock(&meta->lock);

	return ret;
}


int vmeta_session_proto_release_unpacked_rw(struct vmeta_session_proto *meta,
					    Vmeta__SessionMetadata *proto_meta)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!proto_meta, EINVAL);

	pthread_mutex_lock(&meta->lock);

	if (!meta->w_lock) {
		ULOGE("%s called with no write-lock held", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (!meta->unpacked || proto_meta != meta->meta) {
		ULOGE("%s called with a wrong proto_meta", __func__);
		ret = -EPROTO;
		goto out;
	}

	meta->w_lock = 0;

	if (meta->unpacked) {
		free(meta->buf);
		meta->buf = NULL;
		meta->len = 0;
		meta->packed = 0;
	}

out:
	pthread_mutex_unlock(&meta->lock);

	return ret;
}


int vmeta_session_proto_get_buffer(struct vmeta_session_proto *meta,
				   const uint8_t **buf,
				   size_t *len)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!len, EINVAL);

	pthread_mutex_lock(&meta->lock);

	ret = vmeta_session_proto_pack(meta);
	if (ret < 0)
		goto out;

	if (meta->w_lock) {
		ret = -EBUSY;
		goto out;
	}

	*buf = meta->buf;
	*len = meta->len;
	meta->rp_lock++;

out:
	pthread_mutex_unlock(&meta->lock);

	return ret;
}


int vmeta_session_proto_release_buffer(struct vmeta_session_proto *meta,
				       const uint8_t *buf)
{
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!buf, EINVAL);

	pthread_mutex_lock(&meta->lock);

	if (!meta->rp_lock) {
		ULOGE("%s called with no packed-read-lock held", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (!meta->packed || buf != meta->buf) {
		ULOGE("%s called with a wrong buffer", __func__);
		ret = -EPROTO;
		goto out;
	}

	meta->rp_lock--;

out:
	pthread_mutex_unlock(&meta->lock);

	return ret;
}


ssize_t vmeta_session_proto_get_packed_size(struct vmeta_session_proto *meta)
{
	ssize_t ret;

	ULOG_ERRNO_RETURN_ERR_IF(!meta, EINVAL);

	pthread_mutex_lock(&meta->lock);

	if (meta->packed)
		ret = meta->len;
	else if (meta->unpacked)
		ret = vmeta__session_metadata__get_packed_size(meta->meta);
	else
		ret = -EINVAL;

	pthread_mutex_unlock(&meta->lock);

	return ret;
}


Vmeta__CameraType
vmeta_session_camera_type_vmeta_to_proto(enum vmeta_camera_type type)
{
	Vmeta__CameraType out = VMETA__CAMERA_TYPE__CT_UNKNOWN;
	switch (type) {
	case VMETA_CAMERA_TYPE_FRONT:
		out = VMETA__CAMERA_TYPE__CT_FRONT;
		break;
	case VMETA_CAMERA_TYPE_FRONT_STEREO:
		out = VMETA__CAMERA_TYPE__CT_FRONT_STEREO;
		break;
	case VMETA_CAMERA_TYPE_FRONT_STEREO_LEFT:
		out = VMETA__CAMERA_TYPE__CT_FRONT_STEREO_LEFT;
		break;
	case VMETA_CAMERA_TYPE_FRONT_STEREO_RIGHT:
		out = VMETA__CAMERA_TYPE__CT_FRONT_STEREO_RIGHT;
		break;
	case VMETA_CAMERA_TYPE_VERTICAL:
		out = VMETA__CAMERA_TYPE__CT_VERTICAL;
		break;
	case VMETA_CAMERA_TYPE_DISPARITY:
		out = VMETA__CAMERA_TYPE__CT_DISPARITY;
		break;
	case VMETA_CAMERA_TYPE_HORIZONTAL_STEREO:
		out = VMETA__CAMERA_TYPE__CT_HORIZONTAL_STEREO;
		break;
	case VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_LEFT:
		out = VMETA__CAMERA_TYPE__CT_HORIZONTAL_STEREO_LEFT;
		break;
	case VMETA_CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT:
		out = VMETA__CAMERA_TYPE__CT_HORIZONTAL_STEREO_RIGHT;
		break;
	case VMETA_CAMERA_TYPE_DOWN_STEREO:
		out = VMETA__CAMERA_TYPE__CT_DOWN_STEREO;
		break;
	case VMETA_CAMERA_TYPE_DOWN_STEREO_LEFT:
		out = VMETA__CAMERA_TYPE__CT_DOWN_STEREO_LEFT;
		break;
	case VMETA_CAMERA_TYPE_DOWN_STEREO_RIGHT:
		out = VMETA__CAMERA_TYPE__CT_DOWN_STEREO_RIGHT;
		break;
	case VMETA_CAMERA_TYPE_EXTERNAL:
		out = VMETA__CAMERA_TYPE__CT_EXTERNAL;
		break;
	default:
		break;
	}
	return out;
}


Vmeta__CameraSpectrum vmeta_session_camera_spectrum_vmeta_to_proto(
	enum vmeta_camera_spectrum spectrum)
{
	return vmeta_frame_camera_spectrum_vmeta_to_proto(spectrum);
}


Vmeta__VideoMode
vmeta_session_video_mode_vmeta_to_proto(enum vmeta_video_mode mode)
{
	Vmeta__VideoMode out = VMETA__VIDEO_MODE__VM_UNKNOWN;
	switch (mode) {
	case VMETA_VIDEO_MODE_STANDARD:
		out = VMETA__VIDEO_MODE__VM_STANDARD;
		break;
	case VMETA_VIDEO_MODE_HYPERLAPSE:
		out = VMETA__VIDEO_MODE__VM_HYPERLAPSE;
		break;
	case VMETA_VIDEO_MODE_SLOWMOTION:
		out = VMETA__VIDEO_MODE__VM_SLOWMOTION;
		break;
	case VMETA_VIDEO_MODE_STREAMREC:
		out = VMETA__VIDEO_MODE__VM_STREAMREC;
		break;
	default:
		break;
	}
	return out;
}


Vmeta__VideoStopReason vmeta_session_video_stop_reason_vmeta_to_proto(
	enum vmeta_video_stop_reason reason)
{
	Vmeta__VideoStopReason out = VMETA__VIDEO_STOP_REASON__VSR_UNKNOWN;
	switch (reason) {
	case VMETA_VIDEO_STOP_REASON_USER:
		out = VMETA__VIDEO_STOP_REASON__VSR_USER;
		break;
	case VMETA_VIDEO_STOP_REASON_RECONFIGURATION:
		out = VMETA__VIDEO_STOP_REASON__VSR_RECONFIGURATION;
		break;
	case VMETA_VIDEO_STOP_REASON_POOR_STORAGE_PERF:
		out = VMETA__VIDEO_STOP_REASON__VSR_POOR_STORAGE_PERF;
		break;
	case VMETA_VIDEO_STOP_REASON_STORAGE_FULL:
		out = VMETA__VIDEO_STOP_REASON__VSR_STORAGE_FULL;
		break;
	case VMETA_VIDEO_STOP_REASON_RECOVERY:
		out = VMETA__VIDEO_STOP_REASON__VSR_RECOVERY;
		break;
	case VMETA_VIDEO_STOP_REASON_END_OF_STREAM:
		out = VMETA__VIDEO_STOP_REASON__VSR_END_OF_STREAM;
		break;
	case VMETA_VIDEO_STOP_REASON_SHUTDOWN:
		out = VMETA__VIDEO_STOP_REASON__VSR_SHUTDOWN;
		break;
	case VMETA_VIDEO_STOP_REASON_INTERNAL_ERROR:
		out = VMETA__VIDEO_STOP_REASON__VSR_INTERNAL_ERROR;
		break;
	default:
		break;
	}
	return out;
}


Vmeta__DynamicRange
vmeta_session_dynamic_range_vmeta_to_proto(enum vmeta_dynamic_range range)
{
	Vmeta__DynamicRange out = VMETA__DYNAMIC_RANGE__DR_UNKNOWN;
	switch (range) {
	case VMETA_DYNAMIC_RANGE_SDR:
		out = VMETA__DYNAMIC_RANGE__DR_SDR;
		break;
	case VMETA_DYNAMIC_RANGE_HDR8:
		out = VMETA__DYNAMIC_RANGE__DR_HDR8;
		break;
	case VMETA_DYNAMIC_RANGE_HDR10:
		out = VMETA__DYNAMIC_RANGE__DR_HDR10;
		break;
	default:
		break;
	}
	return out;
}


Vmeta__ToneMapping
vmeta_session_tone_mapping_vmeta_to_proto(enum vmeta_tone_mapping mapping)
{
	Vmeta__ToneMapping out = VMETA__TONE_MAPPING__TM_UNKNOWN;
	switch (mapping) {
	case VMETA_TONE_MAPPING_STANDARD:
		out = VMETA__TONE_MAPPING__TM_STANDARD;
		break;
	case VMETA_TONE_MAPPING_P_LOG:
		out = VMETA__TONE_MAPPING__TM_P_LOG;
		break;
	default:
		break;
	}
	return out;
}
