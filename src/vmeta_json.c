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


int vmeta_json_add_location(struct json_object *jobj,
			    const char *name,
			    const struct vmeta_location *val)
{
	if (!val->valid)
		return 0;

	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "latitude", val->latitude);
	vmeta_json_add_double(jobj_val, "longitude", val->longitude);
	if (!isnan(val->altitude_wgs84ellipsoid)) {
		vmeta_json_add_double(jobj_val,
				      "altitude_wgs84ellipsoid",
				      val->altitude_wgs84ellipsoid);
	}
	if (!isnan(val->altitude_egm96amsl)) {
		vmeta_json_add_double(jobj_val,
				      "altitude_egm96amsl",
				      val->altitude_egm96amsl);
	}
	if (val->horizontal_accuracy != 0.) {
		vmeta_json_add_double(jobj_val,
				      "horizontal_accuracy",
				      val->horizontal_accuracy);
	}
	if (val->vertical_accuracy != 0.) {
		vmeta_json_add_double(
			jobj_val, "vertical_accuracy", val->vertical_accuracy);
	}
	if (val->sv_count != VMETA_LOCATION_INVALID_SV_COUNT)
		vmeta_json_add_int(jobj_val, "sv_count", val->sv_count);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_quaternion(struct json_object *jobj,
			      const char *name,
			      const struct vmeta_quaternion *val)
{
	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "w", val->w);
	vmeta_json_add_double(jobj_val, "x", val->x);
	vmeta_json_add_double(jobj_val, "y", val->y);
	vmeta_json_add_double(jobj_val, "z", val->z);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_euler(struct json_object *jobj,
			 const char *name,
			 const struct vmeta_euler *val)
{
	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "yaw", val->yaw);
	vmeta_json_add_double(jobj_val, "pitch", val->pitch);
	vmeta_json_add_double(jobj_val, "roll", val->roll);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_xyz(struct json_object *jobj,
		       const char *name,
		       const struct vmeta_xyz *val)
{
	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "x", val->x);
	vmeta_json_add_double(jobj_val, "y", val->y);
	vmeta_json_add_double(jobj_val, "z", val->z);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_ned(struct json_object *jobj,
		       const char *name,
		       const struct vmeta_ned *val)
{
	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "north", val->north);
	vmeta_json_add_double(jobj_val, "east", val->east);
	vmeta_json_add_double(jobj_val, "down", val->down);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_fov(struct json_object *jobj,
		       const char *name,
		       const struct vmeta_fov *val)
{
	if ((!val->has_horz) && (!val->has_vert))
		return 0;

	struct json_object *jobj_val = json_object_new_object();

	if (val->has_horz)
		vmeta_json_add_double(jobj_val, "horz", val->horz);

	if (val->has_vert)
		vmeta_json_add_double(jobj_val, "vert", val->vert);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_thermal_conversion(
	struct json_object *jobj,
	const char *name,
	const struct vmeta_thermal_conversion *val)
{
	if (!val->valid)
		return 0;

	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "r", val->r);
	vmeta_json_add_double(jobj_val, "b", val->b);
	vmeta_json_add_double(jobj_val, "f", val->f);
	vmeta_json_add_double(jobj_val, "o", val->o);
	vmeta_json_add_double(jobj_val, "tau_win", val->tau_win);
	vmeta_json_add_double(jobj_val, "t_win", val->t_win);
	vmeta_json_add_double(jobj_val, "t_bg", val->t_bg);
	vmeta_json_add_double(jobj_val, "emissivity", val->emissivity);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}


int vmeta_json_add_thermal_spot(struct json_object *jobj,
				const char *name,
				const struct vmeta_thermal_spot *val)
{
	if (!val->valid)
		return 0;

	struct json_object *jobj_val = json_object_new_object();

	vmeta_json_add_double(jobj_val, "x", val->x);
	vmeta_json_add_double(jobj_val, "y", val->y);
	vmeta_json_add_double(jobj_val, "temp", val->temp);

	json_object_object_add(jobj, name, jobj_val);
	return 0;
}
