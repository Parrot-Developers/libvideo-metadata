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

#ifndef _VMETA_JSON_H_
#define _VMETA_JSON_H_

#ifdef BUILD_JSON


static inline int
vmeta_json_add_int(struct json_object *jobj, const char *name, int val)
{
	json_object_object_add(jobj, name, json_object_new_int(val));
	return 0;
}


static inline int
vmeta_json_add_int64(struct json_object *jobj, const char *name, int64_t val)
{
	json_object_object_add(jobj, name, json_object_new_int64(val));
	return 0;
}


static inline int
vmeta_json_add_double(struct json_object *jobj, const char *name, double val)
{
	json_object_object_add(jobj, name, json_object_new_double(val));
	return 0;
}


static inline int
vmeta_json_add_str(struct json_object *jobj, const char *name, const char *val)
{
	json_object_object_add(jobj, name, json_object_new_string(val));
	return 0;
}


int vmeta_json_add_location(struct json_object *jobj,
			    const char *name,
			    const struct vmeta_location *val);


int vmeta_json_add_quaternion(struct json_object *jobj,
			      const char *name,
			      const struct vmeta_quaternion *val);


int vmeta_json_add_euler(struct json_object *jobj,
			 const char *name,
			 const struct vmeta_euler *val);


int vmeta_json_add_xyz(struct json_object *jobj,
		       const char *name,
		       const struct vmeta_xyz *val);


int vmeta_json_add_ned(struct json_object *jobj,
		       const char *name,
		       const struct vmeta_ned *val);


int vmeta_json_add_fov(struct json_object *jobj,
		       const char *name,
		       const struct vmeta_fov *val);


int vmeta_json_add_thermal_conversion(
	struct json_object *jobj,
	const char *name,
	const struct vmeta_thermal_conversion *val);


#endif /* BUILD_JSON */

#endif /* !_VMETA_JSON_H_ */
