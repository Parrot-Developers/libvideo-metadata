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


size_t vmeta_csv_add_location(char *str,
			      size_t maxlen,
			      const struct vmeta_location *val)
{
	size_t len = 0;

	if (val->valid) {
		VMETA_STR_PRINT(
			str,
			len,
			maxlen,
			"%d %.8lf %.8lf %.2lf %d",
			val->valid,
			val->latitude,
			val->longitude,
			val->altitude,
			(val->sv_count != VMETA_LOCATION_INVALID_SV_COUNT)
				? val->sv_count
				: 0);
	} else {
		VMETA_STR_PRINT(str,
				len,
				maxlen,
				"%d %.8lf %.8lf %.2lf %d",
				0,
				0.,
				0.,
				0.,
				0);
	}

	return len;
}


size_t vmeta_csv_add_quaternion(char *str,
				size_t maxlen,
				const struct vmeta_quaternion *val)
{
	size_t len = 0;

	VMETA_STR_PRINT(str,
			len,
			maxlen,
			"%.5f %.5f %.5f %.5f",
			val->w,
			val->x,
			val->y,
			val->z);

	return len;
}


size_t
vmeta_csv_add_euler(char *str, size_t maxlen, const struct vmeta_euler *val)
{
	size_t len = 0;

	VMETA_STR_PRINT(str,
			len,
			maxlen,
			"%.4f %.4f %.4f",
			val->yaw,
			val->pitch,
			val->roll);

	return len;
}


size_t vmeta_csv_add_xyz(char *str, size_t maxlen, const struct vmeta_xyz *val)
{
	size_t len = 0;

	VMETA_STR_PRINT(
		str, len, maxlen, "%.3f %.3f %.3f", val->x, val->y, val->z);

	return len;
}


size_t vmeta_csv_add_ned(char *str, size_t maxlen, const struct vmeta_ned *val)
{
	size_t len = 0;

	VMETA_STR_PRINT(str,
			len,
			maxlen,
			"%.3f %.3f %.3f",
			val->north,
			val->east,
			val->down);

	return len;
}
