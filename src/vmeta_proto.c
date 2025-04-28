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


Vmeta__CameraSubtype
vmeta_camera_subtype_vmeta_to_proto(enum vmeta_camera_subtype subtype)
{
	Vmeta__CameraSubtype out = VMETA__CAMERA_SUBTYPE__CST_UNKNOWN;
	switch (subtype) {
	case VMETA_CAMERA_SUBTYPE_LEFT:
		out = VMETA__CAMERA_SUBTYPE__CST_LEFT;
		break;
	case VMETA_CAMERA_SUBTYPE_RIGHT:
		out = VMETA__CAMERA_SUBTYPE__CST_RIGHT;
		break;
	case VMETA_CAMERA_SUBTYPE_WIDE:
		out = VMETA__CAMERA_SUBTYPE__CST_WIDE;
		break;
	case VMETA_CAMERA_SUBTYPE_TELE:
		out = VMETA__CAMERA_SUBTYPE__CST_TELE;
		break;
	case VMETA_CAMERA_SUBTYPE_DISPARITY:
		out = VMETA__CAMERA_SUBTYPE__CST_DISPARITY;
		break;
	case VMETA_CAMERA_SUBTYPE_DEPTH:
		out = VMETA__CAMERA_SUBTYPE__CST_DEPTH;
		break;
	default:
		break;
	}
	return out;
}


VMETA_API enum vmeta_camera_subtype
vmeta_camera_subtype_proto_to_vmeta(Vmeta__CameraSubtype subtype)
{
	enum vmeta_camera_subtype out = VMETA_CAMERA_SUBTYPE_UNKNOWN;
	switch (subtype) {
	case VMETA__CAMERA_SUBTYPE__CST_LEFT:
		out = VMETA_CAMERA_SUBTYPE_LEFT;
		break;
	case VMETA__CAMERA_SUBTYPE__CST_RIGHT:
		out = VMETA_CAMERA_SUBTYPE_RIGHT;
		break;
	case VMETA__CAMERA_SUBTYPE__CST_WIDE:
		out = VMETA_CAMERA_SUBTYPE_WIDE;
		break;
	case VMETA__CAMERA_SUBTYPE__CST_TELE:
		out = VMETA_CAMERA_SUBTYPE_TELE;
		break;
	case VMETA__CAMERA_SUBTYPE__CST_DISPARITY:
		out = VMETA_CAMERA_SUBTYPE_DISPARITY;
		break;
	case VMETA__CAMERA_SUBTYPE__CST_DEPTH:
		out = VMETA_CAMERA_SUBTYPE_DEPTH;
		break;
	default:
		break;
	}
	return out;
}
