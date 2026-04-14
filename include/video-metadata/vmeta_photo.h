/**
 * Copyright (c) 2026 Parrot Drones SAS
 */

#ifndef _VMETA_PHOTO_H_
#define _VMETA_PHOTO_H_

#include <photo-metadata-defs/pmeta_defs.h>
#include <video-metadata/vmeta_frame.h>
#include <video-metadata/vmeta_session.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * 'Exif' or 'XMP' item writing callback function.
 *
 * This callback is triggered for each metadata entry to be written to the
 * photo payload (JPEG/XMP or DNG). It uses a strongly-typed approach
 * relying on constant definitions for performance and safety.
 *
 * @param dest:      Item destination (PMETA_DEFS_DEST_EXIF or XMP).
 * @param exif_def:  For EXIF: pointer to the constant tag definition.
 * For XMP: NULL.
 * @param xmp_def:   For XMP: pointer to the constant XMP definition.
 * For EXIF: NULL.
 * @param value:     Item value formatted as a null-terminated string.
 * @param userdata:  User-defined pointer (e.g., muxer instance).
 */
typedef void (*vmeta_photo_write_cb_t)(
	enum pmeta_defs_dest dest,
	const struct pmeta_defs_exif_def *exif_def,
	const struct pmeta_defs_xmp_def *xmp_def,
	const char *value,
	void *userdata);


/**
 * Write session metadata as Exif or XMP items.
 * The function is called for a whole session metadata structure and calls the
 * cb callback function for each Exif or XMP item that should be written.
 * For each call to the cb function, the item type, key, tag ID and value are
 * given. Both key and value are null-terminated.
 * @param meta: pointer to the session metadata structure
 * @param cb: 'Exif' or 'XMP' item writing callback function
 * @param userdata: 'Exif' or 'XMP' item writing callback function user data
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_session_photo_write(const struct vmeta_session *meta,
			      vmeta_photo_write_cb_t cb,
			      void *userdata);


/**
 * Write frame metadata as Exif or XMP items.
 * The function is called for a frame metadata structure and calls the
 * cb callback function for each Exif or XMP item that should be written.
 * For each call to the cb function, the item type, key, tag ID and value are
 * given. Both key and value are null-terminated.
 * @param meta: pointer to the frame metadata structure
 * @param cb: 'Exif' or 'XMP' item writing callback function
 * @param userdata: 'Exif' or 'XMP' item writing callback function user data
 * @return 0 on success, negative errno value in case of error
 */
VMETA_API
int vmeta_frame_photo_write(const struct vmeta_frame *meta,
			    vmeta_photo_write_cb_t cb,
			    void *userdata);


#ifdef __cplusplus
}
#endif

#endif /* _VMETA_PHOTO_H_ */
