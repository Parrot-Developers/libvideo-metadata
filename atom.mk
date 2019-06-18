
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libvideo-metadata
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Parrot Drones video metadata library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBVIDEOMETADATA_HEADERS=$\
	$(LOCAL_PATH)/include/video-metadata/vmeta.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_v1.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_v2.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_v3.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_session.h;

LOCAL_CFLAGS := -DVMETA_API_EXPORTS -fvisibility=hidden -std=gnu99

LOCAL_SRC_FILES := \
	src/vmeta_session.c \
	src/vmeta_frame.c \
	src/vmeta_json.c \
	src/vmeta_csv.c \
	src/vmeta_frame_v1.c \
	src/vmeta_frame_v2.c \
	src/vmeta_frame_v3.c \
	src/vmeta_utils.c

LOCAL_LIBRARIES := \
	libfutils \
	libulog

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:json

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)


ifeq ("$(TARGET_OS_FLAVOUR)","native")
  ifeq ("$(shell which pcap-config &> /dev/null; echo $$?)", "0")
    include $(CLEAR_VARS)
    LOCAL_MODULE := libpcap
    LOCAL_EXPORT_CFLAGS := $(shell pcap-config --cflags)
    LOCAL_EXPORT_LDLIBS := $(shell pcap-config --libs)
    $(call local-register-prebuilt-overridable)
  endif
endif


include $(CLEAR_VARS)

LOCAL_MODULE := vmeta-extract
LOCAL_DESCRIPTION := Parrot Drones video metadata extractor tool
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/vmeta_extract.c

LOCAL_LIBRARIES := \
	libulog \
	libvideo-metadata

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libmp4 \
	OPTIONAL:libpcap \
	OPTIONAL:json

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)
