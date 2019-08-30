
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libvideo-metadata-protobuf
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := protobuf generated code for libvideo-metadata
LOCAL_EXPORT_C_INCLUDES := $(call local-get-build-dir)/generated
LOCAL_C_INCLUDES := $(LOCAL_EXPORT_C_INCLUDES)
LOCAL_LIBRARIES := protobuf-c
LOCAL_CUSTOM_MACROS := \
	protoc-c-macro:c,generated,$(LOCAL_PATH)/proto/vmeta.proto

include $(BUILD_LIBRARY)


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
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_proto.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_v1.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_v2.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_frame_v3.h:$\
	$(LOCAL_PATH)/include/video-metadata/vmeta_session.h;

LOCAL_CFLAGS := -DVMETA_API_EXPORTS -fvisibility=hidden -std=gnu99

LOCAL_SRC_FILES := \
	src/vmeta_session.c \
	src/vmeta_frame.c \
	src/vmeta_json.c \
	src/vmeta_json_proto.c \
	src/vmeta_csv.c \
	src/vmeta_frame_proto.c \
	src/vmeta_frame_v1.c \
	src/vmeta_frame_v2.c \
	src/vmeta_frame_v3.c \
	src/vmeta_utils.c

LOCAL_LIBRARIES := \
	libvideo-metadata-protobuf \
	libfutils \
	libulog \
	protobuf-c \
	json

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
	libvideo-metadata \
	json

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libmp4 \
	OPTIONAL:libpcap

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := vmeta-json-to-csv
LOCAL_DESCRIPTION := Parrot Drones video metadata json to csv converter
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/vmeta_json_to_csv.py

LOCAL_COPY_FILES := $(LOCAL_SRC_FILES:=:usr/bin/)

include $(BUILD_CUSTOM)


ifdef TARGET_TEST

include $(CLEAR_VARS)
LOCAL_MODULE := tst-vmeta
LOCAL_C_INCLUDES := $(LOCAL_PATH)/src

LOCAL_SRC_FILES := \
	tests/vmeta_test.c \
	tests/vmeta_test_proto.c \
	tests/vmeta_test_utils.c

LOCAL_LIBRARIES := \
	libcunit \
	libfutils \
	libulog \
	libvideo-metadata

include $(BUILD_EXECUTABLE)

endif
