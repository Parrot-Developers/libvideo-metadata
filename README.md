# libvideo-metadata - Parrot Drones video metadata library

libvideo-metadata is a C library that handles the serialization and
de-serialization of video metadata in both streaming and recording.

## Features

* serializing/de-serializing of frame (timed) metadata
* serializing/de-serializing of session (untimed) metadata
* metadata conversion to CSV or JSON formats
* various helper functions

## Dependencies

The library depends on the following Alchemy modules:

* libfutils
* libulog
* json (optional)

## Building

Building is activated by enabling _libvideo-metadata_ in the Alchemy build
configuration.

## Operation

### Threading model

The API functions are not thread safe and should always be called from the
same thread, or it is the caller's resposibility to synchronize calls if
multiple threads are used.

### Writer

#### Frame metadata

As a writer the library takes as input _vmeta_frame_ structures and serializes
the data as _vmeta_buffer_ objects, ready to use in a stream sender (see
_libvideo-streaming_) or in a MP4 muxer.

#### Session metadata

As a writer the library takes as input a _vmeta_session_ structure and
serializes the data as either data for a stream sender (SDES RTCP packets -
see _libvideo-streaming_ - and/or SDP attributes - see _libsdp_) or for a
MP4 muxer (in _meta_ or _udta_ boxes).

### Reader

#### Frame metadata

As a reader the library takes as input _vmeta_buffer_ objects coming either from
a stream receiver (see _libvideo-streaming_) or a MP4 demuxer (see _libmp4_)
and de-serializes the data as _vmeta_frame_ structures.

#### Session metadata

As a writer the library takes as input data coming either from a stream
receiver (SDES RTCP packets - see _libvideo-streaming_ - and/or SDP
attributes - see _libsdp_) or a MP4 demuxer (values from _meta_ or _udta_
boxes - see _libmp4_) and de-serializes the data as a _vmeta_session_
structure.

## Testing

The library can be tested using the provided _vmeta-extract_ command-line tool
which takes as input a MP4 video file or a *.pcap packet capture and optionally
outputs CSV, JSON and KML files.

To build the tool, enable _vmeta-extract_ in the Alchemy build configuration.
To use with mp4 files as input, _libmp4_ must be enabled in the build
configuration. To use with *.pcap files, the _libpcap-dev_ package must be
installed on the system. For JSON output _json_ must be enabled in the build
configuration.

For a list of available options, run

    $ vmeta-extract -h
