#!/usr/bin/env python3
# Copyright (c) 2016 Parrot Drones SAS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Parrot Drones SAS Company nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import json
import sys
import logging
import os.path as path
import multiprocessing as mproc


def flatten_dict(obj, column_name=[]):
    """
        Turns a multilevel dict generated from parsing a json file
        into a single level dict where keys are concatenations of
        multiple nested objects field names joined by underscores
        ({aggregated_key_from_nested_objects: non iterable value, ...})

        Column names can be null strings
        Values in arrays have column names suffixed by their indices
        Empty arrays are stored as an empty string at keys suffixed by '0'
        prevent adding extra non suffixed key if array has elem in other dicts
    """
    frame_infos = dict()

    if obj == dict() or obj is None:
        frame_infos.update({'_'.join(column_name): ''})
    elif obj == list():
        frame_infos.update({'_'.join(column_name + ['0']): ''})
    elif isinstance(obj, dict):
        for key in obj.keys():
            frame_infos.update(flatten_dict(obj[key], column_name + [key]))
    elif isinstance(obj, list):
        for i, elem in enumerate(obj):
            frame_infos.update(flatten_dict(elem, column_name + [str(i)]))
    else:
        frame_infos.update({'_'.join(column_name): obj})
    return frame_infos


def get_all_column_names(data_dicts):
    """
        Get the superset of all column names in all flattened dict

        It handles cases where dict1 = {"a": []} and dict2 = {"a": [{"b": b}]}
        the flatten_dict function will have stored at key 'a_0' an empty string
        while it should be under a_0_b, so it discards column name a_0
        when an encountered column name contains it (like in a_0_b)
    """
    column_names = []
    for frame in data_dicts:
        frame_column_names = frame.keys()
        column_names += [col for col in frame_column_names
                         if col not in column_names]

    sorted_col_names = sorted(column_names)
    # discard the alphabetically last column_name as potential garbage
    # (because it can't be contained in another column_name)
    potential_col_index_garbage = list(range(len(sorted_col_names) - 1))
    for frame in data_dicts:
        for col_id in potential_col_index_garbage:
            if frame.get(sorted_col_names[col_id], '') != '':
                potential_col_index_garbage.remove(col_id)
                if not len(potential_col_index_garbage):
                    break
    for col_id in potential_col_index_garbage:
        if sorted_col_names[col_id] in sorted_col_names[col_id + 1] and \
           sorted_col_names[col_id][-1] == '0':
            column_names.remove(sorted_col_names[col_id])
    return column_names


def write_csv_from_dicts(data_dicts, csv_file):
    with open(csv_file, 'w') as fo:
        if len(data_dicts):
            column_names = get_all_column_names(data_dicts)
            fo.write(','.join(column_names))
            fo.write('\n')
            for frame_data_dict in data_dicts:
                fo.write(','.join([str(frame_data_dict.get(col_name, ''))
                                   for col_name in column_names]))
                fo.write('\n')
        else:
            logging.debug('Empty value under key "frame" provided.')


def generate_csv(json_file, csv_file):
    with open(json_file) as f:
        try:
            json_data = json.load(f)

            # select frame metadata (discard session ...)
            json_data = json_data["frame"]

            # parallelized flattening data
            MAX_AVAILABLE_PROCESSES = mproc.cpu_count() - 1
            p = mproc.Pool(processes=MAX_AVAILABLE_PROCESSES)
            frames_info_dicts = list(p.map(flatten_dict, json_data))
            p.close()

            write_csv_from_dicts(frames_info_dicts, csv_file)

        except TypeError:
            logging.error('{} should contain an object as root, found {}.'
                          .format(json_file, type(json_data).__name__))
        except json.decoder.JSONDecodeError as e:
            logging.error('{} induce json loading error : {}.'
                          .format(json_file, str(e)))
            sys.exit(-1)
        except KeyError:
            logging.error('{} should contain field "frame" under root object.'
                          .format(json_file))


def args_comply(i_file, o_file):
    i_file = path.abspath(i_file)
    compliant = path.exists(i_file)
    if compliant:
        ibase_path, iext = path.splitext(i_file)
        if iext.lower() != '.json':
            compliant = False
            raise Exception('{} must be a json file.'.format(i_file))
        elif o_file is not None:
            compliant = path.splitext(o_file)[-1].lower() == '.csv'
            if not compliant:
                raise Exception('{} must be a csv file.'.format(o_file))
        else:
            o_file = ibase_path + '.csv'
            compliant = True
    else:
        raise Exception('Json file {} must exist.'.format(i_file))
    return o_file


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-i', '--input_file', help='Input json file.',
                   required=True)
    p.add_argument('-o', '--output_file', help='Output csv file.')
    p.add_argument('-v', '--verbose', action='store_true', default=False,
                   help='Be extra verbose in printing of log messages.')
    opts = p.parse_args()

    if opts.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)
    FORMAT = '%(asctime)-15s %(levelname)s %(funcName)-20s %(message)s'
    logging.basicConfig(format=FORMAT)

    output_file = args_comply(opts.input_file, opts.output_file)
    generate_csv(opts.input_file, output_file)


if __name__ == "__main__":
    main()
