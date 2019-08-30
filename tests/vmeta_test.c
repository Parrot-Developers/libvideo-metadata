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

#include "vmeta_test.h"

ULOG_DECLARE_TAG(ULOG_TAG);

static CU_SuiteInfo s_suites[] = {
	{(char *)"vmeta utils", NULL, NULL, s_utils_tests},
	{(char *)"vmeta frame protobuf", NULL, NULL, s_proto_tests},
	CU_SUITE_INFO_NULL,
};

static CU_SuiteInfo s_monkey_suites[] = {
	{(char *)"protobuf monkey tests", NULL, NULL, s_proto_monkey},
	CU_SUITE_INFO_NULL,
};

static CU_SuiteInfo s_dump_suites[] = {
	{(char *)"protobuf reference buffer gen", NULL, NULL, s_proto_gen},
	CU_SUITE_INFO_NULL,
};

int main(int argc, char *argv[])
{
	int i;
	CU_initialize_registry();
	CU_register_suites(s_suites);

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "monkey") == 0)
			CU_register_suites(s_monkey_suites);
		if (strcmp(argv[i], "dump") == 0) {
			CU_cleanup_registry();
			CU_initialize_registry();
			CU_register_suites(s_dump_suites);
			break;
		}
	}

	if (getenv("CUNIT_OUT_NAME") != NULL)
		CU_set_output_filename(getenv("CUNIT_OUT_NAME"));
	if (getenv("CUNIT_AUTOMATED") != NULL) {
		CU_automated_run_tests();
		CU_list_tests_to_file();
	} else {
		CU_basic_set_mode(CU_BRM_VERBOSE);
		CU_basic_run_tests();
	}
	CU_cleanup_registry();
	return 0;
}