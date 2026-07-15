/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 *
 * Declarations of parser for the ZYFlow-3901U optical flow sensor
 */

#include "zyflow_parser.h"
#include <string.h>
#include <stdlib.h>

//#define ZYFLOW_DEBUG

#ifdef ZYFLOW_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_UNSYNC",
	"1_HEADER",
	"2_NBYTES",
	"3_XM_L",
	"4_XM_H",
	"5_YM_L",
	"6_YM_H",
	"7_HT_L",
	"8_HT_H",
	"9_CHECKSUM",
	"10_QUALITY",
	"11_FOOTER"
};
#endif

bool zyflow_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum ZYFLOW_PARSE_STATE *state,
		     optical_flow_s *flow)
{
	bool parsed_packet = false;

	switch (*state) {
	case ZYFLOW_PARSE_STATE11_FOOTER:
		if (c == 0xFE) {
			*state = ZYFLOW_PARSE_STATE1_HEADER;

		} else {
			*state = ZYFLOW_PARSE_STATE0_UNSYNC;
		}

		break;

	case ZYFLOW_PARSE_STATE0_UNSYNC:
		if (c == 0xFE) {
			*state = ZYFLOW_PARSE_STATE1_HEADER;
		}

		break;

	case ZYFLOW_PARSE_STATE1_HEADER:
		if (c == 0x04) {
			*state = ZYFLOW_PARSE_STATE2_NBYTES;

		} else {
			*state = ZYFLOW_PARSE_STATE0_UNSYNC;
		}

		break;

	case ZYFLOW_PARSE_STATE2_NBYTES:
		*state = ZYFLOW_PARSE_STATE3_XM_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case ZYFLOW_PARSE_STATE3_XM_L:
		*state = ZYFLOW_PARSE_STATE4_XM_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case ZYFLOW_PARSE_STATE4_XM_H:
		*state = ZYFLOW_PARSE_STATE5_YM_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case ZYFLOW_PARSE_STATE5_YM_L:
		*state = ZYFLOW_PARSE_STATE6_YM_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;
	case ZYFLOW_PARSE_STATE6_YM_H:
		*state = ZYFLOW_PARSE_STATE7_HT_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;
	case ZYFLOW_PARSE_STATE7_HT_L:
		*state = ZYFLOW_PARSE_STATE8_HT_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case ZYFLOW_PARSE_STATE8_HT_H: {
			unsigned char cksm = 0;

			// Calculate checksum over motion values
			for (int i = 0; i < 6; i++) {
				cksm += parserbuf[i];
			}

			if (c == cksm) {
				// Checksum valid, populate sensor report
				int16_t delta_x = uint16_t(parserbuf[1]) << 8 | parserbuf[0];
				int16_t delta_y = uint16_t(parserbuf[3]) << 8 | parserbuf[2];
				int16_t height = uint16_t(parserbuf[5]) << 8 | parserbuf[4];
				flow->pixel_flow_x_integral = static_cast<float>(delta_x) * (2.0e-3f);
				flow->pixel_flow_y_integral = static_cast<float>(delta_y) * (2.0e-3f);
				flow->ground_distance_m = static_cast<float>(height) * (1.0e-2f);
				*state = ZYFLOW_PARSE_STATE9_CHECKSUM;

			} else {
				*state = ZYFLOW_PARSE_STATE0_UNSYNC;
			}

			*parserbuf_index = 0;
		}

		break;

	case ZYFLOW_PARSE_STATE9_CHECKSUM:
		*state = ZYFLOW_PARSE_STATE10_QUALITY;
		flow->quality = uint8_t(c);

		break;

	case ZYFLOW_PARSE_STATE10_QUALITY:
		if (c == 0xAA) {
			*state = ZYFLOW_PARSE_STATE11_FOOTER;
			parsed_packet = true;

		} else {
			*state = ZYFLOW_PARSE_STATE0_UNSYNC;
		}

		break;

	}

#ifdef ZYFLOW_DEBUG
	printf("state: ZYFLOW_PARSE_STATE%s, got char: %#02x\n", parser_state[*state], c);
#endif

	return parsed_packet;
}
