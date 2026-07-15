/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the uart pmw3901 opticalflow series
 */

#include "nloopuwb_parser.h"
#include <string.h>
#include <stdlib.h>

//#define NLOOPUWB_DEBUG

#ifdef NLOOPUWB_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_UNSYNC",
	"1_HEADER",
	"2_FUNCTION_MARK",
	"3_PAYLOAD",
	"4_GOT_CHECKSUM",
};
#endif

int nlinkuwb_parse(uint8_t c, uint8_t *parserbuf, unsigned *parserbuf_index, enum NLOOPUWB_PARSE_STATE *state, nlinkframe *dframe)
{
	static uint8_t _crc_expected = 0;
	uint msgbuf_len_max = 256;
	uint frame_len = 128;
	int ret = -1 ;
	//char *end;

	switch (*state) {
	case NLOOPUWB_PARSE_STATE0_UNSYNC:
		*parserbuf_index = 0;
		if (c == 0x55) {
			*state = NLOOPUWB_PARSE_STATE1_HEADER;
			//printf("state: %s\n", parser_state[*state]);
			_crc_expected = c;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = NLOOPUWB_PARSE_STATE0_UNSYNC;
		}

		break;

	case NLOOPUWB_PARSE_STATE1_HEADER:
		if (c == 0x1) {//data length
			*state = NLOOPUWB_PARSE_STATE2_FUNCTION_MARK;
			//printf("state: %s\n", parser_state[*state]);
			parserbuf[*parserbuf_index] = c;
			_crc_expected += c;

			(*parserbuf_index)++;

		} else {
			*state = NLOOPUWB_PARSE_STATE0_UNSYNC;
			*parserbuf_index = 0;
		}

		break;

	case NLOOPUWB_PARSE_STATE2_FUNCTION_MARK:
		*state = NLOOPUWB_PARSE_STATE3_PAYLOAD;
		//printf("state: %s\n", parser_state[*state]);

		parserbuf[*parserbuf_index] = c;
		_crc_expected += c;
		(*parserbuf_index)++;
		break;

	case NLOOPUWB_PARSE_STATE3_PAYLOAD:
		if (*parserbuf_index < msgbuf_len_max) {
			parserbuf[*parserbuf_index] = c;
        }
		if (*parserbuf_index >= frame_len-1) {
            *state = NLOOPUWB_PARSE_STATE0_UNSYNC;
            // check crc
			//printf("crc %x ==? %x\n", c, _crc_expected);
			if(c  == _crc_expected)
				return 0;
			else
				return -1;
        } else {
           	_crc_expected += c;
			(*parserbuf_index)++;
        }
        break;
    }
#ifdef NLOOPUWB_DEBUG
	printf("state: NLOOPUWB_PARSE_STATE%s\n", parser_state[*state]);
#endif
	return ret;
}
void nlinkuwb_parse_msgbuf(uint8_t* parserbuf, nlinkframe*dframe)
{
	dframe->pos_x = ((int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSX+2] << 24 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSX+1] << 16 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSX] << 8) >> 8;
	dframe->pos_y = ((int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSY+2] << 24 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSY+1] << 16 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSY] << 8) >> 8;
	dframe->pos_z = ((int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSZ+2] << 24 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSZ+1] << 16 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_POSZ] << 8) >> 8;

	dframe->vel_x = ((int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELX+2] << 24 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELX+1] << 16 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELX] << 8) >> 8;
	dframe->vel_y = ((int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELY+2] << 24 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELY+1] << 16 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELY] << 8) >> 8;
	dframe->vel_z = ((int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELZ+2] << 24 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELZ+1] << 16 | (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_VELZ] << 8) >> 8;

	dframe->eop_x =  (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_EOPX];
	dframe->eop_y =  (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_EOPY];
	dframe->eop_z =  (int32_t)parserbuf[NOOPLOOP_TAG_FRAME0_EOPZ];
}
