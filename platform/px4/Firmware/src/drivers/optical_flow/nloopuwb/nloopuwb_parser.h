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
 * Declarations of parser for the Benewake PMW3901UART laser rangefinder series
 */

#pragma once

// Data Format for Benewake PMW3901UART
// ===============================
// 9 bytes total per message:
// 1) 0x59
// 2) 0x59
// 3) Dist_L (low 8bit)
// 4) Dist_H (high 8bit)
// 5) Strength_L (low 8bit)
// 6) Strength_H (high 8bit)
// 7) Reserved bytes
// 8) Original signal quality degree
// 9) Checksum parity bit (low 8bit), Checksum = Byte1 + Byte2 +...+Byte8. This is only a low 8bit though


enum NLOOPUWB_PARSE_STATE {
	NLOOPUWB_PARSE_STATE0_UNSYNC = 0,
	NLOOPUWB_PARSE_STATE1_HEADER,
	NLOOPUWB_PARSE_STATE2_FUNCTION_MARK,
	NLOOPUWB_PARSE_STATE3_PAYLOAD,
};
typedef  struct nlinkframe {
	uint8_t id;
	uint8_t role;
	int32_t pos_x;
	int32_t pos_y;
	int32_t pos_z;
	int32_t vel_x;
	int32_t vel_y;
	int32_t vel_z;
	int32_t dis_0;
	int32_t dis_1;
	int32_t dis_2;
	int32_t dis_3;
	int32_t dis_4;
	int32_t dis_5;
	int32_t dis_6;
	int32_t dis_7;
	float g_x;
	float g_y;
	float g_z;
	float a_x;
	float a_y;
	float a_z;
	int16_t angle_x;
	int16_t angle_y;
	int16_t angle_z;
	float q_0;
	float q_1;
	float q_2;
	float q_3;
	uint32_t local_time;
	uint32_t system_time;
	uint8_t eop_x;
	uint8_t eop_y;
	uint8_t eop_z;
	uint16_t voltage;
} nlinkframe;

int nlinkuwb_parse(uint8_t c, uint8_t *parserbuf, unsigned *parserbuf_index, enum NLOOPUWB_PARSE_STATE *state, nlinkframe *dframe);
void nlinkuwb_parse_msgbuf(uint8_t* parserbuf, nlinkframe*dframe);

#define NOOPLOOP_TAG_FRAME0_POSX           4      // start of 3 bytes holding x position in m*1000
#define NOOPLOOP_TAG_FRAME0_POSY           7      // start of 3 bytes holding y position in m*1000
#define NOOPLOOP_TAG_FRAME0_POSZ           10      // start of 3 bytes holding z position in m*1000
#define NOOPLOOP_TAG_FRAME0_VELX           13      // start of 3 bytes holding x velocity in m/s*10000
#define NOOPLOOP_TAG_FRAME0_VELY           16      // start of 3 bytes holding y velocity in m/s*10000
#define NOOPLOOP_TAG_FRAME0_VELZ           19      // start of 3 bytes holding z velocity in m/s*10000
#define NOOPLOOP_TAG_FRAME0_EOPX           117      // start of 1 bytes holding x eop in m*100
#define NOOPLOOP_TAG_FRAME0_EOPY           118     // start of 1 bytes holding y eop in m*100
#define NOOPLOOP_TAG_FRAME0_EOPZ           119     // start of 1 bytes holding z eop in m*100