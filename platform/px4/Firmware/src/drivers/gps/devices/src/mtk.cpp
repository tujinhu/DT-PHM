/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mtk.cpp
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#include "mtk.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <math.h>


GPSDriverMTK::GPSDriverMTK(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	// decodeInit();
}

int
GPSDriverMTK::configure(unsigned &baudrate, const GPSConfig &config)
{

// 	if (config.output_mode != OutputMode::GPS) {
// 		GPS_WARN("MTK: Unsupported Output Mode %i", (int)config.output_mode);
// 		return -1;
// 	}

// 	if (baudrate > 0 && baudrate != MTK_BAUDRATE) {
// 		return -1;
// 	}

	/* set baudrate first */

	if (GPSHelper::setBaudrate(MTK_BAUDRATE) != 0) {
		return -1;
	}

// 	/* Write config messages, don't wait for an answer */
// 	if (strlen(MTK_OUTPUT_5HZ) != write(MTK_OUTPUT_5HZ, strlen(MTK_OUTPUT_5HZ))) {
// 		goto errout;
// 	}

// 	gps_usleep(10000);

// 	if (strlen(MTK_SET_BINARY) != write(MTK_SET_BINARY, strlen(MTK_SET_BINARY))) {
// 		goto errout;
// 	}

// 	gps_usleep(10000);

// 	if (strlen(MTK_SBAS_ON) != write(MTK_SBAS_ON, strlen(MTK_SBAS_ON))) {
// 		goto errout;
// 	}

// 	gps_usleep(10000);

// 	if (strlen(MTK_WAAS_ON) != write(MTK_WAAS_ON, strlen(MTK_WAAS_ON))) {
// 		goto errout;
// 	}

// 	gps_usleep(10000);

// 	if (strlen(MTK_NAVTHRES_OFF) != write(MTK_NAVTHRES_OFF, strlen(MTK_NAVTHRES_OFF))) {
// 		goto errout;
// 	}

	return 0;

// errout:
// 	GPS_WARN("mtk: config write failed");
// 	return -1;
}

int
GPSDriverMTK::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];
	// packet_d7_t packet{};

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	// int times_w = 0;
	// uint8_t len;
	// decodeInit();
	// printf("new state: %d",_decode_state);
	GPS_PARSE = 0;
	while (true) {
		int ret = read(buf, sizeof(buf), timeout);
		// printf("reeeeet:%d\n",ret);
		if (ret > 0) {
			/* first read whatever is left */
			for (int j=0; j<ret; j++) {
				uint8_t kk = parseChar(buf[j]);
				if (kk > 0) {
					handleMessage();
					if (GPS_PARSE > 2)
						return 1;
				}
				// printf("state:%d, char:%#02x, count:%d , parseKK: %#02x, ret%d, retj:%d , times_w:%d\n",_decode_state,buf[j], _rx_count,kk,ret,ret - j,times_w);
			}


		} else {
			gps_usleep(20000);
		}

		/* in case we keep trying but only get crap from GPS */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			return -1;
		}
	}
}

void
GPSDriverMTK::decodeInit()
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_double_dle = 0;
	_numPosType = 0;
	_numSV = 0;
	_numsv_cnt = 0;
	_decode_state = GPS_PARSE_STATE0_DLE;
}

unsigned int
GPSDriverMTK::crc_calculate(uint8_t *dat,unsigned int dat_l)
{
	unsigned int data;
	unsigned int xx,yy,zz;
	unsigned int i;
	xx=yy=zz=0;
	dat++;
	for(i=0;i<dat_l;i++)
	{
		data = (unsigned int)(*(dat+1)) << 8;
		data = (data | (*dat));
		// printf("crc:%d",data);
		yy=crc_table[data&0xff];
		yy ^= ((xx>>8)&0xff);
		zz=crc_table[xx&0xff];
		xx=yy^zz;
		dat = dat + 2;                              /* next word */
	}
	return xx;
}

int
GPSDriverMTK::parseChar(uint8_t b)
{
	int ret = 0;
	switch (_decode_state) {
		case GPS_PARSE_STATE0_DLE:
			// printf(" DLE0 count:%d , packet: %#02x\n", _rx_count-1,((uint8_t *)(&packet))[_rx_count-1]);
			_rx_count = 0;
			if (b == 0x10) {
				_decode_state = GPS_PARSE_STATE1_ID;
			} else {
				decodeInit();
			}
			break;

		case GPS_PARSE_STATE1_ID:
			// printf("   ID count:%d , packet: %#02x\n", _rx_count-1,((uint8_t *)(&packet))[_rx_count-1]);
			_rx_count = 0;
			if (b == 0xD7) {
				_decode_state = GPS_PARSE_D7_DATA0;
				GPS_PARSE++;
				// printf("GPS_PARSE_D7_DATA0 \n");

			} else if(b == 0x47){
				_decode_state = GPS_PARSE_47_DATA0;
				GPS_PARSE++;
				// printf("GPS_PARSE_47_DATA0 \n");
			}else{
				decodeInit();
			}
			break;

		case GPS_PARSE_D7_DATA0:
			// printf("DATA0 count:%d , packet: %#02x\n", _rx_count-1,((uint8_t *)(&packet))[_rx_count-1]);
			if (b == 0x10){
				_double_dle++;
				if (_double_dle == 2){
					((uint8_t *)(&_packet_d7))[_rx_count] = b;
					_rx_count++;
					_double_dle = 0;
				}
			}else if((b != 0x10) && (_double_dle == 0)){
				((uint8_t *)(&_packet_d7))[_rx_count] = b;
				_rx_count++;
			}else{
				decodeInit();
			}

			// if ((51+3*61) < (_rx_count) ){
			if(sizeof(packet_d7_t) == _rx_count){
				_rx_count = 0;
				_decode_state = GPS_PARSE_D7_DATA1;
				_numPosType = 0;
			}
			break;

		case GPS_PARSE_D7_DATA1:
			if (b == 0x10){
				_double_dle++;
				if (_double_dle == 2){
					((uint8_t *)(&pos_msg_bd2[_numPosType]))[_rx_count] = b;
					_rx_count++;
					_double_dle = 0;
				}
			}else if((b != 0x10) && (_double_dle == 0)){
				((uint8_t *)(&pos_msg_bd2[_numPosType]))[_rx_count] = b;
				_rx_count++;
			}else{
				decodeInit();
			}
			if(sizeof(pos_msg) == _rx_count)
			{
				_rx_count = 0;
				_numPosType++;
				if (_numPosType >= 3){
					_decode_state = GPS_PARSE_STATE3_CRC1;
				}
			}
			break;

		case GPS_PARSE_47_DATA0:
			// printf("DATA0 count:%d , packet: %#02x\n", _rx_count-1,((uint8_t *)(&packet))[_rx_count-1]);
			if (b == 0x10){
				_double_dle++;
				if (_double_dle == 2){
					((uint8_t *)(&_packet_47))[_rx_count] = b;
					_rx_count++;
					_double_dle = 0;
				}
			}else if((b != 0x10) && (_double_dle == 0)){
				((uint8_t *)(&_packet_47))[_rx_count] = b;
				_rx_count++;
			}else{
				decodeInit();
			}
			// printf("%#02x \t",((uint8_t *)(&_packet_d7))[4]);
			if (_rx_count == sizeof(_packet_47)){
				_numSV  = ((uint8_t *)(&_packet_47))[0] + ((uint8_t *)(&_packet_47))[1]  + ((uint8_t *)(&_packet_47))[2] ;
				// _gps_position->satellites_used = ((uint8_t *)(&_packet_47))[2];
				if (_numSV == 0){
					_decode_state = GPS_PARSE_STATE3_CRC1;
				}else{
					_decode_state = GPS_PARSE_47_DATA1;
				}


				_double_dle = 0;
				_rx_count = 0;
			}

			break;

		case GPS_PARSE_47_DATA1:
			// printf("DATA1 count:%d , packet: %#02x\n", _rx_count-1,((uint8_t *)(&packet))[_rx_count-1]);
			if (b == 0x10){
				_double_dle++;
				if (_double_dle == 2){
					((uint8_t *)(&satellite_info[_numsv_cnt]))[_rx_count] = b;
					_rx_count++;
					_double_dle = 0;
				}
			}else if((b != 0x10) && (_double_dle == 0)){
				((uint8_t *)(&satellite_info[_numsv_cnt]))[_rx_count] = b;
				_rx_count++;
			}else{
				decodeInit();
			}


			if (_rx_count == sizeof(nmuSV)){
				_rx_count = 0;
				_numsv_cnt++;
				if (_numsv_cnt == _numSV){
					// crc_result = crc_calculate((uint8_t *)(&_packet_d7),_rx_count);
					// printf("%d \n",crc_result);
					_decode_state = GPS_PARSE_STATE3_CRC1;

				}
			}
			break;


		case GPS_PARSE_STATE3_CRC1:
			if (b == 0x10){
				_double_dle++;
				if (_double_dle == 2){
					_decode_state = GPS_PARSE_STATE4_CRC2;
					_double_dle = 0;
				}
			}else if((b != 0x10) && (_double_dle == 0)){
				_decode_state = GPS_PARSE_STATE4_CRC2;
			}else{
				decodeInit();
			}

			break;

		case GPS_PARSE_STATE4_CRC2:
			if (b == 0x10){
				_double_dle++;
				if (_double_dle == 2){
					_decode_state = GPS_PARSE_STATE5_DLE;
					_double_dle = 0;
				}
			}else if((b != 0x10) && (_double_dle == 0)){
				_decode_state = GPS_PARSE_STATE5_DLE;
			}else{
				decodeInit();
			}

			break;

		case GPS_PARSE_STATE5_DLE:
			// printf(" DLE1 count:%d , packet: %#02x\n", _rx_count-1,((uint8_t *)(&packet))[_rx_count-1]);
			// printf("state:%d, char:%#02x \n",_decode_state,b);
			if (b == 0x10){
				_decode_state = GPS_PARSE_STATE6_ETX;
				// printf("1111111111111\n");
			}
			else
				_decode_state = GPS_PARSE_STATE0_DLE;
			break;

		case GPS_PARSE_STATE6_ETX:
			// printf("state:%d, char:%#02x \n",_decode_state,b);
			if (b == 0x03){
				_decode_state = GPS_PARSE_STATE0_DLE;
				// len = _rx_count+1;
				// printf("len: %d \n",len);
				_rx_count = 0;
				ret = 1;
				decodeInit();
			}else
				_decode_state = GPS_PARSE_STATE0_DLE;
			break;

		default:
			break;
	}
	return ret;
}

void
GPSDriverMTK::handleMessage(void)
{

	_gps_position->lat = pos_msg_bd2[0].latitude; // from degrees*1e6 to degrees*1e7
	_gps_position->lon = pos_msg_bd2[0].longitude; // from degrees*1e6 to degrees*1e7
	_gps_position->alt = (int32_t)(pos_msg_bd2[0].msl_altitude * 10); // from cm to mm

	if (pos_msg_bd2[0].fix_type & 0x40){ //Real-Time Kinematic, INT,
		_gps_position->fix_type = 6;
	}else if(pos_msg_bd2[0].fix_type & 0x20){ //Real-Time Kinematic, float,
		_gps_position->fix_type = 5;
	}else if(pos_msg_bd2[0].fix_type & 0x10){ //伪距差分
		_gps_position->fix_type = 3;
	}else if(pos_msg_bd2[0].fix_type & 0x08){ //广域差分解
		_gps_position->fix_type = 3;
	}else if(pos_msg_bd2[0].fix_type & 0x04){ //单点
		_gps_position->fix_type = 3;
	}else{
		_gps_position->fix_type = 0;
	}

	_gps_position->satellites_used = _packet_47.num_satellites_b3;


	_gps_position->hdop = pos_msg_bd2[0].PDOP;
	_gps_position->vdop = pos_msg_bd2[0].PDOP;

	_gps_position->vel_n_m_s = pos_msg_bd2[0].n_vel;
	_gps_position->vel_e_m_s = pos_msg_bd2[0].e_vel;
	_gps_position->vel_d_m_s = -1 * pos_msg_bd2[0].up_vel;


	// timeinfo.tm_isdst = 0;
	_gps_position->time_utc_usec = _packet_d7.sec_utc * 1000000ULL;
	_gps_position->timestamp = gps_absolute_time();
	_gps_position->timestamp_time_relative = 0;

	// Position and velocity update always at the same time
	// _rate_count_vel++;
	// _rate_count_lat_lon++;
}

void
GPSDriverMTK::addByteToChecksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}
