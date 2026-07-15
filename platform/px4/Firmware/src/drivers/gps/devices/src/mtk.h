/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mtk.h
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

#define MTK_SYNC1_V16 0xd0
#define MTK_SYNC1_V19 0xd1
#define MTK_SYNC2 0xdd

#define MTK_OUTPUT_5HZ		"$PMTK220,200*2C\r\n"
#define MTK_SET_BINARY		"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SBAS_ON	       	"$PMTK313,1*2E\r\n"
#define MTK_WAAS_ON		"$PMTK301,2*2E\r\n"
#define MTK_NAVTHRES_OFF 	"$PMTK397,0*23\r\n"

#define MTK_TIMEOUT_5HZ 400
#define MTK_BAUDRATE 115200

typedef enum {
	GPS_PARSE_STATE0_DLE = 0,
	GPS_PARSE_STATE1_ID,
	GPS_PARSE_D7_DATA0,
	GPS_PARSE_D7_DATA1,
	GPS_PARSE_47_DATA0,
	GPS_PARSE_47_DATA1,
	GPS_PARSE_STATE3_CRC1,
	GPS_PARSE_STATE4_CRC2,
	GPS_PARSE_STATE5_DLE,
	GPS_PARSE_STATE6_ETX,
	GPS_PARSE_STATE7
} mtk_decode_state_t;

/** the structures of the binary packets */
#pragma pack(push, 1)

// typedef struct {
// 	uint8_t payload; ///< Number of payload bytes
// 	int32_t latitude;  ///< Latitude in degrees * 10^7
// 	int32_t longitude; ///< Longitude in degrees * 10^7
// 	uint32_t msl_altitude;  ///< MSL altitude in meters * 10^2
// 	uint32_t ground_speed; ///< velocity in m/s
// 	int32_t heading; ///< heading in degrees * 10^2
// 	uint8_t satellites; ///< number of satellites used
// 	uint8_t fix_type;  ///< fix type: XXX correct for that
// 	uint32_t date;
// 	uint32_t utc_time;
// 	uint16_t hdop; ///< horizontal dilution of position (without unit)
// 	uint8_t ck_a;
// 	uint8_t ck_b;
// } gps_mtk_packet_t;


typedef struct {
	uint8_t		id;
	uint8_t		type;
	uint8_t		status;
	uint8_t		ch_status;
	uint8_t		pitch;
	uint16_t	yaw;
	uint8_t		snr;
	u_int32_t	undefine;

	// uint16_t	crc16;

} nmuSV;


typedef struct {
	uint8_t		num_satellites_l1;
	uint8_t		num_satellites_b1;
	uint8_t		num_satellites_b3;
	uint32_t	tic_cnt;
	// uint16_t	crc16;

} packet_47_t;

typedef struct{
	uint8_t		fix_type;
	int32_t		x_ecef;/* data */
	int32_t		y_ecef;
	int32_t		z_ecef;
	float		vx_ecef;
	float		vy_ecef;
	float		vz_ecef;
	int32_t		latitude;
	int32_t		longitude;
	float		msl_altitude;
	float		e_vel;
	float		n_vel;
	float		up_vel;
	float		PDOP;
	float		clock_error;
	float		clock_offset;
}pos_msg;

typedef struct {
	uint32_t	reserve0;/* data */
	uint8_t		time_vali;
	uint16_t	reserve1;
	double		reserve2;
	uint16_t	reserve3;
	double		reserve4;
	uint16_t	week_bd2;
	double		sec_db2;
	uint16_t	year_utc;
	uint8_t		moth_utc;
	uint8_t		day_utc;
	uint8_t		hour_utc;
	uint8_t		min_utc;
	double		sec_utc;
	uint8_t		num_postpye;
	// pos_msg		bd2_b3;
	// pos_msg		bd2_0;
	// pos_msg		bd2_1;
	// uint16_t	crc16;

} packet_d7_t;



#pragma pack(pop)

#define MTK_RECV_BUFFER_SIZE 40

class GPSDriverMTK : public GPSHelper
{
public:
	GPSDriverMTK(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position);
	virtual ~GPSDriverMTK() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, const GPSConfig &config) override;

private:
	/**
	 * Parse the binary MTK packet
	 */
	int parseChar(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	void handleMessage(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit();

	unsigned int crc_calculate(uint8_t *dat,unsigned int dat_l);
	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void addByteToChecksum(uint8_t);

	sensor_gps_s *_gps_position {nullptr};
	mtk_decode_state_t _decode_state{};
	uint8_t _mtk_revision{0};

	packet_d7_t _packet_d7{0};
	packet_47_t _packet_47{0};

	pos_msg pos_msg_bd2[3]{0};

	nmuSV	satellite_info[40]{0};

	unsigned _rx_count{};
	unsigned _double_dle{};
	unsigned _numPosType{0};
	unsigned _numSV{0};

	uint8_t GPS_PARSE{0};

	unsigned _numsv_cnt{0};

	uint8_t _rx_ck_a{};
	uint8_t _rx_ck_b{};

	unsigned int crc_result ;
	unsigned int crc_table[256]={
					0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,0x8c48,
					0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7,0x1081,0x0108,0x3393,
					0x221A,0x56A5,0x472C,0x75B7,0x643E,0x9CC9,0x8D40,0xBFDB,0xAE52,0xDAED,
					0xCB64,0xF9FF,0xE876,0x2102,0x308B,0x0210,0x1399,0x6726,0x76AF,0x4434,
					0x55BD,0xAD4A,0xBCC3,0x8E58,0x9FD1,0xEB6E,0xFAE7,0xC87C,0xD9F5,0x3183,
					0x200A,0x1291,0x0318,0x77A7,0x662E,0x54B5,0x453C,0xBDCB,0xAC42,0x9ED9,
					0x8F50,0xFBEF,0xEA66,0xD8FD,0xC974,0x4204,0x538D,0x6116,0x709F,0x0420,
					0x15A9,0x2732,0x36BB,0xCE4C,0xDFC5,0xED5E,0xFCD7,0x8868,0x99E1,0xAB7A,
					0xBAF3,0x5285,0x430C,0x7197,0x601E,0x14A1,0x0528,0x37B3,0x263A,0xDECD,
					0xCF44,0xFDDF,0xEC56,0x98E9,0x8960,0xBBFB,0xAA72,0x6306,0x728F,0x4014,
					0x519D,0x2522,0x34AB,0x0630,0x17B9,0xEF4E,0xFEC7,0xCC5C,0xDDD5,0xA96A,
					0xB8E3,0x8A78,0x9BF1,0x7387,0x620E,0x5095,0x411C,0x35A3,0x242A,0x16B1,
					0x0738,0xFFCF,0xEE46,0xDCDD,0xCD54,0xB9EB,0xA862,0x9AF9,0x8B70,0x8408,
					0x9581,0xA71A,0xB693,0xC22C,0xD3A5,0xE13E,0xF0B7,0x0840,0x19C9,0x2B52,
					0x3ADB,0x4E64,0x5FED,0x6D76,0x7CFF,0x9489,0x8500,0xB79B,0xA612,0xD2AD,
					0xC324,0xF1BF,0xE036,0x18C1,0x0948,0x3BD3,0x2A5A,0x5EE5,0x4F6C,0x7DF7,
					0x6C7E,0xA50A,0xB483,0x8618,0x9791,0xE32E,0xF2A7,0xC03C,0xD1B5,0x2942,
					0x38CB,0x0A50,0x1BD9,0x6F66,0x7EEF,0x4C74,0x5DFD,0xB58B,0xA402,0x9699,
					0x8710,0xF3AF,0xE226,0xD0BD,0xC134,0x39C3,0x284A,0x1AD1,0x0B58,0x7FE7,
					0x6E6E,0x5CF5,0x4D7C,0xC60C,0xD785,0xE51E,0xF497,0x8028,0x91A1,0xA33A,
					0xB2B3,0x4A44,0x5BCD,0x6956,0x78DF,0x0C60,0x1DE9,0x2F72,0x3EFB,0xD68D,
					0xC704,0xF59F,0xE416,0x90A9,0x8120,0xB3BB,0xA232,0x5AC5,0x4B4C,0x79D7,
					0x685E,0x1CE1,0x0D68,0x3FF3,0x2E7A,0xE70E,0xF687,0xC41C,0xD595,0xA12A,
					0xB0A3,0x8238,0x93B1,0x6B46,0x7ACF,0x4854,0x59DD,0x2D62,0x3CEB,0x0E70,
					0x1FF9,0xF78F,0xE606,0xD49D,0xC514,0xB1AB,0xA022,0x92B9,0x8330,0x7BC7,
					0x6A4E,0x58D5,0x495C,0x3DE3,0x2C6A,0x1EF1,0x0F78};
};
