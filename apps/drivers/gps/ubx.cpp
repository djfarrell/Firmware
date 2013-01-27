/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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

/* @file U-Blox protocol implementation */


#include <unistd.h>
#include <assert.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_gps.h>

#include "ubx.h"


UBX::UBX()
{
	_decode_state = UBX_DECODE_UNINIT;
	_rx_count = 0;
}

UBX::~UBX()
{
}

void
UBX::configure(uint8_t* buffer, int& length, const unsigned max_length)
{

	int current_gps_speed = 38400; //XXX fix this

	assert(sizeof(type_gps_bin_cfg_prt_packet_t) <= max_length);

	type_gps_bin_cfg_prt_packet_t* packet = (type_gps_bin_cfg_prt_packet_t*)buffer;

	packet->clsID		= UBX_CLASS_CFG;
	packet->msgID		= UBX_MESSAGE_CFG_PRT;
	packet->length		= UBX_CFG_PRT_LENGTH;
	packet->portID		= UBX_CFG_PRT_PAYLOAD_PORTID;
	packet->mode		= UBX_CFG_PRT_PAYLOAD_MODE;
	packet->baudRate	= current_gps_speed;
	packet->inProtoMask	= UBX_CFG_PRT_PAYLOAD_INPROTOMASK;
	packet->outProtoMask= UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK;

	//TODO: set CRC here

	length = sizeof(type_gps_bin_cfg_prt_packet_t);
}

unsigned
UBX::parse(uint8_t b)
{

	return 0;
}
