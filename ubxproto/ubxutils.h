/*
 * ubxproto
 * Copyright (c) 2014, Alexey Edelev aka semlanik, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * Additionally to GNU Lesser General Public License you MUST NOT
 * static link this library and MUST add link to author
 * and source of this library in your application.
 *
 * Actual LGPL text https://www.gnu.org/licenses/lgpl.html
 *
 * File: ubxutils.h
 */
/*! \file */

#ifndef UBLOXUTILS_H
#define UBLOXUTILS_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

//Additional API will be replaced with checkMessage(const UBXMsgBuffer* buffer);
extern void fletcherChecksum(unsigned char* buffer, int size, unsigned char* checkSumA, unsigned char* checkSumB);
extern void clearUBXMsgBuffer(const UBXMsgBuffer *buffer);

/* SealHAT additions ******
 *  The following functions are created as part of the 2018 UCSC SealHAT project
 * TODO: license/document this correctly
 */

/**
 * alignUBXmessage
 *
 * Aligns a UBXMsg struct to the first detected UBX message in a uint8_t/character buffer. 
 * This allows for easy parsing of received data using UBXMsg attribute access from ubx.h
 *
 * @param msg pointer to a UBXMsg struct
 * @param BUFF pointer to an array containing a ubx message preceded by a number of 0xff chars
 * @param SIZE size of the BUFF array
 * @returns the offset to the start of the message
 */
extern uint16_t alignUBXmessage(UBXMsg **msg, const uint8_t *BUFF, const int SIZE);

extern UBXMsgBuffer getACK_ACK();

#ifdef __cplusplus
}
#endif

#endif // UBLOXUTILS_H
