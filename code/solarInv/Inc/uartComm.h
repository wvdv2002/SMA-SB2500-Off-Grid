/*
 * uartComm.h
 *
 *  Created on: 23 aug. 2017
 *      Author: wvdv2
 */

#ifndef UARTCOMM_H_
#define UARTCOMM_H_

void startCommandThread(void const * argument);
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);
void controlUsartHandler(void);
#endif /* UARTCOMM_H_ */
