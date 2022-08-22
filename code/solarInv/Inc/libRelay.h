/*
 * libRelay.h
 *
 *  Created on: 13 feb. 2018
 *      Author: wvdv2
 */

#ifndef LIBRELAY_H_
#define LIBRELAY_H_
#include "stdbool.h"

void closeBridgeRelay(void);
void openBridgeRelay(void);
bool isBridgeRelayClosed(void);
void closePrechargeRelay(void);
void openPrechargeRelay(void);
bool isPrechargeRelayClosed(void);

#endif /* LIBRELAY_H_ */
