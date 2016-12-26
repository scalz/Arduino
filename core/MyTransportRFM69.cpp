/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyConfig.h"
#include "MyTransport.h"
#include <stdint.h>
#include "drivers/RFM69/RFM69.h"

bool transportInit(void) {
	const bool result = RFM69_initialise(MY_RFM69_FREQUENCY);
	#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_RFM69_ATC_MODE_DISABLED)
		// only enable ATC mode nodes
		RFM69_ATCmode(true, MY_RFM69_ATC_TARGET_RSSI_DBM);
	#endif
	
	#ifdef MY_RFM69_ENABLE_ENCRYPTION
		uint8_t _psk[16];
		hwReadConfigBlock((void*)_psk, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
		RFM69_encrypt((const char*)_psk);
		memset(_psk, 0, 16); // Make sure it is purged from memory when set
	#endif
	
	return result;
}

void transportSetAddress(const uint8_t address) {
	RFM69_setAddress(address);
}

uint8_t transportGetAddress(void) {
	return RFM69_getAddress();
}

bool transportSend(const uint8_t to, const void* data, uint8_t len) {
	return RFM69_sendWithRetry(to, data, len);
}

bool transportAvailable(void) {
	return RFM69_available();
}

bool transportSanityCheck(void) {
	return RFM69_sanityCheck();
}

uint8_t transportReceive(void* data) {
	const uint8_t len = RFM69_recv((uint8_t*)data);
	if (RFM69_isACKRequested()) {
		RFM69_ACKSend();
	}
	return len;

}	

void transportPowerDown(void) {
	(void)RFM69_sleep();
}

int16_t transportGetRSSI(void) {
	return RFM69_getRSSI();
}

uint8_t transportGetTxPower(void) {
	return RFM69_getTxPower();
}

bool transportSetTxPower(const uint8_t powerLevel) {
	// range 0..23
	return RFM69_setTxPower(powerLevel);
}

void  transportSetRSSI(int16_t targetSignalStrength) {
	#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_RFM69_ATC_MODE_DISABLED)
	RFM69_ATCmode(true, targetSignalStrength);
	#endif
}
