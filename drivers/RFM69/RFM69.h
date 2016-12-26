/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2016 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * RFM69 driver refactored for Mysensors
 * 
 * Based on :
 * - LowPowerLab RFM69 Lib Copyright Felix Rusu (2014), felix@lowpowerlab.com
 * - Automatic Transmit Power Control class derived from RFM69 library.
 *	  Discussion and details in this forum post: https://lowpowerlab.com/forum/index.php/topic,688.0.html
 *	  Copyright Thomas Studwell (2014,2015)
 * - Mysensors generic radio driver implementation Copyright (C) 2016 Olivier Mauti <olivier@mysensors.org>
 *
 *
 * Changes by : @tekka, @scalz, @marceloagno
 *
 * Definitions for Semtech SX1231/H radios:
 * http://www.semtech.com/images/datasheet/sx1231.pdf
 * http://www.semtech.com/images/datasheet/sx1231h.pdf
 */

/**
* @file RFM69.h
*
* @defgroup RFM69grp RFM69
* @ingroup internals
* @{
*
* RFM69 driver-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
* - [!] Exclamation mark is prepended in case of error
*
* This section is WIP!
*
* |E| SYS	 | SUB      | Message				| Comment
* |-|-------|----------|-----------------------|---------------------------------------------------------------------
* | | RFM69 | INIT     | ...					| Initialise RFM69 radio
* | | RFM69 | ...      | ...					| ...
* | | RFM69 | ...      | ...					| ...
* | | RFM69 | ...      | ...					| ...
*
*
* @brief API declaration for RFM69
*
*/

#ifndef _RFM69_h
#define _RFM69_h

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
#define RFM69_IRQ_PIN		(2)
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
#define RFM69_IRQ_PIN		(2)
#elif defined(__AVR_ATmega32U4__)
#define RFM69_IRQ_PIN		(3)
#elif defined(LINUX_ARCH_RASPBERRYPI)
#define RFM69_IRQ_PIN	(22)
#elif defined(__arm__)
#define RFM69_IRQ_PIN		(10)
#else
#define RFM69_IRQ_PIN		(2)			//!< RFM69_IRQ_PIN
#endif

#if defined(MY_RFM69_RST_PIN)
#define RFM69_RST_PIN MY_RFM69_RST_PIN //!< RFM69_RST_PIN 
#endif

#define RFM69_SPI_CS		(SS)

// SPI settings
#define RFM69_SPI_DATA_ORDER	MSBFIRST		//!< SPI data order
#define RFM69_SPI_DATA_MODE		SPI_MODE0		//!< SPI mode

#if !defined(MY_RFM69_SPI_MAX_SPEED)
#define RFM69_SPI_MAX_SPEED		(2000000ul)	//!< SPI speed
#else
#define RFM69_SPI_MAX_SPEED		MY_RFM69_SPI_MAX_SPEED	//!< SPI speed
#endif

#if defined (ARDUINO) && !defined (__arm__) && !defined (_SPI)
#include <SPI.h>
#if defined(MY_SOFTSPI)
SoftSPI<MY_SOFT_SPI_MISO_PIN, MY_SOFT_SPI_MOSI_PIN, MY_SOFT_SPI_SCK_PIN, RFM69_SPI_DATA_MODE> _SPI;
#else
#define _SPI SPI
#endif
#else
#if defined(__arm__)
#include <SPI.h>
#else
extern HardwareSPI SPI;		//!< SPI
#endif

#if !defined(_SPI)
#define _SPI SPI			//!< SPI
#endif
#endif

// MY_RFM69 interface - SPI
#define RFM69_IRQ_DIO_PIN		MY_RFM69_IRQ_PIN
#define RFM69_CS_PIN			MY_RFM69_SPI_CS	//!< SPI CS/SS pin

// MY_RFM69 interface - radio settings
#define RFM69_TX_POWER_DBM		MY_RFM69_TX_POWER_DBM
#define RFM69_FREQUENCY			MY_RFM69_FREQUENCY
#define RFM69_NETWORKID			MY_RFM69_NETWORKID
#if (MY_RFM69HW==true)
// RFM69H(C)W
#define RFM69_VERSION_HW	// change this
#define RFM69_MIN_POWER_LEVEL_DBM		((rfm69_powerlevel_t)-2)	//!< min. power level, -18dBm
#define RFM69_MAX_POWER_LEVEL_DBM		((rfm69_powerlevel_t)20)	//!< max. power level, +20dBm
#else
// RFM69(C)W
#define RFM69_MIN_POWER_LEVEL_DBM		((rfm69_powerlevel_t)-18)	//!< min. power level, -18dBm
#define RFM69_MAX_POWER_LEVEL_DBM		((rfm69_powerlevel_t)13)	//!< max. power level, +13dBm
#endif

// Bit rate definition
#define RFM69_BITRATE_MSB		RFM69_BITRATEMSB_55555
#define RFM69_BITRATE_LSB		RFM69_BITRATELSB_55555

// debug
#if defined(MY_DEBUG_VERBOSE_RFM69)
#define RFM69_DEBUG(x,...) debug(x, ##__VA_ARGS__)	//!< Debug print
#else
#define RFM69_DEBUG(x,...)							//!< Debug null
#endif

#define RFM69_FIFO_SIZE					(0xFFu)			//!< Max number of bytes the Rx/Tx FIFO can hold
#define RFM69_MAX_PACKET_LEN			(0x40u)			//!< This is the maximum number of bytes that can be carried 

#define RFM69_CAD_TIMEOUT_MS			(2*1000ul)		//!< channel activity detection timeout
#define RFM69_PACKET_SENT_TIMEOUT_MS	(2*1000ul)		//!< packet sent timeout
#define RFM69_ATC_TARGET_RANGE_PERCENT	(5)				//!< ATC target range +/-%
#define RFM69_PACKET_HEADER_VERSION		(1u)			//!< RFM69 packet header version
#define RFM69_MIN_PACKET_HEADER_VERSION (1u)			//!< Minimal RFM69 packet header version
#define RFM69_RETRIES					(5u)			//!< Retries in case of failed transmission
#define RFM69_RETRY_TIMEOUT_MS			(50ul)			//!< Timeout for ACK, adjustments needed if modem configuration changed (air time different)
#define RFM69_MODE_READY_TIMEOUT_MS		(50ul)			//!< Timeout for mode ready

#define RFM69_ACK_REQUESTED				(7u)			//!< RFM69 header, controlFlag, bit 7
#define RFM69_ACK_RECEIVED				(6u)			//!< RFM69 header, controlFlag, bit 6
#define RFM69_ACK_RSSI_REPORT			(5u)			//!< RFM69 header, controlFlag, bit 5

#define RFM69_BROADCAST_ADDRESS			(255u)			//!< Broadcasting address 
#define RFM69_TARGET_RSSI_DBM			(-60)			//!< RSSI target
#define RFM69_HIGH_POWER_DBM			(18)			//!< High power threshold, dBm
#define RFM69_PROMISCUOUS				(false)			//!< RFM69 promiscuous mode

#define RFM69_RSSItoInternal(__value)	((uint8_t)-(__value<<1))
#define RFM69_internaltoRSSI(__value)	((int16_t)-(__value>>1))


#define RFM69_getACKRequested(__value) ((bool)bitRead(__value,RFM69_ACK_REQUESTED))						//!< getACKRequested
#define RFM69_setACKRequested(__value, __flag) bitWrite(__value,RFM69_ACK_REQUESTED,__flag)				//!< setACKRequested
#define RFM69_getACKReceived(__value) ((bool)bitRead(__value,RFM69_ACK_RECEIVED))						//!< getACKReceived
#define RFM69_setACKReceived(__value, __flag) bitWrite(__value,RFM69_ACK_RECEIVED,__flag)				//!< setACKReceived
#define RFM69_setACKRSSIReport(__value, __flag) bitWrite(__value,RFM69_ACK_RSSI_REPORT,__flag)			//!< setACKRSSIReport
#define RFM69_getACKRSSIReport(__value) ((bool)bitRead(__value,RFM69_ACK_RSSI_REPORT))					//!< getACKRSSIReport


// Register access
#define RFM69_READ_REGISTER		(0x7Fu)	//!< reading register
#define RFM69_WRITE_REGISTER	(0x80u)	//!< writing register


#define RFM69_CSMA_LIMIT_DBM        (-90) // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RFM69_CSMA_TIMEOUT_MS		(1000ul)
// available frequency bands
#define RFM69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RFM69_433MHZ            43
#define RFM69_868MHZ            86
#define RFM69_915MHZ            91

#define COURSE_TEMP_COEF		-90 // puts the temperature reading in the ballpark, user can fine tune the returned value

#define RFM69_TX_LIMIT_MS		(1000ul)
#define RFM69_FXOSC				(32000000ul)
#define RFM69_FSTEP				(RFM69_FXOSC / 524288ul) // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

#if defined(MY_RFM69_ENABLE_LISTENMODE)
#define  RFM69_LISTEN_RX_US MY_RFM69_DEFAULT_LISTEN_RX_US
#define  RFM69_LISTEN_IDLE_US MY_RFM69_DEFAULT_LISTEN_IDLE_US
#endif


/**
* @brief Radio modes
*/
typedef enum {
   RFM69_RADIO_MODE_RX = 0,					//!< RX mode
   RFM69_RADIO_MODE_TX = 1,					//!< TX mode
   RFM69_RADIO_MODE_CAD = 2,					//!< CAD mode
   RFM69_RADIO_MODE_SLEEP = 3,					//!< SLEEP mode
   RFM69_RADIO_MODE_STDBY = 4,					//!< STDBY mode
   RFM69_RADIO_MODE_SYNTH = 5,					//!< SYNTH mode
   RFM69_RADIO_MODE_LISTEN = 6					//!< LISTEN mode
} rfm69_radio_mode_t;

/**
* @brief Sequence number data type
*/
typedef uint8_t rfm69_sequenceNumber_t;
/**
* @brief RSSI data type
*/
typedef uint8_t rfm69_RSSI_t;
/**
* @brief SNR data type
*/
typedef int8_t rfm69_SNR_t;
/**
* @brief Control flag data type
*/
typedef uint8_t rfm69_controlFlags_t;
/**
* @brief Control flag data type
*/
typedef int8_t rfm69_powerlevel_t;

/**
* @brief RFM69 header
* IMPORTANT: Do not change order (see datasheet for packet structure)
*/
typedef struct {
  uint8_t packetLen;								//!< Packet length
  uint8_t recipient;								//!< Payload recipient
	uint8_t version;								//!< Header version
  uint8_t sender;									//!< Payload sender
  rfm69_controlFlags_t controlFlags;						//!< Control flags, used for ACK
	rfm69_sequenceNumber_t sequenceNumber;			//!< Packet sequence number, used for ACK 
} __attribute__((packed)) rfm69_header_t;
/**
* @brief RFM69  ACK packet structure
*/
typedef struct {
   rfm69_sequenceNumber_t sequenceNumber;			//!< sequence number
   rfm69_RSSI_t RSSI;								//!< RSSI
} __attribute__((packed)) rfm69_ack_t;


#define RFM69_HEADER_LEN sizeof(rfm69_header_t)		//!< Size header inside  payload
#define RFM69_MAX_PAYLOAD_LEN (RFM69_MAX_PACKET_LEN - RFM69_HEADER_LEN)	//!< Max payload length

/**
* @brief Packet structure
*/
typedef struct {
   union {
      struct {
         rfm69_header_t header;						//!< Packet header
         union {
            uint8_t payload[RFM69_MAX_PAYLOAD_LEN];	//!< Union: Data Payload, i.e. MySensors message
            rfm69_ack_t ACK;						//!< Union: ACK payload (internal)
         };
      };
      uint8_t data[RFM69_MAX_PACKET_LEN];				//!< RAW data access
   };
   uint8_t payloadLen;									//!< Length of payload (excluding header)
   rfm69_RSSI_t RSSI;									//!< RSSI of current packet, RSSI = value - 137
} __attribute__((packed)) rfm69_packet_t;


/**
* @brief RFM69 internal variables
*/
typedef struct {
   uint8_t address;							//!< Node address
   rfm69_packet_t currentPacket;				//!< Buffer for current packet
   rfm69_sequenceNumber_t txSequenceNumber;	//!< RFM69_txSequenceNumber
   rfm69_powerlevel_t powerLevel;				//!< TX power level dBm
   uint8_t ATCtargetRSSI;						//!< ATC: target RSSI
   // 8 bit
   rfm69_radio_mode_t radioMode : 3;			//!< current transceiver state
   bool cad : 1;								//!< RFM69_cad
   bool dataSent : 1;							//!< Data sent
   bool dataReceived : 1;						//!< data received
   bool ackReceived : 1;						//!< ack received
   bool ATCenabled : 1;						//!< ATC enabled

	bool listenModeEnabled : 1;			//!< Listen Mode enabled

} rfm69_internal_t;

#if defined(MY_RFM69_ENABLE_LISTENMODE)
uint8_t rxListenCoef;
uint8_t rxListenResolution;
uint8_t idleListenCoef;
uint8_t idleListenResolution;
uint32_t listenCycleDurationUs;
#endif

#define LOCAL static		//!< static

/**
* @brief Initialise the driver transport hardware and software
* @param frequency
* @return True if initialisation succeeded
*/
LOCAL bool RFM69_initialise(const float frequency);

/**
* @brief Set the driver/node address
* @param addr
*/
LOCAL void RFM69_setAddress(const uint8_t addr);

/**
* @brief Get driver/node address
* @return Node address
*/
LOCAL uint8_t RFM69_getAddress(void);

/**
* @brief Tests whether a new message is available
* @return True if a new, complete, error-free uncollected message is available to be retreived by @ref RFM95_recv()
*/
LOCAL bool RFM69_available(void);

/**
* @brief If a valid message is received, copy it to buf and return length. 0 byte messages are permitted.
* @param buf Location to copy the received message
* @return Number of bytes
*/
LOCAL uint8_t RFM69_recv(uint8_t* buf);

/**
* @brief RFM69_sendFrame
* @param packet
* @param increaseSequenceCounter
* @return True if packet sent
*/
LOCAL bool RFM69_sendFrame(rfm69_packet_t &packet, const bool increaseSequenceCounter = true);

/**
* @brief RFM69_send
* @param recipient
* @param data
* @param len
* @param flags
* @param increaseSequenceCounter
* @return True if frame sent
*/
LOCAL bool RFM69_send(const uint8_t recipient, uint8_t* data, const uint8_t len, 
	const rfm69_controlFlags_t flags, const bool increaseSequenceCounter = true);

/**
* @brief Sets the transmitter and receiver centre frequency
* @param centre Frequency in MHz (137.0 to 1020.0)
*/
LOCAL void RFM69_setFrequency(const float centre);

/**
* @brief Sets the transmitter power output level, and configures the transmitter pin
* @param newPowerLevel Transmitter power level in dBm (-18 to +20dBm)
* @return True power level adjusted
*/
LOCAL bool RFM69_setTxPower(rfm69_powerlevel_t newPowerLevel);

/**
* @brief Sets the radio into low-power sleep mode
* @return true if sleep mode was successfully entered
*/
LOCAL bool RFM69_sleep(void);

/**
* @brief RFM69_sendACK
* @param recipient
* @param sequenceNumber
* @param RSSI (rfm95_RSSI_t)
*/
LOCAL void RFM69_sendACK(const uint8_t recipient, const rfm69_sequenceNumber_t sequenceNumber, 
	const rfm69_RSSI_t RSSI);

/**
* @brief RFM69_sendWithRetry
* @param recipient
* @param buffer
* @param bufferSize
* @param retries
* @param retryWaitTime
* @return True if packet successfully sent
*/
LOCAL bool RFM69_sendWithRetry(const uint8_t recipient, const void* buffer, const uint8_t bufferSize, 
	const uint8_t retries = RFM69_RETRIES, const uint32_t retryWaitTime = RFM69_RETRY_TIMEOUT_MS);

/**
* @brief RFM69_waitCAD
* @return True if no channel activity detected
*/
LOCAL bool RFM69_waitCAD(void);

/**
* @brief RFM69_waitPacketSent
* @return True if packet sent
*/
LOCAL bool RFM69_waitPacketSent(void);

/**
* @brief RFM69_setRadioMode
* @param newRadioMode
* @return True if mode changed
*/
LOCAL bool RFM69_setRadioMode(const rfm69_radio_mode_t newRadioMode);

/**
* @brief Low level interrupt handler
*/
LOCAL void RFM69_interruptHandler(void);

/**
* @brief RFM69_clearRxBuffer
*/
LOCAL void RFM69_clearRxBuffer(void);

/**
* @brief RFM69_getRSSI
* @return RSSI Signal strength of last packet
*/
LOCAL int16_t RFM69_getRSSI(void);

/**
* @brief RFM69_executeATC
* @param currentRSSI
* @param targetRSSI
* @return True if power level adjusted
*/
LOCAL bool RFM69_executeATC(const rfm69_RSSI_t currentRSSI, const rfm69_RSSI_t targetRSSI);

/**
* @brief RFM69_getTxPower
* @return Current power level
*/
LOCAL rfm69_powerlevel_t RFM69_getTxPower(void);

/**
* @brief RFM69_ATCmode
* @param targetRSSI Target RSSI for transmitter (default -60)
* @param onOff True to enable ATC
*/
LOCAL void RFM69_ATCmode(const bool onOff, const int16_t targetRSSI = RFM69_TARGET_RSSI_DBM);


// TEMP ADDED
/**
* @brief RFM69_setConfiguration Set general radio register configuration TODO temp use setmodemregisters
*/
LOCAL void RFM69_setConfiguration(void);

/**
* @brief RFM69_setBitrate Set the radio bitrate
* @param bitrate_msb
* @param bitrate_lsb
*/
LOCAL void RFM69_setBitrate(const uint8_t bitrate_msb, const uint8_t bitrate_lsb);

/**
* @brief RFM69_isModeReady
* @return True if Mode Ready is ok, false is timeout
*/
LOCAL bool RFM69_isModeReady(void);

/**
* @brief RFM69_sanityCheck detect HW defect, configuration errors or interrupted SPI line
* @param currentRSSI
* @param targetRSSI
* @return True if radio sanity check passed
*/
LOCAL bool RFM69_sanityCheck(void);

/**
* @brief RFM69_readTemperature COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
* @param calFactor
* @param targetRSSI
* @return centigrad
*/
LOCAL uint8_t RFM69_readTemperature(const uint8_t calFactor);

/**
* @brief RFM69_rcCalibration
*/
LOCAL void RFM69_rcCalibration(void);

/**
* @brief RFM69_setLNA used for power level testing and used to disable LNA AGC for testing purposes
* @param newReg
* @return LNA
*/
LOCAL uint8_t RFM69_setLNA(const uint8_t newReg);

/**
* @brief RFM69_encrypt Set encryption mode
* @param key if key is null, encryption is disabled. Key has to be 16 bytes!
*/
LOCAL void RFM69_encrypt(const char* key);

/**
* @brief RFM69_setHighPowerRegs
* @param onOff
*/
LOCAL void RFM69_setHighPowerRegs(const bool onOff);

/**
* @brief RFM69_setHighPower for RFM69HW only: you must call setHighPower(true) after initialize() 
*        or else transmission won't work
* @param onOff
*/
LOCAL void RFM69_setHighPower(const bool onOff);

/**
* @brief RFM69_setNetwork
* @param networkID
*/
LOCAL void RFM69_setNetwork(const uint8_t networkID);

/**
* @brief RFM69_readRSSI Read rssi
* @param force
* @return rssi (internal format)
*/
LOCAL rfm69_RSSI_t RFM69_readRSSI(bool forceTrigger = false);

/**
* @brief RFM69_readAllRegs Read all RFM69 registers, except the two last PA (todo)
*/
LOCAL void RFM69_readAllRegs(void);







/**
* @brief RFM69_listenModeApplyHighSpeedSettings Set high speed settings
*/
LOCAL void RFM69_listenModeApplyHighSpeedSettings(void);

/**
* @brief RFM69_ListenModeStart Switch radio to listen Mode in prep for sleep until burst
*/
LOCAL void RFM69_ListenModeStart(void);

/**
* @brief RFM69_listenModeEnd Exit listen mode and reinit the radio
* @return 
*/
LOCAL bool RFM69_listenModeEnd(void);

/**
* @brief RFM69_listenModeReset Reset listen mode
*/
LOCAL void RFM69_listenModeReset(void);

/**
* @brief RFM69_reinitRadio Restore previous radio settigns for normal mode
* @return 
*/
LOCAL bool RFM69_reinitRadio(void);

/**
* @brief RFM69_listenModeSendBurst This repeatedly sends the message to the target node for 
 *   the duration of an entire listen cycle. The amount of time remaining in the burst is 
 *   transmitted to the receiver, and it is expected that the receiver
 *   wait for the burst to end before attempting a reply.
* @param 
* @param
* @param
*/
LOCAL void RFM69_listenModeSendBurst(const uint8_t recipient, uint8_t* data, const uint8_t len);

/**
* @brief getUsForResolution 
* @param 
* @return 
*/
LOCAL uint32_t RFM69_getUsForResolution(uint8_t resolution);

/**
* @brief getCoefForResolution
* @param
* @param
* @return
*/
LOCAL uint32_t RFM69_getCoefForResolution(uint8_t resolution, uint32_t duration);

/**
* @brief RFM69_chooseResolutionAndCoef
* @param
* @param
* @param
* @param
* @return
*/
LOCAL bool RFM69_chooseResolutionAndCoef(uint8_t *resolutions, uint32_t duration, uint8_t& resolOut, 
	uint8_t& coefOut);

/**
* @brief RFM69_listenModeSetDurations
* @param
* @param
* @return
*/
LOCAL bool RFM69_listenModeSetDurations(uint32_t& rxDuration, uint32_t& idleDuration);

/**
* @brief RFM69_listenModeGetDurations
* @param
* @param
*/
LOCAL void RFM69_listenModeGetDurations(uint32_t &rxDuration, uint32_t &idleDuration);

#endif

/** @}*/