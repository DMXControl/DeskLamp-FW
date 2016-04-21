/**
* @file  	main.c
* @author  	J. Mintenbeck, S. Krupop
* @date    	2013/11/09
* @version 	1.0
* @brief	Main Function for Desklamp
*
* Functions:
*  LED-Driver via Hardware-PWM
*  RGB Color
*  128 brightness steps
*
* Open items:
*
* Fuses:
*  High: 0xDF
*  Low:  0xEE
*  Extended: 0xFF
*/

#include <avr/signature.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usbconfig.h"
#include "usbdrv/usbdrv.h"
#include "desklamp.h"

FUSES = {
	.low = 0xEE,
	.high = 0xDF,
	.extended = 0xFF,
};

static uchar buffer[8];
static uchar currentPosition, bytesRemaining;

/** USB Descriptor */
PROGMEM const char usbHidReportDescriptor[124] = {
    0x05, 0x08,                    // USAGE_PAGE (LEDs)
    0x09, 0x4b,                    // USAGE (Generic Indicator)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x3d,                    //     USAGE (Indicator On)
    0x85, 0x01,                    //     REPORT_ID (1)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x91, 0x00,                    //     OUTPUT (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x3d,                    //     USAGE (Indicator On)
    0x85, 0x02,                    //     REPORT_ID (2)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x91, 0x00,                    //     OUTPUT (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x47,                    //     USAGE (Usage Indicator Color)
    0x85, 0x03,                    //     REPORT_ID (3)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x91, 0x00,                    //     OUTPUT (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x3e,                    //     USAGE (Indicator Flash)
    0x85, 0x04,                    //     REPORT_ID (4)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x91, 0x00,                    //     OUTPUT (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x47,                    //     USAGE (Usage Indicator Color)
    0x85, 0x05,                    //     REPORT_ID (5)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0xb1, 0x00,                    //     FEATURE (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x3d,                    //     USAGE (Indicator On)
    0x85, 0x06,                    //     REPORT_ID (6)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0xb1, 0x00,                    //     FEATURE (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x47,                    //     USAGE (Usage Indicator Color)
    0x85, 0x07,                    //     REPORT_ID (7)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0xb1, 0x00,                    //     FEATURE (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x47,                    //     USAGE (Usage Indicator Color)
    0x85, 0x08,                    //     REPORT_ID (8)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0xb1, 0x00,                    //     FEATURE (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x47,                    //     USAGE (Usage Indicator Color)
    0x85, 0x09,                    //     REPORT_ID (9)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0xb1, 0x00,                    //     FEATURE (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x47,                    //     USAGE (Usage Indicator Color)
    0x85, 0x0A,                    //     REPORT_ID (10)
    0x95, 0x04,                    //     REPORT_COUNT (4)
    0x91, 0x00,                    //     OUTPUT (Data,Ary,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

#define SERIAL_NUMBER_LENGTH 8

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB Interface ----------------------------- */
/* ------------------------------------------------------------------------- */

/**
 * USB-Data-Handler (host -> device). Handles data that is received from the
 * USB-Interface. In our case the data contains settings for the LEDs.
 * \param data The received data, up to 8 bytes.
 * \param len Length of the received data.
 * \return 1 if we have received the entire payload successfully, 0 if we expect more data. We don't, so we always return 1.
 */

/**
*  @brief	USB-Data-Handler (host -> device).
*
* Handles data that is received from the USB-Interface.
*
* @param   *data	Pointer to Data Array
* @param 	len 	Length of data
* @return	real chuck size
*/
uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar i;

	if(len > bytesRemaining)                // if this is the last incomplete chunk
		len = bytesRemaining;               // limit to the amount we can store
	bytesRemaining -= len;

	for(i = 0; i < len; i++)
		buffer[currentPosition++] = data[i];

	desklamp_set_state(DESKLAMP_STATE_RX_DATA);

	return bytesRemaining == 0;             // return 1 if we have all data
}


/**
*  @brief	USB-Data-Handler
*
* Handles setup-calls that are received from the USB-Interface.
*
* @param   	setupData 	Eight bytes of data.
* @return	The number of returned bytes (in buffer[]).
*/
usbMsgLen_t usbFunctionSetup(uchar setupData[8]) {
	static uchar replyBuf[4];
	usbRequest_t *rq = (void *)setupData;   // cast to structured data for parsing

	usbMsgPtr = replyBuf;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            replyBuf[0] = rq->wValue.bytes[0];
        	switch (rq->wValue.bytes[0]) {
        		case DESKLAMP_CMD_GET_RGB:				/** get RGB VALUES */
                    replyBuf[1] = desklamp_get_rgb('r');
                    replyBuf[2] = desklamp_get_rgb('g');
                    replyBuf[3] = desklamp_get_rgb('b');
                    return 4;

        		case DESKLAMP_CMD_GET_DIMMER:				/** get Dimmer value */
                    replyBuf[1] = desklamp_get_dimmer();
                    return 2;

        		case DESKLAMP_CMD_GET_COLORMODE:		/** get Colormode */
                    replyBuf[1] = desklamp_get_colormode();
                    return 2;

        		case DESKLAMP_CMD_GET_EXTUSB:		/** get external USB state */
#if USBADAPTER == 1
                    replyBuf[1] = desklamp_chk_extusb();
#else
                    replyBuf[1] = 0;
#endif
                    return 2;
        	}
        	return 0; // should not get here
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
        	switch (rq->wValue.bytes[0]) {
        		case DESKLAMP_CMD_SET_LED:
        		case DESKLAMP_CMD_SET_DIMMER:
        		case DESKLAMP_CMD_SET_RGB:
        		case DESKLAMP_CMD_SET_STROBE:
        		case DESKLAMP_CMD_SET_COLORMODE:
        		case DESKLAMP_CMD_SET_SERIAL:
        			currentPosition = 0;                // initialize position index
        			bytesRemaining = rq->wLength.word;  // store the amount of data requested
        			if(bytesRemaining > sizeof(buffer)) // limit to buffer size
        				bytesRemaining = sizeof(buffer);
        			return USB_NO_MSG;        			// tell driver to use usbFunctionWrite()
        	}
            return 0;
        }
    }
    return 0;
}

/**
*  @brief	USB Event Reset Ready
*
*/
void usbEventResetReady(void) {

}

uchar usbFunctionDescriptor(usbRequest_t *rq) {
	static int serialBuf[SERIAL_NUMBER_LENGTH + 1];

	usbMsgPtr = 0;
	if (rq->wValue.bytes[1] == USBDESCR_STRING && rq->wValue.bytes[0] == 3) { // 3 is the type of string descriptor, in this case the device serial number
		usbMsgPtr = (uchar*)&serialBuf[0];

		serialBuf[0] = USB_STRING_DESCRIPTOR_HEADER(SERIAL_NUMBER_LENGTH);

		uint32_t serial = desklamp_get_serial();
		for (uint8_t i = 1; i <= 8; ++i) {
			uint8_t val = (serial >> (32 - (i << 2))) & 0xF;
			serialBuf[i] = val <= 9 ? '0' + val : 'A' - 10 + val;
		}

		return sizeof(serialBuf);
	}
	return 0;
}

/* ------------------------------------------------------------------------- */

/**
*  @brief	Main function
*/
int main(void) {
	uint8_t  i;
#if USBADAPTER
	uint8_t lastExtUSB = 1;
#endif
#if STROBE == 1
	uint16_t lastStrobe = 0;
	uint16_t count = 0;
#endif

	/* set LED-ports to output */
	desklamp_init();

#if USBADAPTER == 1
#else
	// Enable PWM
	desklamp_init_pwm();
#endif

	/* enable Watchdog */
	wdt_enable(WDTO_1S);

	/* We fake an USB disconnect by pulling D+ and D- to 0 during reset. This is
		 * necessary if we had a watchdog reset or brownout reset to notify the host
		 * that it should re-enumerate the device. Otherwise the host's and device's
		 * concept of the device-ID would be out of sync.
		 */
	usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
	for(i = 0; i<250; i++) { 	// wait 500 ms
		wdt_reset(); 			// keep the watchdog happy
		_delay_ms(2);
	}
	usbDeviceConnect();

	usbInit();
	sei();

	while(1){    							// main event loop
		wdt_reset();
		usbPoll();

#if USBADAPTER == 1
		uint8_t extUSB = desklamp_chk_extusb();
		if (extUSB != lastExtUSB) {
			if (extUSB) {
				// Disable PWM when external device using USB data lines is connected
				desklamp_set_led(1, ON);
				desklamp_config_channel(1, DISABLE);
			} else {
				// Enable PWM
				desklamp_init_pwm();
			}
			lastExtUSB = extUSB;
		}

		// check ext_usb device
		if (!extUSB) {
#endif

			// desklamp statemachine
			switch (desklamp_get_state()) {
				case DESKLAMP_STATE_IDLE:
					break;

				case DESKLAMP_STATE_RX_DATA:
					desklamp_set_state(DESKLAMP_STATE_BUSY);
					/** Desklamp Set Commands */
					switch (buffer[0]) { // Report ID
						case DESKLAMP_CMD_SET_LED:
							desklamp_set_led_intensity(buffer[1], buffer[2]);
							break;
						case DESKLAMP_CMD_SET_DIMMER:
							desklamp_set_dimmer(buffer[1]);
							break;
						case DESKLAMP_CMD_SET_RGB:
							desklamp_set_rgb(buffer[1], buffer[2], buffer[3]);
							break;
						case DESKLAMP_CMD_SET_STROBE:
							desklamp_set_strobe(buffer[1]);
							break;
						case DESKLAMP_CMD_SET_COLORMODE:
							desklamp_set_colormode(buffer[1]);
							break;
						case DESKLAMP_CMD_SET_SERIAL:
							desklamp_set_serial((uint32_t)buffer[1] << 24 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]);
							break;
					}

					desklamp_set_state(DESKLAMP_STATE_IDLE);
					break;
			}

#if STROBE == 1
			if(TIFR0 & (1 << TOV0)) {
				uint16_t curStrobe = desklamp_get_strobe();
				// Strobe off
				if (curStrobe == 0 && lastStrobe != 0) {
					desklamp_set_blackout(0);
					desklamp_update_pwm();
					lastStrobe = curStrobe;
				} else if (curStrobe > 0) { // Strobe on [ 0... 15Hz -> 0...255 ]
					curStrobe = 2000 + (50 * (255 - curStrobe));
					if (lastStrobe == 0) {
						desklamp_set_blackout(1);
						desklamp_update_pwm();
						count = 0;
					}
					if (count == curStrobe) {
						desklamp_set_blackout(0);
						desklamp_update_pwm();
					} else if (count >= curStrobe + 2000) {
						desklamp_set_blackout(1);
						desklamp_update_pwm();
						count = 0;
					}
					count++;
					lastStrobe = curStrobe;
				}
				TIFR0 &= ~(1 << TOV0);
			}
#endif
#if USBADAPTER == 1
		}
#endif
	}
	return 0;

}

/* --------------------------------- End Of File ------------------------------ */
