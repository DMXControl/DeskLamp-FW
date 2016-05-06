/**
 * @file 	desklamp.h
 * @author  J. Mintenbeck
 * @date    2013/11/09
 * @version 1.0
 * @brief	Desklamp API
 *
 */

#ifndef __DESKLAMP_H
#define __DESKLAMP_H

/** CPU Clock */
#ifndef F_CPU
#define F_CPU 12000000UL
#endif

/**
 * @name Port definitions
 * @{
 */
#define DESKLAMP_LED_DDR		DDRA
#define DESKLAMP_LED_PORT		PORTA
#define DESKLAMP_PIN_LED1		PA7
#define DESKLAMP_PIN_LED2		PA6
#define DESKLAMP_PIN_LED3		PA5
#define DESKLAMP_PIN_DP_EXT		PA3
#define DESKLAMP_PIN_DM_EXT		PA2
/** @} */

#define LOWBYTE(var)    (((uchar *)&(var))[0])
#define HIGHBYTE(var)   (((uchar *)&(var))[1])

/**
 * @name Desklamp Commands
 * @{
 */
#define DESKLAMP_CMD_SET_LED		1
#define DESKLAMP_CMD_SET_COLORMODE	5
#define DESKLAMP_CMD_SET_RGB		3
#define DESKLAMP_CMD_SET_STROBE		4
#define DESKLAMP_CMD_SET_DIMMER		2
#define DESKLAMP_CMD_SET_SERIAL		10

#define DESKLAMP_CMD_GET_RGB		7
#define DESKLAMP_CMD_GET_COLORMODE	8
#define DESKLAMP_CMD_GET_DIMMER		6
#define DESKLAMP_CMD_GET_EXTUSB		9
/** @} */

/**
 * @name Desklamp states
 * @{
 */
#define DESKLAMP_STATE_IDLE			0
#define DESKLAMP_STATE_BUSY			1
#define DESKLAMP_STATE_RX_DATA 		2		// received Data
#define DESKLAMP_STATE_TX_DATA 		3		// transmit Data
/** @} */

/**
 * @name Colormodes
 * @{
 */
#define DESKLAMP_COLORMODE_RGB 		0
#define DESKLAMP_COLORMODE_MONO		1
/** @} */

/** OPTIONS */
#define COLORMODE					DESKLAMP_COLORMODE_MONO
#define USBADAPTER					1
#define STROBE						1

enum {OFF, ON};				// Values for OFF = 0 , ON = 1
enum {DISABLE, ENABLE};		// Values for DISABLE = 0 , ENABLE = 1


/** Typdef for the desklamp structure */
typedef struct {
	uint8_t state;
	uint8_t colormode;		/** colormode: RGB or Single Color */
	uint8_t r;				/** red */
	uint8_t g;				/** green */
	uint8_t b;				/** blue */
	uint8_t dimmer;			/** dimmer */
	uint8_t strobe;			/** strobe value */
	uint8_t blackout;
	uint8_t usb_ext;		/** ext USB check */
	uint32_t serial;
}desklamp_t;


void desklamp_init(void);
void desklamp_config_channel(uint8_t channel, uint8_t state);
void desklamp_init_pwm(void);
void desklamp_update_pwm(void);
void desklamp_set_led(uint8_t led, uint8_t onoff);
void desklamp_set_led_intensity(uint8_t led, uint8_t intensity);
void desklamp_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void desklamp_set_dimmer(uint8_t dimmer);
void desklamp_set_colormode(uint8_t colormode);
void desklamp_set_serial(uint32_t serial);
void desklamp_set_strobe(uint8_t strobe);
void desklamp_set_blackout(uint8_t blackout);
void desklamp_set_state(uint8_t state);
uint8_t desklamp_get_state(void);
uint8_t desklamp_get_colormode(void);
uint32_t desklamp_get_serial(void);
uint8_t desklamp_get_rgb(char c);
uint8_t desklamp_get_dimmer(void);
uint16_t desklamp_get_hsv(char c);
uint8_t desklamp_get_strobe(void);
uint8_t desklamp_chk_extusb(void);

#endif /* __DESKLAMP_H */

/* --------------------------------- End Of File ------------------------------ */
