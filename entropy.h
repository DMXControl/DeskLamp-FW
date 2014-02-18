/*
 * entropy.h
 *
 *  Created on: 13.02.2014
 *      Author: Stefan Krupop
 */

#ifndef ENTROPY_H_
#define ENTROPY_H_

#include <stdint.h>

void entropy_init(void);
uint32_t entropy_random(void);

#endif /* ENTROPY_H_ */
