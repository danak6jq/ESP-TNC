/* ========================================
 *
 * Copyright © 2019, Dana H. Myers.
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Dana H. Myers.
 *
 * ========================================
*/

#ifndef AXFCS_H_
#define AXFCS_H_

#define VALID_FCS   0xf0b8

/*
 * Initialize FCS value to 0xffff
 * Each call to fcs_update() updates value 
 * FCS is valid when == VALID_FCS
 */
void fcs_update(uint16_t *fcs, uint8_t v);

/*
 * Return FCS value of frame ready to transmit
 */
uint16_t fcs_value(uint8_t *data, uint16_t len);


#endif  /* AXFCS_H_ */


