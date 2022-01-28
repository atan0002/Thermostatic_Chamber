/*
 * user_interface.h
 *
 *  Created on: Jan 3, 2022
 *      Author: Natalia Wiśniewska
 */

#ifndef INC_USER_INTERFACE_H_
#define INC_USER_INTERFACE_H_

#include "lcd_i2c.h"
#include <stdbool.h>
#include <stdio.h>
#include "gpio.h"
#include "stdbool.h"

//page one
#define PAGEONE_MODE1 "   Select temp  "



//last page
#define LINE1 "ACTUAL TEMP:"
#define LINE2 "TARGET TEMP:"


struct lcd_disp disp;


bool initpage;
bool lastpage;
int val;


int ButtonsUsage(void);
void FirstPage(int i);
void DisplayResults(int target,float actual_temp);
void UserInterfaceInit(void);
void InitPage(void);






#endif /* INC_USER_INTERFACE_H_ */
