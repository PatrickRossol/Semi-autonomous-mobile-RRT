/*
 * drawMenu.h
 *
 *  Created on: May 27, 2021
 *      Author: Patrick
 */

#ifndef INC_DRAWMENU_H_
#define INC_DRAWMENU_H_

#include "drawMenu.h"
#include "SSD1331.h"
#include "Fonts.h"

void drawMenu();
void drawManualMode();
void drawSeqMode(int mode);
void drawCoordMode();
void clearMenu();


#endif /* INC_DRAWMENU_H_ */
