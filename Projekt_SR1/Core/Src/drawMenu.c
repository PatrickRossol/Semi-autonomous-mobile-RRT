/*
 * drawMenu.c
 *
 *  Created on: May 27, 2021
 *      Author: Patrick
 */

#include "drawMenu.h"

void drawMenu()
{
	  clearMenu();
	  ssd1331_display_string(7, 0, "1.Reczny", FONT_1206, GREEN);
	  ssd1331_display_string(7, 11, "2.Sekwencja", FONT_1206, GREEN);
	  ssd1331_display_string(7, 22, "3.Coords", FONT_1206, GREEN);

	  ssd1331_draw_line(0,50,95,50, WHITE);
	  ssd1331_display_string(83, 51, "OK", FONT_1206, GREEN);
	  ssd1331_draw_line(0,63,95,63, WHITE);
}


void drawManualMode()
{
	clearMenu();
	ssd1331_display_string(16, 0, "TRYB RECZNY", FONT_1206, GREEN);

//	ssd1331_display_string(4,  15, "X:       Y:         Z:", FONT_1206, GREEN);
	ssd1331_display_string(7,  15, "Pojazd", FONT_1206, GREEN);
	ssd1331_display_string(7,  26, "Manipulator", FONT_1206, GREEN);


	ssd1331_display_string(2, 51, "<", FONT_1206, WHITE);
}

void drawSeqMode(int mode)
{

	clearMenu();
	ssd1331_display_string(0, 0, "TRYB SEKWENCYJNY", FONT_1206, GREEN);

	if(mode == 0)
	{
		  ssd1331_display_string(7, 12, "1.New Seq", FONT_1206, GREEN);
		  ssd1331_display_string(7, 23, "2.Play Seq", FONT_1206, GREEN);

		  ssd1331_display_string(2, 51, "<", FONT_1206, WHITE);
		  ssd1331_display_string(83, 51, "OK", FONT_1206, GREEN);
	}

	if(mode == 1)
	{
		ssd1331_display_string(4,  15, "X:      Y:          Z:", FONT_1206, GREEN);

		ssd1331_draw_line(13,50,13,63, WHITE);
		ssd1331_draw_line(43,50,43,63, WHITE);
		ssd1331_draw_line(72,50,72,63, WHITE);

		ssd1331_draw_rect(0, 37, 8, 11, BLUE);

		ssd1331_display_string(2, 51, "<", FONT_1206, WHITE);
		ssd1331_display_string(17, 51, "Undo", FONT_1206, WHITE);
		ssd1331_display_string(46, 51, "Next", FONT_1206, WHITE);
		ssd1331_display_string(76, 51, "Sav", FONT_1206, WHITE);
	}

	if(mode == 2)
	{
		ssd1331_display_string(7, 12, "1.Sekwencja 1", FONT_1206, GREEN);
		ssd1331_display_string(7, 23, "2.Sekwencja 2", FONT_1206, GREEN);
		ssd1331_display_string(7, 34, "3.Sekwencja 3", FONT_1206, GREEN);

		ssd1331_display_string(2, 51, "<", FONT_1206, WHITE);
		//ssd1331_display_string(17, 51, "Undo", FONT_1206, WHITE);
		ssd1331_display_string(40, 51, "Stop", FONT_1206, WHITE);
		ssd1331_display_string(72, 51, "Play", FONT_1206, WHITE);
	}
}

void drawCoordMode(int mode)
{
	clearMenu();

	ssd1331_display_string(3, 0, "TRYB COORDINATE", FONT_1206, GREEN);

	if(mode == 0)
	{
		  ssd1331_display_string(7, 12, "1.Pojazd", FONT_1206, GREEN);
		  ssd1331_display_string(7, 23, "2.Manipulator", FONT_1206, GREEN);

		  ssd1331_display_string(2, 51, "<", FONT_1206, WHITE);
		  ssd1331_display_string(83, 51, "OK", FONT_1206, GREEN);
	}

	if(mode == 1)
	{
		ssd1331_display_string(4,  15, "X:      Y:          Z:", FONT_1206, GREEN);

		ssd1331_draw_line(13,50,13,63, WHITE);
		ssd1331_draw_line(43,50,43,63, WHITE);
		ssd1331_draw_line(72,50,72,63, WHITE);

		ssd1331_display_string(2, 51, "<", FONT_1206, WHITE);
		ssd1331_display_string(17, 51, "Undo", FONT_1206, WHITE);
		ssd1331_display_string(46, 51, "Next", FONT_1206, WHITE);
		ssd1331_display_string(76, 51, "New", FONT_1206, WHITE);
	}
}

void clearMenu()
{
	ssd1331_fill_rect(0,0,95,49,BLACK);
	ssd1331_fill_rect(0,51,95,12,BLACK);
}
