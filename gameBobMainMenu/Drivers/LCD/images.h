#ifndef __FONT_H
#define __FONT_H

#include "ugui.h"

//extern UG_BMP fry;
typedef struct{
	const uint8_t (*pic)[11];
	uint16_t width;
	uint16_t height;
}PICTURE;

extern PICTURE play_game_pic;
extern PICTURE load_game_pic;
extern PICTURE power_off_pic;

//extern UG_BMP play_game_pic;
//extern UG_BMP load_game_pic;
//extern UG_BMP power_off_pic;


#endif
