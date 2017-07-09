#include "MKL17Z4.h"              // Device header
#include "gpio.h"
#include "delay.h"

#ifndef ILI9325_H
#define ILI9325_H


//registers

const uint16_t DRIVER_OUTPUT_CONTROL_1 = 0x01;
const uint16_t LCD_DRIVING_CONTROL     = 0x02;
const uint16_t ENTRY_MODE         		 = 0x03;
const uint16_t RESIZE_CONTROL          = 0x04;
const uint16_t DISPLAY_CONTROL_1       = 0x07;
const uint16_t DISPLAY_CONTROL_2       = 0x08;
const uint16_t DISPLAY_CONTROL_3       = 0x09;
const uint16_t DISPLAY_CONTROL_4       = 0x0A;
const uint16_t RGB_DISPLAY_INTERFACE_CONTROL_1 = 0x0C;
const uint16_t FRAME_MARKER_POSITION   = 0x0D;
const uint16_t RGB_DISPLAY_INTERFACE_CONTROL_2 = 0x0F;
const uint16_t POWER_CONTROL_1         = 0x10;
const uint16_t POWER_CONTROL_2         = 0x11;
const uint16_t POWER_CONTROL_3         = 0x12;
const uint16_t POWER_CONTROL_4         = 0x13;
const uint16_t POWER_CONTROL_7         = 0x29;
const uint16_t FRAME_RATE_COLOR_CONTROL= 0x2B;
const uint16_t HORIZONTAL_ADDRESS_SET  = 0x20;
const uint16_t VERTICAL_ADDRESS_SET    = 0x21;
const uint16_t GAMMA_CONTROL_1         = 0x30;
const uint16_t GAMMA_CONTROL_2         = 0x31;
const uint16_t GAMMA_CONTROL_3         = 0x32;
const uint16_t GAMMA_CONTROL_4         = 0x35;
const uint16_t GAMMA_CONTROL_5         = 0x36;
const uint16_t GAMMA_CONTROL_6         = 0x37;
const uint16_t GAMMA_CONTROL_7         = 0x38;
const uint16_t GAMMA_CONTROL_8         = 0x39;
const uint16_t GAMMA_CONTROL_9         = 0x3C;
const uint16_t GAMMA_CONTROL_10        = 0x3D;
const uint16_t WINDOW_HORIZONTAL_ADDRESS_START = 0x50;
const uint16_t WINDOW_HORIZONTAL_ADDRESS_END   = 0x51;
const uint16_t WINDOW_VERTICAL_ADDRESS_START   = 0x52;
const uint16_t WINDOW_VERTICAL_ADDRESS_END     = 0x53;
const uint16_t GATE_SCAN_CONTROL_1     = 0x60;
const uint16_t GATE_SCAN_CONTROL_2     = 0x61;
const uint16_t GATE_SCAN_CONTROL_3     = 0x6A;
const uint16_t PARTIAL_IMAGE_1_DISPLAY_POSITION       = 0x80;
const uint16_t PARTIAL_IMAGE_1_RAM_START_LINE_ADDRESS = 0x81;
const uint16_t PARTIAL_IMAGE_1_RAM_END_LINE_ADDRESS   = 0x82;
const uint16_t PARTIAL_IMAGE_2_DISPLAY_POSITION       = 0x83;
const uint16_t PARTIAL_IMAGE_2_RAM_START_LINE_ADDRESS = 0x84;
const uint16_t PARTIAL_IMAGE_2_RAM_END_LINE_ADDRESS   = 0x85;
const uint16_t PANEL_INTERFACE_CONTROL_1              = 0x90;
const uint16_t PANEL_INTERFACE_CONTROL_2              = 0x92;
const uint16_t PANEL_INTERFACE_CONTROL_4              = 0x95;

//colors
const uint16_t BLACK   =	 0x0000;
const uint16_t BLUE	   =	 0x001F;
const uint16_t RED 	   =	 0xF800;
const uint16_t GREEN   =	 0x07E0;
const uint16_t CYAN	   =	 0x07FF;
const uint16_t MAGENTA =	 0xF81F;
const uint16_t YELLOW  =	 0xFFE0;
const uint16_t WHITE   =	 0xFFFF;

const uint16_t GRAY	   =	 0xE79C;
const uint16_t SKY     =	 0x5d1C;
const uint16_t ORANGE  =   0xFCA0;
const uint16_t PINK    =	 0xF97F;
const uint16_t BROWN   =	 0x8200;
const uint16_t VIOLET  =	 0x9199;
const uint16_t SILVER  =   0xA510;
const uint16_t GOLD    =	 0xA508;
const uint16_t BEGH    =	 0xF77B;
const uint16_t NAVY    =	 0x000F;
const uint16_t DARK_GREEN =  0x03E0;
const uint16_t DARK_CYAN  =	 0x03EF;
const uint16_t MAROON     =  0x7800;
const uint16_t PURPLE     =	 0x7BE0;
const uint16_t LIGHT_GREY =	 0xC618;
const uint16_t DARK_GREY  =  0x7BEF;


/*
************registers****************
*/


//GRAM area
const unsigned char h_Gram_start = 0x50;
const unsigned char h_Gram_end   = 0x51;
const unsigned char v_Gram_start = 0x52;
const unsigned char v_Gram_end   = 0x53;

//Entry mode
const unsigned char entry_mode = 0x03;

//Resize
const unsigned char resize = 0x04;


class ili9325
{
//variables
public:
	
protected:
private:
	enum commPins {RST=4, WR=2 , CS=0 , RS=1 ,RD=3};


	Gpio pinData;
	Gpio pinCommand;
	unsigned int x_start, x_end, y_start, y_end;

//functions
public:
	ili9325();
	void point (uint16_t x , uint16_t y, uint16_t color);
	void fill_screen (uint16_t color);
	void set_area (uint16_t x1, uint16_t y1, uint16_t x2 , uint16_t y2);
	void set_cursor (uint16_t x , uint16_t y);
	void data (uint16_t dta);	
	void putchar (uint16_t x , uint16_t y , char * ch , uint16_t color , uint16_t background);
	uint16_t device_code ();
	uint16_t read_reg(uint16_t indx);
protected:
private:
	void init ();
	void index (uint16_t indx);
	void init_bus_output ();
	void init_bus_input ();


	//void wr_reg (unsigned int indx , unsigned int dta);
	void wr_reg (uint16_t indx , uint16_t dta);
	uint16_t read_data();



}; //ili9325

#endif //__ILI9325_H__
