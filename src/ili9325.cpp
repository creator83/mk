#include "ili9325.h"

#define BIT8


#ifdef BIT8
ili9325::ili9325()
:pinData (Gpio::C), pinCommand (Gpio::C)
{
	pinCommand.setOutPort(1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);

	init();
}

void ili9325::index(uint16_t indx)
{
	//отправляем команду
	pinCommand.clearPin(RS);
	pinCommand.clearPin(CS);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (indx >> 8);
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (static_cast <uint8_t>(indx));
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinCommand.setPin(CS);
}

void ili9325::data(uint16_t dta)
{
	//отправляем данные
	pinCommand.setPin(RS);
	pinCommand.clearPin(CS);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (dta >> 8);
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (static_cast <uint8_t>(dta));
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinCommand.setPin(CS);
}
/*
void ili9325::wr_reg (uint16_t indx , uint16_t dta)
{
	pinCommand.clearPin(CS);
	pinCommand.clearPin(RS);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (indx >> 8);
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (static_cast <uint8_t>(indx));
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	//отправляем данные
	pinCommand.setPin(RS);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (dta >> 8);
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinData.clearValPort (1 << RST|1 << WR|1 << CS|1 << RS|1 << RD);
	pinData.setValPort (static_cast <uint8_t>(dta));
	pinCommand.clearPin(WR);
	pinCommand.setPin(WR);
	pinCommand.setPin(CS);
}
*/
void ili9325::wr_reg (uint16_t indx , uint16_t dta)
{
	index(indx);
	data(dta);
}

#else
ili9325::ili9325()
:pinDataLow (Gpio::D) , pinDataHigh (Gpio::C) , pinCommand (Gpio::B)
{
	pinDataLow.setOutPort(0xFF);
	pinDataHigh.setOutPort(0xFF);
	pinCommand.setOutPin(CS);
	pinCommand.setOutPin(RS);	
	pinCommand.setOutPin(RST);
	pinCommand.setOutPin(RD);	
	pinCommand.setOutPin(WR);
	pinCommand.setPin(CS);
	pinCommand.setPin(RS);
	pinCommand.setPin(WR);
	pinCommand.setPin(RD);
	init();
} //ili9325

void ili9325::index(uint16_t indx)
{

	//отправляем команду
	pinCommand.clearPin(CS);
	pinCommand.clearPin(RS);
	pinDataLow.clearValPort (0xFF);
	pinDataHigh.clearValPort (0xFF);
	pinDataLow.setValPort(0xFF&indx);
	pinDataHigh.setValPort (indx >> 8);
	pinCommand.clearPin(WR);
	delay_us(5);
	pinCommand.setPin(WR);
	pinCommand.setPin(CS);
}

void ili9325::data(uint16_t dta)
{
	//отправляем данные
	pinCommand.clearPin(CS);
	pinCommand.setPin(RS);
	pinDataLow.clearValPort (0xFF);
	pinDataHigh.clearValPort (0xFF);
	pinDataLow.setValPort(0xFF&dta);
	pinDataHigh.setValPort (dta >> 8);
	pinCommand.clearPin(WR);
	delay_us(5);
	pinCommand.setPin(WR);
	pinCommand.setPin(CS);

}

void ili9325::wr_reg (uint16_t indx , uint16_t dta)
{
	pinCommand.clearPin(CS);
	pinCommand.clearPin(RS);
	pinDataLow.clearValPort (0xFF);
	pinDataHigh.clearValPort (0xFF);
	pinDataLow.setValPort(0xFF&indx);
	pinDataHigh.setValPort (indx >> 8);
	pinCommand.clearPin(WR);
	delay_us(5);
	pinCommand.setPin(WR);
	pinCommand.setPin(RS);
	pinDataLow.clearValPort (0xFF);
	pinDataHigh.clearValPort (0xFF);
	pinDataLow.setValPort(0xFF&dta);
	pinDataHigh.setValPort (dta >> 8);
	pinCommand.clearPin(WR);
	delay_us(5);
	pinCommand.setPin(WR);
	pinCommand.setPin(CS);
}

#endif
void ili9325::init()
{
	pinCommand.clearPin(RST);
	delay_ms(100);
	pinCommand.setPin(RST);
	delay_ms(100);
	
	
	/*
	wr_reg(0x00E5,0x78F0);
	wr_reg(0x0001,0x0100);
	wr_reg(0x0002,0x0700);
	wr_reg(0x0003,0x1030);
	wr_reg(0x0004,0x0000);
	wr_reg(0x0008,0x0202);
	wr_reg(0x0009,0x0000);
	wr_reg(0x000A,0x0000);
	wr_reg(0x000C,0x0000);
	wr_reg(0x000D,0x0000);
	wr_reg(0x000F,0x0000);
	//power on sequence VGHVGL
	wr_reg(0x0010,0x0000);
	wr_reg(0x0011,0x0007);
	wr_reg(0x0012,0x0000);
	wr_reg(0x0013,0x0000);
	wr_reg(0x0007,0x0000);
	//vgh
	wr_reg(0x0010,0x1690);
	wr_reg(0x0011,0x0227);
	_delay_ms(100);
	//vregiout
	wr_reg(0x0012,0x009D); //0x001b
	_delay_ms(100);
	//vom amplitude
	wr_reg(0x0013,0x1900);
	_delay_ms(100);
	//vom H
	wr_reg(0x0029,0x0025);
	wr_reg(0x002B,0x000D);
	//gamma
	wr_reg(0x0030,0x0007);
	wr_reg(0x0031,0x0303);
	wr_reg(0x0032,0x0003);// 0006
	wr_reg(0x0035,0x0206);
	wr_reg(0x0036,0x0008);
	wr_reg(0x0037,0x0406);
	wr_reg(0x0038,0x0304);//0200
	wr_reg(0x0039,0x0007);
	wr_reg(0x003C,0x0602);// 0504
	wr_reg(0x003D,0x0008);
	//ram
	wr_reg(0x0050,0x0000);
	wr_reg(0x0051,0x00EF);
	wr_reg(0x0052,0x0000);
	wr_reg(0x0053,0x013F);
	wr_reg(0x0060,0xA700);
	wr_reg(0x0061,0x0001);
	wr_reg(0x006A,0x0000);
	wr_reg(0x0080,0x0000);
	wr_reg(0x0081,0x0000);
	wr_reg(0x0082,0x0000);
	wr_reg(0x0083,0x0000);
	wr_reg(0x0084,0x0000);
	wr_reg(0x0085,0x0000);
	wr_reg(0x0090,0x0010);
	wr_reg(0x0092,0x0600);
	wr_reg(0x0007,0x0133);
	wr_reg(0x00,0x0022);
	
	
	
	
	//=== Start Initial Sequence ===//

	wr_reg(0xE3, 0x3008); // Set internal timing
	wr_reg(0xE7, 0x0012); // Set internal timing
	wr_reg(0xEF, 0x1231); // Set internal timing
	wr_reg(0x01, 0x0100); // set SS and SM bit
	wr_reg(0x02, 0x0700); // set 1 line inversion
	wr_reg(0x03, 0x1020); // set GRAM write direction and BGR=1.
	wr_reg(0x04, 0x0000); // Resize register
	wr_reg(0x08, 0x0202); // set the back porch and front porch
	wr_reg(0x09, 0x0000); // set non-display area refresh cycle ISC[3:0]
	wr_reg(0x0A, 0x0000); // FMARK function
	wr_reg(0x0C, 0x0000); // RGB interface setting
	wr_reg(0x0D, 0x0000); // Frame marker Position
	wr_reg(0x0F, 0x0000); // RGB interface polarity

	//===Power On sequence ===//

	wr_reg(0x10, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
	wr_reg(0x11, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
	wr_reg(0x12, 0x0000); // VREG1OUT voltage
	wr_reg(0x13, 0x0000); // VDV[4:0] for VCOM amplitude
	delay_ms(200); // Dis-charge capacitor power voltage
	wr_reg(0x10, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
	wr_reg(0x11, 0x0227); // R11h=0x0221 at VCI=3.3V, DC1[2:0], DC0[2:0], VC[2:0]
	delay_ms(50); // Delay 50ms
	wr_reg(0x12, 0x001C); // External reference voltage= Vci;
	delay_ms(50); // Delay 50ms
	wr_reg(0x13, 0x1800); // R13=1200 when R12=009D;VDV[4:0] for VCOM amplitude
	wr_reg(0x29, 0x001C); // R29=000C when R12=009D;VCM[5:0] for VCOMH
	wr_reg(0x2B, 0x000D); // Frame Rate = 91Hz
	delay_ms(50); // Delay 50ms
	wr_reg(0x20, 0x0000); // GRAM horizontal Address
	wr_reg(0x21, 0x0000); // GRAM Vertical Address

	//===Adjust the Gamma Curve===//

	wr_reg(0x30, 0x0007);
	wr_reg(0x31, 0x0302);
	wr_reg(0x32, 0x0105);
	wr_reg(0x35, 0x0206);
	wr_reg(0x36, 0x0808);
	wr_reg(0x37, 0x0206);
	wr_reg(0x38, 0x0504);
	wr_reg(0x39, 0x0007);
	wr_reg(0x3C, 0x0105);
	wr_reg(0x3D, 0x0808);

	//===Set GRAM area===//

	wr_reg(0x50, 0x0000); // Horizontal GRAM Start Address
	wr_reg(0x51, 0x00EF); // Horizontal GRAM End Address
	wr_reg(0x52, 0x0000); // Vertical GRAM Start Address
	wr_reg(0x53, 0x013F); // Vertical GRAM Start Address
	wr_reg(0x60, 0xA700); // Gate Scan Line
	wr_reg(0x61, 0x0001); // NDL,VLE, REV
	wr_reg(0x6A, 0x0000); // set scrolling line

	//===Partial Display Control===//

	wr_reg(0x80, 0x0000);
	wr_reg(0x81, 0x0000);
	wr_reg(0x82, 0x0000);
	wr_reg(0x83, 0x0000);
	wr_reg(0x84, 0x0000);
	wr_reg(0x85, 0x0000);

	//===Panel Control===//

	wr_reg(0x90, 0x0010);
	wr_reg(0x92, 0x0000);
	wr_reg(0x93, 0x0003);
	wr_reg(0x95, 0x0110);
	wr_reg(0x97, 0x0000);
	wr_reg(0x98, 0x0000);
	wr_reg(0x07, 0x0133); // 262K color and display ON
            */

  wr_reg(0x0000, 0x0001);                   
  delay_ms(10);                              

  wr_reg(0x00E3, 0x3008);
  wr_reg(0x00E7, 0x0012);
  wr_reg(0x00EF, 0x1231);
           
  wr_reg(0x0000, 0x0001);                    //start internal osc
  wr_reg(DRIVER_OUTPUT_CONTROL_1, 0x0100);        //SM = 0, SS = 1
  wr_reg(LCD_DRIVING_CONTROL, 0x0700);          //B/C = 1, EOR = 1                     
  wr_reg(ENTRY_MODE, 0x1030);                  //
  wr_reg(RESIZE_CONTROL, 0x0000);                  //RSZ0 = 0, RSZ1 = 0, RCH0 = 0, RCH1 = 0, RCV0 = 0, RCV1 = 0         
  wr_reg(DISPLAY_CONTROL_2, 0x0202);                 //BP[3:0] = 0010, FP[3:0] = 0010
  wr_reg(DISPLAY_CONTROL_3, 0x0000);               //PTS[2:0] = 000, PTG[1:0] = 00, ISC[3:0] = 0000
  wr_reg(DISPLAY_CONTROL_4, 0x0000);             //FMI[2:0] = 000 (???????? ?????? - 1 ?????),
                                                //FMARKOE = 0 (FMARK ?????? ????????)         
  wr_reg(RGB_DISPLAY_INTERFACE_CONTROL_1, 0x0000);   //RIM[1:0] = 00 (18-bit RGB ?????????),
                                                //DM[1:0] = 00 (?????????? ????????????),
                                                //RM = 0 (????????? ?????????/VSYNC ?????????),
                                                //ENC[2:0] = 000 (1 ???? ?????? ? GRAM)           
  wr_reg(FRAME_MARKER_POSITION, 0x0000);          //FMP[8:0] = 000000000 (??????? ?????? FMARK - ?????? 0)         
  wr_reg(RGB_DISPLAY_INTERFACE_CONTROL_2, 0x0000);   //DPL = 0 (???? ?????? ?? ???????????? ?????? DOTCLK),
                                                //EPL = 0 (?????? ?????? ??? ENABLE = 0),
                                                //HSPL = 0 (???????? ??????? HSYNC - ??????),
                                                //VSPL = 0 (???????? ??????? VSYNC - ??????)
  wr_reg(POWER_CONTROL_1, 0x0000);               //STB = 0 (standby mode ????????),
                                                //SLP = 0 (sleep mode ????????),
                                                //DSTB = 0 (deep standby mode ????????),
                                                //AP[2:0] = 000 (????????? gamma driver - ???????????; ????????? source driver - ???????????),
                                                //APE = 0 (power supply ????????),
                                                //BT[2:0] = 000 (DDVDH = Vci1*2, VCL = -Vci1, VGH = Vci1*6, VGL = -Vci1*5),
                                                //SAP = 0
  wr_reg(POWER_CONTROL_2, 0x0007);               //VC[2:0] = 111 (Vci1 = Vci*1,0),
                                                //DC0[2:0] = 000 (Fdcdc1 = Fosc),
                                                //DC1[2:0] = 000 (Fdcdc2 = Fosc/4)
  wr_reg(POWER_CONTROL_3, 0x0000);                 //VRH[3:0] = 0000 (VREG1OUT - ??????????),
                                                //VCIRE = 0 (??????? ???????? ?????????? Vci),
                                                //PON = 0 (VGL ????? ????????)                                               
  wr_reg(POWER_CONTROL_4, 0x0000);               //VDV[4:0] = 00000 (Vcom = VREG1OUT*0,70)
  wr_reg(DISPLAY_CONTROL_1, 0x0001);               //D[1:0] = 01 (source = GND, ????? Vcom = GND, ?????????? ????????),
                                                //CL = 0 (262144 ?????),
                                                //DTE = 0, GON = 0 (????? gate G1~G320 = VGH),
                                                //BASEE = 0 (base image ????????),
                                                //PTDE[1:0] = 00 (partial image 1 ? partial image 2 ?????????)
                       
  delay_ms(50);                                 //????????

  wr_reg(POWER_CONTROL_1, 0x1090);               //STB = 0 (standby mode ????????),
                                                //SLP = 0 (sleep mode ????????),
                                                //DSTB = 0 (deep standby mode ????????),
                                                //AP[2:0] = 001 (????????? gamma driver -> 1,0; ????????? source driver -> 1,0),
                                                //APE = 1 (power supply ????????),
                                                //BT[2:0] = 000 (DDVDH = Vci1*2, VCL = -Vci1, VGH = Vci1*6, VGL = -Vci1*5),
                                                //SAP = 1 (source driver ????????)
  wr_reg(POWER_CONTROL_2, 0x0227);               //VC[2:0] = 111 (Vci1 = Vci*1,0),
                                                //DC0[2:0] = 010 (Fdcdc1 = Fosc/4),
                                                //DC1[2:0] = 010 (Fdcdc2 = Fosc/16)

  delay_ms(50);                                 //????????

  wr_reg(POWER_CONTROL_3, 0x001f);             //VRH[3:0] = 1111 (VREG1OUT = Vci*1,95),
                                                //VCIRE = 0 (??????? ???????? ?????????? Vci),
                                                //PON = 1 (VGL ????? ????????)
                       
  delay_ms(50);                                 //????????

  wr_reg(POWER_CONTROL_4, 0x1500);               //VDV[4:0] = 10101 (Vcom = VREG1OUT*1,04)
  wr_reg(POWER_CONTROL_7, 0x0027);               //VCM[5:0] = 010111 (VcomH = VREG1OUT*0,8)
  wr_reg(FRAME_RATE_COLOR_CONTROL, 0x000d);      //FRS[3:0] = 1101 (frame rate = 128)

  delay_ms(50);                                 //????????

  wr_reg(HORIZONTAL_ADDRESS_SET, 0x0000);          //AD[16:0] = 00000000000000000                                                   
  wr_reg(VERTICAL_ADDRESS_SET, 0x0000);          //AD[16:0] = 00000000000000000
             
  delay_ms(50);                                 //????????

  wr_reg(GAMMA_CONTROL_1, 0x0000);             //KP1[2:0] = 000, KP0[2:0] = 000
  wr_reg(GAMMA_CONTROL_2, 0x0707);               //KP3[2:0] = 111, KP2[2:0] = 111
  wr_reg(GAMMA_CONTROL_3, 0x0307);               //KP5[2:0] = 011, KP4[2:0] = 111
  wr_reg(GAMMA_CONTROL_4, 0x0200);               //RP1[2:0] = 010, RP0[2:0] = 000
  wr_reg(GAMMA_CONTROL_5, 0x0008);             //VRP1[4:0] = 00000, VRP0[3:0] = 1000
  wr_reg(GAMMA_CONTROL_6, 0x0004);               //KN1[2:0] = 000, KN0[2:0] = 100
  wr_reg(GAMMA_CONTROL_7, 0x0000);                 //KN3[2:0] = 000, KN2[2:0] = 000
  wr_reg(GAMMA_CONTROL_8, 0x0707);              //KN5[2:0] = 111, KN4[2:0] = 111
  wr_reg(GAMMA_CONTROL_9, 0x0002);               //RN1[2:0] = 000, RN0[2:0] = 010
  wr_reg(GAMMA_CONTROL_10, 0x1d04);            //VRN1[4:0] = 11101, VRN0[3:0] = 0100

  delay_ms(50);                                 //????????

  wr_reg(WINDOW_HORIZONTAL_ADDRESS_START, 0x0000);           //HSA[7:0] = 00000000
  wr_reg(WINDOW_HORIZONTAL_ADDRESS_END, 0x00ef);         //HEA[7:0] = 11101111
  wr_reg(WINDOW_VERTICAL_ADDRESS_START, 0x0000);           //VSA[8:0] = 000000000
  wr_reg(WINDOW_VERTICAL_ADDRESS_END, 0x013f);            //VEA[8:0] = 100111111
  wr_reg(GATE_SCAN_CONTROL_1, 0xa700);                    //SCN[5:0] = 000000 (???????????? ? G320),
                                                      //NL[5:0] = 100111 (320 ????? LCD),
                                                      //GS = 1 (???????????? G320~G1)
  wr_reg(GATE_SCAN_CONTROL_2, 0x0001);                //NDL = 0 (V63 - ?????????????, V0 - ?????????????),
                                                      //VLE = 0 (???????????? ????????? ?????????),
                                                      //REV = 1 (???????? ????? ??????)
  wr_reg(GATE_SCAN_CONTROL_3, 0x0000);                  //VL[8:0] = 000000000 (0 ????? ?????????)

  wr_reg(PARTIAL_IMAGE_1_DISPLAY_POSITION, 0x0000);
  wr_reg(PARTIAL_IMAGE_1_RAM_START_LINE_ADDRESS, 0x0000);   
  wr_reg(PARTIAL_IMAGE_1_RAM_END_LINE_ADDRESS, 0x0000);
  wr_reg(PARTIAL_IMAGE_2_DISPLAY_POSITION, 0x0000);
  wr_reg(PARTIAL_IMAGE_2_RAM_START_LINE_ADDRESS, 0x0000);
  wr_reg(PARTIAL_IMAGE_2_RAM_END_LINE_ADDRESS, 0x0000);
     
  wr_reg(PANEL_INTERFACE_CONTROL_1, 0x0010);               //RTNI[4:0] = 10000 (16 ?????? ?? ??????),
                                                      //DIVI[1:0] = 00 (????????? ???????? ??????? Fosc)
  wr_reg(PANEL_INTERFACE_CONTROL_2, 0x0600);              //NOWI[2:0] = 101 (5 ?????? ??????? ?????? gate)

  wr_reg(DISPLAY_CONTROL_1, 0x0133);                  //D[1:0] = 11 (??????????? base image, ?????????? ????????),
                                                      //CL = 0 (262144 ?????),
                                                      //DTE = 1, GON = 1 (?????????? ???????????),
                                                      //BASEE = 1 (base image ????????),
                                                      //PTDE[1:0] = 00 (partial image 1 ? partial image 2 ?????????)
   
  wr_reg(HORIZONTAL_ADDRESS_SET, 0x0000);                  //AD[16:0] = 00000000000000000                                         
  wr_reg(VERTICAL_ADDRESS_SET, 0x0000);               //AD[16:0] = 00000000000000000

}




/*void ili9325::wr_reg (uint16_t indx , uint16_t dta)
{
	index (indx);
	data (dta);
}
*/

void ili9325::set_cursor (uint16_t x , uint16_t y)
{
	wr_reg (0x20 , x);
	wr_reg(0x21 , y);
	index(0x0022);
}

void ili9325::point (uint16_t x , uint16_t y, uint16_t color)
{
	set_cursor(x,y);
	data(color);
}

void ili9325::fill_screen (uint16_t color)
{
	set_cursor(0,0);
	for (long i=0;i<76800;++i)
	{
		data(color);
	}
}

void ili9325::set_area (uint16_t x1 , uint16_t y1 , uint16_t x2 , uint16_t y2)
{
	x_start = x1;
	x_end = x2;
	y_start = y1;
	y_end = y2;
	wr_reg(h_Gram_start,x1);
	wr_reg(v_Gram_start,y1);
	wr_reg(h_Gram_end,x2);
	wr_reg(v_Gram_end,y2);
}

void ili9325::putchar (uint16_t x , uint16_t y , char * ch , uint16_t color , uint16_t background)
{
	set_cursor(x,y);
	
}

