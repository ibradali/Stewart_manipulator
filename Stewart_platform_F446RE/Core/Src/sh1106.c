

#include "sh1106.h"


extern I2C_HandleTypeDef hi2c2;

static uint8_t buffer[8][132];


/*
 * I2C control bytes:
 *
 * 	Last control byte Only data bytes to follow & data byte is command: 0x00
 *
 * 	Next two bytes are a data byte and a control byte & data byte is command: 0x80
 *
 * 	Last control byte, Only data bytes to follow & data byte is RAM operation: 0x40
 *
 */




static const uint8_t font[] = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 94, 0, 0, 0, 0,		// !
	0, 0, 4, 3, 4, 3, 0, 0,
	0, 36, 126, 36, 36, 126, 36, 0,
	0, 36, 74, 255, 82, 36, 0, 0,
	0, 70, 38, 16, 8, 100, 98, 0,
	0, 52, 74, 74, 52, 32, 80, 0,
	0, 0, 0, 4, 3, 0, 0, 0,
	0, 0, 0, 126, 129, 0, 0, 0,
	0, 0, 0, 129, 126, 0, 0, 0,
	0, 42, 28, 62, 28, 42, 0, 0,
	0, 8, 8, 62, 8, 8, 0, 0,
	0, 0, 0, 128, 96, 0, 0, 0,
	0, 8, 8, 8, 8, 8, 0, 0,
	0, 0, 0, 0, 96, 0, 0, 0,
	0, 64, 32, 16, 8, 4, 2, 0,
	0, 62, 65, 73, 65, 62, 0, 0,
	0, 0, 66, 127, 64, 0, 0, 0,
	0, 0, 98, 81, 73, 70, 0, 0,
	0, 0, 34, 73, 73, 54, 0, 0,
	0, 0, 14, 8, 127, 8, 0, 0,
	0, 0, 35, 69, 69, 57, 0, 0,
	0, 0, 62, 73, 73, 50, 0, 0,
	0, 0, 1, 97, 25, 7, 0, 0,
	0, 0, 54, 73, 73, 54, 0, 0,
	0, 0, 6, 9, 9, 126, 0, 0,
	0, 0, 0, 102, 0, 0, 0, 0,
	0, 0, 128, 102, 0, 0, 0, 0,
	0, 0, 8, 20, 34, 65, 0, 0,
	0, 0, 20, 20, 20, 20, 0, 0,
	0, 0, 65, 34, 20, 8, 0, 0,
	0, 2, 1, 81, 9, 6, 0, 0,
	0, 28, 34, 89, 89, 82, 12, 0,
	0, 0, 126, 9, 9, 126, 0, 0,
	0, 0, 127, 73, 73, 54, 0, 0,
	0, 0, 62, 65, 65, 34, 0, 0,
	0, 0, 127, 65, 65, 62, 0, 0,
	0, 0, 127, 73, 73, 65, 0, 0,
	0, 0, 127, 9, 9, 1, 0, 0,
	0, 0, 62, 65, 81, 50, 0, 0,
	0, 0, 127, 8, 8, 127, 0, 0,
	0, 0, 65, 127, 65, 0, 0, 0,
	0, 0, 32, 64, 64, 63, 0, 0,
	0, 0, 127, 8, 20, 99, 0, 0,
	0, 0, 127, 64, 64, 64, 0, 0,
	0, 127, 2, 4, 2, 127, 0, 0,
	0, 127, 6, 8, 48, 127, 0, 0,
	0, 0, 62, 65, 65, 62, 0, 0,
	0, 0, 127, 9, 9, 6, 0, 0,
	0, 0, 62, 65, 97, 126, 64, 0,
	0, 0, 127, 9, 9, 118, 0, 0,
	0, 0, 38, 73, 73, 50, 0, 0,
	0, 1, 1, 127, 1, 1, 0, 0,
	0, 0, 63, 64, 64, 63, 0, 0,
	0, 31, 32, 64, 32, 31, 0, 0,
	0, 63, 64, 48, 64, 63, 0, 0,
	0, 0, 119, 8, 8, 119, 0, 0,
	0, 3, 4, 120, 4, 3, 0, 0,
	0, 0, 113, 73, 73, 71, 0, 0,
	0, 0, 127, 65, 65, 0, 0, 0,
	0, 2, 4, 8, 16, 32, 64, 0,
	0, 0, 0, 65, 65, 127, 0, 0,
	0, 4, 2, 1, 2, 4, 0, 0,
	0, 64, 64, 64, 64, 64, 64, 0,
	0, 0, 1, 2, 4, 0, 0, 0,
	0, 0, 48, 72, 40, 120, 0, 0,
	0, 0, 127, 72, 72, 48, 0, 0,
	0, 0, 48, 72, 72, 0, 0, 0,
	0, 0, 48, 72, 72, 127, 0, 0,
	0, 0, 48, 88, 88, 16, 0, 0,
	0, 0, 126, 9, 1, 2, 0, 0,
	0, 0, 80, 152, 152, 112, 0, 0,
	0, 0, 127, 8, 8, 112, 0, 0,
	0, 0, 0, 122, 0, 0, 0, 0,
	0, 0, 64, 128, 128, 122, 0, 0,
	0, 0, 127, 16, 40, 72, 0, 0,
	0, 0, 0, 127, 0, 0, 0, 0,
	0, 120, 8, 16, 8, 112, 0, 0,
	0, 0, 120, 8, 8, 112, 0, 0,
	0, 0, 48, 72, 72, 48, 0, 0,
	0, 0, 248, 40, 40, 16, 0, 0,
	0, 0, 16, 40, 40, 248, 0, 0,
	0, 0, 112, 8, 8, 16, 0, 0,
	0, 0, 72, 84, 84, 36, 0, 0,
	0, 0, 8, 60, 72, 32, 0, 0,
	0, 0, 56, 64, 32, 120, 0, 0,
	0, 0, 56, 64, 56, 0, 0, 0,
	0, 56, 64, 32, 64, 56, 0, 0,
	0, 0, 72, 48, 48, 72, 0, 0,
	0, 0, 24, 160, 160, 120, 0, 0,
	0, 0, 100, 84, 84, 76, 0, 0,
	0, 0, 8, 28, 34, 65, 0, 0,
	0, 0, 0, 126, 0, 0, 0, 0,
	0, 0, 65, 34, 28, 8, 0, 0,
	0, 0, 4, 2, 4, 2, 0, 0,
	0, 120, 68, 66, 68, 120, 0, 0,
};





// send command

void send_command(uint8_t command) {


	uint8_t control = 0x00;

	uint8_t buff[2] = {control, command};

	if (HAL_I2C_Master_Transmit(&hi2c2, SH1106_DEV_ADDR, buff, 2, 100) != HAL_OK) {
		Error_Handler();
	}

}


void draw_point(uint8_t x, uint8_t y){

	uint8_t page = y / 8;
	uint8_t line = y % 8;

	buffer[page][x] |= (1 << line);

}



void disp_data(void) {


	for (int page=0; page<8; page++) {

		send_command(SH1106_SET_PAGE_ADDR_CMD + page);
		send_command(SH1106_SET_LOW_COL_ADDR_CMD);
		send_command(SH1106_SET_HIGH_COL_ADDR_CMD);


		uint8_t page_buff[133] = {0x40};

		for (int column=0;column<132;column++) {

			page_buff[column+1] = buffer[page][column];
		}

		if( HAL_I2C_Master_Transmit(&hi2c2, SH1106_DEV_ADDR , page_buff, 133, 10) != HAL_OK) {
			Error_Handler();
		}
	}
}



void display_string(uint8_t line, char* str) {

	uint8_t i, j;
	uint8_t c;


	if (line <0 || line > 7) {
		return;
	}


	for (i=0;i<16;i++) {

		c = *str;
		str++;

		for (j=0; j<8; j++) {

			uint32_t font_index = 8*c+j;
			uint8_t buffer_column = i*8+j;
			buffer[line][buffer_column] = font[font_index];

		}
	}
}


void buff_init(void) {

	for (int row=0; row<8;row++) {

		for (int column = 0; column<132; column++) {

			buffer[row][column] = (uint8_t) 0;
		}
	}
}




void disp_init(void) {


	if (HAL_I2C_IsDeviceReady(&hi2c2, SH1106_DEV_ADDR, 1, 1000) != HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(100);

	send_command(SH1106_DISP_OFF_CMD);		// checked

	// set page address 0
	send_command(SH1106_SET_PAGE_ADDR_CMD);		// checked

	// set contrast
	send_command(SH1106_SET_CONTRAST_CMD_MODE);			// checked
	send_command(0xFF);					// checked

	// set segment remap
	send_command(SH1106_SET_SEG_REMAP_CMD);				// checked

	// Set normal display
	send_command(SH1106_SET_NORMAL_DISP_CMD);		// checked

	// set multiplexer
	send_command(SH1106_SET_MULTIPLEX_CMD);		// checked
	send_command(0x3F);							// checked


	// set DC/DC pump ON
	send_command(SH1106_SET_CONTROL_MODE);		// checked
	send_command(SH1106_SET_CONTROL);			// checked
	send_command(0x30|0x02);					// checked

	// set scan direction
	send_command(0xC8);							// checked


	//set display offset
	send_command(SH1106_SET_DISP_OFFSET_CMD);		// checked
	send_command(0x00);								// checked


	// set clock and divider
	send_command(SH1106_SET_CLKDIV_CMD);	// checked
	send_command(0x80);						// checked


	// set precharge
	send_command(SH1106_SET_PRECHARGE_CMD_MODE);		// checked
	send_command(SH1106_SET_PRECHARGE_CMD);				// checked

	// configure com pins
	send_command(SH1106_SET_PIN_CONFIG_CMD);			// checked
	send_command(0x12);									// checked


	// set VCOM
	send_command(SH1106_SET_VCOM_CMD_MODE);				// checked
	send_command(SH1106_SET_VCOM_CMD);					// checked


	buff_init();

	disp_data();


	// Turn on display
	send_command(SH1106_TURN_DISP_ON_CMD);


	// set start line
	send_command(SH1106_SET_START_LINE_CMD | 0x00);


	// set column address
	send_command(SH1106_SET_LOW_COL_ADDR_CMD);
	send_command(SH1106_SET_HIGH_COL_ADDR_CMD);





}


