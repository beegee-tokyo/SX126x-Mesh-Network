#include "main.h"

#ifdef HAS_DISPLAY
#include <SH1106Wire.h>

/** Number of elements in the mesh map */
extern uint8_t numElements;

/** Singleton of the display class */
SH1106Wire display(0x3c, OLED_SDA, OLED_SCL);

/** Task to control the LEDs */
TaskHandle_t displayHandler = NULL;

/**
 * Initialize the display
 */
void initDisplay(void)
{
	myLog_d("Display init");
	display.init();
	display.displayOn();
	display.flipScreenVertically();
	display.setContrast(255);
	display.setFont(ArialMT_Plain_10);

	dispWriteHeader();
	display.display();
}

/**
 * Write the top line of the display
 */
void dispWriteHeader(void)
{
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	// int strWidth = display.getStringWidth("LoRa Mesh Test #dev " + String(numElements + 1));
	// display.drawString(64 - (strWidth / 2), 0, "LoRa Mesh Test #dev " + String(numElements + 1));
	char lineString[64] = {0};
	sprintf(lineString, "LR Mesh %08X #n %d", deviceID, numOfNodes() + 1);
	display.drawString(0, 0, lineString);
	display.display();
}

/**
 * Write text to the display
 * @param text
 * 		Text to be written
 * @param x
 * 		X position where text starts
 * @param y
 * 		y position where text starts
 */
void dispWrite(String text, int x, int y)
{
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.drawString(x, y, text);
}

/**
 * Update the display content
 */
void dispUpdate(void)
{
	display.display();
}
#endif