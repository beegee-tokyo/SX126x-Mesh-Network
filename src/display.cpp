#include "main.h"

#ifdef HAS_DISPLAY
#include <SH1106Wire.h>

SH1106Wire display(0x3c, OLED_SDA, OLED_SCL);
// SH1106Wire display(0x3c, 13, 32);

/** Task to control the LEDs */
TaskHandle_t displayHandler = NULL;

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

void dispWriteHeader(void)
{
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	int strWidth = display.getStringWidth("LoRa Mesh Test #dev " + String(numElements+1));
	display.drawString(64 - (strWidth / 2), 0, "LoRa Mesh Test #dev " + String(numElements + 1));
	display.display();
}

void dispWrite(String text, int x, int y)
{
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.drawString(x, y, text);
}

void dispUpdate(void)
{
	display.display();
}
#endif