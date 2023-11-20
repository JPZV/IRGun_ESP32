#ifndef ABSMOUSE_h
#define ABSMOUSE_h

#include "NimBLEHIDDevice.h"

#define MOUSE_LEFT 0x01
#define MOUSE_RIGHT 0x02
#define MOUSE_MIDDLE 0x04

class AbsMouse_
{
private:
	uint8_t _buttons;
	uint16_t _x;
	uint16_t _y;
	uint32_t _width;
	uint32_t _height;
	bool _autoReport;
	NimBLEService* m_deviceInfoService; //0x180a
	NimBLEService* m_hidService; //0x1812
	NimBLECharacteristic *inputMouse;

	static void taskServer(void *pvParameter);

public:
	AbsMouse_(void);
	void init(uint16_t width = 32767, uint16_t height = 32767, bool autoReport = true);
	void report(void);
	void move(uint16_t x, uint16_t y);
	void press(uint8_t b = MOUSE_LEFT);
	void release(uint8_t b = MOUSE_LEFT);
};
extern AbsMouse_ AbsMouse;
#endif
