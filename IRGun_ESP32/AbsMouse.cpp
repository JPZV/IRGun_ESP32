#include "AbsMouse.h"
#include <NimBLEServer.h>
#include <NimBLEDevice.h>

static uint8_t HID_REPORT_DESCRIPTOR[] PROGMEM = {
    0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x85, 0x01,       //     Report ID (1)
    0x05, 0x09,       //     Usage Page (Button)
    0x19, 0x01,       //     Usage Minimum (0x01)
    0x29, 0x03,       //     Usage Maximum (0x03)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x95, 0x03,       //     Report Count (3)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x05,       //     Report Size (5)
    0x81, 0x03,       //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,       //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x16, 0x00, 0x00, //     Logical Minimum (0)
    0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
    0x36, 0x00, 0x00, //     Physical Minimum (0)
    0x46, 0xFF, 0x7F, //     Physical Maximum (32767)
    0x75, 0x10,       //     Report Size (16)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x02,       //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,             //   End Collection
    0xC0              // End Collection
};

AbsMouse_::AbsMouse_(void) : _buttons(0), _x(0), _y(0)
{
}

void AbsMouse_::init(uint16_t width, uint16_t height, bool autoReport)
{
    _width = width;
    _height = height;
    _autoReport = autoReport;

    xTaskCreate(this->taskServer, "mouse_server", 20000, (void *)this, 5, NULL);
}

void AbsMouse_::taskServer(void *pvParameter)
{
    AbsMouse_ *_this = (AbsMouse_ *)pvParameter;
    NimBLEServer *pServer;
    do
    {
        pServer = NimBLEDevice::getServer();
        if (pServer == nullptr)
        {
            delay(500);
        }
    } while (pServer == nullptr);
    do
    {
        _this->m_deviceInfoService = pServer->getServiceByUUID(NimBLEUUID((uint16_t)0x180a));
        _this->m_hidService = pServer->getServiceByUUID(NimBLEUUID((uint16_t)0x1812));
        if (_this->m_deviceInfoService == nullptr || _this->m_hidService == nullptr)
        {
            delay(500);
        }
    } while (_this->m_deviceInfoService == nullptr || _this->m_hidService == nullptr);

    _this->inputMouse = _this->m_hidService->createCharacteristic((uint16_t)0x2a4d, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC);
    NimBLEDescriptor *inputReportDescriptor = _this->inputMouse->createDescriptor((uint16_t)0x2908, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC);

    uint8_t desc1_val[] = {0x01, 0x01};
    inputReportDescriptor->setValue((uint8_t *)desc1_val, 2);

    uint8_t *HidReportDescriptor = new uint8_t[sizeof(HID_REPORT_DESCRIPTOR)];
    memcpy(HidReportDescriptor, HID_REPORT_DESCRIPTOR, sizeof(HID_REPORT_DESCRIPTOR));

    NimBLECharacteristic *reportChar = nullptr;
    do
    {
        reportChar = _this->m_hidService->getCharacteristic((uint16_t)0x2a4b);
        if (reportChar == nullptr)
        {
            delay(500);
        }
    } while (reportChar == nullptr);

    NimBLEAttValue reporterVal;
    do
    {
        reporterVal = reportChar->getValue();
        if (reporterVal.length() <= 0)
        {
            delay(500);
        }
    } while (reporterVal.length() <= 0);

    NimBLEAttValue newVal = reporterVal.append((uint8_t *)HidReportDescriptor, sizeof(HID_REPORT_DESCRIPTOR));
    reportChar->setValue(newVal.data(), newVal.length());
    vTaskDelete(NULL);
}

void AbsMouse_::report(void)
{
    if (this->inputMouse == nullptr)
    {
        return;
    }
    uint8_t buffer[5];
    buffer[0] = _buttons;
    buffer[1] = _x & 0xFF;
    buffer[2] = (_x >> 8) & 0xFF;
    buffer[3] = _y & 0xFF;
    buffer[4] = (_y >> 8) & 0xFF;
    this->inputMouse->setValue(buffer, sizeof(buffer));
    this->inputMouse->notify();
}

void AbsMouse_::move(uint16_t x, uint16_t y)
{
    _x = (uint16_t)((32767l * ((uint32_t)x)) / _width);
    _y = (uint16_t)((32767l * ((uint32_t)y)) / _height);

    if (_autoReport)
    {
        report();
    }
}

void AbsMouse_::press(uint8_t button)
{
    _buttons |= button;

    if (_autoReport)
    {
        report();
    }
}

void AbsMouse_::release(uint8_t button)
{
    _buttons &= ~button;

    if (_autoReport)
    {
        report();
    }
}

AbsMouse_ AbsMouse;
