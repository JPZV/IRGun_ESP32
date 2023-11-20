# IR « Light » Gun with ESP32
Project forked from [IRGun_Arduino](https://github.com/makotoworkshop/IRGun_Arduino) by [makotoworkshop](https://github.com/makotoworkshop)

## Description

Following the great work from both Makoto and [Sam Ballantyne](https://github.com/samuelballantyne/IR-Light-Gun), I wanted to convert the IRGun_Arduino project for ESP32, so the IRGun could be connected by Bluetooth instead of USB, but leaving anything else just as it was.

## Building and flashing

Aside of the same libraries used by IRGun_Arduino, you'll need to install [Arduino-ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html), [ESP32-BLE-Gamepad@0.5.3](https://github.com/lemmingDev/ESP32-BLE-Gamepad/releases/tag/v0.5.3) and [NimBLE@1.4.1](https://github.com/h2zero/NimBLE-Arduino/releases/tag/1.4.1) on your Arduino Library folder (NimBle can be installed from Arduino Library Manager, just check that you're installing 1.4.1, as the code was made using that version).

Next just clone the project (either by using Git/GitHub Desktop, or by clicking on "<>Code -> Download ZIP" at the top), and finally build it and upload it with Arduino IDE (or VSCode if you want)


## Pinout

As obvious it is, the pinout has changed between the Arduino version and the ESP32 one, so below you have a table comparing the pinout from Arduino and from ESP32

|Pin Name|Arduino pin|ESP32 GPIO|
|---|---|---|
|Trigger|14|GPIO39|
|Start|15|GPIO34|
|Reload|16|GPIO36|
|Auto Reload Switch|4|GPIO35|
|Disable Mouse Switch|7|GPIO14|
|Disable Joystick Switch|8|GPIO12|
|Fire Led|A0|GPIO13|
|Disabled Mouse Led|A1|GPIO25|
|Disabled Joystick Led|A2|GPIO26|
|Auto Reload Led|A3|GPIO27|
|Red Led|9|GPIO16|
|Green Led|6|GPIO17|
|Blue Led|5|GPIO18|
|Solenoid|10|GPIO19|
|I²C SDA|2|GPIO21|
|I²C SCL|3|GPIO22|

### Hardware

Keep in mind that any file from the Kicad directory is not yet updated to work with ESP32, so don't use them without modifying them first and double checking that the pinout is correct.

## Libraries used

This project uses the following libraries (those marked as *Integrated* means that the library is inside this project's code, so no need to install them manually)

- [DFRobotIRPosition](https://github.com/DFRobot/DFRobotIRPosition) by [DFRobo](https://github.com/DFRobot) ([License](https://www.gnu.org/licenses/lgpl-3.0.en.html)) (Integrated)
- [ABSMouse](https://github.com/jonathanedgecombe/absmouse) by [Jonathan Edgecombe](https://github.com/jonathanedgecombe) ([License](https://github.com/jonathanedgecombe/absmouse/blob/master/LICENSE)) (Integrated) (Modified)
- [Arduino-ESP32](https://github.com/espressif/arduino-esp32) by [espressif](https://github.com/espressif) ([License](https://github.com/espressif/arduino-esp32/blob/master/LICENSE.md))
- [ESP32-BLE-Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad) by [lemmingDev](https://github.com/lemmingDev) ([License](https://github.com/lemmingDev/ESP32-BLE-Gamepad/blob/master/license.txt))
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) by [h2zero](https://github.com/h2zero) ([License](https://github.com/h2zero/NimBLE-Arduino/blob/release/1.4/LICENSE))

## License

Because this project derivate from IRGun_Arduino, which also derivate from [IR-Light-Gun](https://github.com/samuelballantyne/IR-Light-Gun), the license being used is [GNU General Public License v3.0](https://github.com/JPZV/IRGun_ESP32/blob/master/LICENSE), which in summary means that any project derived by this one must be licensed by the same license (i.e. GPL V3.0), and its source code must be opened as well.