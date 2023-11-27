/* ######################
   # ORIGINAL WORK FROM :
   ######################
   @file Samco_2.0_4IR_32u4_BETA.ino
   @brief 10 Button Light Gun sketch for 4 LED setup
   @n INO file for Samco Light Gun 4 LED setup
   @copyright   Samco, https://github.com/samuelballantyne, June 2020
   @copyright   GNU Lesser General Public License
   @copyright   Makoto, https://github.com/makotoworkshop, March 2023
   @copyright   GNU Lesser General Public License
   @author [Sam Ballantyne](samuelballantyne@hotmail.com)
   @author [Makoto](makotoworkshop.org)
   @version  V1.0
   @date  2023
   GNU General Public License v3.0
   https://github.com/samuelballantyne/IR-Light-Gun/blob/master/LICENSE
   https://github.com/makotoworkshop/IRGun_Arduino/blob/master/LICENSE
  ———————————————————————————————————————————————————————————————————————
   ######################
   # MODIFIED By JPZV : https://github.com/JPZV
   ######################
   IRGun_ESP32.ino - 11/2023
   Hack for Pistolet X-SHOT LASER360 ("laser" game toy)
   Work with 4 LEDs IR sources setup (2 up and 2 down to the screen)
   Upload to an ESP32

   TODO LIST:
        - Fix bug: If you activate the reload and fire at the same time, when switching to the Automatic "Machine Gun" firing mode, the gun starts firing alone non-stop
*/

/* Firing mode:
    - 1 hit  -> 1 clack SolenoidPin -> Yellow
    - 2 hits -> 2 clacks -> Green
    - 3 hits -> 3 clacks -> Blue
    - 4 hits -> 4 clacks -> White
    - 5 hits -> 5 clacks -> Cyan
    - 6 hits -> 6 clacks -> Violet
    - Machine Gun -> Automatic clacks -> Fading light
    - 1 hit (maintain) -> Automatic clacks -> Red
*/
/* HOW TO CALIBRATE:
   Step 1: Push Calibration Button
   Step 2: Pull Trigger
   Step 3: Shoot Center of the Screen (try an do this as accurately as possible)
   Step 4: Mouse should lock to vertical axis, use Start and Reload buttons to adjust mouse up/down
   Step 5: Pull Trigger
   Step 6: Mouse should lock to horizontal axis, use Start and Reload buttons to adjust mouse left/right
   Step 7: Pull Trigger to finish
   Step 8: Offset are now saved to EEPROM
*/

#include <Wire.h>
#include <BleGamepad.h>
#include "AbsMouse.h"
#include "DFRobotIRPosition.h"
#include "SamcoBeta.h"
#include <EEPROM.h>

#ifndef ARDUINO_ARCH_ESP32
#error "This project must run on an ESP32."
#endif

int xCenter = 512; // Open serial monitor and update these values to save calibration manually
int yCenter = 450;
float xOffset = 147;
float yOffset = 82;
// CALIBRATION:     Cam Center x/y: 421, 356     Offsets x/y: 121.00, 71.00

//**************************
//* Joystick Variables *
//**************************
BleGamepad Joystick("IRGun ESP32");
BleGamepadConfiguration JoystickConfig;

//******************************
//* Variables for IRCamera *
//******************************
int condition = 1;
int finalX; // Values after tilt correction
int finalY;
int see0; // IR LED N°1 seen by the camera. (otherwise = 0)
int see1; // IR LED N°2 seen by the camera. (otherwise = 0)
int see2; // IR LED N°3 seen by the camera. (otherwise = 0)
int see3; // IR LED N°4 seen by the camera. (otherwise = 0)

//****************************
//* Mouse Variables *
//****************************
int MoveXAxis; // Unconstrained mouse postion
int MoveYAxis;
int conMoveXAxis; // Constrained mouse postion
int conMoveYAxis;
int count = -2; // Set initial count

//***************************
//* Input defines *
//***************************
#define TriggerPin 39               // Label Pin to Joystick buttons
#define StartPin 34                 // Also used for calibration (minus)
#define ReloadPin 36                // Also used for calibration (plus)
#define SwitchAutoReloadPin 35      // Auto reload On/Off
#define ChangeHIDModePin 14         // Changes the HID Mode between Mouse, Joystick and Hibryd

//***************************
//* Output defines *
//***************************
#define LedFirePin 13
#define LedMouseModePin 25
#define LedJoystickModePin 26
#define LedAutoReloadPin 27
#define LedRedPin 16
#define LedGreenPin 17
#define LedBluePin 18
#define SolenoidPin 19
#define CadenceTir 90
#define ToggleLED(x) digitalWrite(x, !digitalRead(x))

//**********************
//* Misc *
//**********************
#define ResolutionAxisX 1023
#define ResolutionAxisY 768
unsigned int shotCount = 0;
bool shouldFire = false;
const int R = 1;
const int G = 1;
const int B = 1;
enum RainbowSequence
{
    RAINBOW_RED,
    RAINBOW_YELLOW,
    RAINBOW_GREEN,
    RAINBOW_CYAN,
    RAINBOW_BLUE,
    RAINBOW_VIOLET,
    RAINBOW_MAX
};
unsigned short currentRainbowSequence;
unsigned int varR = 255;
unsigned int varG = 0;
unsigned int varB = 0;

enum AutomaticModes
{
    AUTOMATIC_1_SHOT,
    AUTOMATIC_2_SHOT,
    AUTOMATIC_3_SHOT,
    AUTOMATIC_4_SHOT,
    AUTOMATIC_5_SHOT,
    AUTOMATIC_6_SHOT,
    AUTOMATIC_MACHINE_GUN,
    AUTOMATIC_SHOT_SUSTAINED,
    AUTOMATIC_MAX
};
unsigned short currentAutomaticMode = AUTOMATIC_1_SHOT;
unsigned int burstCount = 0;

enum HIDModes
{
    HID_MODE_MOUSE,
    HID_MODE_JOYSTICK,
    HID_MODE_HIBRYD,
    HID_MODE_MAX
};
unsigned short currentHIDMode = HID_MODE_MOUSE;

// Buttons states
unsigned int AutoReload = 0;
unsigned int LastAutoReload = 0;
int ButtonPlus = 0;
int LastButtonPlus = 0;
int ButtonMinus = 0;
int LastButtonMinus = 0;
int ButtonValid = 0;
int LastButtonValid = 0;
int ButtonState_Calibration = 0;
int LastButtonState_Calibration = 0;
bool LastButtonState_Trigger = 0;
bool LastButtonState_Reload = 0;
bool LastButtonState_Start = 0;
bool LastButtonState_ChangeHIDMode = 0;

bool plus = 0;
bool minus = 0;

unsigned long lastFlashLEDTime = 0;
unsigned long lastSolenoidTime = 0;
unsigned long lastAutoReloadTime = 0;
unsigned long lastFireTime = 0;

//************************
//* IRCamera Statement *
//************************
DFRobotIRPosition myDFRobotIRPosition;
SamcoBeta mySamco;

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void setup()
{

    myDFRobotIRPosition.begin(); // Start IR Camera

    Serial.begin(9600); // For saving calibration (make sure your serial monitor has the same baud rate)

    JoystickConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
    JoystickConfig.setAutoReport(true);
    JoystickConfig.setButtonCount(3);                // Button Count
    JoystickConfig.setHatSwitchCount(0);             // Hat Switch Count
    JoystickConfig.setWhichAxes(true, true, false,   // X and Y, but no Z Axis
                                false, false, false, // No Rx, Ry, or Rz
                                false, false);       // No rudder or throttle
    JoystickConfig.setAxesMin(0);
    JoystickConfig.setAxesMax(255);
    Joystick.begin(&JoystickConfig);

    loadSettings();

    // Switchs
    pinMode(SwitchAutoReloadPin, INPUT_PULLUP);
    pinMode(ChangeHIDModePin, INPUT_PULLUP);

    // Leds
    pinMode(LedMouseModePin, OUTPUT);
    pinMode(LedJoystickModePin, OUTPUT);
    pinMode(LedAutoReloadPin, OUTPUT);
    pinMode(LedFirePin, OUTPUT);
    pinMode(LedRedPin, OUTPUT);
    pinMode(LedGreenPin, OUTPUT);
    pinMode(LedBluePin, OUTPUT);

    // Mechanics
    pinMode(SolenoidPin, OUTPUT);
    digitalWrite(SolenoidPin, LOW); //  LOW = rest

    // Joystick buttons
    pinMode(TriggerPin, INPUT_PULLUP); // Set pin modes
    pinMode(ReloadPin, INPUT_PULLUP);
    pinMode(StartPin, INPUT_PULLUP);

    AbsMouse.init(ResolutionAxisX, ResolutionAxisY);

    AbsMouse.move((ResolutionAxisX / 2), (ResolutionAxisY / 2)); // Set mouse position to centre of the screen

    delay(500);

    // Turn off the RGB Led
    ledRGB(0, 0, 0);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #############
// # Program #
// #############
void loop()
{
    // ------------------ START/PAUSE MOUSE ----------------------

    if (count > 3)
    {

        skipCalibration();
        mouseCount();
        //printResults();
    }
    // ---------------------- CENTER ---------------------------
    // START of the calibration procedure
    else if (count > 2)
    {
        AbsMouse.move((ResolutionAxisX / 2), (ResolutionAxisY / 2));

        mouseCount();
        getPosition();

        xCenter = finalX;
        yCenter = finalY;

        //printResults();
    }
    // -------------------- OFFSET -------------------------
    else if (count > 1)
    {
        mouseCount();
        AbsMouse.move(conMoveXAxis, conMoveYAxis);
        getPosition();

        MoveYAxis = map(finalY, (yCenter + ((mySamco.H() * (yOffset / 100)) / 2)), (yCenter - ((mySamco.H() * (yOffset / 100)) / 2)), 0, ResolutionAxisY);
        conMoveXAxis = ResolutionAxisX / 2;
        conMoveYAxis = constrain(MoveYAxis, 0, ResolutionAxisY);

        if (plus)
        {
            yOffset = yOffset + 1;
            delay(10);
        }

        if (minus)
        {
            yOffset = yOffset - 1;
            delay(10);
        }

        //printResults();
    }
    else if (count > 0)
    {
        mouseCount();
        AbsMouse.move(conMoveXAxis, conMoveYAxis);
        getPosition();

        MoveXAxis = map(finalX, (xCenter + ((mySamco.H() * (xOffset / 100)) / 2)), (xCenter - ((mySamco.H() * (xOffset / 100)) / 2)), 0, ResolutionAxisX);
        conMoveXAxis = constrain(MoveXAxis, 0, ResolutionAxisX);
        conMoveYAxis = ResolutionAxisY / 2;

        if (plus)
        {
            xOffset = xOffset + 1;
            delay(10);
        }

        if (minus)
        {
            xOffset = xOffset - 1;
            delay(10);
        }

        //printResults();
    }
    else if (count > -1)
    {
        count = count - 1;

        EEPROM.write(0, xCenter - 256);
        EEPROM.write(1, yCenter - 256);
        EEPROM.write(2, xOffset);
        EEPROM.write(3, yOffset);
    }
    // End of the calibration procedure
    // ---------------------- LET'S GO ---------------------------
    // Normal use of the gun
    else
    {
        bool ButtonState_ChangeHIDMode = !digitalRead(ChangeHIDModePin);

        if (ButtonState_ChangeHIDMode && ButtonState_ChangeHIDMode != LastButtonState_ChangeHIDMode)
        {
            currentHIDMode = (currentHIDMode + 1) % HID_MODE_MAX;
            ButtonState_ChangeHIDMode = LastButtonState_ChangeHIDMode;
        }
        else if (ButtonState_ChangeHIDMode != LastButtonState_ChangeHIDMode)
        {
            ButtonState_ChangeHIDMode = LastButtonState_ChangeHIDMode;
        }

        handleHIDLeds();
        handleButtons();
        handleAxis();

        getPosition(); // Retrieving IR camera values

        MoveXAxis = map(finalX, (xCenter + ((mySamco.H() * (xOffset / 100)) / 2)), (xCenter - ((mySamco.H() * (xOffset / 100)) / 2)), 0, ResolutionAxisX);
        MoveYAxis = map(finalY, (yCenter + ((mySamco.H() * (yOffset / 100)) / 2)), (yCenter - ((mySamco.H() * (yOffset / 100)) / 2)), 0, ResolutionAxisY);
        conMoveXAxis = constrain(MoveXAxis, 0, ResolutionAxisX);
        conMoveYAxis = constrain(MoveYAxis, 0, ResolutionAxisY);

        //printResults();
        resetCalibration();
    }
    modeAutomatic();
    ledRGBautomatic();
}

//        -----------------------------------------------       
// --------------------------- METHODS -------------------------
//        -----------------------------------------------       

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ####################
// # IRCamera Methods #
// ####################
// Get tilt adjusted position from IR positioning camera
void getPosition()
{

    myDFRobotIRPosition.requestPosition();
    if (myDFRobotIRPosition.available())
    {
        mySamco.begin(myDFRobotIRPosition.readX(0), myDFRobotIRPosition.readY(0), myDFRobotIRPosition.readX(1), myDFRobotIRPosition.readY(1), myDFRobotIRPosition.readX(2), myDFRobotIRPosition.readY(2), myDFRobotIRPosition.readX(3), myDFRobotIRPosition.readY(3), xCenter, yCenter);
        finalX = mySamco.X();
        finalY = mySamco.Y();
        see0 = mySamco.testSee(0);
        see1 = mySamco.testSee(1);
        see2 = mySamco.testSee(2);
        see3 = mySamco.testSee(3);
        //Serial.print("finalX: ");
        //Serial.print(finalX);
        //Serial.print(",  finalY: ");
        //Serial.print(finalY);
    }
    else
    {
        Serial.println("Device not available!");
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ##################################
// # Joystick movements  #
// ##################################
void joystickStick()
{
    int xAxis = map(conMoveXAxis, 0, 1023, 0, 255); // Reverse axis required
    int yAxis = map(conMoveYAxis, 0, 768, 0, 255);

    //Serial.print("Joystick xAxis = ");
    //Serial.print(xAxis);
    //Serial.print(",  yAxis = ");
    //Serial.println(yAxis);

    Joystick.setX(xAxis);
    Joystick.setY(yAxis);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ####################
// # LEDs Methods #
// ####################

//  http://www.mon-club-elec.fr/pmwiki_mon_club_elec/pmwiki.php?n=MAIN.ArduinoInitiationLedsDiversTestLedRVBac
//---- function to set the LED color (Binary) ----
void ledRGB(bool Red, bool Green, bool Blue)
{
    if (Red)
        digitalWrite(LedRedPin, LOW); // Turn on Red
    else
        digitalWrite(LedRedPin, HIGH); // Turn off Red

    if (Green)
        digitalWrite(LedGreenPin, LOW); // Turn on Green
    else
        digitalWrite(LedGreenPin, HIGH); // Turn off Green

    if (Blue)
        digitalWrite(LedBluePin, LOW); // Turn on Blue
    else
        digitalWrite(LedBluePin, HIGH); // Turn off Blue
}

//---- Function to set the LED color (progressive) ----
//Each values are from 0 to 255, where 0 is 0% and 255 is 100%
void ledRGBProgressive(unsigned short pwmRed, unsigned short pwmGreen, unsigned short pwmBlue)
{
    analogWrite(LedRedPin, 255 - pwmRed);
    analogWrite(LedGreenPin, 255 - pwmGreen);
    analogWrite(LedBluePin, 255 - pwmBlue);
}

//---- Rainbow fade function ----
//Sequence:
//- Red
//- Yellow
//- Green
//- Cyan
//- Blue
//- Violet
void rainbowFade()
{
    ledRGBProgressive(varR, varG, varB); // Set the color in the LED

    switch (currentRainbowSequence)
    {
        default:
        case RAINBOW_RED:
            varG++;
            if (varG == 255)
                currentRainbowSequence = RAINBOW_YELLOW;
            break;
        case RAINBOW_YELLOW:
            varR--;
            if (varR == 0)
                currentRainbowSequence = RAINBOW_GREEN;
            break;
        case RAINBOW_GREEN:
            varB++;
            if (varB == 255)
                currentRainbowSequence = RAINBOW_CYAN;
            break;
        case RAINBOW_CYAN:
            varG--;
            if (varG == 0)
                currentRainbowSequence = RAINBOW_BLUE;
            break;
        case RAINBOW_BLUE:
            varR++;
            if (varR == 255)
                currentRainbowSequence = RAINBOW_VIOLET;
            break;
        case RAINBOW_VIOLET:
            varB--;
            if (varB == 0)
                currentRainbowSequence = RAINBOW_RED;
            break;
    }

    //Serial.print("");
    //Serial.print("varR : ");
    //Serial.print(varR);
    //Serial.print(" | varG : ");
    //Serial.print(varG);
    //Serial.print(" | varB : ");
    //Serial.println(varB);
    //delay(4);
}

void ledRGBautomatic()
{
    switch (currentAutomaticMode)
    {
        default:
        case AUTOMATIC_1_SHOT:
            ledRGB(R, G, 0); // Yellow
            break;
        case AUTOMATIC_2_SHOT:
            ledRGB(0, G, 0); // Green
            break;
        case AUTOMATIC_3_SHOT:
            ledRGB(0, 0, B); // Blue
            break;
        case AUTOMATIC_4_SHOT:
            ledRGB(R, G, B); // White
            break;
        case AUTOMATIC_5_SHOT:
            ledRGB(0, G, B); // Cyan
            break;
        case AUTOMATIC_6_SHOT:
            ledRGB(R, 0, B); // Violet
            break;
        case AUTOMATIC_MACHINE_GUN:
            rainbowFade(); // Do the Rainbow effect
            break;
        case AUTOMATIC_SHOT_SUSTAINED:
            ledRGB(R, 0, 0); // Red
            break;
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #####################################
// # Opto-mechanical event management #
// #####################################

// #############################
// # Handling buttons #
// #############################
// Turn on/off the HID mode Leds
void handleHIDLeds()
{
    switch (currentHIDMode)
    {
        case HID_MODE_MOUSE:
            digitalWrite(LedMouseModePin, HIGH);
            digitalWrite(LedJoystickModePin, LOW);
            break;
        case HID_MODE_JOYSTICK:
            digitalWrite(LedMouseModePin, LOW);
            digitalWrite(LedJoystickModePin, HIGH);
            break;
        default:
        case HID_MODE_HIBRYD:
            digitalWrite(LedMouseModePin, HIGH);
            digitalWrite(LedJoystickModePin, HIGH);
            break;
    }
}

void flashLED()
{
    unsigned long currentTime = millis();
    if (currentTime - lastFlashLEDTime >= 90)
    {
        lastFlashLEDTime = currentTime;
        digitalWrite(LedFirePin, HIGH);
    }
    else
    {
        digitalWrite(LedFirePin, LOW); // turn the LED off by making the voltage LOW
    }
}

void fireSolenoid()
{
    unsigned long currentTime = millis();
    if (currentTime - lastSolenoidTime >= CadenceTir)
    {
        lastSolenoidTime = currentTime;
        // Unlike the LED, it takes a little time for the mechanism to change state
        unsigned long startTime = millis();
        // hold it by 25 µs so the mechanism has time to change state
        while (millis() < startTime + 25)
        {
            digitalWrite(SolenoidPin, HIGH); // Hold
        }
    }
    else
    {
        digitalWrite(SolenoidPin, LOW); // Free
    }
    delay(15); // breathing delays (otherwise the rate of fire does not pass!)
    digitalWrite(SolenoidPin, LOW);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #######################
// # Firing mode #
// #######################

//Firing mode selection:
//- 1 shot
//- 2 shots
//- 3 shots
//- 4 shots
//- 5 shots
//- 6 shots
//- MachineGun
//- 1 shot sustained
void modeAutomatic()
{ 
    int ButtonState_Reload = digitalRead(ReloadPin);
    AutoReload = digitalRead(SwitchAutoReloadPin);
    //If Reload mode is activated with the Reload switch
    if (AutoReload == LOW)
    {
        ledRGBautomatic();
        if (ButtonState_Reload == LOW)
        {
            unsigned long currentTime = millis();
            if (currentTime - lastAutoReloadTime >= 300)
            {
                lastAutoReloadTime = currentTime;
                currentAutomaticMode = (currentAutomaticMode + 1) % AUTOMATIC_MAX;
            }
        }
    }
}

void shotBurst()
{
    if (shouldFire)
    {
        unsigned long currentTime = millis();
        if (currentTime - lastFireTime >= CadenceTir)
        {
            lastFireTime = currentTime;
            switch (currentHIDMode)
            {
                case HID_MODE_MOUSE:
                    setMouseButton(MOUSE_LEFT, true);
                    break;
                case HID_MODE_JOYSTICK:
                    setJoystickButton(0, true);
                    break;
                default:
                case HID_MODE_HIBRYD:
                    setMouseButton(MOUSE_LEFT, true);
                    setJoystickButton(0, true);
                    break;
            }
            digitalWrite(LedFirePin, HIGH);
            fireSolenoid();
            burstCount++;
            if (burstCount == shotCount)
            {
                shouldFire = false;
                burstCount = 0;
            }
        }
        else
        {
            setMouseButton(MOUSE_LEFT, false);
            setJoystickButton(0, false);
            digitalWrite(LedFirePin, LOW);
        }
    }
    else
    {
        // If fire differs from 1, turn off the Led and don't fire the gun
        setMouseButton(MOUSE_LEFT, false);
        setJoystickButton(0, false);
        digitalWrite(LedFirePin, LOW);
    }
    delay(10);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #############################
// # Handling buttons #
// #############################
// Check the buttons and send it to HID Device
void handleButtons()
{
    bool ButtonState_Trigger    = !digitalRead(TriggerPin);
    bool ButtonState_Reload     = !digitalRead(ReloadPin);
    bool ButtonState_Start      = !digitalRead(StartPin);

    switch (currentAutomaticMode)
    {
        default:
        case AUTOMATIC_1_SHOT:
        case AUTOMATIC_2_SHOT:
        case AUTOMATIC_3_SHOT:
        case AUTOMATIC_4_SHOT:
        case AUTOMATIC_5_SHOT:
        case AUTOMATIC_6_SHOT:
            if (ButtonState_Trigger != LastButtonState_Trigger)
            {
                if (ButtonState_Trigger)
                {
                    shouldFire = true;
                }
                LastButtonState_Trigger = ButtonState_Trigger;
            }
            shotCount = currentAutomaticMode + 1;
            shotBurst();
            break;
        case AUTOMATIC_MACHINE_GUN:
            if (ButtonState_Trigger)
            {
                shouldFire = true;
            }
            shotCount = 1;
            shotBurst();
            break;
        case AUTOMATIC_SHOT_SUSTAINED:
            // Button pressed and held for certain games (Alien, T2, etc...)
            if (ButtonState_Trigger)
            {
                switch (currentHIDMode)
                {
                    case HID_MODE_MOUSE:
                        setMouseButton(MOUSE_LEFT, true);
                        break;
                    case HID_MODE_JOYSTICK:
                        setJoystickButton(0, true);
                        break;
                    default:
                    case HID_MODE_HIBRYD:
                        setMouseButton(MOUSE_LEFT, true);
                        setJoystickButton(0, true);
                        break;
                }
                flashLED();
                fireSolenoid();
            }
            else
            {
                setMouseButton(MOUSE_LEFT, false);
                setJoystickButton(0, false);
                digitalWrite(LedFirePin, LOW);
            }

            delay(10);
            break;
    }

    // Reload button
    if (ButtonState_Reload != LastButtonState_Reload)
    {
        if (ButtonState_Reload)
        {
            switch (currentHIDMode)
            {
                case HID_MODE_MOUSE:
                    setMouseButton(MOUSE_RIGHT, true);
                    break;
                case HID_MODE_JOYSTICK:
                    setJoystickButton(1, true);
                    break;
                default:
                case HID_MODE_HIBRYD:
                    setMouseButton(MOUSE_RIGHT, true);
                    setJoystickButton(1, true);
                    break;
            }
        }
        else
        {
            setMouseButton(MOUSE_RIGHT, false);
            setJoystickButton(1, false);
        }

        delay(10);
        LastButtonState_Reload = ButtonState_Reload;
    }

    // Start button
    if (ButtonState_Start != LastButtonState_Start)
    {
        if (ButtonState_Start)
        {
            switch (currentHIDMode)
            {
                case HID_MODE_MOUSE:
                    setMouseButton(MOUSE_MIDDLE, true);
                    break;
                case HID_MODE_JOYSTICK:
                    setJoystickButton(2, true);
                    break;
                default:
                case HID_MODE_HIBRYD:
                    setMouseButton(MOUSE_MIDDLE, true);
                    setJoystickButton(2, true);
                    break;
            }
        }
        else
        {
            setMouseButton(MOUSE_MIDDLE, false);
            setJoystickButton(2, false);
        }

        delay(10);
        LastButtonState_Start = ButtonState_Start;
    }

    // Automatic reload mode if the cursor leaves the screen, managed by software
    AutoReload = digitalRead(SwitchAutoReloadPin);
    // If Reload mode is activated with the Reload switch
    if (AutoReload == LOW)
    {
        //Turn on the AutoReload Led
        digitalWrite(LedAutoReloadPin, HIGH);

        // If the gun leaves the screen
        if (see0 == 0 || see1 == 0 || see2 == 0 || see3 == 0)
        {
            switch (currentHIDMode)
            {
                case HID_MODE_MOUSE:
                    setMouseButton(MOUSE_RIGHT, true);
                    break;
                case HID_MODE_JOYSTICK:
                    setJoystickButton(1, true);
                    break;
                default:
                case HID_MODE_HIBRYD:
                    setMouseButton(MOUSE_RIGHT, true);
                    setJoystickButton(1, true);
                    break;
            }
        }
        else
        {
            setMouseButton(MOUSE_RIGHT, false);
            setJoystickButton(1, false);
        }
        
        delay(10);
    }
    else
    {
        // Serial.println("Mode Reload disabled");
        digitalWrite(LedAutoReloadPin, LOW); // Turn off the AutoReload Led
        setMouseButton(MOUSE_RIGHT, false);
        setJoystickButton(1, false);
    }
    delay(10);
}

void setMouseButton(uint8_t button, bool value)
{
    if (value)
    {
        AbsMouse.press(button);
    }
    else
    {
        AbsMouse.release(button);
    }
}

void setJoystickButton(uint8_t button, bool value)
{
    if (value)
    {
        Joystick.press(button);
    }
    else
    {
        Joystick.release(button);
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #############################
// # Handling axis/movements #
// #############################
// Check the movement and send it to HID Device
void handleAxis()
{
    switch (currentHIDMode)
    {
        default:
        case HID_MODE_MOUSE:
        case HID_MODE_HIBRYD:
            AbsMouse.move(conMoveXAxis, conMoveYAxis);
            //Center the Joystick
            Joystick.setX(127);
            Joystick.setY(127);
            break;
        case HID_MODE_JOYSTICK:
            joystickStick();
            break;
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ###################################
// # Calibration methods #
// ###################################

// Setup Start Calibration Button
void startCalibration()
{
    // ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin));
    ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin) | digitalRead(ReloadPin));
    if (ButtonState_Calibration != LastButtonState_Calibration)
    {
        if (ButtonState_Calibration == LOW)
        {
            count--;
        }
        delay(50);
    }
    LastButtonState_Calibration = ButtonState_Calibration;
}

// Calibration procedure
void mouseCount()
{
    ButtonValid = digitalRead(TriggerPin);
    ButtonPlus = digitalRead(StartPin);
    ButtonMinus = digitalRead(ReloadPin);

    // Validation
    if (ButtonValid != LastButtonValid)
    {
        if (ButtonValid == LOW)
        {
            count--;
        }
        delay(10);
    }

    // Tuning Up
    if (ButtonPlus != LastButtonPlus)
    {
        plus = !ButtonPlus;
        delay(10);
    }

    // Tuning down
    if (ButtonMinus != LastButtonMinus)
    {
        plus = !ButtonMinus;
        delay(10);
    }

    LastButtonValid = ButtonValid;
    LastButtonPlus = ButtonPlus;
    LastButtonMinus = ButtonMinus;
}

// Pause/Re-calibrate button
void resetCalibration()
{
    //ButtonState_Calibration = (!digitalRead(StartPin) | !digitalRead(TriggerPin));
    ButtonState_Calibration = (!digitalRead(StartPin) | !digitalRead(TriggerPin) | !digitalRead(ReloadPin));
    if (ButtonState_Calibration != LastButtonState_Calibration)
    {
        if (ButtonState_Calibration)
        {
            count = 4;
            delay(50);
        }
        delay(50);
    }
    LastButtonState_Calibration = ButtonState_Calibration;
}

// Unpause button
void skipCalibration()
{
    // ButtonState_Calibration = (!digitalRead(StartPin) | !digitalRead(TriggerPin));
    ButtonState_Calibration = (!digitalRead(StartPin) | !digitalRead(TriggerPin) | !digitalRead(ReloadPin));
    if (ButtonState_Calibration != LastButtonState_Calibration)
    {
        if (ButtonState_Calibration)
        {
            count = 0;
            delay(50);
        }
        delay(50);
    }
    LastButtonState_Calibration = ButtonState_Calibration;
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ##########################
// # EEPROM Memory management #
// ##########################

// Reload calibration values from the EEPROM memory
void loadSettings()
{
    if (EEPROM.read(1023) == 'T')
    {
        // Settings have been initialized, read them
        xCenter = EEPROM.read(0) + 256;
        yCenter = EEPROM.read(1) + 256;
        xOffset = EEPROM.read(2);
        yOffset = EEPROM.read(3);
    }
    else
    {
        // First time run, settings were never set
        EEPROM.write(0, xCenter - 256);
        EEPROM.write(1, yCenter - 256);
        EEPROM.write(2, xOffset);
        EEPROM.write(3, yOffset);
        EEPROM.write(1023, 'T');
    }
}

// Print results for saving calibration
void printResults()
{
    Serial.print("CALIBRATION:");
    Serial.print("\tCam Center x/y: ");
    Serial.print(xCenter);
    Serial.print(", ");
    Serial.print(yCenter);
    Serial.print("\tOffsets x/y: ");
    Serial.print(xOffset);
    Serial.print(", ");
    Serial.println(yOffset);
}
