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

int xCenter = 512; // Open serial monitor and update these values to save calibration manualy
int yCenter = 450;
float xOffset = 147;
float yOffset = 82;
// CALIBRATION:     Cam Center x/y: 421, 356     Offsets x/y: 121.00, 71.00

//**************************
//* Joystick Variables *
//**************************
BleGamepad Joystick("IRGun ESP32");
BleGamepadConfiguration JoystickConfig;
const int pinToButtonMap = 14;      // Constant that maps the physical pin to joystick button. (pins 14-15-16)
int LastButtonState[3] = {0, 0, 0}; // Last state of the 3 buttons

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
int count = -2; // Set intial count

//***************************
//* Input defines *
//***************************
#define TriggerPin 39               // Label Pin to Joystick buttons
#define StartPin 34                 // Also used for calibration (minus)
#define ReloadPin 36                // Also used for calibration (plus)
#define SwitchAutoReloadPin 35      // Auto reload On/Off
#define SwitchSuspendMousePin 14    // (Active by default if not wired) Disables the Mouse (the HID is present, but the data is no longer sent to the mouse)
#define SwitchSuspendJoystickPin 12 // (Active by default if not wired) Disables the joystick. Note: Could Fail in ESP32?

//***************************
//* Output defines *
//***************************
#define LedFirePin 13
#define LedSuspendMousePin 25
#define LedSuspendJoystickPin 26
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
unsigned int NBdeTirs = 0;
unsigned int fire = 0;
const int R = 1;
const int G = 1;
const int B = 1;
unsigned int varR = 255;
unsigned int varG = 0;
unsigned int varB = 0;
unsigned int SequenceRed = 1;
unsigned int SequenceYellow = 0;
unsigned int SequenceGreen = 0;
unsigned int SequenceCyan = 0;
unsigned int SequenceBlue = 0;
unsigned int SequenceViolet = 0;
unsigned int Automatic = 1;
unsigned int burstCount = 0;

// Buttons states
unsigned int SuspendMouse = 0;
unsigned int SuspendJoystick = 0;
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
// int ButtonState_Trigger = 0;
int LastButtonState_Trigger = 0;
// int ButtonState_Reload = 0;
int LastButtonState_Reload = 0;
// int ButtonState_Start = 0;
int LastButtonState_Start = 0;

int plus = 0;
int minus = 0;

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
    pinMode(SwitchSuspendMousePin, INPUT_PULLUP);
    pinMode(SwitchSuspendJoystickPin, INPUT_PULLUP);

    // Leds
    pinMode(LedSuspendMousePin, OUTPUT);
    pinMode(LedSuspendJoystickPin, OUTPUT);
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
    ledRVB(0, 0, 0);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #############
// # Program #
// #############
void loop()
{

    /* ------------------ START/PAUSE MOUSE ---------------------- */

    if (count > 3)
    {

        skip();
        mouseCount();
        //  PrintResults();
    }

    /* ---------------------- CENTER --------------------------- */
    // START of the calibration procedure

    else if (count > 2)
    {

        AbsMouse.move((ResolutionAxisX / 2), (ResolutionAxisY / 2));

        mouseCount();
        getPosition();

        xCenter = finalX;
        yCenter = finalY;

        //  PrintResults();
    }

    /* -------------------- OFFSET ------------------------- */

    else if (count > 1)
    {

        mouseCount();
        AbsMouse.move(conMoveXAxis, conMoveYAxis);
        getPosition();

        MoveYAxis = map(finalY, (yCenter + ((mySamco.H() * (yOffset / 100)) / 2)), (yCenter - ((mySamco.H() * (yOffset / 100)) / 2)), 0, ResolutionAxisY);
        conMoveXAxis = ResolutionAxisX / 2;
        conMoveYAxis = constrain(MoveYAxis, 0, ResolutionAxisY);

        if (plus == 1)
        {
            yOffset = yOffset + 1;
            delay(10);
        }
        else
        {
        }

        if (minus == 1)
        {
            yOffset = yOffset - 1;
            delay(10);
        }
        else
        {
        }

        //   PrintResults();
    }

    else if (count > 0)
    {

        mouseCount();
        AbsMouse.move(conMoveXAxis, conMoveYAxis);
        getPosition();

        MoveXAxis = map(finalX, (xCenter + ((mySamco.H() * (xOffset / 100)) / 2)), (xCenter - ((mySamco.H() * (xOffset / 100)) / 2)), 0, ResolutionAxisX);
        conMoveXAxis = constrain(MoveXAxis, 0, ResolutionAxisX);
        conMoveYAxis = ResolutionAxisY / 2;

        if (plus == 1)
        {
            xOffset = xOffset + 1;
            delay(10);
        }
        else
        {
        }

        if (minus == 1)
        {
            xOffset = xOffset - 1;
            delay(10);
        }
        else
        {
        }

        //  PrintResults();
    }

    else if (count > -1)
    {

        count = count - 1;

        EEPROM.write(0, xCenter - 256);
        EEPROM.write(1, yCenter - 256);
        EEPROM.write(2, xOffset);
        EEPROM.write(3, yOffset);
    }
    // EnND of the calibration procedure
    /* ---------------------- LET'S GO --------------------------- */
    // Normal use of the gun

    else
    {

        SuspendMouse = digitalRead(SwitchSuspendMousePin);
        // Enabling mouse movements (if LOW then the mouse is disabled by the Switch)
        if ((SuspendMouse == HIGH) && (SuspendJoystick == LOW))
        {
            //Serial.println("Mouse Enabled");
            digitalWrite(LedSuspendMousePin, HIGH);     // Turn on the Led
            AbsMouse.move(conMoveXAxis, conMoveYAxis);  // Movements sent to HID
            mouseButtons();                             // Clicks sent to HID
                                                        //    Serial.print("conMoveXAxis :");
                                                        //    Serial.print(conMoveXAxis);
                                                        //    Serial.print(",  conMoveYAxis :");
                                                        //    Serial.println(conMoveYAxis);
        }
        // Completely disabling the mouse
        else
        {
            //Serial.println("Mouse Disabled");
            //Serial.println(" ");
            digitalWrite(LedSuspendMousePin, LOW); // Turn off the Led
        }

        SuspendJoystick = digitalRead(SwitchSuspendJoystickPin);
        // Enabling joystick movements (if LOW then the joystick is disabled by the Switch)
        if ((SuspendMouse == LOW) && (SuspendJoystick == HIGH))
        {
            //Serial.println("Joystick enabled");
            digitalWrite(LedSuspendJoystickPin, HIGH);  // Turn on the Led
            Joystick_device_Buttons();                  // Buttons sent to HID
            Joystick_device_Stick();                    // Movements sent to HID
        }
        // Completely disabling the joystick
        else
        {
            //Serial.println("Joystick disabled");
            //Serial.println(" ");
            digitalWrite(LedSuspendJoystickPin, LOW);   // Turn off the Led
            Joystick.setX(127);                         // centering de axis
            Joystick.setY(127);
        }

        // Enabling Hybrid mode (Mouse movement only) and (mouse button + joystick button)
        if ((SuspendMouse == HIGH) && (SuspendJoystick == HIGH))
        {
            //Serial.println("Hybrid mode enabled");
            digitalWrite(LedSuspendMousePin, HIGH);     // Turn on the Led
            digitalWrite(LedSuspendJoystickPin, HIGH);  // Turn on the Led
            AbsMouse.move(conMoveXAxis, conMoveYAxis);  // Movements sent to HID
            Hybride_Buttons();                          // Buttons sent to HID
        }

        getPosition(); // Retrieving IR camera values

        MoveXAxis = map(finalX, (xCenter + ((mySamco.H() * (xOffset / 100)) / 2)), (xCenter - ((mySamco.H() * (xOffset / 100)) / 2)), 0, ResolutionAxisX);
        MoveYAxis = map(finalY, (yCenter + ((mySamco.H() * (yOffset / 100)) / 2)), (yCenter - ((mySamco.H() * (yOffset / 100)) / 2)), 0, ResolutionAxisY);
        conMoveXAxis = constrain(MoveXAxis, 0, ResolutionAxisX);
        conMoveYAxis = constrain(MoveYAxis, 0, ResolutionAxisY);

        //PrintResults();
        reset();
    }
    ModeAutomatic();
    LedRGBautomatic();
}

/*        -----------------------------------------------        */
/* --------------------------- METHODS ------------------------- */
/*        -----------------------------------------------        */

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ####################
// # Gestion IRCamera #
// ####################
// Get tilt adjusted position from IR postioning camera
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
        //    Serial.print("finalX: ");
        //    Serial.print(finalX);
        //    Serial.print(",  finalY: ");
        //    Serial.print(finalY);
    }
    else
    {
        Serial.println("Device not available!");
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ##################################
// # Gestion mouvements de Joystick #
// ##################################
void Joystick_device_Stick()
{
    int xAxis = map(conMoveXAxis, 0, 1023, 0, 255); // Avec inversion des axes requise
    int yAxis = map(conMoveYAxis, 0, 768, 0, 255);

    //  Serial.print("Joystick xAxis = ");
    //  Serial.print(xAxis);
    //  Serial.print(",  yAxis = ");
    //  Serial.println(yAxis);

    Joystick.setX(xAxis);
    Joystick.setY(yAxis);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ####################
// # Gestion LEDs RGB #
// ####################
//  http://www.mon-club-elec.fr/pmwiki_mon_club_elec/pmwiki.php?n=MAIN.ArduinoInitiationLedsDiversTestLedRVBac
//---- fonction pour combiner couleurs ON/OFF ----
void ledRVB(int Red, int Green, int Blue)
{
    if (Red == 1)
        digitalWrite(LedRedPin, LOW); // allume couleur
    if (Red == 0)
        digitalWrite(LedRedPin, HIGH); // éteint couleur

    if (Green == 1)
        digitalWrite(LedGreenPin, LOW); // allume couleur
    if (Green == 0)
        digitalWrite(LedGreenPin, HIGH); // éteint couleur

    if (Blue == 1)
        digitalWrite(LedBluePin, LOW); // allume couleur
    if (Blue == 0)
        digitalWrite(LedBluePin, HIGH); // éteint couleur
}

//---- fonction pour variation progressive des couleurs ----
void ledRVBpwm(int pwmRed, int pwmGreen, int pwmBlue)
{                                          // reçoit valeur 0-255 par couleur
    analogWrite(LedRedPin, 255 - pwmRed); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
    analogWrite(LedGreenPin, 255 - pwmGreen);  // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
    analogWrite(LedBluePin, 255 - pwmBlue);   // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
}

//---- fonction pour fondu arc-en-ciel ----
void FonduCouleurs()
{
    //  Séquence :
    //  255,0,0 red
    //  255,255,0 yellow
    //  0,255,0 green
    //  0,255,255 cyan
    //  0,0,255 blue
    //  255,0,255 violet
    //  On démarre en red avec varR,varG,varB = 255,0,0

    ledRVBpwm(varR, varG, varB); // génère impulsion largeur voulue pour la couleur

    if (SequenceRed == 1)
    {           // 255,0,0 Red
        varG++; // augmente le Green
        if (varG == 255)
        { // on obtient Yellow (255,255,0)
            SequenceRed = 0;
            SequenceYellow = 1; // On active la prochaine séquence
            SequenceGreen = 0;
            SequenceCyan = 0;
            SequenceBlue = 0;
            SequenceViolet = 0;
        }
    }

    if (SequenceYellow == 1)
    {           // 255,255,0 Yellow
        varR--; // on baisse le Red
        if (varR == 0)
        { // ne reste que le Green  (0,255,0)
            SequenceRed = 0;
            SequenceYellow = 0;
            SequenceGreen = 1; // On active la prochaine séquence
            SequenceCyan = 0;
            SequenceBlue = 0;
            SequenceViolet = 0;
        }
    }

    if (SequenceGreen == 1)
    {           // 0,255,0 Green
        varB++; // augmente le blue
        if (varB == 255)
        { // on obtient le Cyan  (0,255,255)
            SequenceRed = 0;
            SequenceYellow = 0;
            SequenceGreen = 0;
            SequenceCyan = 1; // On active la prochaine séquence
            SequenceBlue = 0;
            SequenceViolet = 0;
        }
    }

    if (SequenceCyan == 1)
    {           // 0,255,255 Cyan
        varG--; // on baisse le green
        if (varG == 0)
        { // on obtient de blue (0,0,255)
            SequenceRed = 0;
            SequenceYellow = 0;
            SequenceGreen = 0;
            SequenceCyan = 0;
            SequenceBlue = 1; // On active la prochaine séquence
            SequenceViolet = 0;
        }
    }

    if (SequenceBlue == 1)
    {           // 0,0,255 Blue
        varR++; // augmenter le Red
        if (varR == 255)
        { // on obtient le Violet (255,0,255)
            SequenceRed = 0;
            SequenceYellow = 0;
            SequenceGreen = 0;
            SequenceCyan = 0;
            SequenceBlue = 0;
            SequenceViolet = 1; // On active la prochaine séquence
        }
    }

    if (SequenceViolet == 1)
    {           // 255,0,255 Violet
        varB--; // on baisse le Blue
        if (varB == 0)
        { // on obtient le Red (255,0,0)
            Serial.print("SequenceRed");
            SequenceRed = 1; // On active la prochaine séquence
            SequenceYellow = 0;
            SequenceGreen = 0;
            SequenceCyan = 0;
            SequenceBlue = 0;
            SequenceViolet = 0;
        }
    }
    //  Serial.print("");
    //  Serial.print("varR : ");
    //  Serial.print(varR);
    //  Serial.print(" | varG : ");
    //  Serial.print(varG);
    //  Serial.print(" | varB : ");
    //  Serial.println(varB);
    //  delay(4);
}

void LedRGBautomatic()
{
    switch (Automatic)
    {
    case 1:
        ledRVB(R, G, 0); // yellow
        break;
    case 2:
        ledRVB(0, G, 0); // green
        break;
    case 3:
        ledRVB(0, 0, B); // blue
        break;
    case 4:
        ledRVB(R, G, B); // blanc
        break;
    case 5:
        ledRVB(0, G, B); // cyan
        break;
    case 6:
        ledRVB(R, 0, B); // violet
        break;
    case 7:
        FonduCouleurs(); // Fondu de couleur red-orange-yellow-green-cyan-blue-violet-rose
        break;
    case 8:
        ledRVB(R, 0, 0); // red
        break;
    default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #####################################
// # Gestion événements opto-mécanique #
// #####################################

void clignotteLED()
{
    static unsigned long initTime = 0;
    unsigned long maintenant = millis();
    if (maintenant - initTime >= 90)
    {
        initTime = maintenant;
        digitalWrite(LedFirePin, HIGH);
    }
    else
    {
        digitalWrite(LedFirePin, LOW); // turn the LED off by making the voltage LOW
    }
}

void ActionneSolenoid()
{
    static unsigned long initTime = 0;
    unsigned long maintenant = millis();
    if (maintenant - initTime >= CadenceTir)
    {
        initTime = maintenant;
        // Contrairement à la Led, il faut un peu de temps au mécanisme pour changer d'état
        unsigned long startTime = millis();
        unsigned long duree = 25; // maintient 25 µs pour que le mécanisme ait le temps de changer d'état
        while (millis() < startTime + duree)
        {
            digitalWrite(SolenoidPin, HIGH); // collé
        }
    }
    else
    {
        digitalWrite(SolenoidPin, LOW); // repos
    }
    delay(15); // délais de respiration (sinon la cadence de tir ne passe pas !)
    digitalWrite(SolenoidPin, LOW);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #######################
// # Gestion Mode de tir #
// #######################
// Sélection du mode de tir : 1 coup, 2 coups, 3 coups, 4 coups, 5 coups, 6 coups, MachineGun, 1 coup maintenu
void ModeAutomatic()
{ 
    int ButtonState_Reload = digitalRead(ReloadPin);
    AutoReload = digitalRead(SwitchAutoReloadPin);
    if (AutoReload == 0) // Si le mode Reload est activé avec le switch Reload
    {
        LedRGBautomatic();
        if (ButtonState_Reload == 0)
        {
            static unsigned long initTime = 0;
            unsigned long maintenant = millis();
            if (maintenant - initTime >= 300)
            {
                initTime = maintenant;
                Automatic++;
                if (Automatic >= 9)
                {
                    Automatic = 1;
                }
            }
        }
    }
}

void TIRenRafaleSouris()
{
    if (fire == 1)
    {
        static unsigned long initTime = 0;
        unsigned long maintenant = millis();
        if (maintenant - initTime >= CadenceTir)
        {
            initTime = maintenant;
            AbsMouse.press(MOUSE_LEFT);
            digitalWrite(LedFirePin, HIGH);
            ActionneSolenoid();
            burstCount++;
            if (burstCount == NBdeTirs)
            {
                fire = 0;
                burstCount = 0;
            }
        }
        else
        {
            AbsMouse.release(MOUSE_LEFT);
            digitalWrite(LedFirePin, LOW);
        }
    }
    else
    { // si fire différent de 1, éteindre Led et ne pas activer le tir
        AbsMouse.release(MOUSE_LEFT);
        digitalWrite(LedFirePin, LOW);
    }
    delay(10); // délais de respiration (sinon ça plante…)
}

void TIRenRafaleJoyStick()
{
    if (fire == 1)
    {
        static unsigned long initTime = 0;
        unsigned long maintenant = millis();
        if (maintenant - initTime >= CadenceTir)
        {
            initTime = maintenant;
            setButton(0, HIGH); // Bouton appuyé
            digitalWrite(LedFirePin, HIGH);
            ActionneSolenoid();
            burstCount++;
            if (burstCount == NBdeTirs)
            {
                fire = 0;
                burstCount = 0;
            }
        }
        else
        {
            setButton(0, LOW); // Bouton relâché
            digitalWrite(LedFirePin, LOW);
        }
    }
    else
    {                      // si fire différent de 1, éteindre Led et ne pas activer le tir
        setButton(0, LOW); // Bouton relâché
        digitalWrite(LedFirePin, LOW);
    }
    delay(15); // délais de respiration (sinon la cadence de tir ne passe pas !)
}

void TIRenRafaleHybride()
{
    if (fire == 1)
    {
        static unsigned long initTime = 0;
        unsigned long maintenant = millis();
        if (maintenant - initTime >= CadenceTir)
        { // boucle pour un intervalle de temps
            initTime = maintenant;
            AbsMouse.press(MOUSE_LEFT);
            setButton(0, HIGH); // Bouton appuyé puis
            digitalWrite(LedFirePin, HIGH);
            ActionneSolenoid();
            burstCount++;
            if (burstCount == NBdeTirs)
            {
                fire = 0;
                burstCount = 0;
            }
        }
        else
        {
            AbsMouse.release(MOUSE_LEFT);
            setButton(0, LOW); // Bouton relâché immédiatement
            digitalWrite(LedFirePin, LOW);
        }
    }
    else
    { // si fire différent de 1, éteindre Led et ne pas activer le tir
        AbsMouse.release(MOUSE_LEFT);
        setButton(0, LOW); // Bouton relâché
        digitalWrite(LedFirePin, LOW);
    }
    delay(15); // délais de respiration (sinon la cadence de tir ne passe pas !)
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// #############################
// # Gestion Boutons de Souris #
// #############################
// Valeurs pour la souris envoyées au HID Device
void mouseButtons()
{

    int ButtonState_Trigger = digitalRead(TriggerPin);
    int ButtonState_Reload = digitalRead(ReloadPin);
    int ButtonState_Start = digitalRead(StartPin);

    switch (Automatic)
    {
    case 1:
        if (ButtonState_Trigger != LastButtonState_Trigger)
        {
            if (ButtonState_Trigger == LOW)
            {
                fire = 1;
            }
            LastButtonState_Trigger = ButtonState_Trigger;
        }
        NBdeTirs = 1;
        TIRenRafaleSouris();
        break;
    case 2:
        if (ButtonState_Trigger != LastButtonState_Trigger)
        {
            if (ButtonState_Trigger == LOW)
            {
                fire = 1;
            }
            LastButtonState_Trigger = ButtonState_Trigger;
        }
        NBdeTirs = 2;
        TIRenRafaleSouris();
        break;
    case 3:
        if (ButtonState_Trigger != LastButtonState_Trigger)
        {
            if (ButtonState_Trigger == LOW)
            {
                fire = 1;
            }
            LastButtonState_Trigger = ButtonState_Trigger;
        }
        NBdeTirs = 3;
        TIRenRafaleSouris();
        break;
    case 4:
        if (ButtonState_Trigger != LastButtonState_Trigger)
        {
            if (ButtonState_Trigger == LOW)
            {
                fire = 1;
            }
            LastButtonState_Trigger = ButtonState_Trigger;
        }
        NBdeTirs = 4;
        TIRenRafaleSouris();
        break;
    case 5:
        if (ButtonState_Trigger != LastButtonState_Trigger)
        {
            if (ButtonState_Trigger == LOW)
            {
                fire = 1;
            }
            LastButtonState_Trigger = ButtonState_Trigger;
        }
        NBdeTirs = 5;
        TIRenRafaleSouris();
        break;
    case 6:
        if (ButtonState_Trigger != LastButtonState_Trigger)
        {
            if (ButtonState_Trigger == LOW)
            {
                fire = 1;
            }
            LastButtonState_Trigger = ButtonState_Trigger;
        }
        NBdeTirs = 6;
        TIRenRafaleSouris();
        break;
    case 7: // Autofire MachineGun
        if (ButtonState_Trigger == LOW)
        {
            fire = 1;
        }
        NBdeTirs = 1;
        TIRenRafaleSouris();
        break;
    case 8: // 1 coup avec Tir maintenu (spécial Alien3, Terminator…)
        if (ButtonState_Trigger == LOW)
        {
            AbsMouse.press(MOUSE_LEFT); // Bouton appuyé et maintenu pour certains jeux (Alien, T2,…)
            clignotteLED();
            ActionneSolenoid();
        }
        else
        {
            AbsMouse.release(MOUSE_LEFT); // Bouton relâché
            digitalWrite(LedFirePin, LOW);
        }
        delay(10);
        break;
    default:
        break;
    }

    if (ButtonState_Reload != LastButtonState_Reload)
    { // ReloadPin
        if (ButtonState_Reload == LOW)
        {
            AbsMouse.press(MOUSE_RIGHT);
        }
        else
        {
            AbsMouse.release(MOUSE_RIGHT);
        }
        delay(10);
        LastButtonState_Reload = ButtonState_Reload;
    }

    if (ButtonState_Start != LastButtonState_Start)
    { // StartPin
        if (ButtonState_Start == LOW)
        {
            AbsMouse.press(MOUSE_MIDDLE);
        }
        else
        {
            AbsMouse.release(MOUSE_MIDDLE);
        }
        delay(10);
        LastButtonState_Start = ButtonState_Start;
    }

    /* Mode RELOAD automatique si le curseur sort de l'écran, géré par logiciel */
    AutoReload = digitalRead(SwitchAutoReloadPin);
    if (AutoReload == 0) // Si le mode Reload est activé avec le switch Reload
    {
        digitalWrite(LedAutoReloadPin, HIGH); // allume la led Autoreload

        if ((see0 == 0) || (see1 == 0) || (see2 == 0) || (see3 == 0)) // si on sort de l'écran
        {
            // Serial.println("Reload !!! Clic Droit");
            AbsMouse.press(MOUSE_RIGHT);
        }
        else
        {
            AbsMouse.release(MOUSE_RIGHT);
        }
        delay(10);
    }
    else
    {
        // Serial.println("Mode Reload désactivé");
        digitalWrite(LedAutoReloadPin, LOW); // Éteind la led Autoreload
        AbsMouse.release(MOUSE_RIGHT);
    }
    delay(10);
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ###########################
// # Gestion boutons Hybride #
// ###########################
// Valeurs des boutons Souris + Joystick envoyées au HID Device
void Hybride_Buttons()
{

    int ButtonState_Trigger = !digitalRead(0 + pinToButtonMap); // pin 14
    int ButtonState_Start = !digitalRead(1 + pinToButtonMap);  // pin 15
    int ButtonState_Reload = !digitalRead(2 + pinToButtonMap); // pin 16

    // Routine pour tirer en rafale (appuie momentané)
    switch (Automatic)
    {
    case 1:
        if (ButtonState_Trigger != LastButtonState[0])
        {
            if (ButtonState_Trigger == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = ButtonState_Trigger;
        }
        NBdeTirs = 1;
        TIRenRafaleHybride();
        break;
    case 2:
        if (ButtonState_Trigger != LastButtonState[0])
        {
            if (ButtonState_Trigger == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = ButtonState_Trigger;
        }
        NBdeTirs = 2;
        TIRenRafaleHybride();
        break;
    case 3:
        if (ButtonState_Trigger != LastButtonState[0])
        {
            if (ButtonState_Trigger == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = ButtonState_Trigger;
        }
        NBdeTirs = 3;
        TIRenRafaleHybride();
        break;
    case 4:
        if (ButtonState_Trigger != LastButtonState[0])
        {
            if (ButtonState_Trigger == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = ButtonState_Trigger;
        }
        NBdeTirs = 4;
        TIRenRafaleHybride();
        break;
    case 5:
        if (ButtonState_Trigger != LastButtonState[0])
        {
            if (ButtonState_Trigger == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = ButtonState_Trigger;
        }
        NBdeTirs = 5;
        TIRenRafaleHybride();
        break;
    case 6:
        if (ButtonState_Trigger != LastButtonState[0])
        {
            if (ButtonState_Trigger == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = ButtonState_Trigger;
        }
        NBdeTirs = 6;
        TIRenRafaleHybride();
        break;
    case 7: // Autofire MachineGun
        if (ButtonState_Trigger == HIGH)
        {
            fire = 1;
        }
        NBdeTirs = 1;
        TIRenRafaleHybride();
        break;
    case 8: // 1 coup avec Tir maintenu (spécial Alien3, Terminator…)
        if (ButtonState_Trigger == HIGH)
        {
            AbsMouse.press(MOUSE_LEFT);
            setButton(0, HIGH); // Bouton appuyé
            digitalWrite(LedFirePin, HIGH);
            ActionneSolenoid();
        }
        else
        {
            AbsMouse.release(MOUSE_LEFT);
            setButton(0, LOW); // Bouton relâché
            digitalWrite(LedFirePin, LOW);
        }
        delay(15);
        break;
    default:
        break;
    }

    if (ButtonState_Start != LastButtonState[1])
    { // StartPin
        if (ButtonState_Start == LOW)
        {
            AbsMouse.press(MOUSE_MIDDLE);
            setButton(1, HIGH); // Bouton appuyé
        }
        else
        {
            AbsMouse.release(MOUSE_MIDDLE);
            setButton(1, LOW); // Bouton relâché
        }
        delay(15);
        LastButtonState[1] = ButtonState_Start;
    }

    if (ButtonState_Reload != LastButtonState[2])
    { // ReloadPin
        if (ButtonState_Reload == LOW)
        {
            AbsMouse.press(MOUSE_RIGHT);
            setButton(2, HIGH); // Bouton appuyé
        }
        else
        {
            AbsMouse.release(MOUSE_RIGHT);
            setButton(2, LOW); // Bouton relâché
        }
        delay(15);
        LastButtonState[2] = ButtonState_Reload;
    }

    /* Mode RELOAD automatique si le curseur sort de l'écran, géré par logiciel */
    AutoReload = digitalRead(SwitchAutoReloadPin);
    if (AutoReload == 0) // Si le mode auto Reload est activé avec le switch AutoReload
    {
        digitalWrite(LedAutoReloadPin, HIGH); // Allume la led Autoreload

        if ((see0 == 0) || (see1 == 0) || (see2 == 0) || (see3 == 0)) // si on sort de l'écran
        {
            // Serial.println("Reload bouton !!!");
            AbsMouse.press(MOUSE_RIGHT);
            setButton(2, HIGH); // pin 16
        }
        else
        {
            AbsMouse.release(MOUSE_RIGHT);
            setButton(2, LOW); // pin 16
        }
        delay(15);
    }
    else if (LastButtonState[2] == LOW)
    {
        // Serial.println("Mode Reload désactivé");
        digitalWrite(LedAutoReloadPin, LOW); // Éteind la led Autoreload
        AbsMouse.release(MOUSE_RIGHT);
        setButton(2, LOW);
    }
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ###############################
// # Gestion Boutons de Joystick #
// ###############################
// Valeurs des boutons du Joystick envoyées au HID Device
void Joystick_device_Buttons()
{

    //  for (int index = 0; index < 3; index++)
    //  {
    //    int currentButtonState = !digitalRead(index + pinToButtonMap);
    //    if (currentButtonState != LastButtonState[index])
    //    {
    //      setButton(index, currentButtonState);
    //      LastButtonState[index] = currentButtonState;
    //    }
    //  }

    int Etat_bouton_Tir = !digitalRead(0 + pinToButtonMap);    // pin 14
    int Etat_bouton_start = !digitalRead(1 + pinToButtonMap);  // pin 15
    int Etat_bouton_Reload = !digitalRead(2 + pinToButtonMap); // pin 16

    // Routine pour tirer (appuie maintenu)
    //  if (Etat_bouton_Tir != LastButtonState[0]) {
    //    setButton(0, Etat_bouton_Tir);
    //    LastButtonState[0] = Etat_bouton_Tir;
    //  }

    // Routine alternative pour tirer (appuie maintenu)
    //  if (Etat_bouton_Tir != LastButtonState[0]) {
    //    if (Etat_bouton_Tir == HIGH) {
    //      setButton(0, HIGH);  // Bouton appuyé
    //    }
    //    else {
    //      setButton(0, LOW);  // Bouton relâché
    //    }
    //    LastButtonState[0] = Etat_bouton_Tir;
    //  }

    // Routine pour tirer en rafale (appuie momentané)
    switch (Automatic)
    {
    case 1: // 1 coup
        if (Etat_bouton_Tir != LastButtonState[0])
        {
            if (Etat_bouton_Tir == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = Etat_bouton_Tir;
        }
        NBdeTirs = 1;
        TIRenRafaleJoyStick();
        break;
    case 2: // 2 coups
        if (Etat_bouton_Tir != LastButtonState[0])
        {
            if (Etat_bouton_Tir == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = Etat_bouton_Tir;
        }
        NBdeTirs = 2;
        TIRenRafaleJoyStick();
        break;
    case 3: // 3 coups
        if (Etat_bouton_Tir != LastButtonState[0])
        {
            if (Etat_bouton_Tir == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = Etat_bouton_Tir;
        }
        NBdeTirs = 3;
        TIRenRafaleJoyStick();
        break;
    case 4: // 4 coups
        if (Etat_bouton_Tir != LastButtonState[0])
        {
            if (Etat_bouton_Tir == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = Etat_bouton_Tir;
        }
        NBdeTirs = 4;
        TIRenRafaleJoyStick();
        break;
    case 5: // 5 coups
        if (Etat_bouton_Tir != LastButtonState[0])
        {
            if (Etat_bouton_Tir == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = Etat_bouton_Tir;
        }
        NBdeTirs = 5;
        TIRenRafaleJoyStick();
        break;
    case 6: // 6 coups
        if (Etat_bouton_Tir != LastButtonState[0])
        {
            if (Etat_bouton_Tir == HIGH)
            {
                fire = 1;
            }
            LastButtonState[0] = Etat_bouton_Tir;
        }
        NBdeTirs = 6;
        TIRenRafaleJoyStick();
        break;
    case 7: // Autofire MachineGun
        if (Etat_bouton_Tir == HIGH)
        {
            fire = 1;
        }
        NBdeTirs = 1;
        TIRenRafaleJoyStick();
        break;
    case 8: // 1 coup avec Tir maintenu (spécial Alien3, Terminator…)
        if (Etat_bouton_Tir == HIGH)
        {
            setButton(0, HIGH); // Bouton appuyé
            clignotteLED();
            ActionneSolenoid();
        }
        else
        {
            setButton(0, LOW); // Bouton relâché
            digitalWrite(LedFirePin, LOW);
        }
        delay(15);
        break;
    default:
        break;
    }

    if (Etat_bouton_start != LastButtonState[1])
    {
        setButton(1, Etat_bouton_start);
        LastButtonState[1] = Etat_bouton_start;
    }

    if (Etat_bouton_Reload != LastButtonState[2])
    {
        setButton(2, Etat_bouton_Reload);
        LastButtonState[2] = Etat_bouton_Reload;
    }

    /* Mode RELOAD automatique si le curseur sort de l'écran, géré par logiciel */
    AutoReload = digitalRead(SwitchAutoReloadPin);
    if (AutoReload == 0) // Si le mode auto Reload est activé avec le switch AutoReload
    {
        digitalWrite(LedAutoReloadPin, HIGH); // Allume la led Autoreload

        if ((see0 == 0) || (see1 == 0) || (see2 == 0) || (see3 == 0)) // si on sort de l'écran
        {
            // Serial.println("Reload bouton !!!");
            setButton(2, HIGH); // pin 16
        }
        else
        {
            setButton(2, LOW); // pin 16
        }
    }
    else if (LastButtonState[2] == LOW)
    {
        // Serial.println("Mode Reload désactivé");
        digitalWrite(LedAutoReloadPin, LOW); // Éteind la led Autoreload
        setButton(2, LOW);
    }
}

void setButton(uint8_t button, bool value)
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
// ###################################
// # Gestion Boutons de Calibrations #
// ###################################

// Setup Start Calibration Button
void go()
{
    // ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin));
    ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin) | digitalRead(ReloadPin));
    if (ButtonState_Calibration != LastButtonState_Calibration)
    {
        if (ButtonState_Calibration == LOW)
        {
            count--;
        }
        else
        { // do nothing
        }
        delay(50);
    }
    LastButtonState_Calibration = ButtonState_Calibration;
}

// Procédure de calibration
void mouseCount()
{
    ButtonValid = digitalRead(TriggerPin);
    ButtonPlus = digitalRead(StartPin);
    ButtonMinus = digitalRead(ReloadPin);

    if (ButtonValid != LastButtonValid)
    { // validation
        if (ButtonValid == LOW)
        {
            count--;
        }
        else
        {
        }
        delay(10);
    }

    if (ButtonPlus != LastButtonPlus)
    { // réglage +
        if (ButtonPlus == LOW)
        {
            plus = 1;
        }
        else
        {
            plus = 0;
        }
        delay(10);
    }

    if (ButtonMinus != LastButtonMinus)
    { // réglage -
        if (ButtonMinus == LOW)
        {
            minus = 1;
        }
        else
        {
            minus = 0;
        }
        delay(10);
    }

    LastButtonValid = ButtonValid;
    LastButtonPlus = ButtonPlus;
    LastButtonMinus = ButtonMinus;
}

// Pause/Re-calibrate button
void reset()
{
    // ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin));
    ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin) | digitalRead(ReloadPin));
    if (ButtonState_Calibration != LastButtonState_Calibration)
    {
        if (ButtonState_Calibration == LOW)
        {
            count = 4;
            delay(50);
        }
        else
        { // do nothing
        }
        delay(50);
    }
    LastButtonState_Calibration = ButtonState_Calibration;
}

// Unpause button
void skip()
{
    // ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin));
    ButtonState_Calibration = (digitalRead(StartPin) | digitalRead(TriggerPin) | digitalRead(ReloadPin));
    if (ButtonState_Calibration != LastButtonState_Calibration)
    {
        if (ButtonState_Calibration == LOW)
        {
            count = 0;
            delay(50);
        }
        else
        { // do nothing
        }
        delay(50);
    }
    LastButtonState_Calibration = ButtonState_Calibration;
}

// ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ##########################
// # Gestion Mémoire eeprom #
// ##########################
// Re-charger les valeurs de calibration depuis la mémoire EEPROM
void loadSettings()
{
    if (EEPROM.read(1023) == 'T')
    {
        // settings have been initialized, read them
        xCenter = EEPROM.read(0) + 256;
        yCenter = EEPROM.read(1) + 256;
        xOffset = EEPROM.read(2);
        yOffset = EEPROM.read(3);
    }
    else
    {
        // first time run, settings were never set
        EEPROM.write(0, xCenter - 256);
        EEPROM.write(1, yCenter - 256);
        EEPROM.write(2, xOffset);
        EEPROM.write(3, yOffset);
        EEPROM.write(1023, 'T');
    }
}

// Print results for saving calibration
void PrintResults()
{
    Serial.print("CALIBRATION:");
    Serial.print("     Cam Center x/y: ");
    Serial.print(xCenter);
    Serial.print(", ");
    Serial.print(yCenter);
    Serial.print("     Offsets x/y: ");
    Serial.print(xOffset);
    Serial.print(", ");
    Serial.println(yOffset);
}
