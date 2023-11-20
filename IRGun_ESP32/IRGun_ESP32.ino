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
   
   Ok - Remplacement du clavier HID par un Joystick HID (Stick XY et boutons (Start, Reload, Gachette))
   Ok - Mode Reload automatique si la souris sort de l'écran.
   Ok - Calibration/pause enclenchée par l'appuis sur les 3 boutons simultanéments (Start, Reload, Gachette)
   Ok - Switch de sélection pour désactiver le mode Reload automatique.
   Ok - Mode Souris - Mode Joystick - Mode Hybride (Mouvement de souris seulement) et (bouton de souris + bouton de joystick) (pour supermodel2)
   Ok - Switch de sélection pour désactiver la Souris.
   Ok - Switch de sélection pour désactiver le Joystick.
   Ok - Switch de sélection pour activer le mode hybride.
   Ok - Gestion Solenoid, qui claque à chaque tir.  https://arduinodiy.wordpress.com/2012/05/02/using-mosfets-with-ttl-levels/
   Ok - Bouton caroussel de sélection du mode de tir Automatic (pour 1 pression gachette = 1 coup, 2 coups, 3 coups, 4 coups, 5 coups, 6 coups, MachineGun)
   Ok - Gestion LED Mode de tir Automatic (Rouge, Vert, Bleu, Jaune, violet, Cyan, Blanc, fondu de couleur)
           Mode de tir :  Led 4 couleurs  2,3v (rouge) ou 3,4v (vert et bleu). 20mA.
          - 1 coup  -> 1 clac Solenoid -> Jaune
          - 2 coups -> 2 clacs -> Vert
          - 3 coups -> 3 clacs -> Bleu
          - 4 coups -> 4 clacs -> Blanc
          - 5 coups -> 5 clacs -> Cyan
          - 6 coups -> 6 clacs -> Violet
          - MachineGun -> clacs automatiques -> Fondu de couleur
          - 1 coup (maintenu) -> clacs automatiques -> Rouge
   Ok - Gestion LED Fire (led rouge qui crépite à chaque tir)
   Ok - Gestion des 3 LEDs témoins pour indiquer l'activation/désactivation des modes : Led 1+2 Hybride, Led 1 Souris seule, Led 2 Joystick seul, Led 3 Reload)
   Ok - Prise en charge du mode Automatic pour la Souris.
   Ok - Prise en charge du mode Automatic pour le Joystick.
   Ok - Debug mode Automatic lorsque que Hybride activé
   Ok - Faire clignotter led en tir standard maintenu.
   Ok - Si reload manuel, le bouton reload, bah il reload (bouton joystick et click souris)
      - régler bug : si on actionne le reload et tir en même temps, au passage du mode de tir Automatic « MachineGun », le flingue se met à tirer seul sans arrêt
*/

/* HOW TO CALIBRATE:
   Step 1: Push Calibration Button
   Step 2: Pull Trigger
   Step 3: Shoot Center of the Screen (try an do this as acuratley as possible)
   Step 4: Mouse should lock to vertiucal axis, use Start and Reload buttons to adjust mouse up/down
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


int xCenter = 512;    // Open serial monitor and update these values to save calibration manualy
int yCenter = 450;
float xOffset = 147;
float yOffset = 82;
//CALIBRATION:     Cam Center x/y: 421, 356     Offsets x/y: 121.00, 71.00

//**************************
//* Définition du Joystick *
//**************************
BleGamepad Joystick("IRGun ESP32");
BleGamepadConfiguration JoystickConfig;
const int pinToButtonMap = 14;            // Constant that maps the physical pin to joystick button. (pins 14-15-16)
int LastButtonState[3] = {0, 0, 0};       // Last state of the 3 buttons

//******************************
//* Variables pour la CameraIR *
//******************************
int condition = 1;
int finalX;                 // Values after tilt correction
int finalY;
int see0;   // Leds IR N°1 vue par la caméra. (sinon = 0)
int see1;   // Leds IR N°2 vue par la caméra. (sinon = 0)
int see2;   // Leds IR N°3 vue par la caméra. (sinon = 0)
int see3;   // Leds IR N°4 vue par la caméra. (sinon = 0)

//****************************
//* Variables pour la Souris *
//****************************
int MoveXAxis;              // Unconstrained mouse postion
int MoveYAxis;
int conMoveXAxis;           // Constrained mouse postion
int conMoveYAxis;
int count = -2;             // Set intial count

//***************************
//* Déclaration des Entrées *
//***************************
#define _tiggerPin 39             // Label Pin to Joystick buttons
#define _startPin 34              // ET aussi  boutom le calibration - 
#define _reloadPin  36            // ET aussi boutom le calibration + 
#define SwitchAutoReload  35      // Mode reload Automatique On/Off
#define SwitchSuspendMouse  14    // (Actif par défaut si non câblé) Désactive la Souris (le HID est présent, mais les data ne sont plus envoyées à la souris)
#define SwitchSuspendJoystick 12  // (Actif par défaut si non câblé) Désactive le joystick. Note: Could Fail in ESP32?

//***************************
//* Déclaration des Sorties *
//***************************
#define LedFire 13
#define LedSuspendMouse 25
#define LedSuspendJoystick  26
#define LedAutoReload 27
#define LedRouge  16
#define LedVerte  17
#define LedBleu 18
#define Solenoid 19
#define CadenceTir 90
#define ToggleLED(x) digitalWrite(x, !digitalRead(x))

//**********************
//* Variables diverses *
//**********************
unsigned int NBdeTirs = 0;
unsigned int fire = 0;
const int R = 1;
const int V = 1;
const int B = 1;
unsigned int varR = 255;
unsigned int varV = 0;
unsigned int varB = 0;
unsigned int SequenceRouge = 1;
unsigned int SequenceJaune = 0;
unsigned int SequenceVert = 0;
unsigned int SequenceCyan = 0;
unsigned int SequenceBleu = 0;
unsigned int SequenceViolet = 0;
unsigned int Automatic = 1;
unsigned int rafaleCount = 0;

// État des boutons
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
int ButtonState_calibration = 0;
int LastButtonState_calibration = 0;
//int ButtonState_tiggerPin = 0;
int LastButtonState_tiggerPin = 0;
//int ButtonState_reloadPin = 0;
int LastButtonState_reloadPin = 0;
//int ButtonState_startPin = 0;
int LastButtonState_startPin = 0;

int plus = 0;
int minus = 0;

//************************
//* Déclaration CaméraIR *
//************************
DFRobotIRPosition myDFRobotIRPosition;
SamcoBeta mySamco;
int res_x = 1023;              // UPDATE: These values do not need to change
int res_y = 768;               // UPDATE: These values do not need to change

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void setup() {

  myDFRobotIRPosition.begin();               // Start IR Camera

  Serial.begin(9600);                        // For saving calibration (make sure your serial monitor has the same baud rate)

  JoystickConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  JoystickConfig.setAutoReport(true);
  JoystickConfig.setButtonCount(3);                 // Button Count
  JoystickConfig.setHatSwitchCount(0);              // Hat Switch Count
  JoystickConfig.setWhichAxes(true, true, false,    // X and Y, but no Z Axis
                              false, false, false,  // No Rx, Ry, or Rz
                              false, false);        // No rudder or throttle
  JoystickConfig.setAxesMin(0);
  JoystickConfig.setAxesMax(255);
  Joystick.begin(&JoystickConfig);

  loadSettings();

  // Switchs
  pinMode(SwitchAutoReload, INPUT_PULLUP);
  pinMode(SwitchSuspendMouse, INPUT_PULLUP);
  pinMode(SwitchSuspendJoystick, INPUT_PULLUP);

  // Leds
  pinMode(LedSuspendMouse, OUTPUT);
  pinMode(LedSuspendJoystick, OUTPUT);
  pinMode(LedAutoReload, OUTPUT);
  pinMode(LedFire, OUTPUT);
  pinMode(LedRouge, OUTPUT);
  pinMode(LedVerte, OUTPUT);
  pinMode(LedBleu, OUTPUT);

  // Mecanique
  pinMode(Solenoid, OUTPUT);
  digitalWrite(Solenoid, LOW); //  LOW = Repos

  // Boutons de Joystick
  pinMode(_tiggerPin, INPUT_PULLUP);         // Set pin modes
  pinMode(_reloadPin, INPUT_PULLUP);
  pinMode(_startPin, INPUT_PULLUP);

  AbsMouse.init(res_x, res_y);

  AbsMouse.move((res_x / 2), (res_y / 2));          // Set mouse position to centre of the screen

  delay(500);

  // Led RGB Éteinte
  ledRVB(0, 0, 0);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//#############
//# Programme #
//#############
void loop() {

  /* ------------------ START/PAUSE MOUSE ---------------------- */

  if (count > 3 ) {

    skip();
    mouseCount();
    //  PrintResults();

  }

  /* ---------------------- CENTRE --------------------------- */
  // DÉBUT de la procédure de calibration

  else if (count > 2 ) {

    AbsMouse.move((res_x / 2), (res_y / 2));

    mouseCount();
    getPosition();

    xCenter = finalX;
    yCenter = finalY;

    //  PrintResults();

  }

  /* -------------------- OFFSET ------------------------- */

  else if (count > 1 ) {

    mouseCount();
    AbsMouse.move(conMoveXAxis, conMoveYAxis);
    getPosition();

    MoveYAxis = map (finalY, (yCenter + ((mySamco.H() * (yOffset / 100)) / 2)), (yCenter - ((mySamco.H() * (yOffset / 100)) / 2)), 0, res_y);
    conMoveXAxis = res_x / 2;
    conMoveYAxis = constrain (MoveYAxis, 0, res_y);

    if (plus == 1) {
      yOffset = yOffset + 1;
      delay(10);
    } else {
    }

    if (minus == 1) {
      yOffset = yOffset - 1;
      delay(10);
    } else {
    }

    //   PrintResults();

  }

  else if (count > 0 ) {

    mouseCount();
    AbsMouse.move(conMoveXAxis, conMoveYAxis);
    getPosition();

    MoveXAxis = map (finalX, (xCenter + ((mySamco.H() * (xOffset / 100)) / 2)), (xCenter - ((mySamco.H() * (xOffset / 100)) / 2)), 0, res_x);
    conMoveXAxis = constrain (MoveXAxis, 0, res_x);
    conMoveYAxis = res_y / 2;

    if (plus == 1) {
      xOffset = xOffset + 1;
      delay(10);
    } else {
    }

    if (minus == 1) {
      xOffset = xOffset - 1;
      delay(10);
    } else {
    }

    //  PrintResults();

  }

  else if (count > -1) {

    count = count - 1;

    EEPROM.write(0, xCenter - 256);
    EEPROM.write(1, yCenter - 256);
    EEPROM.write(2, xOffset);
    EEPROM.write(3, yOffset);
  }
  // FIN de la procédure de calibration
  /* ---------------------- LET'S GO --------------------------- */
  // Utilisation normale du pistolet

  else {

    // Activation des mouvements de la souris (si 0 = Switch basculé sur > mouse désactivée)
    SuspendMouse = digitalRead(SwitchSuspendMouse);
    if ((SuspendMouse == 1) && (SuspendJoystick == 0)) {
      //      Serial.println("Souris Activée");
      digitalWrite(LedSuspendMouse, HIGH); // allume la led
      AbsMouse.move(conMoveXAxis, conMoveYAxis);  // Mouvements envoyés au HID
      mouseButtons();                             // clicks envoyés au HID
      //    Serial.print("conMoveXAxis :");
      //    Serial.print(conMoveXAxis);
      //    Serial.print(",  conMoveYAxis :");
      //    Serial.println(conMoveYAxis);

    }
    else {      // désactivation compléte de la souris
      //      Serial.println("Souris désactivée");
      //      Serial.println(" ");
      digitalWrite(LedSuspendMouse, LOW);  // éteind la led
    }

    // Activation des mouvement du joystick, 1 par défaut (si 0 = Switch basculé sur > stick désactivé)
    SuspendJoystick = digitalRead(SwitchSuspendJoystick);
    if ((SuspendMouse == 0) && (SuspendJoystick == 1)) {
      //      Serial.println("Joystick Activé");
      digitalWrite(LedSuspendJoystick, HIGH); // allume la led
      Joystick_device_Buttons();   // Boutons envoyés au HID
      Joystick_device_Stick();   // Mouvements envoyés au HID
    }
    else {      // désactivation compléte du joystick
      //      Serial.println("Joystick désactivé");
      //      Serial.println(" ");
      digitalWrite(LedSuspendJoystick, LOW);  // éteind la led
      Joystick.setX(127);   // Centrage du stick
      Joystick.setY(127);
    }

    // Activation du mode Hybride (Mouvement de souris seulement) et (bouton de souris + bouton de joystick)
    if ((SuspendMouse == 1) && (SuspendJoystick == 1)) {
      //      Serial.println("mode hybride activé");
      digitalWrite(LedSuspendMouse, HIGH); // allume la led
      digitalWrite(LedSuspendJoystick, HIGH); // allume la led
      AbsMouse.move(conMoveXAxis, conMoveYAxis);  // Mouvements envoyés au HID
      Hybride_Buttons();   // Boutons envoyés au HID
    }

    getPosition();  // Récupération des valeurs de la caméra IR

    MoveXAxis = map (finalX, (xCenter + ((mySamco.H() * (xOffset / 100)) / 2)), (xCenter - ((mySamco.H() * (xOffset / 100)) / 2)), 0, res_x);
    MoveYAxis = map (finalY, (yCenter + ((mySamco.H() * (yOffset / 100)) / 2)), (yCenter - ((mySamco.H() * (yOffset / 100)) / 2)), 0, res_y);
    conMoveXAxis = constrain (MoveXAxis, 0, res_x);
    conMoveYAxis = constrain (MoveYAxis, 0, res_y);

    //    PrintResults();
    reset();

  }
  ModeAutomatic();
  LedRGBautomatic();
}


/*        -----------------------------------------------        */
/* --------------------------- METHODS ------------------------- */
/*        -----------------------------------------------        */

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//####################
//# Gestion CameraIR #
//####################
void getPosition() {    // Get tilt adjusted position from IR postioning camera

  myDFRobotIRPosition.requestPosition();
  if (myDFRobotIRPosition.available()) {
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
  else {
    Serial.println("Device not available!");
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//##################################
//# Gestion mouvements de Joystick #
//##################################
void Joystick_device_Stick() {
  int xAxis = map(conMoveXAxis, 0, 1023, 0, 255); // Avec inversion des axes requise
  int yAxis = map(conMoveYAxis, 0, 768, 0, 255);

  //  Serial.print("Joystick xAxis = ");
  //  Serial.print(xAxis);
  //  Serial.print(",  yAxis = ");
  //  Serial.println(yAxis);

  Joystick.setX(xAxis);
  Joystick.setY(yAxis);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//####################
//# Gestion LEDs RGB #
//####################
// http://www.mon-club-elec.fr/pmwiki_mon_club_elec/pmwiki.php?n=MAIN.ArduinoInitiationLedsDiversTestLedRVBac
//---- fonction pour combiner couleurs ON/OFF ----
void ledRVB(int Rouge, int Vert, int Bleu) {
  if (Rouge == 1) digitalWrite(LedRouge, LOW); // allume couleur
  if (Rouge == 0) digitalWrite(LedRouge, HIGH); // éteint couleur

  if (Vert == 1) digitalWrite(LedVerte, LOW); // allume couleur
  if (Vert == 0) digitalWrite(LedVerte, HIGH); // éteint couleur

  if (Bleu == 1) digitalWrite(LedBleu, LOW); // allume couleur
  if (Bleu == 0) digitalWrite(LedBleu, HIGH); // éteint couleur
}

//---- fonction pour variation progressive des couleurs ----
void ledRVBpwm(int pwmRouge, int pwmVert, int pwmBleu) { // reçoit valeur 0-255 par couleur
  analogWrite(LedRouge, 255 - pwmRouge); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
  analogWrite(LedVerte, 255 - pwmVert); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
  analogWrite(LedBleu, 255 - pwmBleu); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
}

//---- fonction pour fondu arc-en-ciel ----
void FonduCouleurs() {
//  Séquence :
//  255,0,0 rouge
//  255,255,0 jaune
//  0,255,0 vert
//  0,255,255 cyan
//  0,0,255 bleu
//  255,0,255 violet
//  On démarre en rouge avec varR,varV,varB = 255,0,0

  ledRVBpwm(varR,varV,varB); // génère impulsion largeur voulue pour la couleur
    
  if (SequenceRouge == 1) {  // 255,0,0 Rouge
    varV++; // augmente le Vert
    if (varV == 255 ) { // on obtient Jaune (255,255,0)
      SequenceRouge = 0;
      SequenceJaune = 1;  // On active la prochaine séquence
      SequenceVert = 0;
      SequenceCyan = 0;
      SequenceBleu = 0;
      SequenceViolet = 0;  
    }
  }
  
  if (SequenceJaune == 1) { // 255,255,0 Jaune
    varR--;   // on baisse le Rouge 
    if (varR == 0 ) {   // ne reste que le Vert  (0,255,0)
      SequenceRouge = 0;
      SequenceJaune = 0;
      SequenceVert = 1;   // On active la prochaine séquence
      SequenceCyan = 0;
      SequenceBleu = 0;
      SequenceViolet = 0;  
    }
  }
  
  if (SequenceVert == 1) {  // 0,255,0 Vert
    varB++;   // augmente le bleu
    if (varB == 255 ) {   // on obtient le Cyan  (0,255,255)
      SequenceRouge = 0;
      SequenceJaune = 0;
      SequenceVert = 0;
      SequenceCyan = 1;   // On active la prochaine séquence
      SequenceBleu = 0;
      SequenceViolet = 0;  
    }
  }
    
  if (SequenceCyan == 1) {  // 0,255,255 Cyan
    varV--;   // on baisse le vert
    if (varV == 0 ) {    // on obtient de bleu (0,0,255)
      SequenceRouge = 0;
      SequenceJaune = 0;
      SequenceVert = 0;
      SequenceCyan = 0;
      SequenceBleu = 1;   // On active la prochaine séquence
      SequenceViolet = 0;  
    }
  }
  
  if (SequenceBleu == 1) {  // 0,0,255 Bleu
    varR++;   // augmenter le Rouge
    if (varR == 255 ) {     // on obtient le Violet (255,0,255)
      SequenceRouge = 0;
      SequenceJaune = 0;
      SequenceVert = 0;
      SequenceCyan = 0;
      SequenceBleu = 0;
      SequenceViolet = 1;   // On active la prochaine séquence 
    }
  }
  
  if (SequenceViolet == 1) {  // 255,0,255 Violet
    varB--;   // on baisse le Bleu
    if (varB == 0 ) {     // on obtient le Rouge (255,0,0)
      Serial.print("SequenceRouge");
      SequenceRouge = 1;   // On active la prochaine séquence 
      SequenceJaune = 0;
      SequenceVert = 0;
      SequenceCyan = 0;
      SequenceBleu = 0;
      SequenceViolet = 0;
    }
  }
//  Serial.print("");
//  Serial.print("varR : ");
//  Serial.print(varR);
//  Serial.print(" | varV : ");
//  Serial.print(varV);
//  Serial.print(" | varB : ");
//  Serial.println(varB);
//  delay(4);
}

void LedRGBautomatic() {
  switch (Automatic) {
    case 1:
      ledRVB(R, V, 0); // jaune
      break;
    case 2:
      ledRVB(0, V, 0); // vert
      break;
    case 3:
      ledRVB(0, 0, B); // bleu
      break;
    case 4:
      ledRVB(R, V, B); // blanc
      break;
    case 5:
      ledRVB(0, V, B); // cyan
      break;
    case 6:
      ledRVB(R, 0, B); // violet
      break;
    case 7:   
      FonduCouleurs();  // Fondu de couleur rouge-orange-jaune-vert-cyan-bleu-violet-rose
      break;
    case 8:
      ledRVB(R, 0, 0); // rouge
      break; 
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//#####################################
//# Gestion événements opto-mécanique #
//#####################################

void clignotteLED() {
  static unsigned long initTime = 0;
  unsigned long maintenant = millis();
  if (maintenant - initTime >= 90) {
    initTime = maintenant;
    digitalWrite(LedFire, HIGH);
  }
  else {
    digitalWrite(LedFire, LOW);    // turn the LED off by making the voltage LOW
  }
}

void ActionneSolenoid() {
  static unsigned long initTime = 0;
  unsigned long maintenant = millis();
  if (maintenant - initTime >= CadenceTir) {
    initTime = maintenant;
    // Contrairement à la Led, il faut un peu de temps au mécanisme pour changer d'état
    unsigned long startTime = millis();
    unsigned long duree = 25; // maintient 25 µs pour que le mécanisme ait le temps de changer d'état
    while (millis() < startTime + duree) {
      digitalWrite(Solenoid, HIGH);  // collé
    }
  }
  else {
    digitalWrite(Solenoid, LOW);   // repos
  }
  delay(15);  // délais de respiration (sinon la cadence de tir ne passe pas !)
  digitalWrite(Solenoid, LOW);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//#######################
//# Gestion Mode de tir #
//#######################

void ModeAutomatic() {  // Sélection du mode de tir : 1 coup, 2 coups, 3 coups, 4 coups, 5 coups, 6 coups, MachineGun, 1 coup maintenu
  int ButtonState_reloadPin = digitalRead(_reloadPin);
  AutoReload = digitalRead(SwitchAutoReload);
  if (AutoReload == 0)  // Si le mode Reload est activé avec le switch Reload
  {
    LedRGBautomatic();
    if (ButtonState_reloadPin == 0) {
      static unsigned long initTime = 0;
      unsigned long maintenant = millis();
      if (maintenant - initTime >= 300) {
        initTime = maintenant;
        Automatic++;
        if (Automatic >= 9) {
          Automatic = 1;
        }
      }
    }
  }
}

void TIRenRafaleSouris() {
  if (fire == 1) {
    static unsigned long initTime = 0;
    unsigned long maintenant = millis();
    if (maintenant - initTime >= CadenceTir) {
      initTime = maintenant;
      AbsMouse.press(MOUSE_LEFT);
      digitalWrite(LedFire, HIGH);
      ActionneSolenoid();
      rafaleCount++;
      if (rafaleCount == NBdeTirs) {
        fire = 0;
        rafaleCount = 0;
      }
    }
    else {
      AbsMouse.release(MOUSE_LEFT);
      digitalWrite(LedFire, LOW);
    }
  }
  else {  // si fire différent de 1, éteindre Led et ne pas activer le tir
    AbsMouse.release(MOUSE_LEFT);
    digitalWrite(LedFire, LOW);
  }
  delay(10);  // délais de respiration (sinon ça plante…)
}

void TIRenRafaleJoyStick() {
  if (fire == 1) {
    static unsigned long initTime = 0;
    unsigned long maintenant = millis();
    if (maintenant - initTime >= CadenceTir) {
      initTime = maintenant;
      setButton(0, HIGH);  // Bouton appuyé
      digitalWrite(LedFire, HIGH);
      ActionneSolenoid();
      rafaleCount++;
      if (rafaleCount == NBdeTirs) {
        fire = 0;
        rafaleCount = 0;
      }
    }
    else {
      setButton(0, LOW);  // Bouton relâché
      digitalWrite(LedFire, LOW);
    }
  }
  else {  // si fire différent de 1, éteindre Led et ne pas activer le tir
    setButton(0, LOW);  // Bouton relâché
    digitalWrite(LedFire, LOW);
  }
  delay(15);  // délais de respiration (sinon la cadence de tir ne passe pas !)
}

void TIRenRafaleHybride() {
  if (fire == 1) {
    static unsigned long initTime = 0;
    unsigned long maintenant = millis();
    if (maintenant - initTime >= CadenceTir) {  // boucle pour un intervalle de temps
      initTime = maintenant;
      AbsMouse.press(MOUSE_LEFT);
      setButton(0, HIGH);  // Bouton appuyé puis
      digitalWrite(LedFire, HIGH);
      ActionneSolenoid();
      rafaleCount++;
      if (rafaleCount == NBdeTirs) {
        fire = 0;
        rafaleCount = 0;
      }
    }
    else {
      AbsMouse.release(MOUSE_LEFT);
      setButton(0, LOW);  // Bouton relâché immédiatement
      digitalWrite(LedFire, LOW);
    }
  }
  else {  // si fire différent de 1, éteindre Led et ne pas activer le tir
    AbsMouse.release(MOUSE_LEFT);
    setButton(0, LOW);  // Bouton relâché
    digitalWrite(LedFire, LOW);
  }
  delay(15);  // délais de respiration (sinon la cadence de tir ne passe pas !)
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//#############################
//# Gestion Boutons de Souris #
//#############################
void mouseButtons() {    // Valeurs pour la souris envoyées au HID Device

  int ButtonState_tiggerPin = digitalRead(_tiggerPin);
  int ButtonState_reloadPin = digitalRead(_reloadPin);
  int ButtonState_startPin = digitalRead(_startPin);

  switch (Automatic) {
    case 1:
      if (ButtonState_tiggerPin != LastButtonState_tiggerPin) {
        if (ButtonState_tiggerPin == LOW) {
          fire = 1;
        }
        LastButtonState_tiggerPin = ButtonState_tiggerPin;
      }
      NBdeTirs = 1;
      TIRenRafaleSouris();
      break;
    case 2:
      if (ButtonState_tiggerPin != LastButtonState_tiggerPin) {
        if (ButtonState_tiggerPin == LOW) {
          fire = 1;
        }
        LastButtonState_tiggerPin = ButtonState_tiggerPin;
      }
      NBdeTirs = 2;
      TIRenRafaleSouris();
      break;
    case 3:
      if (ButtonState_tiggerPin != LastButtonState_tiggerPin) {
        if (ButtonState_tiggerPin == LOW) {
          fire = 1;
        }
        LastButtonState_tiggerPin = ButtonState_tiggerPin;
      }
      NBdeTirs = 3;
      TIRenRafaleSouris();
      break;
    case 4:
      if (ButtonState_tiggerPin != LastButtonState_tiggerPin) {
        if (ButtonState_tiggerPin == LOW) {
          fire = 1;
        }
        LastButtonState_tiggerPin = ButtonState_tiggerPin;
      }
      NBdeTirs = 4;
      TIRenRafaleSouris();
      break;
    case 5:
      if (ButtonState_tiggerPin != LastButtonState_tiggerPin) {
        if (ButtonState_tiggerPin == LOW) {
          fire = 1;
        }
        LastButtonState_tiggerPin = ButtonState_tiggerPin;
      }
      NBdeTirs = 5;
      TIRenRafaleSouris();
      break;
    case 6:
      if (ButtonState_tiggerPin != LastButtonState_tiggerPin) {
        if (ButtonState_tiggerPin == LOW) {
          fire = 1;
        }
        LastButtonState_tiggerPin = ButtonState_tiggerPin;
      }
      NBdeTirs = 6;
      TIRenRafaleSouris();
      break;
    case 7:   // Autofire MachineGun
      if (ButtonState_tiggerPin == LOW) {
        fire = 1;
      }
      NBdeTirs = 1;
      TIRenRafaleSouris();
      break;
    case 8:   // 1 coup avec Tir maintenu (spécial Alien3, Terminator…)
      if (ButtonState_tiggerPin == LOW) {
        AbsMouse.press(MOUSE_LEFT);  // Bouton appuyé et maintenu pour certains jeux (Alien, T2,…)
        clignotteLED();
        ActionneSolenoid();
      }
      else {
        AbsMouse.release(MOUSE_LEFT);  // Bouton relâché
        digitalWrite(LedFire, LOW);
      }
      delay(10);
      break;
    default:
      break;
  }

  if (ButtonState_reloadPin != LastButtonState_reloadPin) { // _reloadPin
    if (ButtonState_reloadPin == LOW) {
      AbsMouse.press(MOUSE_RIGHT);
    }
    else {
      AbsMouse.release(MOUSE_RIGHT);
    }
    delay(10);
    LastButtonState_reloadPin = ButtonState_reloadPin;
  }

  if (ButtonState_startPin != LastButtonState_startPin) { // _startPin
    if (ButtonState_startPin == LOW) {
      AbsMouse.press(MOUSE_MIDDLE);
    }
    else {
      AbsMouse.release(MOUSE_MIDDLE);
    }
    delay(10);
    LastButtonState_startPin = ButtonState_startPin;
  }

  /* Mode RELOAD automatique si le curseur sort de l'écran, géré par logiciel */
  AutoReload = digitalRead(SwitchAutoReload);
  if (AutoReload == 0)  // Si le mode Reload est activé avec le switch Reload
  {
    digitalWrite(LedAutoReload, HIGH);  // allume la led Autoreload

    if ((see0 == 0) || (see1 == 0) || (see2 == 0) || (see3 == 0))  // si on sort de l'écran
    {
      //Serial.println("Reload !!! Clic Droit");
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
    //Serial.println("Mode Reload désactivé");
    digitalWrite(LedAutoReload, LOW);  // Éteind la led Autoreload
    AbsMouse.release(MOUSE_RIGHT);
  }
  delay(10);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//###########################
//# Gestion boutons Hybride #
//###########################
void Hybride_Buttons() {    // Valeurs des boutons Souris + Joystick envoyées au HID Device

  int ButtonState_tiggerPin = !digitalRead(0 + pinToButtonMap); // pin 14
  int ButtonState_startPin = !digitalRead(1 + pinToButtonMap); // pin 15
  int ButtonState_reloadPin = !digitalRead(2 + pinToButtonMap); // pin 16


  // Routine pour tirer en rafale (appuie momentané)
  switch (Automatic) {
    case 1:
      if (ButtonState_tiggerPin != LastButtonState[0]) {
        if (ButtonState_tiggerPin == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = ButtonState_tiggerPin;
      }
      NBdeTirs = 1;
      TIRenRafaleHybride();
      break;
    case 2:
      if (ButtonState_tiggerPin != LastButtonState[0]) {
        if (ButtonState_tiggerPin == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = ButtonState_tiggerPin;
      }
      NBdeTirs = 2;
      TIRenRafaleHybride();
      break;
    case 3:
      if (ButtonState_tiggerPin != LastButtonState[0]) {
        if (ButtonState_tiggerPin == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = ButtonState_tiggerPin;
      }
      NBdeTirs = 3;
      TIRenRafaleHybride();
      break;
    case 4:
      if (ButtonState_tiggerPin != LastButtonState[0]) {
        if (ButtonState_tiggerPin == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = ButtonState_tiggerPin;
      }
      NBdeTirs = 4;
      TIRenRafaleHybride();
      break;
    case 5:
      if (ButtonState_tiggerPin != LastButtonState[0]) {
        if (ButtonState_tiggerPin == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = ButtonState_tiggerPin;
      }
      NBdeTirs = 5;
      TIRenRafaleHybride();
      break;
    case 6:
      if (ButtonState_tiggerPin != LastButtonState[0]) {
        if (ButtonState_tiggerPin == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = ButtonState_tiggerPin;
      }
      NBdeTirs = 6;
      TIRenRafaleHybride();
      break;
    case 7:   // Autofire MachineGun
      if (ButtonState_tiggerPin == HIGH) {
        fire = 1;
      }
      NBdeTirs = 1;
      TIRenRafaleHybride();
      break;
    case 8:   // 1 coup avec Tir maintenu (spécial Alien3, Terminator…)
      if (ButtonState_tiggerPin == HIGH) {
        AbsMouse.press(MOUSE_LEFT);
        setButton(0, HIGH);  // Bouton appuyé
        digitalWrite(LedFire, HIGH);
        ActionneSolenoid();
      }
      else {
        AbsMouse.release(MOUSE_LEFT);
        setButton(0, LOW);  // Bouton relâché
        digitalWrite(LedFire, LOW);
      }
      delay(15);
      break;
    default:
      break;
  }

  if (ButtonState_startPin != LastButtonState[1]) { // _startPin
    if (ButtonState_startPin == LOW) {
      AbsMouse.press(MOUSE_MIDDLE);
      setButton(1, HIGH);  // Bouton appuyé
    }
    else {
      AbsMouse.release(MOUSE_MIDDLE);
      setButton(1, LOW);  // Bouton relâché
    }
    delay(15);
    LastButtonState[1] = ButtonState_startPin;
  }

  if (ButtonState_reloadPin != LastButtonState[2]) { // _reloadPin
    if (ButtonState_reloadPin == LOW) {
      AbsMouse.press(MOUSE_RIGHT);
      setButton(2, HIGH);  // Bouton appuyé
    }
    else {
      AbsMouse.release(MOUSE_RIGHT);
      setButton(2, LOW);  // Bouton relâché
    }
    delay(15);
    LastButtonState[2] = ButtonState_reloadPin;
  }

  /* Mode RELOAD automatique si le curseur sort de l'écran, géré par logiciel */
  AutoReload = digitalRead(SwitchAutoReload);
  if (AutoReload == 0)  // Si le mode auto Reload est activé avec le switch AutoReload
  {
    digitalWrite(LedAutoReload, HIGH);  // Allume la led Autoreload

    if ((see0 == 0) || (see1 == 0) || (see2 == 0) || (see3 == 0))  // si on sort de l'écran
    {
      //Serial.println("Reload bouton !!!");
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
    //Serial.println("Mode Reload désactivé");
    digitalWrite(LedAutoReload, LOW);  // Éteind la led Autoreload
    AbsMouse.release(MOUSE_RIGHT);
    setButton(2, LOW);
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//###############################
//# Gestion Boutons de Joystick #
//###############################
void Joystick_device_Buttons() {    // Valeurs des boutons du Joystick envoyées au HID Device

  //  for (int index = 0; index < 3; index++)
  //  {
  //    int currentButtonState = !digitalRead(index + pinToButtonMap);
  //    if (currentButtonState != LastButtonState[index])
  //    {
  //      setButton(index, currentButtonState);
  //      LastButtonState[index] = currentButtonState;
  //    }
  //  }

  int Etat_bouton_Tir = !digitalRead(0 + pinToButtonMap); // pin 14
  int Etat_bouton_start = !digitalRead(1 + pinToButtonMap); // pin 15
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
  switch (Automatic) {
    case 1:   // 1 coup
      if (Etat_bouton_Tir != LastButtonState[0]) {
        if (Etat_bouton_Tir == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = Etat_bouton_Tir;
      }
      NBdeTirs = 1;
      TIRenRafaleJoyStick();
      break;
    case 2:   // 2 coups
      if (Etat_bouton_Tir != LastButtonState[0]) {
        if (Etat_bouton_Tir == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = Etat_bouton_Tir;
      }
      NBdeTirs = 2;
      TIRenRafaleJoyStick();
      break;
    case 3:   // 3 coups
      if (Etat_bouton_Tir != LastButtonState[0]) {
        if (Etat_bouton_Tir == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = Etat_bouton_Tir;
      }
      NBdeTirs = 3;
      TIRenRafaleJoyStick();
      break;
    case 4:   // 4 coups
      if (Etat_bouton_Tir != LastButtonState[0]) {
        if (Etat_bouton_Tir == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = Etat_bouton_Tir;
      }
      NBdeTirs = 4;
      TIRenRafaleJoyStick();
      break;
    case 5:   // 5 coups
      if (Etat_bouton_Tir != LastButtonState[0]) {
        if (Etat_bouton_Tir == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = Etat_bouton_Tir;
      }
      NBdeTirs = 5;
      TIRenRafaleJoyStick();
      break;
    case 6:   // 6 coups
      if (Etat_bouton_Tir != LastButtonState[0]) {
        if (Etat_bouton_Tir == HIGH) {
          fire = 1;
        }
        LastButtonState[0] = Etat_bouton_Tir;
      }
      NBdeTirs = 6;
      TIRenRafaleJoyStick();
      break;
    case 7:   // Autofire MachineGun
      if (Etat_bouton_Tir == HIGH) {
        fire = 1;
      }
      NBdeTirs = 1;
      TIRenRafaleJoyStick();
      break;
    case 8:   // 1 coup avec Tir maintenu (spécial Alien3, Terminator…)
      if (Etat_bouton_Tir == HIGH) {
        setButton(0, HIGH);  // Bouton appuyé
        clignotteLED();
        ActionneSolenoid();
      }
      else {
        setButton(0, LOW);  // Bouton relâché
        digitalWrite(LedFire, LOW);
      }
      delay(15);
      break;
    default:
      break;
  }

  if (Etat_bouton_start != LastButtonState[1]) {
    setButton(1, Etat_bouton_start);
    LastButtonState[1] = Etat_bouton_start;
  }

  if (Etat_bouton_Reload != LastButtonState[2]) {
    setButton(2, Etat_bouton_Reload);
    LastButtonState[2] = Etat_bouton_Reload;
  }

  /* Mode RELOAD automatique si le curseur sort de l'écran, géré par logiciel */
  AutoReload = digitalRead(SwitchAutoReload);
  if (AutoReload == 0)  // Si le mode auto Reload est activé avec le switch AutoReload
  {
    digitalWrite(LedAutoReload, HIGH);  // Allume la led Autoreload

    if ((see0 == 0) || (see1 == 0) || (see2 == 0) || (see3 == 0))  // si on sort de l'écran
    {
      //Serial.println("Reload bouton !!!");
      setButton(2, HIGH); // pin 16
    }
    else
    {
      setButton(2, LOW); // pin 16
    }

  }
  else if (LastButtonState[2] == LOW)
  {
    //Serial.println("Mode Reload désactivé");
    digitalWrite(LedAutoReload, LOW);  // Éteind la led Autoreload
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

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//###################################
//# Gestion Boutons de Calibrations #
//###################################

void go() {    // Setup Start Calibration Button

  // ButtonState_calibration = (digitalRead(_startPin) | digitalRead(_tiggerPin));
  ButtonState_calibration = (digitalRead(_startPin) | digitalRead(_tiggerPin) | digitalRead(_reloadPin));
  if (ButtonState_calibration != LastButtonState_calibration) {
    if (ButtonState_calibration == LOW) {
      count--;
    }
    else { // do nothing
    }
    delay(50);
  }
  LastButtonState_calibration = ButtonState_calibration;
}

void mouseCount() {    // Procédure de calibration

  ButtonValid = digitalRead(_tiggerPin);
  ButtonPlus = digitalRead(_startPin);
  ButtonMinus = digitalRead(_reloadPin);

  if (ButtonValid != LastButtonValid) { // validation
    if (ButtonValid == LOW) {
      count--;
    }
    else {
    }
    delay(10);
  }

  if (ButtonPlus != LastButtonPlus) { // réglage +
    if (ButtonPlus == LOW) {
      plus = 1;
    }
    else {
      plus = 0;
    }
    delay(10);
  }

  if (ButtonMinus != LastButtonMinus) { // réglage -
    if (ButtonMinus == LOW) {
      minus = 1;
    }
    else {
      minus = 0;
    }
    delay(10);
  }

  LastButtonValid = ButtonValid;
  LastButtonPlus = ButtonPlus;
  LastButtonMinus = ButtonMinus;
}


void reset() {    // Pause/Re-calibrate button

  // ButtonState_calibration = (digitalRead(_startPin) | digitalRead(_tiggerPin));
  ButtonState_calibration = (digitalRead(_startPin) | digitalRead(_tiggerPin) | digitalRead(_reloadPin));
  if (ButtonState_calibration != LastButtonState_calibration) {
    if (ButtonState_calibration == LOW) {
      count = 4;
      delay(50);
    }
    else { // do nothing
    }
    delay(50);
  }
  LastButtonState_calibration = ButtonState_calibration;
}


void skip() {    // Unpause button

  //ButtonState_calibration = (digitalRead(_startPin) | digitalRead(_tiggerPin));
  ButtonState_calibration = (digitalRead(_startPin) | digitalRead(_tiggerPin) | digitalRead(_reloadPin));
  if (ButtonState_calibration != LastButtonState_calibration) {
    if (ButtonState_calibration == LOW) {
      count = 0;
      delay(50);
    }
    else { // do nothing
    }
    delay(50);
  }
  LastButtonState_calibration = ButtonState_calibration;
}


//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//##########################
//# Gestion Mémoire eeprom #
//##########################
void loadSettings() {     // Re-charger les valeurs de calibration depuis la mémoire EEPROM
  if (EEPROM.read(1023) == 'T') {
    //settings have been initialized, read them
    xCenter = EEPROM.read(0) + 256;
    yCenter = EEPROM.read(1) + 256;
    xOffset = EEPROM.read(2);
    yOffset = EEPROM.read(3);
  } else {
    //first time run, settings were never set
    EEPROM.write(0, xCenter - 256);
    EEPROM.write(1, yCenter - 256);
    EEPROM.write(2, xOffset);
    EEPROM.write(3, yOffset);
    EEPROM.write(1023, 'T');
  }
}

void PrintResults() {    // Print results for saving calibration
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
