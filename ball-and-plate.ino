//===========================================================================
// projet : ball and plate
// auteur : Bouhdid Mounib
// version finale - PID + mode debug + joystick manuel
//===========================================================================

#include <M5Stack.h>
#include "m5_unit_joystick2.hpp"
#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>

// =========================
// SERVOS
// =========================
Servo servoX;
Servo servoY;

int pinServoX = 12;
int pinServoY = 16;

int usMin    = 1220;
int usCentre = 1520;
int usMax    = 1820;

int impulseServoX = 1520;
int impulseServoY = 1520;

// =========================
// DALLE RESISTIVE
// =========================
int positionX = 0;
int positionY = 0;

int consigneX = 1999;
int consigneY = 1999;

int erreurX = 0;
int erreurY = 0;

// =========================
// GAINS PID
// =========================
float Kp = 0.08;
float Ki = 0.015;
float Kd = 0.6;

float termeP_X = 0, termeI_X = 0, termeD_X = 0;
float termeP_Y = 0, termeI_Y = 0, termeD_Y = 0;

float sommeErreurX = 0;
float sommeErreurY = 0;

int erreurPrecedenteX = 0;
int erreurPrecedenteY = 0;

// =========================
// TELEMETRES IR
// =========================
int pinIR1 = 35;
int pinIR2 = 36;

// =========================
// JOYSTICK
// =========================
M5UnitJoystick2 joystick2;
uint16_t joyX = 0;
uint16_t joyY = 0;

// =========================
// MODE
// 0 = regulation pure   (bouton A)
// 1 = debug + affichage (bouton B)
// 2 = joystick manuel   (bouton C)
// =========================
int mode = 0;

// =========================
// TEMPOS
// =========================
unsigned long tempoMilis     = 0;
unsigned long tempoAffichage = 0;
bool memoirePrint = false;

// =========================
// CONVERSION ADC -> CM
// =========================
float adcVersCm(int adc) {
  float v  = adc * (3.3f / 4095.0f);
  float cm = 29.988f * powf(v, -1.173f);
  return cm;
}

// =========================
// LECTURE AXE X
// =========================
int lireX() {
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  digitalWrite(2, HIGH);
  digitalWrite(5, LOW);
  delay(5);
  return analogRead(25);
}

// =========================
// LECTURE AXE Y
// =========================
int lireY() {
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(2, INPUT);
  pinMode(5, INPUT);
  digitalWrite(25, HIGH);
  digitalWrite(26, LOW);
  delay(5);
  return analogRead(2);
}

// =========================
// DETECTION CONTACT
// =========================
bool contactPresent(int x, int y) {
  return (x > 150 && y > 350);
}

// =========================
// CORRECTION PID - AXE X
// =========================
void PID_X() {
  erreurX = positionX - consigneX;

  sommeErreurX += erreurX;
  sommeErreurX = constrain(sommeErreurX, -5000, 5000);
  if (abs(erreurX) < 100) {
    sommeErreurX = 0;
  }

  int deltaErreurX  = erreurX - erreurPrecedenteX;
  erreurPrecedenteX = erreurX;

  termeP_X = erreurX      * Kp;
  termeI_X = sommeErreurX * Ki;
  termeD_X = deltaErreurX * Kd;

  impulseServoX = usCentre - (int)(termeP_X + termeI_X + termeD_X);
  impulseServoX = constrain(impulseServoX, usMin, usMax);
}

// =========================
// CORRECTION PID - AXE Y
// =========================
void PID_Y() {
  erreurY = positionY - consigneY;

  sommeErreurY += erreurY;
  sommeErreurY = constrain(sommeErreurY, -5000, 5000);
  if (abs(erreurY) < 100) {
    sommeErreurY = 0;
  }

  int deltaErreurY  = erreurY - erreurPrecedenteY;
  erreurPrecedenteY = erreurY;

  termeP_Y = erreurY      * Kp;
  termeI_Y = sommeErreurY * Ki;
  termeD_Y = deltaErreurY * Kd;

  // signe + car l axe Y est inverse mecaniquement
  impulseServoY = usCentre + (int)(termeP_Y + termeI_Y + termeD_Y);
  impulseServoY = constrain(impulseServoY, usMin, usMax);
}

// =========================
// AFFICHAGE MODE DEBUG
// =========================
void afficherDebug(float d1, float d2) {
  M5.Lcd.setTextSize(2);

  M5.Lcd.setCursor(0, 30);  M5.Lcd.printf("IR1 : %.1f cm   ", d1);
  M5.Lcd.setCursor(0, 50);  M5.Lcd.printf("IR2 : %.1f cm   ", d2);
  M5.Lcd.setCursor(0, 80);  M5.Lcd.printf("SX  : %d us   ", impulseServoX);
  M5.Lcd.setCursor(0, 100); M5.Lcd.printf("SY  : %d us   ", impulseServoY);
  M5.Lcd.setCursor(0, 130); M5.Lcd.printf("JX  : %u   ", joyX);
  M5.Lcd.setCursor(0, 150); M5.Lcd.printf("JY  : %u   ", joyY);
  M5.Lcd.setCursor(0, 180); M5.Lcd.printf("X   : %d   ", positionX);
  M5.Lcd.setCursor(0, 200); M5.Lcd.printf("Y   : %d   ", positionY);

  static int bxPrecedent = 260;
  static int byPrecedent = 80;

  int bx = constrain(map(positionX, 0, 4095, 308, 211), 211, 308);
  int by = constrain(map(positionY, 0, 4095, 31,  128), 31,  128);

  M5.Lcd.fillCircle(bxPrecedent, byPrecedent, 4, BLACK);
  M5.Lcd.drawRect(210, 30, 100, 100, WHITE);

  if (contactPresent(positionX, positionY)) {
    M5.Lcd.fillCircle(bx, by, 4, WHITE);
    bxPrecedent = bx;
    byPrecedent = by;
  }
}

// =========================
// SETUP
// =========================
void setup() {
  M5.begin();
  Serial.begin(9600);

  servoX.setPeriodHertz(100);
  servoX.attach(pinServoX, usMin, usMax);
  servoX.writeMicroseconds(usCentre);

  servoY.setPeriodHertz(100);
  servoY.attach(pinServoY, usMin, usMax);
  servoY.writeMicroseconds(usCentre);

  analogReadResolution(12);
  analogSetPinAttenuation(pinIR1, ADC_11db);
  analogSetPinAttenuation(pinIR2, ADC_11db);

  joystick2.begin(&Wire, JOYSTICK2_ADDR, 21, 22);
  joystick2.set_rgb_color(0x00ff00);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);   M5.Lcd.println("BALL AND PLATE");
  M5.Lcd.setCursor(0, 40);  M5.Lcd.println("A : REGULATION");
  M5.Lcd.setCursor(0, 70);  M5.Lcd.println("B : DEBUG");
  M5.Lcd.setCursor(0, 100); M5.Lcd.println("C : JOYSTICK");

  tempoMilis     = millis();
  tempoAffichage = millis();
}

// =========================
// LOOP
// =========================
void loop() {
  M5.update();

  // gestion des boutons
  if (M5.BtnA.wasPressed()) {
    mode = 0;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 100); M5.Lcd.println("  REGULATION");
    M5.Lcd.setCursor(0, 130); M5.Lcd.println("  EN COURS...");
    Serial.println("MODE REGULATION");
  }

  if (M5.BtnB.wasPressed()) {
    mode = 1;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);    M5.Lcd.println("BALL AND PLATE");
    M5.Lcd.setCursor(215, 10); M5.Lcd.println("BILLE");
    M5.Lcd.drawRect(210, 30, 100, 100, WHITE);
    Serial.println("MODE DEBUG");
  }

  if (M5.BtnC.wasPressed()) {
    mode = 2;
    servoX.attach(pinServoX, usMin, usMax);
    servoY.attach(pinServoY, usMin, usMax);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 100); M5.Lcd.println("  JOYSTICK");
    M5.Lcd.setCursor(0, 130); M5.Lcd.println("  MANUEL");
    Serial.println("MODE JOYSTICK");
  }

  // mode joystick manuel
  if (mode == 2) {
    joystick2.get_joy_adc_16bits_value_xy(&joyX, &joyY);
    impulseServoX = map(joyX, 0, 65535, usMin, usMax);
    impulseServoY = map(joyY, 0, 65535, usMin, usMax);
    servoX.writeMicroseconds(impulseServoX);
    servoY.writeMicroseconds(impulseServoY);
    Serial.printf("JX=%u  JY=%u  SX=%d  SY=%d\n", joyX, joyY, impulseServoX, impulseServoY);
    return;
  }

  // lectures dalle
  positionX = lireX();
  positionY = lireY();

  if (!contactPresent(positionX, positionY)) {
    // pas de bille : servos detaches et PID remis a zero
    servoX.detach();
    servoY.detach();
    sommeErreurX      = 0;
    sommeErreurY      = 0;
    erreurPrecedenteX = 0;
    erreurPrecedenteY = 0;
    if (!memoirePrint)
      Serial.println("pas de contact - servos libres");
    memoirePrint = true;
  } else {
    if (memoirePrint) {
      servoX.attach(pinServoX, usMin, usMax);
      servoY.attach(pinServoY, usMin, usMax);
      // initialise l erreur precedente pour eviter le pic D au premier passage
      erreurPrecedenteX = positionX - consigneX;
      erreurPrecedenteY = positionY - consigneY;
    }
    memoirePrint = false;

    PID_X();
    PID_Y();

    if (millis() >= tempoMilis + 10) {
      tempoMilis = millis();
      servoX.writeMicroseconds(impulseServoX);
      servoY.writeMicroseconds(impulseServoY);
      Serial.printf("X=%4d errX=%5d P=%6.1f I=%6.1f D=%6.1f impX=%5d\n",
                    positionX, erreurX, termeP_X, termeI_X, termeD_X, impulseServoX);
      Serial.printf("Y=%4d errY=%5d P=%6.1f I=%6.1f D=%6.1f impY=%5d\n",
                    positionY, erreurY, termeP_Y, termeI_Y, termeD_Y, impulseServoY);
    }
  }

  // affichage debug toutes les 500ms
  if (mode == 1 && millis() >= tempoAffichage + 500) {
    tempoAffichage = millis();
    float d1 = adcVersCm(analogRead(pinIR1));
    float d2 = adcVersCm(analogRead(pinIR2));
    joystick2.get_joy_adc_16bits_value_xy(&joyX, &joyY);
    afficherDebug(d1, d2);
  }
} //je teste voir si ca marche 