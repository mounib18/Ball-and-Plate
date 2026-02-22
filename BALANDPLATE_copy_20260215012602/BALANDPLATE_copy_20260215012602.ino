
//===========================================================================
// projet : ball and plate
// auteur : Bouhdid Mounib
// date de la derniere modification : 22/02/26
//===========================================================================

#include <M5Stack.h>              // Ecran + boutons + init M5
#include "m5_unit_joystick2.hpp"  // Librairie du joystick M5 (I2C)

#include <ESP32Servo.h>  // Servomoteurs (attach(pin), write(angle))
#include <math.h>        // powf() pour la formule télémètre (ADC -> cm)
#include <Wire.h>        // I2C (joystick SDA=21 / SCL=22)


int modeActuel = 0;

Servo servo1;
Servo servo2;
int pinServo1 = 12;
int pinServo2 = 16;

int pinIR1 = 35;
int pinIR2 = 36;

int x, y;
int px, py;
M5UnitJoystick2 joystick2;




// =========================
// CONVERSION ADC -> CM
// =========================
float adcVersCm(int adc) {
  float v = adc * (3.3f / 4095.0f);
  float cm = 29.988f * powf(v, -1.173f);
  return cm;
}

// =========================
// AFFICHAGE TITRE
// =========================
void ecranTitre(string titre) {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println(titre);
}

// =========================
// LECTURE DALLE
// =========================
void lireDalle() {
  // Lecture axe X
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  digitalWrite(2, HIGH);
  digitalWrite(5, LOW);
  delay(5);
  x = analogRead(25);

  // Lecture axe Y
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(2, INPUT);
  pinMode(5, INPUT);
  digitalWrite(25, HIGH);
  digitalWrite(26, LOW);
  delay(5);
  y = analogRead(2);

  // Conversion en pixels
  px = map(x, 0, 4095, 0, 320);
  py = map(y, 0, 4095, 0, 240);
}

// =========================
// MODE 0 : DALLE
// =========================
void modeDalle() {
  lireDalle();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("MODE : DALLE");

  if ((x > 150) && (y > 350)) {
    M5.Lcd.fillCircle(px, py, 5, WHITE);
  }

  M5.Lcd.setCursor(0, 210);           // fixer un curseur d'ecriture
  M5.Lcd.printf("x=%d  y=%d", x, y);  // afficher les valeurs de x et y
}

// =========================
// MODE 1 : SERVOS
// =========================
void modeServos() {
  servo1.write(80);
  servo2.write(80);
  delay(600);

  servo1.write(100);
  servo2.write(100);
  delay(600);
}

// =========================
// MODE 2 : TELEMETRES + JOYSTICK
// =========================
void modeTelemetres() {
  int adc1 = analogRead(pinIR1);
  int adc2 = analogRead(pinIR2);

  float d1 = adcVersCm(adc1);
  float d2 = adcVersCm(adc2);

  uint16_t joyX = 0, joyY = 0;
  joystick2.get_joy_adc_16bits_value_xy(&joyX, &joyY);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("MODE : TELEMETRES + JOY");
/*"IR1 : %.1f cm" = le texte modèle

%.1f = un emplacement réservé pour un nombre décimal (f = float)

.1 = 1 chiffre après la virgule

d1 = la valeur qui va remplacer %.1f*/
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf("IR1 : %.1f cm", d1);

  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("IR2 : %.1f cm", d2);

  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf("JoyX: %u", joyX);

  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("JoyY: %u", joyY);

  delay(200);
}

// =========================
// SETUP
// =========================
void setup() {
  M5.begin();
  Serial.begin(9600);

  // Servos
  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  servo1.write(90);
  servo2.write(90);

  // ADC telemetres
  analogReadResolution(12);
  analogSetPinAttenuation(pinIR1, ADC_11db);
  analogSetPinAttenuation(pinIR2, ADC_11db);

  // Joystick
  joystick2.begin(&Wire, JOYSTICK2_ADDR, 21, 22);
  joystick2.set_rgb_color(0x00ff00);

  ecranTitre("MODE : DALLE");
}

// =========================
// LOOP
// =========================
void loop() {
  M5.update();

  // A = DALLE
  if (M5.BtnA.wasPressed()) {
    modeActuel = 0;
    ecranTitre("MODE : DALLE");
  }

  // B = SERVOS
  if (M5.BtnB.wasPressed()) {
    modeActuel = 1;
    ecranTitre("MODE : SERVOS");
  }

  // C = TELEMETRES + JOYSTICK
  if (M5.BtnC.wasPressed()) {
    modeActuel = 2;
    ecranTitre("MODE : TELEMETRES + JOY");
  }

  // Execution du mode
  if (modeActuel == 0) {
    modeDalle();
  } else if (modeActuel == 1) {
    modeServos();
  } else if (modeActuel == 2) {
    modeTelemetres();
  }
}