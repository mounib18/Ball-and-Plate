//===========================================================================
// projet : ball and plate
// auteur : Bouhdid Mounib
// ecole  : INRACI Bruxelles - 6eme electronique
// description : maintien d une bille au centre d une plaque inclinable
//               via un regulateur PID et une dalle resistive 4 fils
//
// composants utilises :
//   - M5Stack Core (ESP32)
//   - 2 servos Miuzei 25kg 270° (GPIO 12 axe X, GPIO 16 axe Y)
//   - dalle resistive 4 fils 322x247mm (GPIO 2, 5, 25, 26)
//   - 2 telemetres IR Sharp GP2Y0A21YK0F (GPIO 35, 36)
//   - joystick M5Stack Unit Joystick2 I2C (SDA GPIO21, SCL GPIO22)
//===========================================================================

#include <M5Stack.h>             // bibliotheque principale du M5Stack (ecran, boutons)
#include "m5_unit_joystick2.hpp" // bibliotheque du joystick I2C M5Stack
#include <ESP32Servo.h>          // bibliotheque servo adaptee a l ESP32 (timers differents de l Arduino classique)
#include <Wire.h>                // communication I2C pour le joystick (SDA GPIO21, SCL GPIO22)
#include <math.h>                // fonctions mathematiques (powf pour la conversion ADC des telemetres IR)

// =========================
// SERVOS
// =========================
Servo servoX;  
Servo servoY;  

int pinServoX = 12;  
int pinServoY = 16;  


int usMin    = 1220; // impulsion minimale en microsecondes
int usCentre = 1520; // impulsion centre en microsecondes - plaque horizontale
int usMax    = 1820; // impulsion maximale en microsecondes

int impulseServoX = 1520;
int impulseServoY = 1520; 

// =========================
// DALLE RESISTIVE 4 FILS
// =========================
int positionX = 0;  
int positionY = 0;  


int consigneX = 2100;
int consigneY = 1900;

int erreurX = 0;  // difference entre position actuelle et consigne sur X
int erreurY = 0;  // difference entre position actuelle et consigne sur Y

// =========================
// GAINS PID
// =========================

float Kp = 0.08;  
float Ki = 0.015; 
float Kd = 0.6;   

float termeP_X = 0, termeI_X = 0, termeD_X = 0; // termes PID axe X (affiches dans le moniteur serie)
float termeP_Y = 0, termeI_Y = 0, termeD_Y = 0; // termes PID axe Y (affiches dans le moniteur serie)

float sommeErreurX = 0; // accumulateur pour le terme integral axe X
float sommeErreurY = 0; // accumulateur pour le terme integral axe Y

int erreurPrecedenteX = 0; // memoire de l erreur precedente pour calculer la derivee axe X
int erreurPrecedenteY = 0; // memoire de l erreur precedente pour calculer la derivee axe Y

// =========================
// TELEMETRES IR
// =========================
// capteurs Sharp GP2Y0A21YK0F
int pinIR1 = 35; 
int pinIR2 = 36; 

// =========================
// JOYSTICK
// =========================
M5UnitJoystick2 joystick2; 
uint16_t joyX = 0;          // valeur brute axe X du joystick (0 a 65535, 16 bits via I2C)
uint16_t joyY = 0;          // valeur brute axe Y du joystick (0 a 65535, 16 bits via I2C)

// =========================
// MODE DE FONCTIONNEMENT
// =========================
// 0 = regulation PID automatique (bouton A)
// 1 = mode debug avec affichage ecran (bouton B)
// 2 = controle manuel par joystick (bouton C)
int mode = 0;

// on l utilise pour executer certaines actions a intervalle fixe sans bloquer le reste du code
unsigned long tempoMilis     = 0; // reference de temps pour l envoi servo (toutes les 10ms)
unsigned long tempoAffichage = 0; // reference de temps pour l affichage debug (toutes les 500ms)

// drapeau qui memorise si on etait sans bille au cycle precedent
// permet de detecter le premier retour de la bille pour rallumer les servos proprement
// et evite de repeter le message "pas de contact" en boucle dans le moniteur serie
bool memoirePrint = false;

// =========================
// CONVERSION ADC -> CM (telemetres IR)
// =========================
float adcVersCm(int adc) {
  float v  = adc * (3.3f / 4095.0f); // conversion valeur ADC 12 bits -> tension en volts
  float cm = 29.988f * powf(v, -1.173f); // loi de puissance du datasheet
  return cm;
}

// =========================
// LECTURE AXE X DE LA DALLE
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
// LECTURE AXE Y DE LA DALLE
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
// DETECTION DE CONTACT
// =========================
bool contactPresent(int x, int y) {
  return (x > 150 && y > 350);
}

// =========================
// CORRECTEUR PID - AXE X
// =========================
void PID_X() {
  // calcul de l erreur : ecart entre position reelle et consigne
  erreurX = positionX - consigneX;

  // terme integral : accumulation de l erreur dans le temps
  sommeErreurX += erreurX;

  // anti-windup : limite l accumulation pour eviter la saturation de l integrateur
  // sans ca l integrale peut devenir enorme si la bille est bloquee longtemps
  sommeErreurX = constrain(sommeErreurX, -5000, 5000);
  // zone morte : si la bille est proche du centre on remet l integrale a zero
  // evite les micro-oscillations dues au bruit quand la bille est stabilisee
  if (abs(erreurX) < 100) {
    sommeErreurX = 0;
  }

  // terme derive : variation de l erreur entre deux cycles
  // permet d anticiper le mouvement et d amortir les depassements
  int deltaErreurX  = erreurX - erreurPrecedenteX;
  erreurPrecedenteX = erreurX; // memorisation pour le prochain cycle

  // calcul des trois termes PID
  termeP_X = erreurX      * Kp; 
  termeI_X = sommeErreurX * Ki;     
  termeD_X = deltaErreurX * Kd;        


  impulseServoX = usCentre - (int)(termeP_X + termeI_X + termeD_X);
  // securite : on ne depasse jamais les limites mecaniques du servo
  impulseServoX = constrain(impulseServoX, usMin, usMax);
}

// =========================
// CORRECTEUR PID - AXE Y
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

void afficherDebug() {
  float d1 = adcVersCm(analogRead(pinIR1)); // lecture et conversion telemetre IR1 en cm
  float d2 = adcVersCm(analogRead(pinIR2)); // lecture et conversion telemetre IR2 en cm
  joystick2.get_joy_adc_16bits_value_xy(&joyX, &joyY); // lecture joystick

  M5.Lcd.setTextSize(2);

  M5.Lcd.setCursor(0, 30);  M5.Lcd.printf("IR1 : %.1f cm   ", d1);
  M5.Lcd.setCursor(0, 50);  M5.Lcd.printf("IR2 : %.1f cm   ", d2);
  M5.Lcd.setCursor(0, 80);  M5.Lcd.printf("SX  : %d us   ", impulseServoX);
  M5.Lcd.setCursor(0, 100); M5.Lcd.printf("SY  : %d us   ", impulseServoY);
  M5.Lcd.setCursor(0, 130); M5.Lcd.printf("JX  : %u   ", joyX);
  M5.Lcd.setCursor(0, 150); M5.Lcd.printf("JY  : %u   ", joyY);
  M5.Lcd.setCursor(0, 180); M5.Lcd.printf("X   : %d   ", positionX);
  M5.Lcd.setCursor(0, 200); M5.Lcd.printf("Y   : %d   ", positionY);

  // map convertit la position de la bille (0 a 4095) en pixels ecran (211 a 308)
// constrain force le cercle a rester dans le carre meme aux bords extremes de la dalle
  int bx = constrain(map(positionX, 0, 4095, 308, 211), 211, 308);
  int by = constrain(map(positionY, 0, 4095, 31,  128), 31,  128);

  M5.Lcd.fillRect(211, 31, 98, 98, BLACK);

  if (contactPresent(positionX, positionY)) {
    M5.Lcd.fillCircle(bx, by, 4, WHITE); // dessine la bille a sa nouvelle position
  }
}


void setup() {
  M5.begin();         // initialisation du M5Stack (ecran, boutons, I2C)
  Serial.begin(9600); // moniteur serie pour le debug

  // configuration et centrage des servos au demarrage
  servoX.setPeriodHertz(100);             // frequence PWM 100Hz pour servo analogique
  servoX.attach(pinServoX, usMin, usMax); // attachement du servo X sur GPIO 12
  servoX.writeMicroseconds(usCentre);     // position centrale au demarrage

  servoY.setPeriodHertz(100);
  servoY.attach(pinServoY, usMin, usMax);
  servoY.writeMicroseconds(usCentre);

  // configuration ADC pour tous les capteurs analogiques (dalle resistive + telemetres IR)
  analogReadResolution(12);                  // resolution 12 bits - valeurs de 0 a 4095
  analogSetPinAttenuation(pinIR1, ADC_11db); 
  analogSetPinAttenuation(pinIR2, ADC_11db); 
  // note : le joystick est en I2C, il n est pas concerne par ces reglages

  // initialisation du joystick I2C
  joystick2.begin(&Wire, JOYSTICK2_ADDR, 21, 22); // adresse 0x63, SDA GPIO21, SCL GPIO22
  joystick2.set_rgb_color(0x00ff00);               // LED verte = systeme pret

  // affichage du menu de demarrage
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);   M5.Lcd.println("BALL AND PLATE");
  M5.Lcd.setCursor(0, 40);  M5.Lcd.println("A : REGULATION");
  M5.Lcd.setCursor(0, 70);  M5.Lcd.println("B : DEBUG");
  M5.Lcd.setCursor(0, 100); M5.Lcd.println("C : JOYSTICK");

  tempoMilis     = millis(); // initialisation du chrono servo
  tempoAffichage = millis(); // initialisation du chrono affichage
}

// =========================
// LOOP - boucle principale repetee en permanence
// =========================
void loop() {
  M5.update(); 
  // --- GESTION DES BOUTONS ---
  if (M5.BtnA.wasPressed()) {
    mode = 0; // passage en mode regulation PID
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 100); M5.Lcd.println("  REGULATION");
    M5.Lcd.setCursor(0, 130); M5.Lcd.println("  EN COURS...");
    Serial.println("MODE REGULATION");
  }
  if (M5.BtnB.wasPressed()) {
    mode = 1; // passage en mode debug
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);    M5.Lcd.println("BALL AND PLATE");
    M5.Lcd.setCursor(215, 10); M5.Lcd.println("BILLE");
    M5.Lcd.drawRect(210, 30, 100, 100, WHITE); // cadre dessine une seule fois ici
    Serial.println("MODE DEBUG");
  }
  if (M5.BtnC.wasPressed()) {
    mode = 2; // passage en mode joystick manuel
    servoX.attach(pinServoX, usMin, usMax); // re-attachement des servos si detaches
    servoY.attach(pinServoY, usMin, usMax);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 100); M5.Lcd.println("  JOYSTICK");
    M5.Lcd.setCursor(0, 130); M5.Lcd.println("  MANUEL");
    Serial.println("MODE JOYSTICK");
  }

  // --- MODE JOYSTICK MANUEL ---
  // le return en fin de bloc empeche le PID de tourner en mode joystick
  if (mode == 2) {
    joystick2.get_joy_adc_16bits_value_xy(&joyX, &joyY); // lecture joystick 16 bits (0 a 65535)
    // conversion de la plage joystick vers la plage servo en microsecondes
    impulseServoX = map(joyX, 0, 65535, usMin, usMax);
    impulseServoY = map(joyY, 0, 65535, usMin, usMax);
    servoX.writeMicroseconds(impulseServoX);
    servoY.writeMicroseconds(impulseServoY);
    return; // sort du loop  le PID ne tourne pas en mode joystick
  }

  // --- LECTURE DE LA DALLE RESISTIVE ---
  // a ce stade on est forcement en mode 0 ou 1 (mode 2 est sorti avec return)
  // les deux modes ont besoin de la position donc on lit une seule fois ici
  positionX = lireX();
  positionY = lireY();

  if (!contactPresent(positionX, positionY)) {
    // pas de bille detectee
    servoX.detach();
    servoY.detach();

    sommeErreurX      = 0;
    sommeErreurY      = 0;
    erreurPrecedenteX = 0;
    erreurPrecedenteY = 0;
    // memoirePrint evite de repeter ce message en boucle dans le moniteur serie
    if (!memoirePrint) // ! = si PAS de bille
      Serial.println("pas de contact - servos libres");
    memoirePrint = true;
  } else {
    // bille presente
    if (memoirePrint) {
      // on re-attache les servos et on initialise l erreur precedente
      // pour eviter le pic derive (terme D enorme) au premier calcul PID
      servoX.attach(pinServoX, usMin, usMax);
      servoY.attach(pinServoY, usMin, usMax);
      erreurPrecedenteX = positionX - consigneX;
      erreurPrecedenteY = positionY - consigneY;
    }
    memoirePrint = false;

    // calcul PID sur les deux axes
    // tourne en mode 0 et en mode 1
    PID_X();
    PID_Y();

    // envoi des commandes servo toutes les 10ms
    // millis() = chronometre en ms depuis le demarrage
    // on attend 10ms entre chaque envoi pour ne pas surcharger les servos
    if (millis() >= tempoMilis + 10) {
      tempoMilis = millis(); // remet le chrono a zero
      servoX.writeMicroseconds(impulseServoX);
      servoY.writeMicroseconds(impulseServoY);
      // affichage moniteur serie : position, erreur, termes PID, impulsion servo
      Serial.printf("X=%4d errX=%5d P=%6.1f I=%6.1f D=%6.1f impX=%5d\n",
                    positionX, erreurX, termeP_X, termeI_X, termeD_X, impulseServoX);
      Serial.printf("Y=%4d errY=%5d P=%6.1f I=%6.1f D=%6.1f impY=%5d\n",
                    positionY, erreurY, termeP_Y, termeI_Y, termeD_Y, impulseServoY);
    }
  }

  // rafraichit l ecran debug toutes les 500ms
  // le && verifie les deux conditions en meme temps :
  // on est en mode debug ET 500ms se sont ecoulees depuis le dernier affichage
  if (mode == 1 && millis() >= tempoAffichage + 500) {
    tempoAffichage = millis(); // remet le chrono affichage a zero
    afficherDebug();
  }
}