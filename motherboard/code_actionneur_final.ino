
#include <PicoSoftwareSerial.h>
#include <SCServo.h>     
#include <Wire.h>
#include "Adafruit_MCP23X17.h"   

//  CONFIGURATION SERVOMOTEURS (STS3215)

SoftwareSerial SerialPompe(13, 12);
SoftwareSerial MonUART(2, 2);   // ligne half‑duplex
SMS_STS sts;                 // objet de communication FT servo bus

//  CONFIGURATION ÉLECTROVANNES (Adafruit MCP23017)

Adafruit_MCP23X17 mcp;
const uint8_t VANNE_1 = 0; 
const uint8_t NUM_VANNES = 1;

const int PUMP_PIN = 15;   // GP15 – compatible PWM sur Pico

// --- Vitesse par défaut (0–255, Arduino analogWrite) ---
const int DEFAULT_SPEED = 200;

//  STRUCTURE REPRESENTANT UN BRAS ROBOTIQUE

struct BrasRobotique {
  int idbras;
  int idServoBase;    
  int idServoMain;    
  int idVanne;        
};
// Uart
BrasRobotique bras1 = {1,0, 1, 2};
BrasRobotique bras2 = {2,3, 4, 5};
BrasRobotique bras3 = {3,7, 8, 9};
BrasRobotique bras4 = {4,10, 11, 12};


void setup() {
  Serial.begin(115200);
  MonUART.begin(115200);
  SerialPompe.begin(9600);
  sts.pSerial = &MonUART;
  if (!mcp.begin_I2C()) {
    while (1);
  }
  mcp.pinMode(VANNE_1, OUTPUT);
  mcp.digitalWrite(VANNE_1, LOW);

}

// FONCTIONS POMPE

void pump(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(PUMP_PIN, speed);
  SerialPompe.println(speed);
}

void stopPump() {
  analogWrite(PUMP_PIN, 0);
}

//  FONCTIONS : ELECTROVANNE

void ouvrirVanne(int vanne) {
  mcp.digitalWrite(vanne, HIGH);
}
void fermerVanne(int vanne) {
  mcp.digitalWrite(vanne, LOW);
}

//  SERVOS INDIVIDUELS

void allerA(int id, int pos, int vitesse = 800) {
  sts.WritePosEx(id, pos, vitesse);
}
void tournerServo(int id, int vitesse) {
  sts.WriteSpe(id, vitesse); 
}
void arreterServo(int id) {
  sts.WriteSpe(id, 0);
}

//  FONCTIONS : BRAS ROBOTIQUE


void tournerBras(BrasRobotique bras, int vitesse) {
  tournerServo(bras.idServoBase, vitesse);
}


void tournerMain(BrasRobotique bras, int vitesse) {
  tournerServo(bras.idServoMain, vitesse);
}

// Arrêter tous les mouvements
void arreterBras(BrasRobotique bras) {
  arreterServo(bras.idServoBase);
  arreterServo(bras.idServoMain);
}

// Placer le bras dans une position coordonnée
void positionBras(BrasRobotique bras, int posBase, int posMain, int vitBase = 800, int vitMain = 800) {
  allerA(bras.idServoBase, posBase, vitBase);
  allerA(bras.idServoMain, posMain, vitMain);
}

void aspirer(BrasRobotique bras) {
  fermerVanne(bras.idVanne);
}

void relacher(BrasRobotique bras) {
  ouvrirVanne(bras.idVanne);
}




//  SEQUENCES 


void Attraper(BrasRobotique bras) {
  if((bras.idbras = 1) ||(bras.idbras = 4) ){
        positionBras(bras, 450, 450);
        delay(100);
        aspirer(bras);
        delay(100);
        positionBras(bras, 300, 300);
        delay(100);
    }
    else{
        positionBras(bras, 450, 450);
        delay(100);
        aspirer(bras);
        delay(100);
        positionBras(bras, 0, 0);
        delay(100);
    }
}

// Déposer un objet
void Deposer(BrasRobotique bras) {
    if((bras.idbras = 1) ||(bras.idbras = 4) ){
        positionBras(bras, 450, 450);
        delay(100);
        relacher(bras);
        delay(100);
        positionBras(bras, 300, 300);
        delay(100);
    }
    else{
        positionBras(bras, 450, 450);
        delay(100);
        relacher(bras);
        delay(100);
        positionBras(bras, 0, 0);
        delay(100);
    }
}

void traiterAlerte(String message) {
    message.trim();   // on nettoie
    // Exemple : on attend le texte "ALERT"
    if (message.equalsIgnoreCase("PICK")) {
        Serial.println("picking");
        pump(DEFAULT_SPEED);
        Attraper(bras1);
        Attraper(bras2);
        Attraper(bras3);
        Attraper(bras4);
        stopPump();
        Serial.println("DONE");
    }
    else {
        if (message.equalsIgnoreCase("DROP")) {
            pump(DEFAULT_SPEED);
            Serial.println("dropping");
            Deposer(bras1);
            Deposer(bras2);
            Deposer(bras3);
            Deposer(bras4);
            stopPump();
            Serial.println("DONE");
        }
        else{
            Serial.println("idle");        
        }
    }
}

String buffer = "";


void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (buffer.length() > 0) {
                traiterAlerte(buffer);
                buffer = "";
            }
        }
        else {
            buffer += c;
        }
    }
}


