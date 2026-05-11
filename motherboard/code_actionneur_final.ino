#include <SoftwareSerial.h>
#include <SCServo.h>     
#include <Wire.h>
#include "Adafruit_MCP23X17.h"   

//  CONFIGURATION SERVOMOTEURS (STS3215)

SoftwareSerial UART(2, 2);   // ligne half‑duplex
SMS_STS sts;                 // objet de communication FT servo bus

//  CONFIGURATION ÉLECTROVANNES (Adafruit MCP23017)

Adafruit_MCP23X17 mcp;
const uint8_t VANNE_1 = 0; 
const uint8_t NUM_VANNES = 1;

//  STRUCTURE REPRESENTANT UN BRAS ROBOTIQUE

struct BrasRobotique {
  int idServoBase;    
  int idServoMain;    
  int idVanne;        
};
// Uart
BrasRobotique bras1 = {1, 2, 0};
BrasRobotique bras2 = {1, 2, 0};
BrasRobotique bras3 = {1, 2, 0};
BrasRobotique bras4 = {1, 2, 0};


void setup() {
  Serial.begin(115200);
  UART.begin(115200);
  sts.pSerial = &UART;
  if (!mcp.begin_I2C()) {
    while (1);
  }
  mcp.pinMode(VANNE_1, OUTPUT);
  mcp.digitalWrite(VANNE_1, LOW);
  Statut = "idle";
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
  sts.WritePos(id, pos, vitesse);
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
  if((bras = bras1) ||(bras = bras4) ){
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
    if((bras = bras1) ||(bras = bras4) ){
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
        Attraper(bras1);
        Attraper(bras2);
        Attraper(bras3);
        Attraper(bras4);
        Serial.println("DONE");
    }
    else {
        if (message.equalsIgnoreCase("DROP")) {
            Serial.println("dropping");
            Deposer(bras1);
            Deposer(bras2);
            Deposer(bras3);
            Deposer(bras4);
            Serial.println("DONE");
        }
        else{
            Serial.println("idle");        
        }
    }
}

String bufferUART = "";


void loop() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
            if (bufferUART.length() > 0) {
                traiterAlerte(bufferUART);
                bufferUART = "";
            }
        }
        else {
            bufferUART += c;
        }
    }
}
