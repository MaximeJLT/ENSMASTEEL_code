#define __FREERTOS 1

#include <Arduino.h>
#include <PicoSoftwareSerial.h>
#include <SCServo.h>     
#include <Wire.h>
#include "Adafruit_MCP23X17.h"

// ============================================================
//  CONFIGURATION
// ============================================================

SoftwareSerial SerialPompe(13, 12);
SoftwareSerial MonUART(0, 1);
SoftwareSerial UARTLidar(4, 5);
SMS_STS sts;

Adafruit_MCP23X17 mcp;

const int PUMP_PIN     = 15;
const int DEFAULT_SPEED = 200;

// Mutex natif Pico pour protéger Serial
mutex_t mutexSerial;

// ============================================================
//  STRUCTURE BRAS ROBOTIQUE
// ============================================================

struct BrasRobotique {
    int idbras;
    int idServoBase;
    int idServoMain;
    int idVanne;
};

BrasRobotique bras1 = {1, 1, 2, 1};
BrasRobotique bras2 = {2, 3, 4, 3};
BrasRobotique bras3 = {3, 5, 6, 2};
BrasRobotique bras4 = {4, 7, 8, 7};

// ============================================================
//  FONCTIONS POMPE
// ============================================================

void pump(int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(PUMP_PIN, speed);
}
void stopPump() { 
    analogWrite(PUMP_PIN, 0); 
}

// ============================================================
//  FONCTIONS ÉLECTROVANNES
// ============================================================

void ouvrirVanne(int vanne) { 
    mcp.digitalWrite(vanne, HIGH); 
}
void fermerVanne(int vanne) { 
    mcp.digitalWrite(vanne, LOW);  
}

// ============================================================
//  SERVOS INDIVIDUELS
// ============================================================

void allerA(int id, int pos, int vitesse = 800, int acc = 50) {
    sts.WritePosEx(id, pos, vitesse, acc);
}
void tournerServo(int id, int vitesse) { 
    sts.WriteSpe(id, vitesse); 
}
void arreterServo(int id){ 
    sts.WriteSpe(id, 0); 
}

// ============================================================
//  FONCTIONS BRAS
// ============================================================

void positionBras(BrasRobotique bras, int posBase, int posMain, int vitBase = 800, int vitMain = 800) {
    allerA(bras.idServoBase, posBase, vitBase);
    allerA(bras.idServoMain, posMain, vitMain);
}
void aspirer(BrasRobotique bras)  { 
    fermerVanne(bras.idVanne); 
}
void relacher(BrasRobotique bras) { 
    ouvrirVanne(bras.idVanne); 
}

// ============================================================
//  SÉQUENCES
// ============================================================

void Attraper(BrasRobotique bras) {
    if (bras.idbras == 1 || bras.idbras == 4) {
        positionBras(bras, 450, 450); delay(100);
        aspirer(bras);               delay(100);
        positionBras(bras, 300, 300); delay(100);
    } else {
        positionBras(bras, 450, 450); delay(100);
        aspirer(bras);               delay(100);
        positionBras(bras, 0, 0);    delay(100);
    }
}

void Deposer(BrasRobotique bras) {
    if (bras.idbras == 1 || bras.idbras == 4) {
        positionBras(bras, 450, 450); delay(100);
        relacher(bras);              delay(100);
        positionBras(bras, 300, 300); delay(100);
    } else {
        positionBras(bras, 450, 450); delay(100);
        relacher(bras);              delay(100);
        positionBras(bras, 0, 0);    delay(100);
    }
}

// Impression protégée par mutex
void serialPrint(const String &msg) {
    mutex_enter_blocking(&mutexSerial);
    Serial.println(msg);
    mutex_exit(&mutexSerial);
}

void traiterAlerte(String message) {
    message.trim();
    if (message.equalsIgnoreCase("PICK")) {
        serialPrint("picking");
        pump(DEFAULT_SPEED);
        Attraper(bras1); Attraper(bras2);
        Attraper(bras3); Attraper(bras4);
        delay(100);
        stopPump();
        serialPrint("DONE");
    } else if (message.equalsIgnoreCase("DROP")) {
        serialPrint("dropping");
        pump(DEFAULT_SPEED);
        Deposer(bras1); Deposer(bras2);
        Deposer(bras3); Deposer(bras4);
        delay(100);
        stopPump();
        serialPrint("DONE");
    } else {
        serialPrint("idle");
    }
}

// ============================================================
//  LIDAR
// ============================================================

bool lireMesure(int &angleDeg, int &distanceMm) {
    if (!UARTLidar.available()) return false;
    if (UARTLidar.read() != 0x54) return false;
    unsigned long t = millis();
    while (UARTLidar.available() < 11) {
        if (millis() - t > 100) return false;
    }
    uint8_t frame[11];
    for (int i = 0; i < 11; i++) frame[i] = UARTLidar.read();
    angleDeg   = (frame[4] << 8 | frame[3]) / 100;
    distanceMm =  frame[6] << 8 | frame[5];
    return true;
}

void detectionObjet(int seuil_mm) {
    int angleDeg, distMm;
    if (lireMesure(angleDeg, distMm)) {
        float angleRad = angleDeg * 3.141592653589793f / 180.0f;
        if (distMm > 0 && distMm < seuil_mm) {
            String msg = "OBST " + String(distMm) + " " + String(angleRad, 4);
            serialPrint(msg);
        } else {
            serialPrint("CLEAR");
        }
    }
}

// ============================================================
//  CŒUR 1 — Lidar en parallèle (loop1)
// ============================================================

void setup1() {
    // Rien à initialiser spécifiquement sur le cœur 1
}

void loop1() {
    detectionObjet(300);   // tourne en permanence sur le cœur 1
}

// ============================================================
//  CŒUR 0 — UART / Servos (setup + loop classiques)
// ============================================================

void setup() {
    Serial.begin(115200);
    MonUART.begin(1000000);
    UARTLidar.begin(230400);
    sts.pSerial = &MonUART;

    if (!mcp.begin_I2C()) { while (1); }
    for (int i = 0; i < 8; i++) {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }

    mutex_init(&mutexSerial);   // initialisation du mutex natif Pico
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
        } else {
            buffer += c;
        }
    }
}