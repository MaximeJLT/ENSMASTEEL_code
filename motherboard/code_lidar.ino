
#include <PicoSoftwareSerial.h>
SoftwareSerial lidar(0, 1); 

void setup() {
    Serial.begin(115200);
    lidar.begin(230400);     
}


bool lireMesure(int &angleDeg, int &distanceMm) {
    if (!lidar.available()) return false;
    if (lidar.read() != 0x54) return false;

    // Attendre les 11 octets restants
    unsigned long debut = millis();
    while (lidar.available() < 11) {
        if (millis() - debut > 100) return false; 
    }

    uint8_t frame[11];
    for (int i = 0; i < 11; i++)
        frame[i] = lidar.read();

    int angleRaw    = (frame[4] << 8 | frame[3]);
    angleDeg        = angleRaw / 100;         
    distanceMm      = (frame[6] << 8 | frame[5]);
    return true;
}


void detectionObjet(int seuil_mm) {
    int angleDeg, distMm;
    if (lireMesure(angleDeg, distMm)) {
        float angleRad = angleDeg * 3.141592653589793f / 180.0f;
        if (distMm > 0 && distMm < seuil_mm) {
            Serial.print("OBST ");
            Serial.print(distMm);
            Serial.print(" ");
            Serial.print(angleRad, 4);
            Serial.println();
        } else {
            Serial.println("CLEAR");
        }
    }
}


void loop() {
    detectionObjet(300);  // seuil = 300 mm
}
