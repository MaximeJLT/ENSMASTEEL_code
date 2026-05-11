
#include <Arduino.h>
void setup() {
    Serial.begin(115200); // Debug USB
    Serial1.begin(230400, SERIAL_8N1, 0, 1);  // UART1 : RX=0  TX=1
}


bool lireMesure(int &angleDeg, int &distanceMm) {
    if (!Serial1.available()) return false;
    if (Serial1.read() != 0x54) return false;
    while (Serial1.available() < 11);
    uint8_t frame[11];
    for (int i = 0; i < 11; i++)
        frame[i] = Serial1.read();
    int angleRaw = (frame[4] << 8 | frame[3]);
    angleDeg = angleRaw / 100;  // -> 0–359°
    distanceMm = (frame[6] << 8 | frame[5]);
    return true;
}


void detectionObjet(int seuil_mm) {
    int angleDeg, distMm;
    if (lireMesure(angleDeg, distMm)) {
        // Conversion degrés → radians
        float angleRad = angleDeg * 3.141592653589793f / 180.0f;
        if (distMm > 0 && distMm < seuil_mm) {
            Serial.print("OBST ");
            Serial.print(distMm);
            Serial.print(" ");
            Serial.print(angleRad, 4);
            Serial.println("");
        }
        else {
            Serial.println("CLEAR");
        }
    }
}


void loop() {
    detectionObjet(300);   // seuil = 300 mm
}
