
void setup() {
    Serial.begin(115200);         // Debug USB
    Serial1.begin(115200, SERIAL_8N1, 0, 1);   // UART1 = pins 0 (RX) et 1 (TX)
}



int lireDistanceDTOF() {
    if (Serial1.available() < 9) return -1;
    // Chercher l’en-tête 0x59 0x59
    if (Serial1.read() != 0x59) return -1;
    if (Serial1.read() != 0x59) return -1;
    uint8_t distL = Serial1.read();
    uint8_t distH = Serial1.read();
    uint8_t strengthL = Serial1.read();
    uint8_t strengthH = Serial1.read();
    uint8_t tempL = Serial1.read();
    uint8_t tempH = Serial1.read();
    uint8_t checksum = Serial1.read();
    int distance = (distH << 8) | distL;
    return distance;
}



int distanceFiltree(int minDist) {
    int dist = lireDistanceDTOF();
    if (dist < 0) return -1;         
    if (dist < minDist) return -1;    
    return dist;
}



void loop() {
    int dist = distanceFiltree(100);   // veut un objet à minimum 100 mm
    if (dist = -1 ) {
        printf("STOP"); //cette ligne sert à gérer les arrets à remplacer par une alerte
    }
    delay(50); // cette ligne est là pour assurer le bon "echantilloanage du lidar"
}