#include <Arduino.h>
#include <math.h>

// ============================================================================
// UNIFIED DUAL-MODE ESP32 FIRMWARE FOR OMNIDIRECTIONAL ROBOT
// Compatible with both:
//   1. Mode 3-Element: <rawX, rawY, rawW>\n       (Digital Twin / ROS 2)
//   2. Mode 4-Element: <rawX, rawY, b6, b7>\n     (Joystick / Diagnostic)
//
// Physical Wheel Layout (Clockwise from Front-Right):
//   FR (2) ----> BR (4) ----> BL (3) ----> FL (1)
// ============================================================================

// Physical Motor Pins:
#define M1_PWM_R 15   // Motor 1: Front-Left (FL)
#define M1_PWM_L 2

#define M2_PWM_R 16   // Motor 2: Front-Right (FR)
#define M2_PWM_L 4

#define M3_PWM_R 18   // Motor 3: Back-Left (BL)
#define M3_PWM_L 19

#define M4_PWM_R 5    // Motor 4: Back-Right (BR)
#define M4_PWM_L 17

const int maxPWM = 255;
const double R_FACTOR = 7.6; // Scale factor for rotation vs translation

String inputString = "";
unsigned long lastPacketTime = 0;
bool isStopped = true;

void setMotor(int motor, int speed) {
    int pwmR = (speed > 0) ? speed : 0;
    int pwmL = (speed < 0) ? -speed : 0;
    if (pwmR > maxPWM) pwmR = maxPWM;
    if (pwmL > maxPWM) pwmL = maxPWM;

    switch (motor) {
        case 1: analogWrite(M1_PWM_R, pwmR); analogWrite(M1_PWM_L, pwmL); break;
        case 2: analogWrite(M2_PWM_R, pwmR); analogWrite(M2_PWM_L, pwmL); break;
        case 3: analogWrite(M3_PWM_R, pwmR); analogWrite(M3_PWM_L, pwmL); break;
        case 4: analogWrite(M4_PWM_R, pwmR); analogWrite(M4_PWM_L, pwmL); break;
    }
}

void stopAllMotors() {
    setMotor(1, 0);
    setMotor(2, 0);
    setMotor(3, 0);
    setMotor(4, 0);
    isStopped = true;
}

void Inverse_Kinematics(double Vx, double Vy, double W) {
    // Kinematika Mecanum / Omniwheel 4 Roda (X-Drive)
    // Vx = Strafe (Kanan = Positif, Kiri = Negatif)
    // Vy = Maju/Mundur (Maju = Positif, Mundur = Negatif)
    // W  = Rotasi (CCW/Putar Kiri = Positif, CW/Putar Kanan = Negatif)
    
    double M1 = Vy + Vx - R_FACTOR * W; // Front-Left  (FL)
    double M2 = Vy - Vx + R_FACTOR * W; // Front-Right (FR)
    double M3 = Vy - Vx - R_FACTOR * W; // Back-Left   (BL)
    double M4 = Vy + Vx + R_FACTOR * W; // Back-Right  (BR)

    // Normalisasi PWM jika melebihi batas 255
    double maxVal = fabs(M1);
    if (fabs(M2) > maxVal) maxVal = fabs(M2);
    if (fabs(M3) > maxVal) maxVal = fabs(M3);
    if (fabs(M4) > maxVal) maxVal = fabs(M4);

    if (maxVal > maxPWM) {
        M1 = (M1 / maxVal) * maxPWM;
        M2 = (M2 / maxVal) * maxPWM;
        M3 = (M3 / maxVal) * maxPWM;
        M4 = (M4 / maxVal) * maxPWM;
    }

    setMotor(1, (int)M1);
    setMotor(2, (int)M2);
    setMotor(3, (int)M3);
    setMotor(4, (int)M4);
    
    isStopped = (M1 == 0 && M2 == 0 && M3 == 0 && M4 == 0);
}

void setup() {
    Serial.begin(115200);

    pinMode(M1_PWM_R, OUTPUT); pinMode(M1_PWM_L, OUTPUT);
    pinMode(M2_PWM_R, OUTPUT); pinMode(M2_PWM_L, OUTPUT);
    pinMode(M3_PWM_R, OUTPUT); pinMode(M3_PWM_L, OUTPUT);
    pinMode(M4_PWM_R, OUTPUT); pinMode(M4_PWM_L, OUTPUT);

    stopAllMotors();
    lastPacketTime = millis();

    Serial.println("ESP32 Ready - Unified Dual-Mode Engine");
}

void loop() {
    // 1. Baca Serial Non-Blocking
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '<') {
            inputString = "";
        } else if (inChar == '>') {
            processPacket(inputString);
            inputString = "";
        } else {
            inputString += inChar;
        }
    }

    // 2. Failsafe Watchdog (Matikan motor jika tidak ada perintah > 500ms)
    if (!isStopped && (millis() - lastPacketTime > 500)) {
        stopAllMotors();
    }
}

void processPacket(String packet) {
    packet.trim();
    if (packet.length() == 0) return;

    lastPacketTime = millis();

    // Hitung jumlah koma untuk menentukan mode
    int commaCount = 0;
    for (size_t i = 0; i < packet.length(); i++) {
        if (packet[i] == ',') commaCount++;
    }

    // ────────────────────────────────────────────────────────────────────────
    // MODE 1: 4 ELEMEN <rawX, rawY, b6, b7> (GitHub / Controller)
    // ────────────────────────────────────────────────────────────────────────
    if (commaCount == 3) {
        int rawX, rawY, b6, b7;
        if (sscanf(packet.c_str(), "%d,%d,%d,%d", &rawX, &rawY, &b6, &b7) == 4) {
            
            // Mode Diagnostik: <999, motorID, speed, 0>
            if (rawX == 999) {
                int motorID = rawY;
                int speed = b6;
                setMotor(1, (motorID == 1) ? speed : 0);
                setMotor(2, (motorID == 2) ? speed : 0);
                setMotor(3, (motorID == 3) ? speed : 0);
                setMotor(4, (motorID == 4) ? speed : 0);
                isStopped = false;
                return;
            }

            // Mode Normal
            double Vx = (double)(rawX - 127); // Strafe
            double Vy = (double)(rawY - 127); // Forward
            
            if (fabs(Vx) < 8.0) Vx = 0.0;
            if (fabs(Vy) < 8.0) Vy = 0.0;

            double W = 0.0;
            if (b6 == 1) W = 5.0;       // Putar Kiri / CCW
            else if (b7 == 1) W = -5.0; // Putar Kanan / CW

            Inverse_Kinematics(Vx, Vy, W);
        }
    }
    // ────────────────────────────────────────────────────────────────────────
    // MODE 2: 3 ELEMEN <rawX, rawY, rawW> (Digital Twin / ROS 2 Analog)
    // ────────────────────────────────────────────────────────────────────────
    else if (commaCount == 2) {
        int rawX, rawY, rawW;
        if (sscanf(packet.c_str(), "%d,%d,%d", &rawX, &rawY, &rawW) == 3) {
            double Vx = (double)(rawX - 127); // Strafe
            double Vy = (double)(rawY - 127); // Forward
            double W  = (double)(rawW - 127) / 25.4; // Continuous Analog (-5.0 s/d +5.0)

            if (fabs(Vx) < 8.0) Vx = 0.0;
            if (fabs(Vy) < 8.0) Vy = 0.0;
            if (fabs(W)  < 0.3) W  = 0.0;

            Inverse_Kinematics(Vx, Vy, W);
        }
    }
}
