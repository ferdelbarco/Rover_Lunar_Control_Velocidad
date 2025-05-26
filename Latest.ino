#include <SoftPWM.h>

// Pines de control de motores
const int IN1 = 2;  // Izquierda adelante
const int IN2 = 3;  // Izquierda atrás
const int IN3 = 4;  // Derecha adelante
const int IN4 = 5;  // Derecha atrás

// Pines sensores Hall
const int hallRightPin = 7;
const int hallLeftPin  = 8;

// Estado anterior de los sensores
bool lastRightState = LOW;
bool lastLeftState  = LOW;

// Variables velocidad derecha
unsigned int pulsesRight = 0;
unsigned long prevTimeRight = 0;
float rpmRight = 0;

// Variables velocidad izquierda
unsigned int pulsesLeft = 0;
unsigned long prevTimeLeft = 0;
float rpmLeft = 0;

// PWM a probar
const int pwmValue = 150;

void setup() {
  // Inicializar PWM software
  SoftPWMBegin();
  SoftPWMSetFadeTime(ALL, 0, 0);

  SoftPWMSet(IN1, 0);
  SoftPWMSet(IN2, 0);
  SoftPWMSet(IN3, 0);
  SoftPWMSet(IN4, 0);

  pinMode(hallRightPin, INPUT_PULLUP);
  pinMode(hallLeftPin, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Tiempo(ms),PWM,RPM_Derecha,RPM_Izquierda");

  // Dirección hacia adelante
  SoftPWMSet(IN1, pwmValue);  // Izquierda adelante
  SoftPWMSet(IN2, 0);
  SoftPWMSet(IN3, 0);  // Derecha adelante
  SoftPWMSet(IN4, pwmValue);

  prevTimeRight = millis();
  prevTimeLeft = millis();
}

void loop() {
  unsigned long now = millis();

  // Leer estado actual
  bool currentRight = digitalRead(hallRightPin);
  bool currentLeft = digitalRead(hallLeftPin);

  // Detectar flanco RISING en sensor derecho
  if (currentRight == HIGH && lastRightState == LOW) {
    unsigned long dtRight = now - prevTimeRight;
    rpmRight = (dtRight > 0) ? (60000.0 / dtRight) : 0;
    prevTimeRight = now;
    pulsesRight++;
  }

  // Detectar flanco RISING en sensor izquierdo
  if (currentLeft == HIGH && lastLeftState == LOW) {
    unsigned long dtLeft = now - prevTimeLeft;
    rpmLeft = (dtLeft > 0) ? (60000.0 / dtLeft) : 0;
    prevTimeLeft = now;
    pulsesLeft++;
  }

  lastRightState = currentRight;
  lastLeftState = currentLeft;

  // Imprimir cada 50 ms
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 50) {
    lastPrint = now;
    Serial.print(now);
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(rpmRight, 2);
    Serial.print(",");
    Serial.println(rpmLeft, 2);
  }
}
