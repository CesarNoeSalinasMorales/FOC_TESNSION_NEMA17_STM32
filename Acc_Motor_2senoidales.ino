#include <Arduino.h>
#include <math.h>

// =====================================================
// BTS7960 - BOBINA A
// =====================================================
#define A_RPWM PA0
#define A_LPWM PA1

// =====================================================
// BTS7960 - BOBINA B
// =====================================================
#define B_RPWM PA6
#define B_LPWM PA7

// =====================================================
// PARAMETROS DEL MOTOR
// =====================================================

float angulo = 0.0;

// Velocidad de giro del campo magnético
// Aumentar este valor = mayor velocidad
float velocidad = 1.0;

// Amplitud PWM
// 0 - 255
int amplitud = 230;

// =====================================================
// SETUP
// =====================================================

void setup() {

  pinMode(A_RPWM, OUTPUT);
  pinMode(A_LPWM, OUTPUT);

  pinMode(B_RPWM, OUTPUT);
  pinMode(B_LPWM, OUTPUT);

  // Inicializar salidas apagadas
  analogWrite(A_RPWM, 0);
  analogWrite(A_LPWM, 0);
  analogWrite(B_RPWM, 0);
  analogWrite(B_LPWM, 0);

  Serial.begin(115200);

  Serial.println("Motor bipolar - Lazo abierto");
  Serial.println("STM32F411 + 2 BTS7960");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // -----------------------------------------------
  // GENERAR LAS DOS SENOIDALES
  // -----------------------------------------------
  float senoA = sin(angulo);
  // 90 grados desfasada
  float senoB = cos(angulo);
  // -----------------------------------------------
  // CONVERTIR A PWM
  // -----------------------------------------------
  int pwmA = abs(senoA * amplitud);
  int pwmB = abs(senoB * amplitud);
  // -----------------------------------------------
  // BOBINA A
  // -----------------------------------------------

  if (senoA >= 0) {

    analogWrite(A_RPWM, pwmA);
    analogWrite(A_LPWM, 0);

  } else {

    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, pwmA);
  }

  // -----------------------------------------------
  // BOBINA B
  // -----------------------------------------------

  if (senoB >= 0) {

    analogWrite(B_RPWM, pwmB);
    analogWrite(B_LPWM, 0);

  } else {

    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, pwmB);
  }

  // -----------------------------------------------
  // AVANZAR ANGULO
  // -----------------------------------------------

  angulo += velocidad;

  if (angulo >= 2.0 * PI) {
    angulo -= 2.0 * PI;
  }

  delayMicroseconds(5000);
}