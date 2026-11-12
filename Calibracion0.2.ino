#include <Wire.h>
#include <math.h>
/* =========================================================
   NEMA 17 + STM32F411 + 2 BTS7960 + AS5600
   CALIBRACION DE ANGULO ELECTRICO / OFFSET

   MOTOR:
       50 pares de polos

   RELACION:
       360° electricos = 7.2° mecanicos

   CABLEADO:

       BTS7960 A
       PA0 -> RPWM
       PA1 -> LPWM

       BTS7960 B
       PA6 -> RPWM
       PA7 -> LPWM

       AS5600
       PB8 -> SCL
       PB9 -> SDA

       R_EN y L_EN de ambos BTS7960 -> 5 V
       GND COMUN OBLIGATORIO
   ========================================================= */
/* =========================================================
   MOTOR
   ========================================================= */
#define POLE_PAIRS              50.0f
/*
   Vector de alineacion.
   45 grados hace trabajar las dos bobinas.
*/
#define ALIGN_ANGLE_E           45.0f


/* =========================================================
   POTENCIA PWM
   ========================================================= */

/*
   Empezamos en 20%.

   Se puede modificar desde el monitor serial:

       +  aumenta
       -  disminuye

   Rango:
       0.05 ... 0.35

   Sin sensor de corriente NO conviene superar
   estos valores durante pruebas largas.
*/
float testPower = 0.25f;
#define POWER_STEP              0.05f
#define POWER_MIN               0.05f
#define POWER_MAX               0.35f
/* =========================================================
   PWM
   ========================================================= */

#define PWM_MAX                 255
#define PWM_FREQUENCY           20000
/* =========================================================
   TIEMPOS
   ========================================================= */

#define ALIGN_TIME_MS           1200
#define DIRECTION_TEST_MS       900
#define VECTOR_SETTLE_MS        40
#define PHASE_TEST_MS           700

#define OFFSET_SAMPLES          100
/* =========================================================
   PINES
   ========================================================= */
/*
   BOBINA A
*/

#define A_RPWM                  PA0
#define A_LPWM                  PA1


/*
   BOBINA B
*/

#define B_RPWM                  PA6
#define B_LPWM                  PA7


/* =========================================================
   AS5600
   ========================================================= */

#define AS5600_ADDRESS          0x36

#define AS5600_ANGLE_H          0x0C
#define AS5600_ANGLE_L          0x0D


/* =========================================================
   VARIABLES DE CALIBRACION
   ========================================================= */

/*
   Este signo NO se supone.

   El programa lo determina automáticamente:

       +1  -> +theta_m genera +theta_e
       -1  -> +theta_m genera -theta_e
*/

float electricalSign = 0.0f;

float electricalOffset = 0.0f;

bool calibrated = false;


/* =========================================================
   UTILIDADES
   ========================================================= */

float wrap360(float angle)
{
  while (angle >= 360.0f)
    angle -= 360.0f;

  while (angle < 0.0f)
    angle += 360.0f;

  return angle;
}

/*
   Diferencia angular con cruce 0/360.

   Resultado:
       -180 ... +180
*/
float angleDiff(float a, float b)
{
  float d = a - b;

  while (d > 180.0f)
    d -= 360.0f;

  while (d < -180.0f)
    d += 360.0f;

  return d;
}

/* =========================================================
   AS5600
   ========================================================= */

uint16_t readAS5600Raw()
{
  Wire.beginTransmission(AS5600_ADDRESS);

  Wire.write(AS5600_ANGLE_H);

  if (Wire.endTransmission(false) != 0)
  {
    return 0xFFFF;
  }

  if (Wire.requestFrom(AS5600_ADDRESS, 2) != 2)
  {
    return 0xFFFF;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  uint16_t raw =
      ((uint16_t)highByte << 8) |
      lowByte;

  raw &= 0x0FFF;

  return raw;
}

float readAS5600()
{
  uint16_t raw = readAS5600Raw();

  if (raw == 0xFFFF)
    return NAN;

  return ((float)raw * 360.0f) / 4096.0f;
}
/* =========================================================
   PROMEDIO CIRCULAR
   ========================================================= */

float circularMeanDegrees(int samples)
{
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  int valid = 0;
  for (int i = 0; i < samples; i++)
  {
    float angle = readAS5600();

    if (!isnan(angle))
    {
      float rad = angle * DEG_TO_RAD;
      sumSin += sinf(rad);
      sumCos += cosf(rad);
      valid++;
    }
    delay(5);
  }
  if (valid == 0)
    return NAN;
  float meanRad =
      atan2f(sumSin, sumCos);
  return wrap360(meanRad * RAD_TO_DEG);
}
/* =========================================================
   FASE A
   ========================================================= */

void setPhaseA(float value)
{
  value = constrain(value, -1.0f, 1.0f);
  int pwm =
      (int)(fabsf(value) * PWM_MAX);

  if (value > 0.0f)
  {
    /*
       Primero apagar dirección opuesta.
    */
    analogWrite(A_LPWM, 0);
    analogWrite(A_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, pwm);
  }
  else
  {
    analogWrite(A_RPWM, 0);
    analogWrite(A_LPWM, 0);
  }
}
/* =========================================================
   FASE B
   ========================================================= */
void setPhaseB(float value)
{
  value = constrain(value, -1.0f, 1.0f);

  int pwm =
      (int)(fabsf(value) * PWM_MAX);

  if (value > 0.0f)
  {
    analogWrite(B_LPWM, 0);
    analogWrite(B_RPWM, pwm);
  }
  else if (value < 0.0f)
  {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, pwm);
  }
  else
  {
    analogWrite(B_RPWM, 0);
    analogWrite(B_LPWM, 0);
  }
}

/* =========================================================
   STOP
   ========================================================= */
void motorStop()
{
  setPhaseA(0.0f);
  setPhaseB(0.0f);
}

/* =========================================================
   VECTOR ELECTRICO
   ========================================================= */

/*
   Por ahora:
       A = cos(theta)
   El signo de B se controla con electricalSign.
   Durante la calibracion todavía no dependemos
   de un signo previamente conocido.
*/
void setElectricalVector(float thetaElectrical, float amplitude, float sign)
{
  thetaElectrical = wrap360(thetaElectrical);
  float thetaRad = thetaElectrical * DEG_TO_RAD;
  float phaseA = amplitude * cosf(thetaRad);
  float phaseB = sign *amplitude *sinf(thetaRad);
  setPhaseA(phaseA);
  setPhaseB(phaseB);
}
/* =========================================================
   PRUEBA A+
   ========================================================= */
void testPhaseA_Positive()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println(" FASE A POSITIVA");
  Serial.println("====================================");

  setPhaseA(testPower);
  setPhaseB(0.0f);

  delay(PHASE_TEST_MS);

  float angle = readAS5600();

  Serial.print("Potencia = ");
  Serial.println(testPower, 3);

  Serial.print("AS5600 = ");

  if (isnan(angle))
    Serial.println("ERROR");
  else
  {
    Serial.print(angle, 4);
    Serial.println(" deg");
  }

  motorStop();
}

/* =========================================================
   PRUEBA B+
   ========================================================= */
void testPhaseB_Positive()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println(" FASE B POSITIVA");
  Serial.println("====================================");

  setPhaseA(0.0f);
  setPhaseB(testPower);

  delay(PHASE_TEST_MS);

  float angle = readAS5600();

  Serial.print("Potencia = ");
  Serial.println(testPower, 3);

  Serial.print("AS5600 = ");

  if (isnan(angle))
    Serial.println("ERROR");
  else
  {
    Serial.print(angle, 4);
    Serial.println(" deg");
  }
  motorStop();
}
/* =========================================================
   PRUEBA A-
   ========================================================= */

void testPhaseA_Negative()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println(" FASE A NEGATIVA");
  Serial.println("====================================");

  setPhaseA(-testPower);
  setPhaseB(0.0f);

  delay(PHASE_TEST_MS);

  float angle = readAS5600();

  Serial.print("Potencia = ");
  Serial.println(testPower, 3);

  Serial.print("AS5600 = ");

  if (isnan(angle))
    Serial.println("ERROR");
  else
  {
    Serial.print(angle, 4);
    Serial.println(" deg");
  }
  motorStop();
}
/* =========================================================
   PRUEBA B-
   ========================================================= */
void testPhaseB_Negative()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println(" FASE B NEGATIVA");
  Serial.println("====================================");
  setPhaseA(0.0f);
  setPhaseB(-testPower);
  delay(PHASE_TEST_MS);
  float angle = readAS5600();
  Serial.print("Potencia = ");
  Serial.println(testPower, 3);
  Serial.print("AS5600 = ");
  if (isnan(angle))
    Serial.println("ERROR");
  else
  {
    Serial.print(angle, 4);
    Serial.println(" deg");
  }

  motorStop();
}
/* =========================================================
   TEST VECTOR 45°
   ========================================================= */
void testVector45()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println(" VECTOR 45 GRADOS");
  Serial.println("====================================");
  /*
     Probamos ambas polaridades posibles de B.
  */
  Serial.println();
  Serial.println("Vector +45 con SIGN +1");
  setElectricalVector(45.0f, testPower, +1.0f);
  delay(1000);
  float a1 = readAS5600();
  Serial.print("AS5600 = ");
  Serial.println(a1, 4);
  motorStop();
  delay(300);
  Serial.println();
  Serial.println("Vector +45 con SIGN -1");
  setElectricalVector(45.0f, testPower,-1.0f);
  delay(1000);
  float a2 = readAS5600();
  Serial.print("AS5600 = ");
  Serial.println(a2, 4);
  motorStop();
}

/* =========================================================
   CALIBRACION INTELIGENTE
   ========================================================= */

/*
   Esta rutina NO acepta un offset si el motor no se movió.
   PASO 1:
       Alinear en 45° eléctricos.
   PASO 2:
       Mover +90° eléctricos.
   PASO 3:
       Medir movimiento mecánico.
   Debemos obtener aproximadamente:
       +1.8°
       o
       -1.8°
   según la orientación física.
   Con esto determinamos automáticamente:
       electricalSign = +1
       o
       electricalSign = -1
   PASO 4:
       Regresar a 45°.
   PASO 5:
       Calcular offset.
*/
void calibrateFOC()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println(" CALIBRACION FOC INTELIGENTE");
  Serial.println("====================================");
  Serial.println();
  Serial.print("Pares de polos = ");
  Serial.println(POLE_PAIRS, 1);
  Serial.print("Potencia actual = ");
  Serial.println(testPower, 3);
  Serial.println();
  /*
     ---------------------------------------------------------
     PROBAR SIGNO +1
     ---------------------------------------------------------
  */
  Serial.println("PROBANDO SIGNO +1");
  /*
     Alinear a 45°.
  */
  setElectricalVector(
      ALIGN_ANGLE_E,
      testPower,
      +1.0f);
  delay(ALIGN_TIME_MS);
  float pos1 =
      circularMeanDegrees(OFFSET_SAMPLES);
  if (isnan(pos1))
  {
    Serial.println("ERROR AS5600");
    motorStop();
    return;
  }
  Serial.print("Posicion inicial = ");
  Serial.print(pos1, 4);
  Serial.println(" deg");
  /*
     Mover +90° eléctricos.
     45 -> 135
  */
  setElectricalVector(
      ALIGN_ANGLE_E + 90.0f,
      testPower,
      +1.0f);
  delay(DIRECTION_TEST_MS);
  float pos2 =
      circularMeanDegrees(OFFSET_SAMPLES);
  if (isnan(pos2))
  {
    Serial.println("ERROR AS5600");
    motorStop();
    return;
  }
  Serial.print("Posicion despues +90E = ");
  Serial.print(pos2, 4);
  Serial.println(" deg");
  float delta1 =
      angleDiff(pos2, pos1);
  Serial.print("Delta mecanico = ");
  Serial.print(delta1, 4);
  Serial.println(" deg");
  /*
     Esperamos aproximadamente ±1.8°.
  */
  if (fabsf(delta1) >= 0.5f)
  {
    /*
       Tenemos movimiento suficiente.
       Determinar signo.
    */
    electricalSign =
        (delta1 > 0.0f)
        ? +1.0f
        : -1.0f;

    Serial.println();
    Serial.println("MOVIMIENTO DETECTADO");
    Serial.print("SIGNO ELECTRICO = ");
    if (electricalSign > 0)
      Serial.println("+1");
    else
      Serial.println("-1");
  }
  else
  {
    /*
       No hubo movimiento suficiente.
       NO calcular offset.
    */
    Serial.println();
    Serial.println("****************************************");
    Serial.println("NO HAY MOVIMIENTO SUFICIENTE");
    Serial.println("NO SE CALCULA OFFSET");
    Serial.println("****************************************");
    Serial.println();
    Serial.println("Prueba aumentando la potencia con '+'");
    Serial.print("Potencia actual = ");
    Serial.println(testPower,3);
    Serial.println();
    motorStop();
    electricalSign = 0.0f;
    calibrated = false;
    return;
  }
  /*
     ---------------------------------------------------------
     VOLVER A 45°
     ---------------------------------------------------------
  */
  setElectricalVector(
      ALIGN_ANGLE_E,
      testPower,
      electricalSign);
  delay(ALIGN_TIME_MS);
  /*
     ---------------------------------------------------------
     LEER POSICION ALINEADA
     ---------------------------------------------------------
  */
  float thetaM =
      circularMeanDegrees(OFFSET_SAMPLES);
  if (isnan(thetaM))
  {
    Serial.println("ERROR AS5600");
    motorStop();
    return;
  }
  /*
     ---------------------------------------------------------
     CALCULAR OFFSET
     ---------------------------------------------------------
     theta_e_cmd =
         sign * P * theta_m
         + offset
     Para la alineacion:
         theta_e_cmd = 45°
     Por tanto:
         offset =
         45 - sign*P*theta_m
  */
  electricalOffset = wrap360(ALIGN_ANGLE_E - electricalSign * POLE_PAIRS * thetaM);
  /*
     ---------------------------------------------------------
     VERIFICAR
     ---------------------------------------------------------
  */

  float thetaCheck =wrap360(electricalSign * POLE_PAIRS * thetaM + electricalOffset);
  calibrated = true;
  /*
     ---------------------------------------------------------
     MOSTRAR
     ---------------------------------------------------------
  */
  Serial.println();
  Serial.println("====================================");
  Serial.println(" RESULTADO DE CALIBRACION");
  Serial.println("====================================");
  Serial.print("Theta mecanico = ");
  Serial.print(thetaM, 6);
  Serial.println(" deg");
  Serial.print("Signo electrico = ");

  if (electricalSign > 0)
    Serial.println("+1");
  else
    Serial.println("-1");
  Serial.print("OFFSET_FOC = ");
  Serial.print(electricalOffset, 6);
  Serial.println(" deg");
  Serial.print("Comprobacion theta_e = ");
  Serial.print(thetaCheck, 6);
  Serial.println(" deg");
  Serial.println();
  Serial.println("El rotor esta alineado en 45 grados electricos.");
  /*
     Mantener alineación.
  */
  setElectricalVector(
      ALIGN_ANGLE_E,
      testPower,
      electricalSign);
}
/* =========================================================
   CALCULO DE ANGULO ELECTRICO
   ========================================================= */
float getElectricalAngle()
{
  if (!calibrated)
    return NAN;

  float thetaM =
      readAS5600();

  if (isnan(thetaM))
    return NAN;

  float thetaE =
      electricalSign *
      POLE_PAIRS *
      thetaM
      + electricalOffset;

  return wrap360(thetaE);
}
/* =========================================================
   TEST 360 ELECTRICOS
   ========================================================= */
/*
   Con 50 pares:
       360 E = 7.2 M
*/
void test360Electrical()
{
  if (!calibrated)
  {
    Serial.println();
    Serial.println("Primero ejecuta C.");
    return;
  }
  Serial.println();
  Serial.println("====================================");
  Serial.println(" TEST 1 VUELTA ELECTRICA");
  Serial.println("====================================");
  /*
     Empezamos en el mismo ángulo
     utilizado durante calibración.
  */
  setElectricalVector(
      ALIGN_ANGLE_E,
      testPower,
      electricalSign);

  delay(ALIGN_TIME_MS);
  float start =
      circularMeanDegrees(OFFSET_SAMPLES);
  if (isnan(start))
  {
    Serial.println("ERROR AS5600");
    motorStop();
    return;
  }
  Serial.println();
  Serial.print("Inicial = ");
  Serial.print(start, 4);
  Serial.println(" deg");
  Serial.println();
  Serial.println("CMD_E\tMEC\tDELTA_M\tDELTA_E" );
  Serial.println("----------------------------------------");
  /*
     45 -> 405

     Esto equivale a:
         360 grados eléctricos
  */

  for (float thetaE = ALIGN_ANGLE_E;
       thetaE <= ALIGN_ANGLE_E + 360.0f;
       thetaE += 2.0f)
  {
    setElectricalVector(
        thetaE,
        testPower,
        electricalSign);

    delay(VECTOR_SETTLE_MS);

    float thetaM =
        readAS5600();

    if (isnan(thetaM))
    {
      Serial.println("ERROR AS5600");
      continue;
    }

    float deltaM =
        angleDiff(thetaM, start);
    /*
       Convertir la mecánica medida
       según el signo determinado.
    */

    float deltaE =
        electricalSign *
        deltaM *
        POLE_PAIRS;

    Serial.print(thetaE, 1);
    Serial.print("\t");

    Serial.print(thetaM, 4);
    Serial.print("\t");

    Serial.print(deltaM, 4);
    Serial.print("\t");

    Serial.println(deltaE, 2);
  }
  /*
     Mantener 45° al terminar.
     405° = 45°
  */
  setElectricalVector(
      ALIGN_ANGLE_E,
      testPower,
      electricalSign);
  delay(500);
  /*
     Posición final
  */

  float finalAngle =
      circularMeanDegrees(OFFSET_SAMPLES);

  if (isnan(finalAngle))
  {
    Serial.println("ERROR AS5600");
    motorStop();
    return;
  }

  /*
     Movimiento total
  */
  float totalMove =
      angleDiff(finalAngle, start);

  float magnitude =
      fabsf(totalMove);

  float expected =
      360.0f / POLE_PAIRS;

  float error =
      magnitude - expected;

  float errorPercent =
      (error / expected) * 100.0f;

  float electricalMeasured =
      electricalSign *
      totalMove *
      POLE_PAIRS;
  /*
     Resultado
  */
  Serial.println();

  Serial.println("====================================");
  Serial.println(" RESULTADO");
  Serial.println("====================================");
  Serial.print("Inicial = ");
  Serial.print(start, 4);
  Serial.println(" deg");

  Serial.print("Final   = ");
  Serial.print(finalAngle, 4);
  Serial.println(" deg");

  Serial.println();

  Serial.print("Delta mecanico = ");
  Serial.print(totalMove, 4);
  Serial.println(" deg");

  Serial.print("Magnitud       = ");
  Serial.print(magnitude, 4);
  Serial.println(" deg");

  Serial.print("Esperado       = ");
  Serial.print(expected, 4);
  Serial.println(" deg");

  Serial.print("Error          = ");
  Serial.print(error, 4);
  Serial.println(" deg");

  Serial.print("Error %        = ");
  Serial.print(errorPercent, 2);
  Serial.println(" %");

  Serial.print("Electrico medido = ");
  Serial.print(electricalMeasured, 2);
  Serial.println(" deg");

  Serial.println();

  if (fabsf(error) < 0.25f)
  {
    Serial.println("OK: desplazamiento cercano a 7.2 grados.");
  }
  else if (fabsf(error) < 0.50f)
  {
    Serial.println("AVISO: desplazamiento razonablemente cercano.");
  }
  else
  {
    Serial.println("AVISO: revisar potencia y seguimiento del rotor.");
  }
  Serial.println();
}
/* =========================================================
   MOSTRAR ANGULOS
   ========================================================= */
void showAngles()
{
  float thetaM =
      readAS5600();
  if (isnan(thetaM))
  {
    Serial.println("AS5600 ERROR");
    return;
  }
  Serial.print("MEC = ");
  Serial.print(thetaM, 4);
  Serial.print(" deg");
  if (calibrated)
  {
    float thetaE =
        getElectricalAngle();
    Serial.print(" | ELEC = ");
    Serial.print(thetaE, 3);
    Serial.print(" deg");
    Serial.print(" | SIGN = ");
    if (electricalSign > 0)
      Serial.print("+1");
    else
      Serial.print("-1");
    Serial.print(" | OFFSET = ");
    Serial.println(electricalOffset,6);
  }
  else
  {
    Serial.println( " | SIN CALIBRAR" );
  }
}
/* =========================================================
   CAMBIAR POTENCIA
   ========================================================= */
void increasePower()
{
  testPower += POWER_STEP;
  if (testPower > POWER_MAX)
    testPower = POWER_MAX;
  Serial.print("TEST_POWER = ");
  Serial.println(testPower,3);
}
void decreasePower()
{
  testPower -= POWER_STEP;
  if (testPower < POWER_MIN)
    testPower = POWER_MIN;
  Serial.print("TEST_POWER = ");
  Serial.println(testPower,3);
}
/* =========================================================
   SETUP
   ========================================================= */
void setup()
{
  Serial.begin(115200);
  delay(1500);
  /*
     ============================================
     GPIO
     ============================================
  */
  pinMode(A_RPWM, OUTPUT);
  pinMode(A_LPWM, OUTPUT);
  pinMode(B_RPWM, OUTPUT);
  pinMode(B_LPWM, OUTPUT);
  /*
     ============================================
     PWM
     ============================================
  */
  analogWriteResolution(8);
  analogWriteFrequency(
    PWM_FREQUENCY
  );
  /*
     Motor apagado
  */
  motorStop();
  /*
     ============================================
     I2C
     ============================================
     PB9 = SDA
     PB8 = SCL
  */
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Wire.setClock(400000);
  /*
     ============================================
     INFORMACIÓN
     ============================================
  */
  Serial.println();
  Serial.println("====================================");
  Serial.println(" STM32F411 BLACK PILL");
  Serial.println(" NEMA17 17HS8501");
  Serial.println(" 2 x BTS7960");
  Serial.println(" AS5600");
  Serial.println("====================================");
  Serial.println();
  Serial.print("POLE_PAIRS = ");
  Serial.println(POLE_PAIRS,1);
  Serial.print("TEST_POWER = ");
  Serial.println(testPower,3);
  Serial.print("PWM = " );
  Serial.print(PWM_FREQUENCY);
  Serial.println(" Hz");
  /*
     ============================================
     AS5600
     ============================================
  */
  float a =
      readAS5600();
  if (isnan(a))
  {
    Serial.println();
    Serial.println("ERROR: AS5600 NO RESPONDE");
  }
  else
  {
    Serial.println();
    Serial.print("AS5600 inicial = ");
    Serial.print(a,4);
    Serial.println(" deg");
  }
  /*
     ============================================
     COMANDOS
     ============================================
  */
  Serial.println();
  Serial.println("COMANDOS:");
  Serial.println();
  Serial.println("1 = Fase A positiva");
  Serial.println("2 = Fase B positiva");
  Serial.println("3 = Fase A negativa");
  Serial.println("4 = Fase B negativa");
  Serial.println("V = Vector de prueba 45 grados");
  Serial.println("C = Calibrar automaticamente");
  Serial.println("T = Test 360 grados electricos");
  Serial.println("A = Mostrar angulos");
  Serial.println("+ = Aumentar potencia");
  Serial.println("- = Disminuir potencia");
  Serial.println("S = Stop");
  Serial.println();
  motorStop();
}
/* =========================================================
   LOOP
   ========================================================= */
void loop()
{
  if (Serial.available())
  {
    char c =
        Serial.read();
    /*
       Fase A+
    */
    if (c == '1')
    {
      testPhaseA_Positive();
    }
    /*
       Fase B+
    */
    else if (c == '2')
    {
      testPhaseB_Positive();
    }
    /*
       Fase A-
    */
    else if (c == '3')
    {
      testPhaseA_Negative();
    }
    /*
       Fase B-
    */
    else if (c == '4')
    {
      testPhaseB_Negative();
    }
    /*
       Vector 45°
    */
    else if (c == 'V' ||
             c == 'v')
    {
      testVector45();
    }
    /*
       Calibración
    */
    else if (c == 'C' ||
             c == 'c')
    {
      calibrateFOC();
    }

    /*
       Test 360°
    */
    else if (c == 'T' || c == 't')
    {
      test360Electrical();
    }
    /*
       Mostrar ángulos
    */
    else if (c == 'A' ||
             c == 'a')
    {
      showAngles();
    }
    /*
       Aumentar potencia
    */
    else if (c == '+')
    {
      increasePower();
    }
    /*
       Disminuir potencia
    */
    else if (c == '-')
    {
      decreasePower();
    }
    /*
       STOP
    */
    else if (c == 'S' ||
             c == 's')
    {
      motorStop();
      Serial.println();
      Serial.println("MOTOR STOP");
      Serial.println();
    }
  }
}