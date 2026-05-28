/*
 * ============================================================
 *  BALL & BEAM — v8.0
 *  BASE: v7.0 (bloque de control sin modificar)
 *  AGREGADO desde v7.1:
 *  - Pantalla LCD 16x2 I2C
 *  - Sensor de referencia dinámico para setpoint
 *  - Calibración automática de setpoint (15 segundos)
 *  - Rango válido: 3-26 cm para ambos sensores
 * ============================================================
 */

#include <NewPing.h>
#include <AccelStepper.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─────────────────────────────────────────────
//  CONFIGURACIÓN LCD I2C
// ─────────────────────────────────────────────
const byte LCD_ADDR = 0x27;
const byte LCD_COLS = 16;
const byte LCD_ROWS = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ─────────────────────────────────────────────
//  *** PARÁMETROS AJUSTABLES ***
// ─────────────────────────────────────────────
double SET_POINT_CM   = 13.0;      // ahora no es const — se actualiza con sensor ref
const int MAX_TILT_STEPS =  500;
const int MIN_TILT_STEPS =  450;

// ── Ganancias PID — INTACTAS de v7.0 ─────────
const double KP = 90.0;
const double KI =  0.6;
const double KD = 45.0;

// ── Ajustes de comportamiento — INTACTOS de v7.0
const double ZONA_MUERTA_CM = 0.8;
const double INTEGRAL_LIMIT = 100.0;
const double ALPHA_D        = 0.4;

// ── Rangos válidos de sensor ─────────────────
const double SENSOR_MIN_CM = 3.0;
const double SENSOR_MAX_CM = 26.0;

// ─────────────────────────────────────────────
//  PINES
// ─────────────────────────────────────────────
const int STEP_PIN   =  3;
const int DIR_PIN    =  2;
const int ENABLE_PIN = 12;
const int M0_PIN     =  7;
const int M1_PIN     =  9;
const int M2_PIN     = 10;
const int TRIG_PIN   =  5;   // sensor bola
const int ECHO_PIN   =  6;
const int TRIG_REF   = 11;   // sensor referencia setpoint
const int ECHO_REF   = 13;
#define MAX_DIST 200

// ─────────────────────────────────────────────
//  MICROSTEPPING 1/32 — INTACTO de v7.0
// ─────────────────────────────────────────────
const int STEP_MODE = 5;
const int STEP_MODES[6][4] = {
  {1,  0, 0, 0},
  {2,  1, 0, 0},
  {4,  0, 1, 0},
  {8,  1, 1, 0},
  {16, 0, 0, 1},
  {32, 1, 1, 1}
};

// ─────────────────────────────────────────────
//  OBJETOS
// ─────────────────────────────────────────────
NewPing sensor(TRIG_PIN, ECHO_PIN, MAX_DIST);
NewPing sensorRef(TRIG_REF, ECHO_REF, MAX_DIST);

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

SimpleKalmanFilter kalman(0.03, 0.03, 0.5);       // bola — INTACTO de v7.0
SimpleKalmanFilter kalmanRef(0.05, 0.05, 0.8);    // referencia

// ─────────────────────────────────────────────
//  VARIABLES PID — INTACTAS de v7.0
// ─────────────────────────────────────────────
double ball_position  = SET_POINT_CM;
double radiansPerStep = 0.0;
bool   sensorListo    = false;

double integral   = 0.0;
double prevError  = 0.0;
double derivFilt  = 0.0;

double accionP = 0.0;
double accionI = 0.0;
double accionD = 0.0;
double output  = 0.0;

const unsigned long CONTROL_MS = 50;
const unsigned long SERIAL_MS  = 100;
const unsigned long LCD_MS     = 200;
unsigned long lastControlMs    = 0;
unsigned long lastSerialMs     = 0;
unsigned long lastLcdMs        = 0;

// ─────────────────────────────────────────────
//  VARIABLES CALIBRACIÓN SETPOINT
// ─────────────────────────────────────────────
bool setpoint_calibrado = false;
const unsigned long TIEMPO_CALIBRACION_MS = 15000;

// ─────────────────────────────────────────────
//  PROTOTIPOS
// ─────────────────────────────────────────────
void stepMove(long target, unsigned int delayUs);
double leerSensorCm();
double leerSensorRefCm();
void actualizarLCD();

// ─────────────────────────────────────────────
//  HELPER: sensor bola — INTACTO de v7.0
// ─────────────────────────────────────────────
double leerSensorCm() {
  unsigned int us = sensor.ping();
  if (us == 0) return -1.0;
  double d_cm = (double)us / 58.0;
  if (d_cm < 2.0 || d_cm > 60.0) return -1.0;
  return d_cm;
}

// ─────────────────────────────────────────────
//  HELPER: sensor referencia setpoint
// ─────────────────────────────────────────────
double leerSensorRefCm() {
  unsigned int us = sensorRef.ping();
  if (us == 0) return -1.0;
  double d_cm = (double)us / 58.0;
  if (d_cm < SENSOR_MIN_CM || d_cm > SENSOR_MAX_CM) return -1.0;
  return d_cm;
}

// ─────────────────────────────────────────────
//  LCD: fila 0 = SP y posición | fila 1 = error
// ─────────────────────────────────────────────
void actualizarLCD() {
  double err = SET_POINT_CM - ball_position;

  lcd.setCursor(0, 0);
  lcd.print("SP:");
  lcd.print(SET_POINT_CM, 1);
  lcd.print(" Po:");
  lcd.print(ball_position, 1);
  lcd.print("  ");   // limpiar caracteres residuales

  lcd.setCursor(0, 1);
  lcd.print("Err:");
  if (err >= 0) lcd.print(" ");   // alinear signo
  lcd.print(err, 2);
  lcd.print(" cm     ");
}

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // ── LCD ──────────────────────────────────────
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Ball & Beam v8.0");
  lcd.setCursor(0, 1); lcd.print("Inicializando...");
  Serial.println("[INIT] LCD OK");

  // ── Pines ────────────────────────────────────
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(M0_PIN,     OUTPUT);
  pinMode(M1_PIN,     OUTPUT);
  pinMode(M2_PIN,     OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(M0_PIN, STEP_MODES[STEP_MODE][1]);
  digitalWrite(M1_PIN, STEP_MODES[STEP_MODE][2]);
  digitalWrite(M2_PIN, STEP_MODES[STEP_MODE][3]);

  double degreesPerStep = 360.0 / (200.0 * STEP_MODES[STEP_MODE][0]);
  radiansPerStep = degreesPerStep * (PI / 180.0);

  stepper.setMaxSpeed(50000);
  stepper.setAcceleration(25000);
  stepper.setMinPulseWidth(5);

  // ── Calentar sensor bola — INTACTO de v7.0 ───
  Serial.println("[INIT] Calentando sensor...");
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Sensor posicion");
  lcd.setCursor(0, 1); lcd.print("calentando...");

  for (int i = 0; i < 15; i++) {
    delay(50);
    double d = leerSensorCm();
    if (d > 0.0) {
      if (!sensorListo) {
        ball_position = d;
        kalman.updateEstimate(d);
        sensorListo = true;
      } else {
        ball_position = kalman.updateEstimate(d);
      }
    }
    delay(150);
  }
  Serial.print("[INIT] Bola en: ");
  Serial.print(ball_position, 2);
  Serial.println(" cm");

  // ── Calibración mecánica — INTACTA de v7.0 ───
  Serial.println("[CAL] Pon la viga HORIZONTAL. 3 segundos...");
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("CALIBRACION");
  lcd.setCursor(0, 1); lcd.print("Viga HORIZONTAL");
  delay(3000);

  digitalWrite(ENABLE_PIN, LOW);
  stepper.setCurrentPosition(0);

  Serial.print("[CAL] Subiendo MAX (+");
  Serial.print(MAX_TILT_STEPS); Serial.println(")...");
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Cal: MAX");
  stepMove(MAX_TILT_STEPS, 300);
  Serial.println("[CAL] 3s..."); delay(3000);

  Serial.println("[CAL] Centro...");
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Cal: CENTRO");
  stepMove(0, 300);
  Serial.println("[CAL] 3s..."); delay(3000);

  Serial.print("[CAL] Bajando MIN (-");
  Serial.print(MIN_TILT_STEPS); Serial.println(")...");
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Cal: MIN");
  stepMove(-MIN_TILT_STEPS, 300);
  Serial.println("[CAL] 3s..."); delay(3000);

  Serial.println("[CAL] Centro...");
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Cal: CENTRO");
  stepMove(0, 300);
  stepper.setCurrentPosition(0);

  // ── Calibración setpoint con sensor referencia ─
  Serial.println("[SETPOINT] Calibrando 15 seg...");
  Serial.println("[SETPOINT] Acerca tu mano al sensor REFERENCIA");
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("SETPOINT");
  lcd.setCursor(0, 1); lcd.print("Acerca la mano");
  delay(2000);

  unsigned long t_cal_start = millis();
  while ((millis() - t_cal_start) < TIEMPO_CALIBRACION_MS) {
    double cm_ref = leerSensorRefCm();
    if (cm_ref > 0.0) {
      SET_POINT_CM = 0.8 * SET_POINT_CM + 0.2 * cm_ref;
      setpoint_calibrado = true;
      Serial.print("[SETPOINT] ");
      Serial.print(cm_ref, 2);
      Serial.print(" cm → SP: ");
      Serial.print(SET_POINT_CM, 2);
      Serial.println(" cm");
    }

    // LCD: mostrar SP actual y tiempo restante
    unsigned int seg = (TIEMPO_CALIBRACION_MS - (millis() - t_cal_start)) / 1000;
    lcd.setCursor(0, 0); lcd.print("SP:");
    lcd.print(SET_POINT_CM, 1); lcd.print(" cm    ");
    lcd.setCursor(0, 1);
    if (seg < 10) lcd.print("0");
    lcd.print(seg); lcd.print("s restantes  ");

    delay(200);
  }

  if (!setpoint_calibrado) {
    Serial.println("[SETPOINT] Sin lectura. Usando 13.0 cm");
    SET_POINT_CM = 13.0;
  }
  Serial.print("[SETPOINT] Final: "); Serial.print(SET_POINT_CM, 2); Serial.println(" cm");

  // ── Reset limpio — INTACTO de v7.0 ───────────
  integral      = 0.0;
  prevError     = 0.0;
  derivFilt     = 0.0;
  output        = 0.0;
  lastControlMs = millis();
  lastSerialMs  = millis();
  lastLcdMs     = millis();

  Serial.println("[INIT] Control activo.");
  Serial.println("─────────────────────────────────────────────");
  Serial.print  ("  SP="); Serial.print(SET_POINT_CM, 2);
  Serial.print  ("  KP="); Serial.print(KP);
  Serial.print  ("  KI="); Serial.print(KI, 2);
  Serial.print  ("  KD="); Serial.println(KD);
  Serial.print  ("  Zona muerta: +-"); Serial.print(ZONA_MUERTA_CM); Serial.println(" cm");
  Serial.print  ("  Integral limit: +-"); Serial.println(INTEGRAL_LIMIT);
  Serial.print  ("  Alpha D: "); Serial.println(ALPHA_D);
  Serial.println("─────────────────────────────────────────────");
  Serial.println("SP Ball Err P I D U Ang Pos");

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Control ACTIVO");
  lcd.setCursor(0, 1); lcd.print("SP:"); lcd.print(SET_POINT_CM, 1);
  delay(2000);
  lcd.clear();
}

// ═════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═════════════════════════════════════════════
void loop() {
  unsigned long nowMs = millis();

  // ══════════════════════════════════════════════
  //  BLOQUE DE CONTROL — INTACTO de v7.0
  //  No se modificó ninguna línea de esta sección
  // ══════════════════════════════════════════════
  if (nowMs - lastControlMs >= CONTROL_MS) {
    double dt = (double)(nowMs - lastControlMs) / 1000.0;
    lastControlMs = nowMs;

    double d = leerSensorCm();
    if (d > 0.0) {
      ball_position = kalman.updateEstimate(d);
    }

    double h     = SET_POINT_CM - ball_position;
    double a     = stepper.currentPosition() * radiansPerStep;
    double error = h * cos(a);

    if (fabs(error) < ZONA_MUERTA_CM) error = 0.0;

    accionP = KP * error;

    double integralCandidate = integral + error * dt;
    double outputEstimate    = -(KP * error + KI * integralCandidate);
    bool saturado    = (outputEstimate >  (double)MAX_TILT_STEPS) ||
                       (outputEstimate < -(double)MAX_TILT_STEPS);
    bool mismaSennal = (outputEstimate > 0 && error < 0) ||
                       (outputEstimate < 0 && error > 0);
    if (!(saturado && mismaSennal)) integral = integralCandidate;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    accionI  = KI * integral;

    double derivRaw = (error - prevError) / dt;
    derivFilt = ALPHA_D * derivRaw + (1.0 - ALPHA_D) * derivFilt;
    accionD   = KD * derivFilt;
    prevError = error;

    output = -(accionP + accionI + accionD);
    output = constrain(output,
                      -(double)MAX_TILT_STEPS,
                       (double)MAX_TILT_STEPS);

    stepper.moveTo((long)output);
  }
  // ══════════════════════════════════════════════
  //  FIN BLOQUE DE CONTROL
  // ══════════════════════════════════════════════

  stepper.run();

  // ── LCD cada 200 ms ───────────────────────────
  if (nowMs - lastLcdMs >= LCD_MS) {
    lastLcdMs = nowMs;
    actualizarLCD();
  }

  // ── Telemetría cada 100 ms — INTACTA de v7.0 ─
  if (nowMs - lastSerialMs >= SERIAL_MS) {
    lastSerialMs = nowMs;
    double angDeg = stepper.currentPosition()
                  * radiansPerStep * (180.0 / PI);

    Serial.print("SP:");    Serial.print(SET_POINT_CM, 1);
    Serial.print(" Ball:"); Serial.print(ball_position, 2);
    Serial.print(" Err:");  Serial.print(SET_POINT_CM - ball_position, 2);
    Serial.print(" P:");    Serial.print(accionP, 1);
    Serial.print(" I:");    Serial.print(accionI, 2);
    Serial.print(" D:");    Serial.print(accionD, 1);
    Serial.print(" U:");    Serial.print(output, 1);
    Serial.print(" Ang:");  Serial.print(angDeg, 1);
    Serial.print(" Pos:");  Serial.println(stepper.currentPosition());
  }
}

// ═════════════════════════════════════════════
//  STEP MOVE — INTACTO de v7.0
// ═════════════════════════════════════════════
void stepMove(long target, unsigned int delayUs) {
  if (target > 0) target = constrain(target, 0L,  (long)MAX_TILT_STEPS);
  else            target = constrain(target, -(long)MIN_TILT_STEPS, 0L);

  long pos = stepper.currentPosition();
  while (pos != target) {
    bool dir = (target > pos);
    if ( dir && pos >=  (long)MAX_TILT_STEPS) break;
    if (!dir && pos <= -(long)MIN_TILT_STEPS) break;

    digitalWrite(DIR_PIN, dir ? HIGH : LOW);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayUs);

    pos += dir ? 1 : -1;
    stepper.setCurrentPosition(pos);
  }

  double ang = stepper.currentPosition() * radiansPerStep * (180.0 / PI);
  Serial.print("[CAL] "); Serial.print(stepper.currentPosition());
  Serial.print(" pasos / "); Serial.print(ang, 1);
  Serial.println(" grados");
}
