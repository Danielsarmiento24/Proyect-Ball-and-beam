/*
 * ============================================================
 *  BALL & BEAM — HYBRID v3.8 + LCD + SENSOR DINÁMICO
 *  - Dos sensores HC-SR04 (bola + referencia)
 *  - 15 segundos para calibrar setpoint automático
 *  - LCD muestra SP dinámico y posición actual
 *  - Control LQR/PID híbrido
 * ============================================================
 */

#include <NewPing.h>
#include <AccelStepper.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─────────────────────────────────────────────
//  PANTALLA LCD I2C
// ─────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ─────────────────────────────────────────────
//  SENSORES HC-SR04
// ─────────────────────────────────────────────
// Sensor PRINCIPAL (posición bola)
NewPing sonar_bola(5, 6, 30);

// Sensor REFERENCIA (setpoint dinámico)
NewPing sonar_ref(11, 13, 30);

SimpleKalmanFilter kalman_bola(0.03, 0.03, 0.5);
SimpleKalmanFilter kalman_ref(0.05, 0.05, 0.8);

// ─────────────────────────────────────────────
//  ESTADO DE CALIBRACIÓN DE SETPOINT
// ─────────────────────────────────────────────
bool setpoint_calibrado = false;
unsigned long t_inicio_calibracion = 0;
const unsigned long TIEMPO_CALIBRACION_MS = 15000;  // 15 segundos
double setpoint_cm = 17.0;  // Valor por defecto si no se calibra

// Bandera de modo calibración
enum Estado_Sistema {
  CALIBRANDO_SETPOINT,
  ESPERANDO_VALIDACION,
  CONTROL_ACTIVO
};
Estado_Sistema estado_sistema = CALIBRANDO_SETPOINT;

// ─────────────────────────────────────────────
//  *** PARÁMETROS AJUSTABLES ***
// ─────────────────────────────────────────────
const int MAX_STEPS_POS = 430;
const int MAX_STEPS_NEG = 450;
const int SOFT_LIMIT_CERCA = 220;

// ── Tolerancia de estabilidad ────────────────
const double TOLERANCIA_ESTABLE_CM   = 0.5;
const double TOLERANCIA_REACTIVAR_CM = 1.0;
const unsigned long TIEMPO_ESTABLE_MS = 800;

// ── Ganancias LQR (zona LEJOS) ───────────────
const double K1 = 380.0;
const double K2 = 160.0;
const double K3 =  15.0;
const double K4 =   2.5;

// ── Ganancias PID (zona CERCA/MEDIA) ─────────
const double KP =  45.0;
const double KI =   1.5;
const double KD =   8.0;

const double ALPHA_D        = 0.12;
const double INTEGRAL_LIMIT = 60.0;

// ── Umbrales de zona ─────────────────────────
const double DIST_LEJOS_CM = 6.0;
const double DIST_CERCA_CM = 2.0;
const double ZONA_MUERTA_CM = 0.6;

// ── Rangos válidos de sensor ─────────────────
const double SENSOR_MIN_CM = 3.0;
const double SENSOR_MAX_CM = 26.0;

const int MOTOR_DIR = +1;

// ─────────────────────────────────────────────
//  PINES
// ─────────────────────────────────────────────
const int STEP_PIN   =  3;
const int DIR_PIN    =  2;
const int ENABLE_PIN = 12;
const int M0_PIN     =  7;
const int M1_PIN     =  9;
const int M2_PIN     = 10;
// Pin 5, 6 = SENSOR BOLA
// Pin 11, 13 = SENSOR REFERENCIA
// A4 (SDA), A5 (SCL) = LCD

// ─────────────────────────────────────────────
//  MICROSTEPPING 1/32
// ─────────────────────────────────────────────
const int    MICROSTEPS   = 32;
const double RAD_PER_STEP = (2.0 * PI) / (200.0 * MICROSTEPS);

// ─────────────────────────────────────────────
//  OBJETOS MOTOR
// ─────────────────────────────────────────────
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ─────────────────────────────────────────────
//  VARIABLES DE ESTADO
// ─────────────────────────────────────────────
double pos_m      = 0.0;
double vel_m      = 0.0;
double alpha_rad  = 0.0;
double alpha_dot  = 0.0;
double pos_prev   = 0.0;
double alpha_prev = 0.0;
double u_control  = 0.0;

// PID
double pid_integral  = 0.0;
double pid_err_prev  = 0.0;
double deriv_filt    = 0.0;
double accionP       = 0.0;
double accionI       = 0.0;
double accionD       = 0.0;

// Motor ON/OFF
bool   motor_activo           = true;
unsigned long t_dentro_tol    = 0;
bool   en_tol_prev            = false;

// Telemetría
double u_lqr_tel  = 0.0;
double u_pid_tel  = 0.0;
double blend_tel  = 0.0;
char   zona_tel[8] = "LEJOS";

double sp_m = 0.0;  // Se actualiza dinámicamente

unsigned long t_prev        = 0;
unsigned long t_last_serial = 0;
unsigned long t_last_lcd    = 0;
const unsigned long DT_CONTROL_US = 10000;
const unsigned long DT_SERIAL_MS  = 100;
const unsigned long DT_LCD_MS     = 200;

// ─────────────────────────────────────────────
//  PROTOTIPOS
// ─────────────────────────────────────────────
bool leerSensor(NewPing &sensor, SimpleKalmanFilter &kalman, double &cm_kalman);
void stepMoveCal(long target, unsigned int delayUs);
void imprimirTelemetria();
void actualizarLCD();
void procesarCalibrationSetpoint();

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // ── Inicializar LCD ────────────────────────
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ball & Beam v3.8");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");
  delay(2000);
  lcd.clear();

  // ── Pines Arduino ──────────────────────────
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(M0_PIN,     OUTPUT);
  pinMode(M1_PIN,     OUTPUT);
  pinMode(M2_PIN,     OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, HIGH);

  stepper.setMaxSpeed(500000);
  stepper.setAcceleration(60000);
  stepper.setMinPulseWidth(3);

  Serial.println(F("========================================="));
  Serial.println(F("   BALL & BEAM — HYBRID v3.8 + SENSOR"));
  Serial.println(F("   Setpoint dinámico + LCD"));
  Serial.println(F("========================================="));
  Serial.print(F("  LQR  K=[")); Serial.print(K1,0); Serial.print(F(","));
  Serial.print(K2,0); Serial.print(F(","));
  Serial.print(K3,0); Serial.print(F(","));
  Serial.print(K4,2); Serial.println(F("]"));
  Serial.print(F("  PID  Kp=")); Serial.print(KP,1);
  Serial.print(F(" Ki="));       Serial.print(KI,2);
  Serial.print(F(" Kd="));       Serial.println(KD,1);
  Serial.println(F("========================================="));

  // ── Calentar sensor PRINCIPAL ──────────────
  Serial.println(F("[INIT] Leyendo sensor BOLA..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Leyendo sensor");
  lcd.setCursor(0, 1);
  lcd.print("bola...");
  double cm_k = 0.0;
  for (int i = 0; i < 20; i++) {
    delay(100);
    if (leerSensor(sonar_bola, kalman_bola, cm_k)) {
      pos_m    = cm_k / 100.0;
      pos_prev = pos_m;
    }
  }
  Serial.print(F("[INIT] Pelota en: "));
  Serial.print(pos_m * 100.0, 2);
  Serial.println(F(" cm"));

  // ── CALIBRACIÓN MECÁNICA ───────────────────
  Serial.println(F("[CAL] Pon la viga HORIZONTAL y espera 3s..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CALIBRACION");
  lcd.setCursor(0, 1);
  lcd.print("Viga HORIZONTAL");
  delay(3000);

  digitalWrite(ENABLE_PIN, LOW);
  stepper.setCurrentPosition(0);

  Serial.print(F("[CAL] Subiendo MAX (+"));
  Serial.print(MAX_STEPS_POS); Serial.println(F(")..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cal: MAX");
  stepMoveCal(MAX_STEPS_POS, 400);
  delay(2000);

  Serial.println(F("[CAL] Centro..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cal: CENTRO");
  stepMoveCal(0, 400);
  delay(2000);

  Serial.print(F("[CAL] Bajando MIN (-"));
  Serial.print(MAX_STEPS_NEG); Serial.println(F(")..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cal: MIN");
  stepMoveCal(-MAX_STEPS_NEG, 400);
  delay(2000);

  Serial.println(F("[CAL] Centro..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cal: CENTRO");
  stepMoveCal(0, 400);
  stepper.setCurrentPosition(0);
  delay(2000);

  // ── CALIBRACIÓN DE SETPOINT DINÁMICO ──────
  Serial.println(F("[CALIBRACION SETPOINT] 15 segundos..."));
  Serial.println(F("[SETPOINT] Acerca tu mano al sensor REFERENCIA"));
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SETPOINT");
  lcd.setCursor(0, 1);
  lcd.print("15 seg...");

  estado_sistema = CALIBRANDO_SETPOINT;
  t_inicio_calibracion = millis();

  // Loop de calibración bloqueante
  unsigned long t_cal_start = millis();
  while ((millis() - t_cal_start) < TIEMPO_CALIBRACION_MS) {
    procesarCalibrationSetpoint();
    
    unsigned long t_restante = TIEMPO_CALIBRACION_MS - (millis() - t_cal_start);
    unsigned int seg = t_restante / 1000;
    
    lcd.setCursor(0, 1);
    lcd.print("SP:");
    lcd.print(setpoint_cm, 1);
    lcd.print(" (");
    if (seg < 10) lcd.print("0");
    lcd.print(seg);
    lcd.print("s)");
  }

  if (!setpoint_calibrado) {
    // Si no se calibró, usar valor por defecto
    Serial.println(F("[SETPOINT] No se capturó. Usando 17.0 cm"));
    setpoint_cm = 17.0;
  } else {
    Serial.print(F("[SETPOINT] Calibrado: "));
    Serial.print(setpoint_cm, 2);
    Serial.println(F(" cm"));
  }

  sp_m = setpoint_cm / 100.0;

  alpha_rad    = 0.0;
  alpha_prev   = 0.0;
  alpha_dot    = 0.0;
  vel_m        = 0.0;
  pid_integral = 0.0;
  pid_err_prev = 0.0;
  deriv_filt   = 0.0;
  motor_activo = true;
  t_prev        = micros();
  t_last_serial = millis();
  t_last_lcd    = millis();

  estado_sistema = CONTROL_ACTIVO;

  Serial.println(F("[INIT] Control HYBRID v3.8 activo."));
  Serial.println(F("========================================="));
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Control ACTIVO");
  delay(2000);
  lcd.clear();
}

// ═════════════════════════════════════════════
//  PROCESAMIENTO CALIBRACIÓN SETPOINT
// ═════════════════════════════════════════════
void procesarCalibrationSetpoint() {
  double cm_ref = 0.0;
  
  if (leerSensor(sonar_ref, kalman_ref, cm_ref)) {
    if (cm_ref >= SENSOR_MIN_CM && cm_ref <= SENSOR_MAX_CM) {
      setpoint_cm = cm_ref;
      setpoint_calibrado = true;
      Serial.print(F("[SETPOINT] Lectura válida: "));
      Serial.print(setpoint_cm, 2);
      Serial.println(F(" cm"));
    } else {
      Serial.print(F("[SETPOINT] Lectura inválida: "));
      Serial.print(cm_ref, 2);
      Serial.println(F(" cm (fuera de rango)"));
    }
  }
  
  delay(100);  // Pequeño delay para dar tiempo al sensor
}

// ═════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═════════════════════════════════════════════
void loop() {
  stepper.run();

  unsigned long ahora_us = micros();
  double dt = (double)(ahora_us - t_prev) / 1e6;
  if (dt < (DT_CONTROL_US / 1e6)) return;
  t_prev = ahora_us;

  unsigned long ahora_ms = millis();

  // ── 1. Leer sensor BOLA ────────────────────
  double cm_k = 0.0;
  if (leerSensor(sonar_bola, kalman_bola, cm_k)) {
    double pos_nueva  = cm_k / 100.0;
    double vel_raw    = (pos_nueva - pos_prev) / dt;
    double abs_err_cm = fabs((pos_nueva - sp_m) * 100.0);
    double lp = (abs_err_cm < DIST_CERCA_CM) ? 0.95 : 0.75;
    vel_m    = lp * vel_m + (1.0 - lp) * vel_raw;
    pos_prev = pos_m;
    pos_m    = pos_nueva;
  }

  // ── 2. Ángulo y velocidad angular ──────────
  alpha_rad = (double)stepper.currentPosition() * RAD_PER_STEP;
  double alpha_dot_raw = (alpha_rad - alpha_prev) / dt;
  alpha_dot  = 0.85 * alpha_dot + 0.15 * alpha_dot_raw;
  alpha_prev = alpha_rad;

  // ── 3. Error ───────────────────────────────
  double abs_error_cm = fabs((pos_m - sp_m) * 100.0);

  // ── 4. Lógica motor ON/OFF ─────────────────
  bool en_tol = (abs_error_cm < TOLERANCIA_ESTABLE_CM);
  if (en_tol) {
    if (!en_tol_prev) t_dentro_tol = ahora_ms;
    if (motor_activo &&
        (ahora_ms - t_dentro_tol) >= TIEMPO_ESTABLE_MS) {
      motor_activo = false;
      stepper.moveTo(stepper.currentPosition());
      digitalWrite(ENABLE_PIN, HIGH);
      strcpy(zona_tel, "DORMIDO");
    }
  } else {
    if (!motor_activo && abs_error_cm > TOLERANCIA_REACTIVAR_CM) {
      motor_activo = true;
      digitalWrite(ENABLE_PIN, LOW);
      pid_err_prev = 0.0;
      deriv_filt   = 0.0;
    }
  }
  en_tol_prev = en_tol;

  if (!motor_activo) {
    if (ahora_ms - t_last_serial >= DT_SERIAL_MS) {
      t_last_serial = ahora_ms;
      imprimirTelemetria();
    }
    if (ahora_ms - t_last_lcd >= DT_LCD_MS) {
      t_last_lcd = ahora_ms;
      actualizarLCD();
    }
    return;
  }

  // ── 5. Errores ─────────────────────────────
  double error_lqr = pos_m - sp_m;
  if (abs_error_cm < ZONA_MUERTA_CM) error_lqr = 0.0;

  double e_pid_m  = error_lqr * cos(alpha_rad);
  double e_pid_cm = e_pid_m * 100.0;

  // ── 6. Salida LQR ──────────────────────────
  double u_lqr_rad = (double)MOTOR_DIR * (K1 * error_lqr
                                         + K2 * vel_m
                                         + K3 * alpha_rad
                                         + K4 * alpha_dot);
  double u_lqr = u_lqr_rad / RAD_PER_STEP;
  u_lqr = constrain(u_lqr, -(double)MAX_STEPS_NEG, (double)MAX_STEPS_POS);

  // ── 7. Salida PID ──────────────────────────
  accionP = KP * e_pid_cm;

  if (abs_error_cm >= DIST_LEJOS_CM) {
    pid_integral = 0.0;
  } else {
    double intCandidate = pid_integral + e_pid_cm * dt;
    double lim = (abs_error_cm > DIST_CERCA_CM)
               ? INTEGRAL_LIMIT
               : INTEGRAL_LIMIT * 0.5;
    double outEst = (double)MOTOR_DIR
                  * (KP * e_pid_cm + KI * intCandidate);
    double soft = (abs_error_cm <= DIST_CERCA_CM)
                ? (double)SOFT_LIMIT_CERCA
                : (double)MAX_STEPS_POS;
    bool sat  = (outEst >  soft) || (outEst < -soft);
    bool same = (outEst > 0 && e_pid_cm < 0) ||
                (outEst < 0 && e_pid_cm > 0);
    if (!(sat && same)) pid_integral = intCandidate;
    pid_integral = constrain(pid_integral, -lim, lim);
  }
  accionI = KI * pid_integral;

  // Derivada con filtro EMA
  double deriv_raw = (e_pid_cm - pid_err_prev) / dt;
  deriv_filt = ALPHA_D * deriv_raw + (1.0 - ALPHA_D) * deriv_filt;
  accionD    = KD * deriv_filt;
  pid_err_prev = e_pid_cm;

  double u_pid_raw = (double)MOTOR_DIR * (accionP + accionI + accionD);
  double u_pid;
  if (abs_error_cm <= DIST_CERCA_CM) {
    u_pid = constrain(u_pid_raw,
                      -(double)SOFT_LIMIT_CERCA,
                       (double)SOFT_LIMIT_CERCA);
  } else {
    u_pid = constrain(u_pid_raw,
                      -(double)MAX_STEPS_NEG,
                       (double)MAX_STEPS_POS);
  }

  // ── 8. Mezcla LQR ↔ PID ───────────────────
  double u_final = 0.0;
  double blend   = 0.0;

  if (abs_error_cm >= DIST_LEJOS_CM) {
    blend   = 0.0;
    u_final = u_lqr;
    strcpy(zona_tel, "LEJOS");

  } else if (abs_error_cm <= DIST_CERCA_CM) {
    blend   = 1.0;
    u_final = u_pid;
    strcpy(zona_tel, "CERCA");

  } else {
    blend = (DIST_LEJOS_CM - abs_error_cm)
           / (DIST_LEJOS_CM - DIST_CERCA_CM);
    double soft_media = SOFT_LIMIT_CERCA
                      + (1.0 - blend)
                      * ((double)MAX_STEPS_POS - SOFT_LIMIT_CERCA);
    u_pid = constrain(u_pid_raw, -soft_media, soft_media);
    u_final = (1.0 - blend) * u_lqr + blend * u_pid;
    strcpy(zona_tel, "MEDIA");
  }

  u_final   = constrain(u_final,
                        -(double)MAX_STEPS_NEG,
                         (double)MAX_STEPS_POS);
  u_control = u_final;
  u_lqr_tel = u_lqr;
  u_pid_tel = u_pid;
  blend_tel = blend;

  // ── 9. Mover motor ─────────────────────────
  stepper.moveTo((long)u_control);

  // ── 10. Telemetría ─────────────────────────
  if (ahora_ms - t_last_serial >= DT_SERIAL_MS) {
    t_last_serial = ahora_ms;
    imprimirTelemetria();
  }

  if (ahora_ms - t_last_lcd >= DT_LCD_MS) {
    t_last_lcd = ahora_ms;
    actualizarLCD();
  }
}

// ═════════════════════════════════════════════
//  LEER SENSOR (genérico para ambos)
// ═════════════════════════════════════════════
bool leerSensor(NewPing &sensor, SimpleKalmanFilter &kalman, double &cm_kalman) {
  unsigned int us = sensor.ping();
  if (us == 0) return false;
  double cm_raw = (double)us / 58.0;
  if (cm_raw < 3.0 || cm_raw > 26.0) return false;
  cm_kalman = kalman.updateEstimate(cm_raw);
  return true;
}

// ═════════════════════════════════════════════
//  ACTUALIZAR LCD
// ═════════════════════════════════════════════
void actualizarLCD() {
  double pos_cm = pos_m * 100.0;
  double err_cm = (pos_m - sp_m) * 100.0;

  // Línea 1: SP y Posición
  lcd.setCursor(0, 0);
  lcd.print("SP:");
  lcd.print(setpoint_cm, 1);
  lcd.print(" Pos:");
  lcd.print(pos_cm, 1);

  // Línea 2: Error y Estado
  lcd.setCursor(0, 1);
  lcd.print("Err:");
  lcd.print(err_cm, 2);
  lcd.print(" ");
  lcd.print(zona_tel);
  
  // Rellenar espacios
  for (int i = 15; i < 16; i++) {
    lcd.setCursor(i, 1);
    lcd.print(" ");
  }
}

// ═════════════════════════════════════════════
//  TELEMETRÍA SERIAL
// ═════════════════════════════════════════════
void imprimirTelemetria() {
  double pos_cm  = pos_m  * 100.0;
  double vel_cms = vel_m  * 100.0;
  double ang_deg = alpha_rad * (180.0 / PI);
  double err_cm  = (pos_m - sp_m) * 100.0;
  long   pasos   = stepper.currentPosition();

  Serial.print(F("SP:"));    Serial.print(setpoint_cm, 1);
  Serial.print(F(" Pos:"));  Serial.print(pos_cm, 2);
  Serial.print(F(" Err:"));  Serial.print(err_cm, 2);
  Serial.print(F(" Vel:"));  Serial.print(vel_cms, 1);
  Serial.print(F(" Ang:"));  Serial.print(ang_deg, 1);
  Serial.print(F(" Pas:"));  Serial.print(pasos);
  Serial.print(F(" U:"));    Serial.print(u_control, 1);
  Serial.print(F(" Bl:"));   Serial.print(blend_tel, 2);
  Serial.print(F(" Int:"));  Serial.print(pid_integral, 2);
  Serial.print(F(" P:"));    Serial.print(accionP, 1);
  Serial.print(F(" D:"));    Serial.print(accionD, 1);
  Serial.print(F(" Z:"));    Serial.print(zona_tel);
  Serial.print(F(" M:"));    Serial.println(motor_activo ? F("ON") : F("OFF"));
}

// ═════════════════════════════════════════════
//  STEP MOVE CALIBRACIÓN
// ═════════════════════════════════════════════
void stepMoveCal(long target, unsigned int delayUs) {
  long pos = stepper.currentPosition();
  int  dir = (target > pos) ? 1 : -1;

  while (pos != target) {
    if ( dir ==  1 && pos >= (long) MAX_STEPS_POS) break;
    if ( dir == -1 && pos <= -(long)MAX_STEPS_NEG) break;

    digitalWrite(DIR_PIN, dir == 1 ? HIGH : LOW);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayUs);

    pos += dir;
    stepper.setCurrentPosition(pos);
  }

  double ang = stepper.currentPosition() * RAD_PER_STEP * (180.0 / PI);
  Serial.print(F("[CAL] ")); Serial.print(stepper.currentPosition());
  Serial.print(F(" pasos = ")); Serial.print(ang, 2);
  Serial.println(F(" deg"));
}