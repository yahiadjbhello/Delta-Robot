// Delta Robot ESP32 with Forward Kinematics + GUI Communication (Fixed)
#include <Wire.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <math.h>

// === AS5600 + TCA9548A ===
#define AS5600_ADDR 0x36
#define TCA_ADDR 0x70
#define SDA_PIN 21
#define SCL_PIN 22

// === Stepper Motor Pins ===
#define STEP_PIN_1 18
#define DIR_PIN_1 19
#define STEP_PIN_2 13
#define DIR_PIN_2 27
#define STEP_PIN_3 26
#define DIR_PIN_3 25

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

// === PID Control ===
double input[3] = {0}, output[3] = {0}, setpoint[3] = {0};
double Kp = 40.0, Ki = 0.0, Kd = 0.125;
PID pid1(&input[0], &output[0], &setpoint[0], Kp, Ki, Kd, DIRECT);
PID pid2(&input[1], &output[1], &setpoint[1], Kp, Ki, Kd, DIRECT);
PID pid3(&input[2], &output[2], &setpoint[2], Kp, Ki, Kd, DIRECT);

int motorSign[3] = {1, 1, 1};
double filteredInput[3] = {0};
const float alpha = 0.8;

// === Trajectory Control ===
String MODE = "CIRCULAR";
float RADIUS = 0.2;
float FREQUENCY = 0.5;
int UPDATE_INTERVAL = 5;
float ALTITUDE = 0.1;
float currentAngle = 0.0;
float pickPoint[3] = {0, 0, -0.3};
float placePoint[3] = {0.1, 0.1, -0.3};

unsigned long startTime;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  startTime = millis();

  stepper1.setMaxSpeed(3000); stepper1.setAcceleration(400);
  stepper2.setMaxSpeed(3000); stepper2.setAcceleration(400);
  stepper3.setMaxSpeed(3000); stepper3.setAcceleration(400);

  pid1.SetMode(AUTOMATIC); pid1.SetOutputLimits(-3000, 3000);
  pid2.SetMode(AUTOMATIC); pid2.SetOutputLimits(-3000, 3000);
  pid3.SetMode(AUTOMATIC); pid3.SetOutputLimits(-3000, 3000);

  Serial.println("System Ready - Real-Time GUI Communication");
}

void loop() {
  handleSerialInput();
  unsigned long now = millis();

  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;

    float x = 0, y = 0, z = -0.3;
    if (MODE == "CIRCULAR") {
      x = RADIUS * cos(currentAngle);
      y = RADIUS * sin(currentAngle);
    } else if (MODE == "UPDOWN") {
      x = 0;
      y = 0;
      z = -0.3 + ALTITUDE * sin(currentAngle);
    } else if (MODE == "POINT") {
      x = pickPoint[0];
      y = pickPoint[1];
      z = pickPoint[2];
    }

    currentAngle += 2 * PI * FREQUENCY * (UPDATE_INTERVAL / 1000.0);
    if (currentAngle > 2 * PI) currentAngle -= 2 * PI;

    double q_deg[3];
    computeInverseKinematics(x, y, z, q_deg);

    for (int i = 0; i < 3; i++) {
      setpoint[i] = q_deg[i];
      filteredInput[i] = alpha * wrapTo180(readAngleFromChannel(3 - i)) + (1 - alpha) * filteredInput[i];
      input[i] = filteredInput[i];
    }

    pid1.Compute(); pid2.Compute(); pid3.Compute();
    applyPID(stepper1, motorSign[0] * output[0]);
    applyPID(stepper2, motorSign[1] * output[1]);
    applyPID(stepper3, motorSign[2] * output[2]);

    float xyz_mes[3];
    fpk(radians(filteredInput[0]), radians(filteredInput[1]), radians(filteredInput[2]), xyz_mes);

    Serial.print("Q:");
    for (int i = 0; i < 3; i++) Serial.print(setpoint[i]), Serial.print(",");
    for (int i = 0; i < 3; i++) Serial.print(filteredInput[i]), Serial.print(",");
    Serial.printf("%.3f,%.3f,%.3f,", x, y, z);
    for (int i = 0; i < 3; i++) Serial.print(xyz_mes[i], 3), Serial.print(i < 2 ? "," : "\n");
  }

  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}

void parseAndUpdateParams(String line) {
  int start = 0;
  while (start < line.length()) {
    int end = line.indexOf(';', start);
    if (end == -1) end = line.length();
    int sep = line.indexOf('=', start);
    if (sep != -1 && sep < end) {
      String key = line.substring(start, sep);
      String val = line.substring(sep + 1, end);
      float fval = val.toFloat();
      if (key == "Kp") { Kp = fval; pid1.SetTunings(Kp, Ki, Kd); pid2.SetTunings(Kp, Ki, Kd); pid3.SetTunings(Kp, Ki, Kd); }
      else if (key == "Ki") { Ki = fval; pid1.SetTunings(Kp, Ki, Kd); pid2.SetTunings(Kp, Ki, Kd); pid3.SetTunings(Kp, Ki, Kd); }
      else if (key == "Kd") { Kd = fval; pid1.SetTunings(Kp, Ki, Kd); pid2.SetTunings(Kp, Ki, Kd); pid3.SetTunings(Kp, Ki, Kd); }
      else if (key == "R")  { RADIUS = fval; }
      else if (key == "F")  { FREQUENCY = fval; }
      else if (key == "U")  { UPDATE_INTERVAL = (int)fval; }
      else if (key == "Z")  { ALTITUDE = fval; }
      else if (key == "MODE") { MODE = val; }
      else if (key == "PICK") {
        sscanf(val.c_str(), "%f,%f,%f", &pickPoint[0], &pickPoint[1], &pickPoint[2]);
      } else if (key == "PLACE") {
        sscanf(val.c_str(), "%f,%f,%f", &placePoint[0], &placePoint[1], &placePoint[2]);
      }
    }
    start = end + 1;
  }
}
// Other unchanged utility functions (applyPID, readAngleFromChannel, getAngleDegrees, etc.) stay the same.
void applyPID(AccelStepper &stepper, double speed) {
  stepper.setSpeed(fabs(speed) < 3 ? 0 : -speed);
}

void selectTCAChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(300);
}

float readAngleFromChannel(uint8_t ch) {
  selectTCAChannel(ch);
  return getAngleDegrees();
}

uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0;
}

float getAngleDegrees() {
  return (readAS5600RawAngle() * 360.0) / 4096.0;
}

float wrapTo180(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}
void handleSerialInput() {
  static String inputLine;  // ✅ Déclaration ici

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseAndUpdateParams(inputLine);
      inputLine = "";  // clear après traitement
    } else {
      inputLine += c;
    }
  }
}


void computeInverseKinematics(float x, float y, float z, double q_deg[3]) {
  const float sb = 0.41268, sp = 0.086, L = 0.150, l = 0.3;
  float wb = sb * sqrt(3.0) / 6.0, wp = sp * sqrt(3.0) / 6.0, up = sp * sqrt(3.0) / 3.0;
  float a = wb - up;
  float b = sp / 2.0 - sqrt(3.0) / 2.0 * wb;
  float c = wp - wb / 2.0;

  float H[3], G[3], F[3], t[3], q[3];
  H[0] = 2.0 * L * (y + a);
  H[1] = -L * (sqrt(3.0) * (x + b) + y + c);
  H[2] = L * (sqrt(3.0) * (x - b) - y - c);

  G[0] = x * x + y * y + z * z + a * a + L * L + 2.0 * y * a - l * l;
  G[1] = x * x + y * y + z * z + b * b + c * c + L * L + 2.0 * (x * b + y * c) - l * l;
  G[2] = x * x + y * y + z * z + b * b + c * c + L * L + 2.0 * (-x * b + y * c) - l * l;

  F[0] = F[1] = F[2] = 2.0 * z * L;

  for (int i = 0; i < 3; i++) {
    float delta = H[i] * H[i] + F[i] * F[i] - G[i] * G[i];
    if (delta < 0) q_deg[i] = 0;
    else {
      t[i] = (-F[i] + sqrt(delta)) / (G[i] - H[i]);
      q[i] = 2.0 * atan(t[i]);
      q_deg[i] = (q[i] * 180.0 / PI) - 175.83;
    }
  }
}
// === Rotation autour de l'axe Z ===
void rot_0Z(float angle, float R[3][3]) {
  R[0][0] = cos(angle);  R[0][1] = -sin(angle); R[0][2] = 0;
  R[1][0] = sin(angle);  R[1][1] =  cos(angle); R[1][2] = 0;
  R[2][0] = 0;           R[2][1] = 0;           R[2][2] = 1;
}

// === Appliquer la rotation à un vecteur ===
void applyRotation(float R[3][3], float v[3], float result[3]) {
  for (int i = 0; i < 3; i++) {
    result[i] = R[i][0]*v[0] + R[i][1]*v[1] + R[i][2]*v[2];
  }
}

// === Fonction FPK : cinématique directe pour Delta robot ===
bool fpk(float th1, float th2, float th3, float P[3]) {
  const float sb = 0.41268;
  const float sp = 0.086;
  const float L  = 0.150;
  const float l  = 0.300;
  const float pi = M_PI;
  th1 =th1+radians(175.83);
  th2 =th2+radians(175.83);
  th3 =th3+radians(175.83);
  float wb = sb * sqrt(3.0) / 6.0;
  float wp = sp * sqrt(3.0) / 6.0;
  float up = sp * sqrt(3.0) / 3.0;

  // Vecteurs des bras en position fixe
  float e1[3] = {(wb - up) + L * cos(th1), 0, -L * sin(th1)};
  float e2[3] = {(wb - up) + L * cos(th2), 0, -L * sin(th2)};
  float e3[3] = {(wb - up) + L * cos(th3), 0, -L * sin(th3)};

  float E1[3], E2[3], E3[3], R[3][3];
  rot_0Z(0, R);            applyRotation(R, e1, E1);
  rot_0Z(2*pi/3, R);       applyRotation(R, e2, E2);
  rot_0Z(4*pi/3, R);       applyRotation(R, e3, E3);

  float M[3][3] = {
    {E1[0], E1[1], E1[2]},
    {E2[0], E2[1], E2[2]},
    {E3[0], E3[1], E3[2]}
  };

  float M1[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      M1[i][j] = -M[i][j];

  float N[2][3], V[2];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      N[i][j] = 2 * (M1[0][j] - M1[i + 1][j]);
    }
    V[i] = (pow(M1[i + 1][0], 2) + pow(M1[i + 1][1], 2) + pow(M1[i + 1][2], 2))
         - (pow(M1[0][0], 2) + pow(M1[0][1], 2) + pow(M1[0][2], 2));
  }

  float D = N[0][0]*N[1][1] - N[1][0]*N[0][1];
  if (fabs(D) < 1e-6) return false;

  float f1 = (N[1][1]*V[0] - N[0][1]*V[1]) / D;
  float f2 = (N[0][0]*V[1] - N[1][0]*V[0]) / D;
  float f3 = (N[0][1]*N[1][2] - N[0][2]*N[1][1]) / D;
  float f4 = (N[1][0]*N[0][2] - N[0][0]*N[1][2]) / D;

  float E = f3*f3 + f4*f4 + 1;
  float F = 2*M1[2][2] + 2*f3*(f1 + M1[2][0]) + 2*f4*(f2 + M1[2][1]);
  float G = pow(f1 + M1[2][0], 2) + pow(f2 + M1[2][1], 2) + pow(M1[2][2], 2) - l*l;

  float delta = F*F - 4*E*G;
  if (delta < 0) return false;

  float z = (-F - sqrt(delta)) / (2*E);
  float x = f1 + f3 * z;
  float y = f2 + f4 * z;

  float rawP[3] = {x, y, z};
  rot_0Z(-pi / 2, R);
  applyRotation(R, rawP, P);

  return true;
}

