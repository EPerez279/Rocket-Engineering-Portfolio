

#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <Adafruit_ST7735.h>
#include <Servo.h>
#include <MadgwickAHRS.h>

Madgwick filter;

// A 2d vector, used for passing of values
struct Vector2Float {
  double x;
  double y;
};

MPU6050 mpu;

Adafruit_BMP280 bmp;

// TVC Servos
Servo SERVO_X;
Servo SERVO_Y;
Servo SERVO_P;

#define TFT_CS   10
#define TFT_RST  9
#define TFT_DC   8
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Servo Pins
const int PIN_X = 16;
const int PIN_Y = 17;
const int PIN_P = 6;

// Offset Angles (Neutral Positions)
const int ANGLE_X = 75;
const int ANGLE_Y = 87;
const int ANGLE_P = 0;

// TVC Angle Limits (degrees)
// NOTE: This is currently +/-5 althought the TVC can take +/-7 but we observed some hard stop while using Y-SERVO.
// update these and re-tune if TVC is different.
const double tvc_min = -5;
const double tvc_max = 5;

// Servo range mapping (degrees around neutral)
// These are the angles that when written to the servo, will give the corresponding tvc_min (-5) or tvc_max (5) values
const double servo_min = -20;
const double servo_max = 20;

// PID Parameters
const double KP = 0.45;
const double KD = 0.06;
const double KI = 1.2;

// Target Orientation (in degrees)
const double TARGET_X = 0;
const double TARGET_Y = 0;

// Axis/frame configuration.
// Defaults below assume the MPU is mounted upright with the rocket/body axes.
// These values must be checked during calibration tilt and gyro-rate signs match the TVC axes.
const bool SWAP_TILT_AXES = false;
const double TILT_X_SIGN = 1.0;
const double TILT_Y_SIGN = 1.0;

enum GyroAxisIndex { GYRO_AXIS_X = 0, GYRO_AXIS_Y = 1, GYRO_AXIS_Z = 2 };
const GyroAxisIndex PID_GYRO_X_AXIS = GYRO_AXIS_X;
const GyroAxisIndex PID_GYRO_Y_AXIS = GYRO_AXIS_Y;
const GyroAxisIndex PID_GYRO_Z_AXIS = GYRO_AXIS_Z;
const double PID_GYRO_X_SIGN = 1.0;
const double PID_GYRO_Y_SIGN = 1.0;
const double PID_GYRO_Z_SIGN = 1.0;

// Offsets computed during calibration
double ANGLE_X_OFFSET = 0;
double ANGLE_Y_OFFSET = 0;

// DMP Variables
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
double GYRO_LSB_PER_DPS = 65.5; // will be set based on gyro full-scale
Quaternion q;

// Parachute variables
bool PARACHUTE_OPEN = false;
unsigned long initial_time = 0;
bool start_timer = false;

// ---------------- PID state ----------------
double PREVIOUS_ERROR_X = 0;
double PREVIOUS_ERROR_Y = 0;
double INTEGRAL_X = 0;
double INTEGRAL_Y = 0;

// Fixed-rate PID timing
const uint32_t PID_PERIOD_US = 5000 * 2;   // 100 Hz (good starting point)
uint32_t last_pid_us = 0;

// Display update (avoid slowing PID)
const uint32_t DISPLAY_PERIOD_MS = 50; // 20 Hz
uint32_t last_display_ms = 0;

double MAXIMUM_ALTITUDE = 0;

// For Integral clamp:
// Integral state is clamped so that KI*I doesn't get crazy.
// This sets max integral contribution to about ~2x tvc_max (still safe due to output clamp).
const double INTEGRAL_MAX = (2.0 * tvc_max) / KI;

// Utility clamp
static inline double clampd(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int16_t selectGyroAxis(int16_t gx, int16_t gy, int16_t gz, GyroAxisIndex axis) {
  switch (axis) {
    case GYRO_AXIS_X: return gx;
    case GYRO_AXIS_Y: return gy;
    default:          return gz;
  }
}


int counter = 0;

void quaternionToDirectionError(const Quaternion& q, VectorFloat &r, double &error_x, double &error_y, double &rocket_angle, VectorFloat &tvc_x, VectorFloat &tvc_y,
                            double X_OFFSET, double Y_OFFSET);

void setup(){
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  // Initialize TFT Display
  tft.initR(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.fillScreen(ST77XX_BLACK);

  // Attach Servos
  SERVO_X.attach(PIN_X);
  SERVO_Y.attach(PIN_Y);
  pinMode(PIN_P, OUTPUT);
  digitalWrite(PIN_P, LOW);

  SERVO_X.write(ANGLE_X);
  SERVO_Y.write(ANGLE_Y);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  mpu.setRate(9);
  mpu.setDLPFMode(2);
  
  mpu.setXAccelOffset(-14);
  mpu.setYAccelOffset(-4);
  mpu.setZAccelOffset(16331);
  mpu.setXGyroOffset(-8);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  filter.begin(100);
  filter.beta = 0.3;

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1) {}
  }

  Serial.println("Initializing DMP...");
  if (mpu.dmpInitialize() == 0) {
    // Explicit gyro full-scale so scaling is known
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500); // +/- 500 dps

    // Determine scale factor
    switch (mpu.getFullScaleGyroRange()) {
      case MPU6050_GYRO_FS_250:  GYRO_LSB_PER_DPS = 131.0; break;
      case MPU6050_GYRO_FS_500:  GYRO_LSB_PER_DPS = 65.5;  break;
      case MPU6050_GYRO_FS_1000: GYRO_LSB_PER_DPS = 32.8;  break;
      default:                   GYRO_LSB_PER_DPS = 16.4;  break; // (MPU6050_GYRO_FS_2000) 
    }
    Serial.print("GYRO: ");
    Serial.println(GYRO_LSB_PER_DPS);

    // Quick gyro calibration (hold still)
    mpu.CalibrateGyro(20);

    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP Ready");
  } else {
    Serial.println("DMP Initialization Failed!");
    while (1) {}
  }

  tft.setCursor(0, 5);
  tft.println("Calibrating MPU...");
  Serial.println("Calibrating MPU...");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Measure initial attitude offsets
  // (Hold rocket in desired starting orientation)
  for (int i = 0; i < 50; i++) {
    delay(50);
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      q.normalize();
    }

    double error_x, error_y;
    VectorFloat r;
    double rocket_roll;
    VectorFloat tvc_x, tvc_y;
    quaternionToDirectionError(q, r, error_x, error_y, rocket_roll, tvc_x, tvc_y, 0, 0);

    ANGLE_X_OFFSET = error_x;
    ANGLE_Y_OFFSET = error_y;
    tft.setCursor(0, 13);
    tft.println("x: " + String(error_x, 2) + "   ");
    tft.println("y: " + String(error_y, 2) + "   ");
  }


  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("Done.");
  Serial.println("Done.");
 
  tft.setCursor(0, 40);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  tft.println(String("Checking BMP280..."));

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 connection failed!");
    while (1);
  }

  tft.setCursor(0, 50);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("Done.");

  tft.setCursor(0, 60);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  tft.println("Initiating System...");
  Serial.println("Initiating System...");

  last_pid_us = micros();
  last_display_ms = millis();

  MAXIMUM_ALTITUDE = bmp.readAltitude(1013.25);

  tft.setCursor(0, 70);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.println("Rocket's Angle:");
  tft.println("Angle-x: ");
  tft.println("Angle-y: ");
  tft.println("Altitude: ");
  tft.setCursor(0, 110);
}

/** The PID controler for the TVC system
 *  r - the quaternion representation of the rocket
 *  error_x, error_y - The error angle of the rocket
 *  Gx, Gy, Gz - The gyro values (aka angular velocity) of the rocket
 *  dt - Delta Time
 *  (Out) mapped_x, mapped_y - the servo angles given as the result of the PID
 */
void main_pid(VectorFloat r, double error_x, double error_y, int16_t Gx, int16_t Gy, int16_t Gz, double dt,
              double &mapped_x, double &mapped_y) {
  // Sign conventions (Gyro signs are now '1')
  double gx_dps = PID_GYRO_X_SIGN * ((double)selectGyroAxis(Gx, Gy, Gz, PID_GYRO_X_AXIS)) / GYRO_LSB_PER_DPS; // deg/s
  double gy_dps = PID_GYRO_Y_SIGN * ((double)selectGyroAxis(Gx, Gy, Gz, PID_GYRO_Y_AXIS)) / GYRO_LSB_PER_DPS; // deg/s
  double gz_dps = PID_GYRO_Z_SIGN * ((double)selectGyroAxis(Gx, Gy, Gz, PID_GYRO_Z_AXIS)) / GYRO_LSB_PER_DPS; // deg/s

  double gx_rps = gx_dps * (M_PI / 180.0);
  double gy_rps = gy_dps * (M_PI / 180.0);
  double gz_rps = gz_dps * (M_PI / 180.0);
 
  // Derivative term uses gyro rate
  // Finds the gyro error (derivative) using (w cross r) cross t, where t=<0,0,1>, r is the rocket vector, and w is the gyro values (aka angular acceleration)
 
  // w cross r
  double d_error_x = gy_rps*r.z - gz_rps*r.y;
  double d_error_y = gz_rps*r.x - gx_rps*r.z;
  double d_error_z = gx_rps*r.y - gy_rps*r.x; // Z is currently unused

  // then cross t (0,0,1)
  double derivative_x = d_error_y;
  double derivative_y = -d_error_x;

  // Start from current integral state
  double I_candidate_x = INTEGRAL_X;
  double I_candidate_y = INTEGRAL_Y;

  // Only integrate once the rocket is actually flying
  if (start_timer) {
    
    double error_for_integral_x = clampd(error_x, -INTEGRAL_MAX, INTEGRAL_MAX);
    double error_for_integral_y = clampd(error_y, -INTEGRAL_MAX, INTEGRAL_MAX);

    // Candidate integral update
    I_candidate_x = INTEGRAL_X + error_for_integral_x * dt;
    I_candidate_y = INTEGRAL_Y + error_for_integral_y * dt;

    // Clamp stored integral state
    I_candidate_x = clampd(I_candidate_x, -INTEGRAL_MAX, INTEGRAL_MAX);
    I_candidate_y = clampd(I_candidate_y, -INTEGRAL_MAX, INTEGRAL_MAX);
  }

  // Provisional PID output using candidate integral
  double u_unsat_x = KP * error_x + KD * derivative_x + KI * I_candidate_x;
  double u_unsat_y = KP * error_y + KD * derivative_y + KI * I_candidate_y;
  u_unsat_x *= 180/M_PI; // To convert the output into degrees
  u_unsat_y *= 180/M_PI;

  // Saturate to TVC limits
  double u_sat_x = clampd(u_unsat_x, tvc_min, tvc_max);
  double u_sat_y = clampd(u_unsat_y, tvc_min, tvc_max);

  // Anti-windup:
  // If saturated and the error pushes farther into saturation,
  // reject the candidate integral update.
  if (start_timer) {
    bool block_integral_x =
      (u_unsat_x > tvc_max && error_x > 0) ||
      (u_unsat_x < tvc_min && error_x < 0);

    bool block_integral_y =
      (u_unsat_y > tvc_max && error_y > 0) ||
      (u_unsat_y < tvc_min && error_y < 0);

    if (!block_integral_x) {
      INTEGRAL_X = I_candidate_x;
    }

    if (!block_integral_y) {
      INTEGRAL_Y = I_candidate_y;
    }
  }

  // Final outputs are the saturated ones
  double output_x = u_sat_x;
  double output_y = u_sat_y;

  // Map TVC output -> servo command range
  mapped_x = servo_min + (servo_max - servo_min) * ((output_x - tvc_min) / (tvc_max - tvc_min));
  mapped_y = servo_min + (servo_max - servo_min) * ((output_y - tvc_min) / (tvc_max - tvc_min));

}


void loop() {
  if (!dmpReady) return;


  // Convert quaternion directly to 2-axis tilt errors (degrees)
  double error_x, error_y;
  double rocket_roll;
  VectorFloat r;
  VectorFloat tvc_x, tvc_y;
  

  // Fixed-rate PID update
  uint32_t now_us = micros();
  uint32_t elapsed_us = now_us - last_pid_us;
  if (elapsed_us >= PID_PERIOD_US) {
    last_pid_us = now_us;
    // Still update display at its own rate even if PID doesn't run
   
    double dt = elapsed_us * 1e-6; // to seconds
    if (dt <= 0) dt = PID_PERIOD_US * 1e-6;
 
    // Gyro rate for D term
    int16_t Gx, Gy, Gz;
    int16_t Ax, Ay, Az;
    mpu.getMotion6(&Ax, &Ay, &Az, &Gx, &Gy, &Gz);
    double gx = (double)Gx / GYRO_LSB_PER_DPS;
    double gy = (double)Gy / GYRO_LSB_PER_DPS;
    double gz = (double)Gz / GYRO_LSB_PER_DPS;
    
    
    double ax = (double)Ax / 16384.0;
    double ay = (double)Ay / 16384.0;
    double az = (double)Az / 16384.0;

    double a_mag = sqrt(ax*ax + ay*ay + az*az);
    if (a_mag > 0.0001) {
      ax /= a_mag;
      ay /= a_mag;
      az /= a_mag;
    }

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    
    q.w = filter.q0;
    q.x = filter.q1;
    q.y = filter.q2;
    q.z = filter.q3;
    
    q.normalize();
    

    quaternionToDirectionError(q, r, error_x, error_y, rocket_roll, tvc_x, tvc_y, ANGLE_X_OFFSET, ANGLE_Y_OFFSET);

    
    if (!start_timer && sqrt(error_x*error_x + error_y*error_y) >= sin(2.0 * (M_PI / 180.0))) {
      initial_time = millis();
      start_timer = true;
    }

    double mapped_x, mapped_y;
    main_pid(r, error_x, error_y, Gx, Gy, Gz, dt, mapped_x, mapped_y); // mapped x and y are changed by main_pid()

    double TVC_MAPPED_X = -mapped_y; // rotate by 90 degress to counteract the rotation made by the cross-product of the error
    double TVC_MAPPED_Y =  mapped_x;
    mapped_x = TVC_MAPPED_X;
    mapped_y = TVC_MAPPED_Y;

    // e cross r
    double TVC_X = mapped_y * r.z;
    double TVC_Y =                - mapped_x * r.z;
    double TVC_Z = mapped_x * r.y - mapped_y * r.x;

   
    double TARGET_TVC_X = (tvc_x.x) * TVC_X + (tvc_x.y * TVC_Y) + (tvc_x.z * TVC_Z);
    double TARGET_TVC_Y = (tvc_y.x) * TVC_X + (tvc_y.y * TVC_Y) + (tvc_y.z * TVC_Z);

    

    SERVO_X.write(ANGLE_X + TARGET_TVC_X);
    SERVO_Y.write(ANGLE_Y + TARGET_TVC_Y);
 
    PREVIOUS_ERROR_X = error_x;
    PREVIOUS_ERROR_Y = error_y;
  }

  
  unsigned long current_time_ms = millis();
  double time_seconds = start_timer ? ((current_time_ms - initial_time) / 1000.0) : 0.0;

  if (bmp.readAltitude(1013.25) > MAXIMUM_ALTITUDE) {
    MAXIMUM_ALTITUDE = bmp.readAltitude(1013.25);
  }

  // Open the parachute if altitude starts decreasing
  if (start_timer && time_seconds > 4.0 && !PARACHUTE_OPEN && bmp.readAltitude(1013.25) + 2.5 < MAXIMUM_ALTITUDE) {
    digitalWrite(PIN_P, HIGH);
    PARACHUTE_OPEN = true;
    Serial.println("Deploy Parachute");
  }

  // Display at slower rate (prevents slowing PID loop)
  if ((current_time_ms - last_display_ms) >= DISPLAY_PERIOD_MS && false) {
    last_display_ms = current_time_ms;

    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);

    tft.setCursor(54, 80);
    tft.print(String(error_x, 2) + "   ");
    tft.setCursor(54, 90);
    tft.print(String(error_y, 2) + "   ");
    tft.setCursor(60, 100);
    tft.print(String(time_seconds, 2) + "   ");
    tft.setCursor(60, 110);
    tft.print(String(MAXIMUM_ALTITUDE - (bmp.readAltitude(1013.25) + 3), 3) + "   ");

    tft.setCursor(0, 110);
    tft.print(String(q.w, 4) + "," + String(q.x, 4) + "   ");
    tft.setCursor(0, 120);
    tft.print(String(q.y, 4) + "," + String(q.z, 4) + "   ");
  }
}

void quaternionToDirectionError(const Quaternion& q, VectorFloat &r, double &error_x, double &error_y, double &rocket_angle, VectorFloat &tvc_x, VectorFloat &tvc_y,
                            double X_OFFSET, double Y_OFFSET) {
  
  // Gets the rotation matrix from the rotations of the global unit vectors, q*p*q^-1
  // The Rocket-z axis (0,0,1)_z
  double zx = 2.0 * (q.x * q.z + q.w * q.y);
  double zy = 2.0 * (q.y * q.z - q.w * q.x);
  double zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y); 

  // The Rocket-x axis (1,0,0)_x
  double xx = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double xy = 2.0 * (q.x * q.y + q.z * q.w);
  double xz = 2.0 * (q.x * q.z - q.y * q.w);

  // The Rocket-y axis (0,1,0)_y
  double yx = 2.0 * (q.x * q.y - q.z * q.w);
  double yy = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
  double yz = 2.0 * (q.y * q.z + q.x * q.w);


  tvc_x.x = xx;
  tvc_x.y = xy;
  tvc_x.z = xz;
  
  tvc_y.x = yx;
  tvc_y.y = yy;
  tvc_y.z = yz;


  r.x = zx;
  r.y = zy;
  r.z = zz;

  rocket_angle = 2.0 * atan2(sqrt(q.x*q.x + q.y*q.y + q.z*q.z), q.w);

  double rocket_roll = atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  double rocket_pitch = asin(2.0 * (q.w * q.y - q.x * q.z));
  double rocket_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Target direction = straight up in world frame, pointing up to <0,0,1> z-axis direction
  // t = (0, 0, 1) t is target vector, r is rocket vector as per the pic of the whiteboard
  // e = r x t = (zy, -zx, 0)--(take <a,b,c> cross_product <0,0,1> = <b,-a,0>)
  double ex = zy;
  double ey = -zx;
  // Optional (axis swap to match hardware layout)
  double ctrl_x = SWAP_TILT_AXES ? ey : ex; // SWAP_TILT_AXES is to set true or false once calibration is performed and get e_x or e_y
  double ctrl_y = SWAP_TILT_AXES ? ex : ey;
 
  ctrl_x *= TILT_X_SIGN;
  ctrl_y *= TILT_Y_SIGN;

  error_x = ctrl_x - X_OFFSET; // the offsets now are the small e_x and e_y that we will get from r_0(initial rocket direction) cross_product t(target <0,0,1>)
  error_y = ctrl_y - Y_OFFSET;


  // Debug values and can be used to make sure the computer is on and working
  counter++;

  if (counter > 10) {
    counter = 0;
 
    Serial.print("R: <");
    Serial.print(zx);
    Serial.print(", ");
    Serial.print(zy);
    Serial.print(", ");
    Serial.print(zz);
    Serial.print(">  Norm: ");
    Serial.print(sqrt(zx*zx+zy*zy+zz*zz));

    Serial.print(" VX: <");
    Serial.print(xx);
    Serial.print(", ");
    Serial.print(xy);
    Serial.print(", ");
    Serial.print(xz);
    Serial.print("> VY: <");
    Serial.print(yx);
    Serial.print(", ");
    Serial.print(yy);
    Serial.print(", ");
    Serial.print(yz);

    Serial.print("> X: <");
    Serial.print(tvc_x.x);
    Serial.print(", ");
    Serial.print(tvc_x.y);
    Serial.print("> Y: <");
    Serial.print(tvc_y.x);
    Serial.print(", ");
    Serial.print(tvc_y.y);
    Serial.print("> ");

    Serial.print(" R: ");
    Serial.print(rocket_roll * 180/M_PI);
    Serial.print(" P: ");
    Serial.print(rocket_pitch * 180/M_PI);
    Serial.print(" Y: ");
    Serial.print(rocket_yaw * 180/M_PI);
  
    Serial.print(" E: <");
    Serial.print(error_x);
    Serial.print(", ");
    Serial.print(error_y);
    Serial.println(">");
  }
}