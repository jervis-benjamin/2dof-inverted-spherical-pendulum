#include <Arduino.h>  
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>
#include <string>
#include <BasicLinearAlgebra.h>

using namespace BLA;
Adafruit_MPU6050 mpu;

/* motor variables and pins */
const int motorNorth = 13;
const int IN1_1 = 5;
const int IN2_1 = 4;

const int motorSouth = 12;
const int IN3_1 = 15;
const int IN4_1 = 2;

const int motorEast = 14;
const int IN1_2 = 27;
const int IN2_2 = 26;

const int motorWest = 25;
const int IN3_2 = 33;
const int IN4_2 = 32;

const int motor[2][2] = {{motorEast, motorWest}, {motorNorth, motorSouth}};
const int motorIN1[2][2] = {{IN1_2, IN3_2}, {IN1_1, IN3_1}};
const int motorIN2[2][2] = {{IN2_2, IN4_2}, {IN2_1, IN4_1}};

// offsets
double ACC_X_OFFSET = 0, ACC_Y_OFFSET = 0, ACC_Z_OFFSET = 0;
double GYR_X_OFFSET = 0, GYR_Y_OFFSET = 0, GYR_Z_OFFSET = 0;

// starting time
unsigned long t0 = 0;
unsigned long start_time = 0;

/* miscellaneous system parameters */
double thrust_limit = 0.56; // max magnetude of trust (in Newtons) from one motor set (two motors)
double supply_voltage = 4.8;
double voltage_drop = 2; // approximate voltage drop in the l298
int pwm_floor = 50, pwm_ceiling = 14000; // bounds for when to cut off motor to prevent buzzing or abort from saturation

/* PID variables for theta */
const double Kp_theta = 0.006, Ki_theta = 0.005036, Kd_theta = 0.01191, N_theta = 856.656;
double theta_setpoint = 0;
double theta_integral = 0;
double prev_theta_error = 0;
double filt_theta_derivative = 0;
double Theta;

/* PID variables for phi */
const double Kp_phi = 0.006, Ki_phi = 0.005036, Kd_phi = 0.01191, N_phi = 856.656;
double phi_setpoint = 0;
double phi_integral = 0;
double prev_phi_error = 0;
double filt_phi_derivative = 0;
double Phi;

/* total PID variables */
double Kp[2] = {Kp_theta, Kp_phi}, Ki[2] = {Ki_theta, Ki_phi}, Kd[2] = {Kd_theta, Kd_phi}, N[2] = {N_theta, N_phi};
double setpoint[2] = {theta_setpoint, phi_setpoint};
double integral[2] = {theta_integral, phi_integral};
double prev_error[2] = {prev_theta_error, prev_phi_error};
double filt_derivative[2] = {filt_theta_derivative, filt_phi_derivative};
double angle[2] = {Theta, Phi};
double PID_gain = 1;

/* PID controller function */
double applyPID(double angle, int angle_mode, double dt){

  if (dt > 0.01) dt = 0.01; // clamp at 20 ms
  if (dt < 0.001) dt = 0.001; // clamp to avoid zero

  double error = setpoint[angle_mode] - angle; // computing error

  /* computing integral */
  double integral_max = 0, integral_min = 0;
  if (Ki[angle_mode] != 0) {
      integral_max = thrust_limit / fabs(Ki[angle_mode]);
      integral_min = -thrust_limit / fabs(Ki[angle_mode]);
  }

  integral[angle_mode] += (error * dt);

  // integral anti-windup
  if(integral[angle_mode] > integral_max){ 
    integral[angle_mode] = integral_max;
  }else if (integral[angle_mode] < integral_min){
    integral[angle_mode] = integral_min;
  }

  /* computing derivative */
  double derivative = (error - prev_error[angle_mode]) / dt;
  double filtered_derivative = (N[angle_mode] * filt_derivative[angle_mode] + derivative) / (N[angle_mode] + 1); // applying a low pass filter on the derivative term
  filt_derivative[angle_mode] = filtered_derivative; // updating previous error
  prev_error[angle_mode] = error; // updating previous error

  // determine PID output
  double force = PID_gain * (Kp[angle_mode] * error) + (Ki[angle_mode] * integral[angle_mode]) + (Kd[angle_mode] * filtered_derivative);

  return force;
}

/* motor control algorthim: takes in PID output and actuates motors*/
double applyCorrectiveTrust(double force, int angle_mode){

  // calculating pwm motor signal to determine motor speed
  int motorSpeed = round(sqrt(fabs(force)/0.0359 ) * (255/(supply_voltage-voltage_drop))); 

  if(motorSpeed > pwm_ceiling){
    motorSpeed = 0; // cutting off thrust when system is saturated
  }else if(motorSpeed > 255){
    motorSpeed = 255; // allowing max thrust when the system is only slightly saturated
  }else if(motorSpeed < pwm_floor){
    motorSpeed = pwm_floor; // turning motor off to prevent buzzing from low pwm
  }

  if((motorSpeed == 0) || (fabs(angle[angle_mode]) > 45)){
    digitalWrite(motorIN1[angle_mode][0], LOW);
    digitalWrite(motorIN2[angle_mode][0], LOW);
    analogWrite(motor[angle_mode][0], 0);
    digitalWrite(motorIN1[angle_mode][1], LOW);
    digitalWrite(motorIN2[angle_mode][1], LOW);
    analogWrite(motor[angle_mode][1], 0);
  }else if(force > 0){
    digitalWrite(motorIN1[angle_mode][0], HIGH);
    digitalWrite(motorIN2[angle_mode][0], LOW);
    analogWrite(motor[angle_mode][0], motorSpeed);
    digitalWrite(motorIN1[angle_mode][1], LOW);
    digitalWrite(motorIN2[angle_mode][1], HIGH);
    analogWrite(motor[angle_mode][1], motorSpeed);
  }else if(force < 0){
    digitalWrite(motorIN1[angle_mode][0], LOW);
    digitalWrite(motorIN2[angle_mode][0], HIGH);
    analogWrite(motor[angle_mode][0], motorSpeed);
    digitalWrite(motorIN1[angle_mode][1], HIGH);
    digitalWrite(motorIN2[angle_mode][1], LOW);
    analogWrite(motor[angle_mode][1], motorSpeed);
  }
  
  return motorSpeed;
}

/* Extended Kalman Filter variables */
Matrix<6,1,double> x_hat = {0, 0, 0, 0, 0, 0}; // state vector x_hat = {theta prediction, phi prediction, theta velocity, phi velocity, gyro bias in x dir, gyro bias in y dir}

Matrix<6,6,double> P = {0.1,   0,     0,     0,     0,     0, // state covariance matrix
                        0,     0.1,   0,     0,     0,     0,
                        0,     0,     0.1,   0,     0,     0,
                        0,     0,     0,     0.1,   0,     0,
                        0,     0,     0,     0,     0.001, 0,
                        0,     0,     0,     0,     0,     0.001};

Matrix<6,6,double> Q = {35,      0,       0,       0,       0,       0, // process noise covariance matrix
                        0,       5,       0,       0,       0,       0,
                        0,       0,       27,      0,       0,       0, 
                        0,       0,       0,       12,      0,       0,
                        0,       0,       0,       0,       0.001,   0,
                        0,       0,       0,       0,       0,       0.002};

Matrix<2,2,double> R = {2000, 0, // measurement noise covariance matrix
                        0, 1000};
                        
Matrix<2,6,double> H = {1, 0, 0, 0, 0, 0, // measurement Jacobian matrix
                        0, 1, 0, 0, 0, 0};

Matrix<6,6,double> F; // state transition Jacobian matrix
Matrix<2,2,double> S; // innovation covariance matrix
Matrix<6,2,double> K; // Kalman gain

Matrix<6,6,double> I = {1,0,0,0,0,0, // identity matrix
                        0,1,0,0,0,0,
                        0,0,1,0,0,0,
                        0,0,0,1,0,0,
                        0,0,0,0,1,0,
                        0,0,0,0,0,1};


/* applies an Extended Kalman Filter */
void KalmanFilter(double gyr_x, double gyr_y, double acc_x, double acc_y, double acc_z, double dt){ 
  
  if (dt > 0.01) dt = 0.01; // clamp at 20 ms
  if (dt < 0.001) dt = 0.001; // clamp to avoid zero

  /* ------------PREDICT------------ */

  // updating state vector
  x_hat(0,0) = x_hat(0,0) + (gyr_y - x_hat(5,0)) * dt; // updating theta prediction
  x_hat(1,0) = x_hat(1,0) + (gyr_x - x_hat(4,0)) * dt; // updating phi prediction
  x_hat(2,0) = gyr_y - x_hat(5,0); // updating theta angular velocity
  x_hat(3,0) = gyr_x - x_hat(4,0); // updating phi angular velocity

  F = {1,  0,  dt, 0,  0,  0, // updating state transition Jacobian matrix
       0,  1,  0,  dt, 0,  0,
       0,  0,  1,  0, -dt, 0,
       0,  0,  0,  1,  0, -dt,
       0,  0,  0,  0,  1,  0,
       0,  0,  0,  0,  0,  1};

  P = F * P * ~F + Q; // covariance prediction


  /* ------------UPDATE------------ */

  S = H * P * ~H + R; // updating the total uncertainty
  S(0,0) += 1e-6; // regularize S Before Inversion
  S(1,1) += 1e-6;
  K = P * ~H * Inverse(S); // computing Kalman gain

  double theta_meas = atan2(acc_x, acc_z); // theta measurment from accelerometer
  double phi_meas = atan2(acc_y, acc_z); // phi measurment from accelerometer
  Matrix<2,1,double> z = {theta_meas, phi_meas}; // measurement vector
  
  x_hat = x_hat + K * (z - (H * x_hat)); // updating state estimates with measurements
  P = (I - K * H) * P; // updating the state covariance matrix
  P = 0.5 * (P + ~P); // forcing symmetry since computational truncation or rounding can prevent symmetry

   // for error checking
  for (int i = 0; i < 6; i++) {
    if (isnan(x_hat(i,0)) || isinf(x_hat(i,0))) {
      Serial.println("Invalid element in x_hat!");
      x_hat(i,0) = 0;
    }
  }2
  for (int i = 0; i < 6; i++) {
    if (isnan(P(i,i)) || isinf(P(i,i))) {
      Serial.println("Invalid element in P!");
      P(i,i) = 0;
    }
  }
}



/*---------------------------------------angle setpoint commands here---------------------------------------*/
bool run_pitch_sweep = false;
unsigned long pitch_sweep_start_t, pitch_sweep_t_elapsed;
void pitchSweep(double pitch_range){ // sweeping pitch angle without ramping
   
  unsigned long elapsed_time = millis() - pitch_sweep_start_t;
  if(elapsed_time < 500){
    theta_setpoint = 0;
  }else if(elapsed_time < 4500){
    theta_setpoint = pitch_range;
  }else if(elapsed_time < 8500){
    theta_setpoint = 0;
  }else if(elapsed_time < 12500){
    theta_setpoint = -pitch_range;
  }else if(elapsed_time < 15500){
    theta_setpoint = 0;
  }else{
    run_pitch_sweep = false;
  }
}

bool run_ramp_pitch_sweep = false;
unsigned long ramp_pitch_sweep_start_t, ramp_pitch_sweep_t_elapsed;
void rampPitchSweep(double pitch_range){ // half a second at 0 to reset, 1 second to ramp to range, 3 seconds to hold, one second to ramp to 0, 1 second to hold (then repeat for negative side), end at 0 and hold for 2 seconds

  pitch_range = fabs(pitch_range); // making sure the range will always be a magnetude
  unsigned long elapsed_time = millis() - ramp_pitch_sweep_start_t;
  if(elapsed_time < 500){
    theta_setpoint = 0;
  }else if(elapsed_time < 1500){
    theta_setpoint += (pitch_range/100);
  }else if(elapsed_time < 4500){
    theta_setpoint = pitch_range;
  }else if(elapsed_time < 5500){
    theta_setpoint -= (pitch_range/100);
  }else if(elapsed_time < 6500){
    theta_setpoint = 0;
  }else if(elapsed_time < 7500){
    theta_setpoint -= (pitch_range/100);
  }else if(elapsed_time < 10500){
    theta_setpoint = -pitch_range;
  }else if(elapsed_time < 11500){
    theta_setpoint += (pitch_range/100);
  }else if(elapsed_time < 13500){
    theta_setpoint = 0;
  }else{
    run_ramp_pitch_sweep = false;
  }
}

/*---------------------------------------------------------------------------------------------------*/



void setup() {
  Serial.begin(115200);
  Serial.setTimeout(0); // changing delay to 0s instead of the default 1s for serial commands 

  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 2; j++){
      pinMode(motor[i][j], OUTPUT);
      pinMode(motorIN1[i][j], OUTPUT);
      pinMode(motorIN2[i][j], OUTPUT);
    }
  }

  while (!Serial){ // waiting until serial console opens
    delay(10);
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1){ 
      delay(10); 
    }
  }

  delay(2000); // delay to prevent serial prompts from being cut

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /*IMU CALIBRATION*/
  Serial.println("IMPORTANT: Calibration sequence assumes custom IMU axis orientation");
  Serial.println("Verify that intended IMU axis and code orientations are similar!");
  Serial.println("===================================================================\n");
  Serial.println("2DOF_PID_CONTROL.cpp");
  Serial.println("Starting IMU calibration in 5 seconds");
  Serial.println("Ensure IMU is stable and upright. Do not move!");
  for(int i = 1; i <= 5; i++){
    Serial.print(i);
    Serial.println("...  ");
    delay(1000);
  }

  Serial.println("\nRecording IMU offsets...");
  sensors_event_t a, g, temp;

  Serial.print("[");
  int offset_sample_count = 2000;
  for(int i = 1; i <= offset_sample_count; i++){
    mpu.getEvent(&a, &g, &temp);
    ACC_X_OFFSET += a.acceleration.x;
    ACC_Y_OFFSET += a.acceleration.y;
    ACC_Z_OFFSET += a.acceleration.z;
    GYR_X_OFFSET += g.gyro.x;
    GYR_Y_OFFSET += g.gyro.y;
    GYR_Z_OFFSET += g.gyro.z;
    
    if (i%(offset_sample_count/5) == 0){
      Serial.print("==");
    }
  }
  
  ACC_X_OFFSET = ACC_X_OFFSET / offset_sample_count;
  ACC_Y_OFFSET = ACC_Y_OFFSET / offset_sample_count;
  ACC_Z_OFFSET = ACC_Z_OFFSET / offset_sample_count - 9.81;
  GYR_X_OFFSET = GYR_X_OFFSET / offset_sample_count;
  GYR_Y_OFFSET = GYR_Y_OFFSET / offset_sample_count;
  GYR_Z_OFFSET = GYR_Z_OFFSET / offset_sample_count;

  Serial.println("] Done!");

  /* Print out offsets */
  Serial.println("------------------------------------------------");
  Serial.print("Acceleration offsets X: ");
  Serial.print(ACC_X_OFFSET);
  Serial.print(", Y: ");
  Serial.print(ACC_Y_OFFSET);
  Serial.print(", Z: ");
  Serial.print(ACC_Z_OFFSET);
  Serial.println(" m/s^2");

  Serial.print("Rotation offsets X: ");
  Serial.print(GYR_X_OFFSET);
  Serial.print(", Y: ");
  Serial.print(GYR_Y_OFFSET);
  Serial.print(", Z: ");
  Serial.print(GYR_Z_OFFSET);
  Serial.println(" rad/s");
  Serial.println("------------------------------------------------");

  
  Serial.println("\n\n-- Beginning control loop in 5 seconds --");
  Serial.println("\nCommands: ");
  Serial.println("test - confirms serial messages and returns dt");
  Serial.println("t# - changes theta setpoint with # being a number between -45 to 45");
  Serial.println("p# - changes phi setpoint with # being a number between -45 to 45");
  Serial.println("a - abort");
  Serial.println("(Units -> accel: m/s^2, gyro: rad/s, angles: deg)");
  
  delay(5000); // delay to prepare for data collection

  Serial.println("\n------------------------------------------------\n\n");
  t0 = micros();
  start_time = micros();
}

void loop() {

  unsigned long t = micros();
  double dt = (t - t0) / 1e6; // since micros() returns in microseconds, we need to convert to seconds
  t0 = t;
  unsigned long elapsed_time = micros() - start_time;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 

  /* Applying offsets */
  double acc_x = -1 * (a.acceleration.x - ACC_X_OFFSET);
  double acc_y = -1 * (a.acceleration.y - ACC_Y_OFFSET);
  double acc_z = a.acceleration.z - ACC_Z_OFFSET;
  double gyr_x = -1 * (g.gyro.x - GYR_X_OFFSET);
  double gyr_y = -1 * (g.gyro.y - GYR_Y_OFFSET);
  double gyr_z = g.gyro.z - GYR_Z_OFFSET;

  /* computing angle values from the Kalman filter */
  KalmanFilter(gyr_x, gyr_y, acc_x, acc_y, acc_z, dt);
  double Theta = x_hat(0,0) * (180/M_PI);
  double Phi = x_hat(1,0) * (180/M_PI);

  double correct_force_x = applyPID(Theta, 0, dt);
  double correct_force_y = applyPID(Phi, 1, dt);

  int motor_speed_x = applyCorrectiveTrust(correct_force_x, 0);
  int motor_speed_y = applyCorrectiveTrust(correct_force_y, 1);

  if(Serial.available()){ // handles serial prompts for user commands
    String cmd = Serial.readStringUntil('\n');
    char cmd_type = cmd[0];

    if(cmd_type == 'a'){ // abort
        
      for(int i = 0; i < 2; i++){
        for(int j = 0; j < 2; j++){
          digitalWrite(motorIN1[i][j], LOW);
          digitalWrite(motorIN2[i][j], LOW);
          analogWrite(motor[i][j], 0);
        }
      }
        
      Serial.println("Program stopped");
      while(1){
        delay(10);
      }
    }

    if(cmd == "pitchsweep"){
      run_pitch_sweep = true;
      Serial.println("Running pitch sweep");
      pitch_sweep_start_t = millis();
    }else if(cmd == "pitchsweepramp"){
      run_ramp_pitch_sweep = true;
      Serial.println("Running pitch sweep (ramp ver.)");
      ramp_pitch_sweep_start_t = millis();
    }

    if(cmd.length() > 1){
      String cmd_val = cmd.substring(1);
      cmd_val.trim();

      const char *cstr = cmd_val.c_str(); // needed to verify if cmd_val can be succesfully converted
      char *endptr;
      double inputNum = strtof(cstr, &endptr);

      if((endptr != cstr) && (*endptr == '\0') && (inputNum >= -90) && (inputNum <= 90)){
        if(cmd_type == 't'){
          theta_setpoint = inputNum;
          Serial.print("New theta setpoint: ");
          Serial.println(theta_setpoint);
        }else if (cmd_type == 'p'){
          phi_setpoint = inputNum;
          Serial.print("New phi setpoint: ");
          Serial.println(phi_setpoint);
        }
      }
    }
  }  

  if(run_pitch_sweep){
    pitchSweep(10);
  }else if(run_ramp_pitch_sweep){
    rampPitchSweep(10);
  }

  // formatting data to be interpreted by Teleplot extension
  Serial.print(">Theta:"); Serial.println(Theta);
  Serial.print(">Phi:"); Serial.println(Phi);
  Serial.print(">theta_setpoint:"); Serial.println(theta_setpoint);
  Serial.print(">phi_setpoint:"); Serial.println(phi_setpoint);

  Serial.print(">motor_speed_x:"); Serial.println(motor_speed_x);
  Serial.print(">motor_speed_y:"); Serial.println(motor_speed_y);

  Serial.print(">time:"); Serial.println(elapsed_time);
  Serial.print(">dt:"); Serial.println(dt);

}