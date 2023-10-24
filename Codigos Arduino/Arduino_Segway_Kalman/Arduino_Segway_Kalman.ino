//software em arduino para aplicação do controle do veículo de duas rodas

#include <Wire.h>
#include <Kalman.h>
#define M1_IN1 4
#define M1_IN2 5
#define M2_IN1 4
#define M2_IN2 5
#define M_PWM 3
#define T_amostragem 2e-3
#define PI_T 3.1415
#define PWM_NVL 4096*6/8
Kalman kalmanX; // Create the Kalman instances
float kalmanY;
/* IMU Data */
double accX, accY, accZ;
double gyroX;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double omega;
double tempo = 0;
float t_estabiliza = 10;
uint32_t N_Ciclo = 0;
uint32_t timer;
uint32_t timer1;
uint8_t i2cData[15]; // Buffer for I2C data
double theta;
double integral = 0;
float uot;
void control()
{
uot = 1.68*(theta * PI_T / 180) + 43.6*(integral*PI_T/180)+
0.0162*(omega*PI_T/180);

 int x;
 x = abs(uot) * PWM_NVL;
 if (x > PWM_NVL) x = PWM_NVL;
 if (uot > 0) {
 analogWrite(M1_IN1, 0);
 analogWrite(M1_IN2, 4096);
 analogWrite(M_PWM, x);
 }
 else if (uot < 0) {
 analogWrite(M1_IN1, 4069);
 analogWrite(M1_IN2, 0);
 analogWrite(M_PWM, x);
 }
 else {
 analogWrite(M1_IN1, 4096);
 analogWrite(M1_IN2, 4096);
 analogWrite(M_PWM, 0);
 }
}
void setup() {
71
 Serial.begin(115200);
 pinMode(M_PWM, OUTPUT);
 pinMode(M1_IN1, OUTPUT);
 pinMode(M1_IN2, OUTPUT);
 pinMode(M2_IN1, OUTPUT);
 pinMode(M2_IN2, OUTPUT);
 pinMode(LED_BUILTIN , OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH );
 Wire.begin();
#if ARDUINO >= 157
 Wire.setClock(100000UL); // Set I2C frequency to 100kHz , Standardmode
#else
 TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set I2C frequency to 100kHz
, Standard-mode
#endif
 i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) =
1000Hz
 i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering,
256 Hz Gyro filtering, 8 KHz sampling
 i2cData[2] = 0x00; // Set Gyro Full Scale Range to Ã‚Â±250deg/s
 i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to Ã‚Â±2g
 delay(100);
 while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four
registers at once
 delay(100);
 while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope
reference and disable sleep mode
 delay(1000);
 while (i2cRead(0x3B, i2cData, 6));
 accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
 accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
 accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
#ifdef RESTRICT_PITCH // Eq. 25 and 26
 double roll = atan2(accY, accZ) * RAD_TO_DEG;
 double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) *
RAD_TO_DEG;
#else // Eq. 28 and 29
 double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) *
RAD_TO_DEG;
 double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
 kalmanX.setAngle(roll); // Set starting angle
 kalmanY = pitch;
 timer = micros();
 timer1 = micros();
}
void loop() {
 /* Update all the values */
 while ((i2cRead(0x3B, i2cData, 14))) {
 }
 accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
 accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
 accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
 gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
72
#ifdef RESTRICT_PITCH // Eq. 25 and 26
 double roll = atan2(accY, accZ) * RAD_TO_DEG;
 double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) *
RAD_TO_DEG;
#else // Eq. 28 and 29
 double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) *
RAD_TO_DEG;
 double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
 omega = (gyroX / 131.0); // Convert to deg/s
 double dt = (double)(micros() - timer) / 1000000; // Calculate delta
time
 timer = micros();
 // This fixes the transition problem when the accelerometer angle
jumps between -180 and 180 degrees
 if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -
90)) {
 kalmanY = pitch;
 } else
 kalAngleY = pitch; // Calculate the angle using a Kalman filter
 if (abs(kalAngleY) > 90) omega = -omega; // Invert rate, so it fits
the restriced accelerometer reading
 //VARIÀVEIS DINÂMICAS//
 omega = omega + 3.8847 + 0.1267;
 kalAngleX = kalmanX.getAngle(roll + 0.1193, omega, dt); // Calculate
the angle using a Kalman filter
 float cft = 0.75;
 theta = (1 - cft) * (0.9 * (theta + omega * dt) + 0.1 * (roll +
0.1193)) + cft * kalAngleX ; // Calculate the angle using a
Complimentary filter
 integral = integral + theta * dt;
 digitalWrite(LED_BUILTIN, HIGH );
 tempo = (double)(micros() - timer1) / 1000000;
 if (tempo > (double) N_Ciclo * T_amostragem) {
 N_Ciclo = N_Ciclo + 1;
 //Serial.print(tempo); Serial.print("\t");
 //Serial.print(omega); Serial.print("\t");
 Serial.print(theta); Serial.print("\t");
 control();
 Serial.print(omega); //Serial.print("\t");
 Serial.print("\r\n");
 }
}
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors
in I2C communication
73
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool
sendStop) {
 return i2cWrite(registerAddress, &data, 1, sendStop); //
Returns 0 on success
}
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data,
uint8_t length, bool sendStop) {
 Wire.beginTransmission(IMUAddress);
 Wire.write(registerAddress);
 Wire.write(data, length);
 uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0
on success
 if (rcode) {
 Serial.print(F("i2cWrite failed: "));
 Serial.println(rcode);
 digitalWrite(LED_BUILTIN, LOW );
 analogWrite(M1_IN1, 4096);
 analogWrite(M1_IN2, 4096);
 analogWrite(M_PWM, 0);
 delay(50000);
 }
 return rcode; // See:
http://arduino.cc/en/Reference/WireEndTransmission
}
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t
nbytes) {
 uint32_t timeOutTimer;
 Wire.beginTransmission(IMUAddress);
 Wire.write(registerAddress);
 uint8_t rcode = Wire.endTransmission(false); // Don't release
the bus
 if (rcode) {
 Serial.print(F("i2cWrite failed: "));
 Serial.println(rcode);
 digitalWrite(LED_BUILTIN, LOW );
 analogWrite(M1_IN1, 4096);
 analogWrite(M1_IN2, 4096);
 analogWrite(M_PWM, 0);
 delay(50000);
74
 return rcode; // See:
http://arduino.cc/en/Reference/WireEndTransmission
 }
 Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send
a repeated start and then release the bus after reading
 for (uint8_t i = 0; i < nbytes; i++) {
 if (Wire.available())
 data[i] = Wire.read();
 else {
 timeOutTimer = micros();
 while (((micros() - timeOutTimer) < I2C_TIMEOUT) &&
!Wire.available());
 if (Wire.available())
 data[i] = Wire.read();
 else {
 Serial.println(F("i2cRead timeout"));
 return 5; // This error value is not already taken by
endTransmission
 }
 }
 }
 return 0; // Success
}
