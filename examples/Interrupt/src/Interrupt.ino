#include <BMI160Gen.h>
#include <Wire.h>

const int bmi160_i2c_addr = 0xE9;
const int bmi160_select_pin = 10;
const int bmi160_interrupt_pin = 32;
volatile int interrupt_raised = 0;
volatile int interrupt_raised_count = 0;
const int interrupt_axis = 7;

void bmi160_intr(void)
{
  interrupt_raised = 1;
  interrupt_raised_count += 1;
}

void setup() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  //BMI160.begin(BMI160GenClass::SPI_MODE, bmi160_select_pin, bmi160_interrupt_pin);
  BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
  BMI160.attachInterrupt(bmi160_intr);
  BMI160.setIntMotionEnabled(true, interrupt_axis);
  BMI160.setInterruptMode(0);
  BMI160.setAccelerometerRange(BMI160_ACCEL_RANGE_16G);
  BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_250);
  BMI160.setDetectionThreshold(CURIE_IMU_MOTION, 250);
  Serial.println("Initializing IMU device...done.");

  /*
  Wire.begin(CUSTOM_I2C_SDA_PIN, CUSTOM_I2C_SCL_PIN);
  for(int i = 0 ; i < 256 ; i++) {
    Wire.beginTransmission(i);
    if( Wire.endTransmission() == 0 ) {
      Serial.print(i);
      Serial.println(" found device.");
    }
  }
  */

}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

void loop() {
  if (interrupt_raised) {
    BMI160.setIntMotionEnabled(false, interrupt_axis);
    //int16_t x = 0, y = 0, z = 0;
    //BMI160.getAcceleration(&x, &y, &z);
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    float gx, gy, gz;

    // read raw gyro measurements from device
    BMI160.readGyro(gxRaw, gyRaw, gzRaw);

    // convert the raw gyro data to degrees/second
    gx = convertRawGyro(gxRaw);
    gy = convertRawGyro(gyRaw);
    gz = convertRawGyro(gzRaw);

    interrupt_raised = 0;
    Serial.print("Interrupted:");
    Serial.println(interrupt_raised_count);
    //Serial.printf("X: %d Y: %d Z: %d\n", x, y, z);
    Serial.printf("X: %f Y: %f Z: %f\n", gx, gy, gz);
    BMI160.setIntMotionEnabled(true, interrupt_axis);
  }
}
