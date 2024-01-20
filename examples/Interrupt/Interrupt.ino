#include <BMI160Gen.h>
#include <Wire.h>

const int bmi160_i2c_addr = 0xE9;
const int bmi160_select_pin = 10;
const int bmi160_interrupt_pin = 32;
volatile int interrupt_raised = 0;
volatile int interrupt_raised_count = 0;
const int interrupt_axis = 1;

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

void loop() {
  if (interrupt_raised) {
    BMI160.setIntMotionEnabled(false, interrupt_axis);
    interrupt_raised = 0;
    Serial.print("Interrupted:");
    Serial.println(interrupt_raised_count);
    BMI160.setIntMotionEnabled(true, interrupt_axis);
  }
}
