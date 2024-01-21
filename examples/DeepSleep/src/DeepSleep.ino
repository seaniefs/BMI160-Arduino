#include <BMI160Gen.h>
#include <Wire.h>
#include <driver/rtc_io.h>

const int bmi160_i2c_addr = 0xE9;
const int bmi160_select_pin = 10;
volatile int interrupt_raised = 0;
volatile int interrupt_raised_count = 0;
const int interrupt_axis = 7;
const gpio_num_t bmi160_interrupt_pin = GPIO_NUM_32;

void enable_motion_wake() {
    // Cold boot
    // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
    // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    // No need to keep that power domain explicitly, unlike EXT1.
    Serial.println("Enabling motion wake");
    rtc_gpio_init(bmi160_interrupt_pin);
    rtc_gpio_set_direction(bmi160_interrupt_pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(bmi160_interrupt_pin);
    rtc_gpio_pulldown_dis(bmi160_interrupt_pin);
    rtc_gpio_hold_en(bmi160_interrupt_pin);
    // can wait for bmi160_interrupt_pin to be falling
    Serial.println("Enabling BMI160 module");
    BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
    BMI160.setAccelerometerRange(BMI160_ACCEL_RANGE_16G);
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_250);
    BMI160.setDetectionThreshold(CURIE_IMU_MOTION, 250);
    BMI160.setInterruptMode(0);
    BMI160.setIntMotionEnabled(true, interrupt_axis);
    delay(100);
    // Start deep sleep
    esp_sleep_enable_ext0_wakeup(bmi160_interrupt_pin, 1); //1 = High, 0 = Low
    Serial.println("Sleeping");
    delay(100);
    esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  esp_reset_reason_t r = esp_reset_reason();

  if(r != ESP_RST_DEEPSLEEP) {
    enable_motion_wake();
  }
  else {
    Serial.println("Woken up!");
    enable_motion_wake();
  }

}

void loop() {
}
