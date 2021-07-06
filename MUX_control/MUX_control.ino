/* MUX 4:1 (x2) MAX350 IC
 *
 *  Pins:
  * CS - to digital pin 10  (SS pin)
  * SDI - to digital pin 11 (MOSI pin)
  * CLK - to digital pin 13 (SCK pin)
  *
  * SPI MSG Bits:
  * D7 = SW0B
  * D6 = SW1B
  * D5 = SW2B
  * D4 = SW3B
  * D3 = SW0A
  * D2 = SW1A
  * D1 = SW2A
  * D0 = SW3A
  * So if the message sent = 10001000, this would turn on SW0B and SW0A

TODO:
1. Make generalised function for obtaining resistance value from each joint
2. Put circuit on a vero board!

*/


// include the SPI library:
#include <SPI.h>

// Declare pins
const int chipSelectPin = 10;
const int RSTPin = 8;
const int StrainSensePin = A1;

// Cmds to activate switch states
byte Res0SW = 0b01000001;
byte Res1SW = 0b00100010;
byte Res2SW = 0b00010100;

int SW_state = 0;

// Const current source value
float I_src = 10e-6;

// ADC const max_val/volt (1024/5)
float ADC_const = 204.8;

// Resistance values
float R0;
float R1;
float R2;

void setup() {
  // Set the RST pin high
  pinMode(RSTPin, OUTPUT);
  digitalWrite(RSTPin, 1);

  // Setup analog strain pin
  pinMode(StrainSensePin, INPUT); // can set INPUT_PULLUP too

  // Set the chipSelectPin as an output:
  pinMode(chipSelectPin, OUTPUT);

  // Init serial
  Serial.begin(9600);

  // initialize SPI:
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.begin();
}



void loop() {
  if (SW_state == 0) {
    writeSPI(Res0SW);
    delay(10); // wait for MUX to switch over
    float R0_val = analogRead(StrainSensePin);
    R0 = (R0_val/ADC_const) / I_src; // R = V_meas/I_src
    // Plot the raw Res0 sensor reading
    Serial.print("R0: "); Serial.print(R0_val); Serial.print("  ");
    SW_state++;
  }
  else if (SW_state == 1) {
    writeSPI(Res1SW);
    delay(10); // wait for MUX to switch over
    float R1_val = analogRead(StrainSensePin);
    R1 = (R1_val/ADC_const) / I_src; // R = V_meas/I_src
    // Plot the raw Res1 sensor reading
    Serial.print("R1: "); Serial.print(R1_val); Serial.print("  ");
    SW_state++;
  }
  else if (SW_state == 2) {
    writeSPI(Res2SW);
    delay(10); // wait for MUX to switch over
    float R2_val = analogRead(StrainSensePin);
    R2 = (R2_val/ADC_const) / I_src; // R = V_meas/I_src
    // Plot the raw Res1 sensor reading
    Serial.print("R2: "); Serial.print(R2_val); Serial.print("  ");
    Serial.println("V");
    SW_state = 0;
  }
  else {
    Serial.println("What's cooking here..."); // Error state
  }
  delay(20);

  // Serial.println(SW_state);

  // Show plot 3 R_vals data points
  
}

void write_data()


void writeSPI(byte thisValue) {
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}
