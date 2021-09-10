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
    R0 = read_resistance();
    SW_state++;
  }
  else if (SW_state == 1) {
    writeSPI(Res1SW);
    delay(10); // wait for MUX to switch over
    R1 = read_resistance();
    SW_state++;
  }
  else if (SW_state == 2) {
    writeSPI(Res2SW);
    delay(10); // wait for MUX to switch over
    R2 = read_resistance();
    SW_state = 0;
    plot_R_vals(R0, R1, R2);
  }
  else {
    Serial.println("What's cooking here..."); // Error state
  }
  delay(20);  
}

float read_resistance(void){
  // Calculate resistance value from measured voltage and known current source
  float R_val = analogRead(StrainSensePin);
  return (R_val/ADC_const) / I_src; // R = V_meas/I_src
}

void plot_R_vals(float R0,float R1,float R2){
  // Plots 3 values in parallel when arduino plotter open
  // >> Could generalise this for any number of simulataneous data points
  Serial.print("R0: "); Serial.print(R0); Serial.print("  ");
  Serial.print("R1: "); Serial.print(R1); Serial.print("  ");
  Serial.print("R2: "); Serial.print(R2); Serial.print("  ");
  Serial.println("V");
  return 0;
}


void writeSPI(byte thisValue) {
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}
