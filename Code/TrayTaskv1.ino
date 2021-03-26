/// Rotating Tray Task Code - to be uploaded to ESP32 board in enclosure
/// Author: Shiv Dalla, sdalla@kumc.edu
/// November 2020

#include "SH1106.h" 
#include "Wire.h"

//OUTPUT PINS
//PINS 25, 26 are DAC 1 and DAC 2, only need 1 
int dacOUT = 25; //analog output, to INTAN
int ledCOutput = 32; //output for PWM to level shifter -> motor driver
int builtinLED = 2; //LED on board, also output to INTAN, also LED output
int ledOUT = builtinLED; //change if external LED added
int dirPin = 18; //direction output to level shifter -> motor driver
int enablePin = 33; //output to enable/disable to level shifter -> motor driver

//INPUT PINS
int pulseInput = 23; //LEDc input to check step
int resPin = 39; //photoresistor pin
int potPin = 36; //potentiometer input 
int calibBPin = 5; // calibration button input pin 
int zeroBPin = 17; // button for 'zero position set' pin 
int dirSwitchPin = 27; //input to switch directions
int enableSwitchPin = 14; //input to enable/disable 

//Other initializations 
bool dirBool = 1;
bool disableBool = 1;
int Position = 0;
long stepsPerRev = 200; //dependent on stepper used, most are 200
long microStepFactor = 8; //dependent on setting on stepper motor driver
long fullRotation = stepsPerRev*microStepFactor; 

// Set a limit on the velocity, in RPM 
float maxVelocity = 60; 
float minVelocity = 0;
// Change that into max pulses freq, in Hz
float maxFreq = maxVelocity*fullRotation/60.0;
float minFreq = minVelocity*fullRotation/60.0;
double freq;

long count = 0;
float angle;
float velocity;
int potValue;
int resValue;
int resFilt;
int angleOut;

//Filter Parameters
const float alpha = 0.15;
double data_filtered[] = {0, 0};
const int n = 1;

//initialize LCD
//SCL on 21, SDA on 22
SH1106 display(0x3c, 21, 22); 


//Can change this value by clicking calibration button
int photoResCutoff = 250;
int photoResCutoffBuffer = 300;

//Intterupt function to increment or decrement the count variable, for position
void IRAM_ATTR isr(){
  //check if the enable switch is enabled/disabled
  if(!disableBool){
    //if the direction is one way
    if(dirBool){
      //decrememnt the count 'position'
      count--;
    }
    else{
      //if its the other direction, increase count
      count++;
    }
    //dont let the value go negative, its between 0-359degrees
    if(count<0){
      count = fullRotation;
    }
  }
}

void setup() {
  //Begin serial usb comms
  Serial.begin(115200);
  //Set all output pins to outputs
  pinMode(ledCOutput, OUTPUT);
  pinMode(builtinLED, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(ledOUT, OUTPUT);
  pinMode(resPin, INPUT);

  //button pins setup, uset internal pullup resistor
  pinMode(calibBPin, INPUT_PULLUP);
  pinMode(zeroBPin, INPUT_PULLUP);

  //Set direction, enable it
  digitalWrite(dirPin, dirBool);
  digitalWrite(enablePin, disableBool);
  
  //LEDC
  ledcAttachPin(ledCOutput, 1); // assign pins to channels
  ledcSetup(1, 0, 10); // Start at 0 Hz PWM, 10-bit resolution
  
  //attach interupt to interupt function above
  attachInterrupt(pulseInput, isr, FALLING);

  //SCREEN INITIALIZE 
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Rebooted!");
  display.display();
  delay(2000);

}

void loop() {
  //check switch states, set direction and enable/disable
  dirBool = digitalRead(dirSwitchPin);
  disableBool = !digitalRead(enableSwitchPin);
  digitalWrite(dirPin, dirBool);
  digitalWrite(enablePin, disableBool);


  resValue = analogRead(resPin); // photores input 0 to 4095, 0.0V to 3.3V

  //Filter the resValue
  // Low Pass Filter
  data_filtered[n] = alpha * resValue + (1 - alpha) * data_filtered[n-1];
  // Store the last filtered data in data_filtered[n-1]
  data_filtered[n-1] = data_filtered[n];
  resFilt = data_filtered[n];
  /*
  Serial.print(resValue); //USB print the photores value, just for debugging
  Serial.print(",");    
  Serial.println(resFilt);
  */
  //check zero button state, set it
  if(!digitalRead(zeroBPin)){
    count = 0;
    //Serial.println("Zero-d");
  }

  //check calib button state, set new calibration value
  if(!digitalRead(calibBPin)){
    photoResCutoff = resFilt - photoResCutoffBuffer;
    //Serial.println("Calib-d");
  }

  
  // consider converting these to integers to decrease runtime? -- tbd
  potValue = analogRead(potPin); // potentiometer input 0 to 4095, 0.0V to 3.3V
  freq = map(potValue, 4095, 0, minFreq, maxFreq); // map the pot input to correct frequency value
  angle = 360.0*(count%fullRotation)/(fullRotation); // convert the count to angular position
  velocity = freq*60.0/fullRotation; //convert the frequency to angular velocity in RPM

  // Outputing stuff, angle and pellet present boolean
  // need to map between 0 and 255, 255 is max output
  angleOut = map(angle, 0, 360, 0, 255);
  dacWrite(dacOUT, angleOut); //write value to dac
  //if the pellet is gone, turn on the LED
  digitalWrite(ledOUT, resFilt<photoResCutoff); // turn on LED if pellet isnt there
  digitalWrite(builtinLED, !(resValue<photoResCutoff));
  
  /*
  // Printing Stuff, for debugging
  Serial.print(angle);
  Serial.print(";    ");
  Serial.print(velocity);
  Serial.print(";    ");
  Serial.print(resValue);
  Serial.print(";    ");
  Serial.println(resValue<photoResCutoff);
  
  Serial.print(dirBool);
  Serial.print("     ");
  */
  //Serial.println(disableBool);
  
  

  //does not work well below 10Hz, but thats okay because 10Hz is < 4RPM (impractically slow)
  if(!disableBool){
    ledcWriteTone(1, freq);
  }
  else{
    ledcWriteTone(1, 0);
  }
  delay(100);//add small delay to make output consistent speed
  
  //LCD displaying stuff
  display.drawString(0, 0, "Speed (RPM):");
  display.drawString(0, 16, String(velocity));
  display.drawString(0, 32, "Position:");
  display.drawString(80, 32, "Reach:");
  display.drawString(0, 32+16, String(angle)+"*");
  display.drawString(80, 32+16, String(resFilt<photoResCutoff));
  display.display();
  display.clear();
  
}
