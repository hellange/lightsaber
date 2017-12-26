/* Presented without warranty, guarantee and support.  
 *  By Bob Clagett - I Like To Make Stuff
 *  
 * This is the code to accompany the lightsaber project at http://www.iliketomakestuff.com/make-lightsaber/
 * Expected hardware is listed in post (Teensy 3.2, Prop Shield, DotStar LEDs)
 * 
 * This is functional v1 of this code.  Lots more I want to add to it including crash detection/sounds.
 * This code is dirty, bulky, poorly documented and ugly but I finally got it all functional.  
 * Cleanup to come later in v2.
*/


#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
// GUItool: begin automatically generated code
AudioPlaySerialflashRaw  playFlashRaw1;  //xy=149,388
AudioPlaySerialflashRaw  playHUMraw;  //xy=149,388
AudioPlaySerialflashRaw  playSwingRaw;  //xy=149,388
AudioMixer4              mixer1;         //xy=445,386
AudioOutputAnalog        dac1;           //xy=591,379
AudioConnection          patchCord4(playSwingRaw, 0, mixer1, 2);
AudioConnection          patchCord1(playFlashRaw1, 0, mixer1, 1);
AudioConnection          patchCord3(playHUMraw, 0, mixer1, 0);
AudioConnection          patchCord2(mixer1, dac1);
// GUItool: end automatically generated code

#include <Adafruit_DotStar.h>
//#include <FastLED.h>
#include <NXPMotionSense.h>

NXPMotionSense imu;
NXPSensorFusion filter;

int motionThreshold = 2;

float lastPitch = 0, lastRoll = 0, lastHeading = 0;

int buttonState = HIGH;
int bladeState = 0; //0 off, 1, fading up, 2 on, 3 fading down
int fadeStep = 0;
int fadeStepSize = 4;
int selectedColor = 0;
int lastSelectedColor = selectedColor;
bool isAnimating = 0;
bool bladeOn = 0;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 20;            
int lastButtonState = HIGH;
int pendingPress = 0;

//BC = Blade Color
int BCbuttonState = HIGH;
long BClastDebounceTime = 0;  // the last time the output pin was toggled
long BCdebounceDelay = 50;            
int BClastButtonState = HIGH;
int BCpendingPress = 0;
int pendingColorPress = 0;

#define COLOR_BUTTON_PIN 1
#define POWER_BUTTON_PIN 0
//#define COLOR_ORDER BGR
#define CHIPSET     APA102
#define NUMPIXELS    144


// Reduce this while programming to avoid current/voltage drop and noise...
#define BRIGHTNESS  120  // 0..255?



#define FRAMES_PER_SECOND 7

#define PROP_AMP_ENABLE 5
#define FLASH_CHIP_SELECT 6
#define LED_BUFFER_SELECT 7

#define VOLUME_POT 15

#define LED_PIN    7
Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DOTSTAR_BRG);
  
//#define NUM_LEDS 5
//CRGB leds[NUM_LEDS];


// orange 0x09af00 (trial and error)
    
int outputValue = 0;// BB RR GG ???
//int presetColors[] = {0x0000FF,0x00FF00,0xFF0000,0x09af00,  0xFFFF00, 0xFF00FF, 0x00FFFF, 0xFFFFFF};
int presetColors[] = {0x0000FF,0x00FF00,0xFF0000,0xFFFF00, 0xFF00FF, 0x00FFFF, 0xFFFFFF,0x09af00 };

int totalPresetColors = (sizeof(presetColors)/sizeof(int));
int selectedColorIndex = 0
;

int swingSounds = 4;
int lastSwingSound = swingSounds;

int clashSounds = 0;
void setup() {
  delay(500); // sanity delay
  imu.begin();
  filter.begin(100);
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

//  FastLED.addLeds<APA102, BGR>(leds, NUM_LEDS);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);  // enable access to LEDs


  pinMode(POWER_BUTTON_PIN, INPUT);
  pinMode(COLOR_BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(POWER_BUTTON_PIN, HIGH);
  digitalWrite(COLOR_BUTTON_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);


  Serial.begin(115200);
 // wait up to 3 seconds for the Serial device to become available
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 3000))
    ;

  Serial.println ("Start prop shield RAW player");
SPI.begin();
  // Enable the amplifier on the prop shield
  pinMode(PROP_AMP_ENABLE, OUTPUT);
  digitalWrite(PROP_AMP_ENABLE, HIGH);
  pinMode(LED_BUFFER_SELECT, OUTPUT);
  digitalWrite(LED_BUFFER_SELECT, HIGH);

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(8);

  // Set initial volume
   mixer1.gain(1, 1.5f); //other sounds
   mixer1.gain(0, 1.5f); //hum

  // Start SerialFlash
  if (!SerialFlash.begin(FLASH_CHIP_SELECT)) {
    while (1)
      {
  Serial.println ("Cannot access SPI Flash chip");
  delay (1000);
      }
  }
  //triggerSound("saberswing1.raw");
  //triggerSound("click.raw");
    //triggerSound("HUM2.wav");

   
    // powerUpBlade();

  

}

void powerUpBlade(){
  
  isAnimating = 1;
  bladeState = 1;
  Serial.println("Turn on blade");
  bladeOn = 1;
    triggerSound("ltsaberon01.wav");
    //triggerSound("HUM2.wav");

  //triggerSound("saberon.raw");
 }
 
void powerDownBlade(){
 //animate DOWN 
 triggerSound("saberoff.raw");
 bladeState = 3;
 fadeStep = NUMPIXELS;
 isAnimating = 1;
 Serial.println("Turn off blade");
 stopHum();
 bladeOn = 0;
}
  
void bladeIsOn(){
  
  if(!playHUMraw.isPlaying()){
      startHum();
  }
  //nothing needed here unless adding animation
}

void bladeIsAnimatingUp(){
  Serial.println("bladeIsAnimatingUp 2");

  int midpoint = NUMPIXELS/2;
  int newSection = fadeStep+fadeStepSize;
  for( int j = fadeStep; j < newSection; j++) {

    strip.setPixelColor(j, selectedColor);
    strip.setPixelColor(NUMPIXELS-j, selectedColor);
       //Serial.println("step");

  }
  fadeStep = newSection;
    if (fadeStep >= midpoint+fadeStepSize){
    
     fadeStep = NUMPIXELS;
      isAnimating=0;
      bladeState = 2; 
      delay(100);
      Serial.println("blade up complete 2");
      startHum();
    }
    }
    
void bladeIsAnimatingDown(){
  //Serial.println("bladeIsAnimatingDown");
  int midpoint = NUMPIXELS/2;
  int newSection = fadeStep-fadeStepSize;
  for( int j = fadeStep; j > newSection; j--) {
    
     strip.setPixelColor(j-midpoint, 0x000000);
     
     strip.setPixelColor(midpoint+NUMPIXELS-j, 0x000000);
  }
  //Serial.println(fadeStep);
  fadeStep = newSection;
  
  if (fadeStep <=midpoint-fadeStepSize){
    fadeStep = 0;
      isAnimating=0;
      bladeState = 0;  
    }
}

boolean maybe_roll=false;
void detectMotion() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
          //Serial.println("check motion");

    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
    // does it need a time threshold too?
    
    float headingDiff = abs(lastHeading - heading);
    float pitchDiff = abs(lastPitch - pitch);
        float rollDiff = abs(lastRoll - roll);

    if(lastHeading != 0){
      if(pitchDiff > motionThreshold || headingDiff > motionThreshold){      
      //cyle through  sounds
      lastSwingSound++;
      if(lastSwingSound>swingSounds){
        lastSwingSound=1;
      }
      String swingFile = "saberswingX.raw";
      swingFile.replace("X",lastSwingSound);
      char charBuf[50];
      swingFile.toCharArray(charBuf, 50); 
      //triggerSwing(charBuf);  // needs sequence to iterate through
      triggerSwing("ltsaberswing08.wav");
            Serial.println("motion detected");

      }
    }

    if (lastRoll !=0) {
      if (headingDiff < motionThreshold * 2) {

         if(rollDiff > motionThreshold *4 && maybe_roll==false){
           Serial.println("Maybe ROLL motion detected !!!!!!!!!!!!!!!!!");
           maybe_roll = true;
           nextColor();

//         } else if( rollDiff > motionThreshold *4 && maybe_roll==true){
//           Serial.println("ROLL motion detected !!!!!!!!!!!!!!!!!");
//           nextColor();
//           maybe_roll = false;
         } else {
           maybe_roll = false;
        }
      } else {
         Serial.println("Dont care about roll when in motion...");
         maybe_roll = false;
      }

    }

    
    lastHeading = heading;
    lastPitch = pitch;
        lastRoll = roll;















    
  }

}

bool firstTime=true;
void loop(){
  if (firstTime) {
    firstTime=false;
   // powerUpBlade();
  }

  // Add entropy to random number generator; we use a lot of it.
 // random16_add_entropy( random() );
  
  //handle color selector button
  int BCreading = digitalRead(COLOR_BUTTON_PIN);
   
  if (BCreading != BClastButtonState) {
    // reset the debouncing timer
    BClastDebounceTime = millis();
  }

  if ((millis() - BClastDebounceTime) > BCdebounceDelay) {
    if (BCreading != BCbuttonState) {
      BCbuttonState = BCreading;
      
      if(BCbuttonState == HIGH){
        Serial.println("CHANGE COLOR BUTTON PRESSED!!!!!!!!!!!!!!!");
        nextColor();
      }
    }
  }
  BClastButtonState = BCreading;
  
  selectedColor = presetColors[selectedColorIndex];
   

if(selectedColor != lastSelectedColor && bladeState == 2 /* on */){
    Serial.println("COLOR CHANGE");
        Serial.println(selectedColorIndex);


    for( int j = 0; j < NUMPIXELS; j++) {
      strip.setPixelColor(j,selectedColor);
      
    }
  }
  lastSelectedColor = selectedColor;
  

//handle blade on/off
  
  int reading = digitalRead(POWER_BUTTON_PIN);
  //debounce
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
    }
  }
  lastButtonState = reading;
   
  if( pendingPress == 1 && buttonState != LOW){
    buttonState = HIGH;
      pendingPress = 0;
      Serial.print("execute pending blade press");
     
       if(bladeOn){
        powerDownBlade();
      } else {  
        powerUpBlade();
      }
    }
      
if (buttonState == LOW && !isAnimating) {
  Serial.println("BLADE BUTTON IS PRESSED");
      pendingPress = 1;
    }
  switch(bladeState){
      case 0:
      //blade is off
        break;
      case 1:
        //blade is animating up
        bladeIsAnimatingUp();
        break;
      case 2:
        //blade is on
        bladeIsOn();
        break;
      case 3:
        //blade is animating down
        bladeIsAnimatingDown();
        break;
      }
      SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
      digitalWrite(LED_PIN, HIGH);  // enable access to LEDs
      strip.setBrightness(BRIGHTNESS);
      strip.show();                     // Refresh strip

//// Move a single white led 
//  for(int n = 0; n < NUM_LEDS; n++) {
//    leds[n] = CRGB::Red;
//    FastLED.show();
//    delay(8);
//    leds[n] = CRGB::Black;
//  }
      
      digitalWrite(LED_PIN, LOW);
      SPI.endTransaction();   // allow other libs to use SPI again

      delay(10);
  
  if(bladeState == 2){
    detectMotion();
  }
        
}
void nextColor(){
    selectedColorIndex++;
    if(selectedColorIndex >= totalPresetColors){
      selectedColorIndex = 0;
    }
 }
void triggerSound(const char *filename){
  playFlashRaw1.play(filename);
}
void triggerSwing(const char *filename){
  if(playSwingRaw.isPlaying()==0){
    playSwingRaw.play(filename);
  } else {
    Serial.println("already swinging");
    }
}
void startHum(){
  //Serial.println("startHum");
  //playHUMraw.play("HUM2.wav");
  //Serial.println(playHUMraw.isPlaying());
  }
void stopHum(){
  playHUMraw.stop();
  }
