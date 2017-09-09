#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputAnalog         adc1;           //xy=255,182
AudioFilterBiquad        biquad1;        //xy=394,182
AudioPlaySerialflashRaw  playFlashRaw1;  //xy=535,319
AudioFilterStateVariable filter1;        //xy=558,189
AudioAnalyzePeak         peak1;          //xy=559,255
AudioMixer4              mixer1;         //xy=710,196
AudioOutputAnalog        dac1;           //xy=844,196
AudioConnection          patchCord1(adc1, biquad1);
AudioConnection          patchCord2(biquad1, 0, filter1, 0);
AudioConnection          patchCord3(biquad1, peak1);
AudioConnection          patchCord4(playFlashRaw1, 0, mixer1, 2);
AudioConnection          patchCord5(filter1, 0, mixer1, 0);
AudioConnection          patchCord6(filter1, 2, mixer1, 1);
AudioConnection          patchCord7(mixer1, dac1);
// GUItool: end automatically generated code



// Pins
const int FLASH_CS = 6;               // Serial flash chip select
const int AMP_ENABLE = 5;             // Amplifier enable pin



void setup() {

  Serial.begin(9600);

  // Initialize amplifier
  AudioMemory(20);
  //dac1.analogReference(EXTERNAL); // much louder!
  delay(50);                      // time for DAC voltage stable
  pinMode(AMP_ENABLE, OUTPUT);

  // Initialize serial flash
  if ( !SerialFlash.begin(FLASH_CS) ) {
    Serial.println( "Unable to access SPI Flash chip" );
  }

  Serial.println("Finished init");
}

void playFile( const char* filename ) {

  Serial.print("Playing file: ");
  Serial.print(filename);

  // Start playing the file
  playFlashRaw1.play(filename);

  // A brief delay for the library read info
  delay(5);

  // Wait for the file to finish playing
  while ( playFlashRaw1.isPlaying() );

  pinMode(AMP_ENABLE, 0);

}



void loop() {
  digitalWrite(AMP_ENABLE, HIGH);
  //playFile("lightsaber_01.wav");

  playFile("lightsaber_03.wav");
  //playFile("lightsaber_04.wav");
  Serial.println("playing...");
    delay(5000);

}

