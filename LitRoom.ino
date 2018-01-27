#define ARM_MATH_CM4
#include <arm_math.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#define STRIP1_SIG 10

#define BASS_SCALER 1.5
#define MID_SCALER 8
#define HIGH_SCALER 10

#define MIN_BRIGHTNESS 20

long switchStart = 0;

int SAMPLE_RATE_HZ = 9000;             // Sample rate of the audio in hertz.
float SPECTRUM_MIN_DB = 10.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 60.0;          // Audio intensity (in decibels) that maps to high LED brightness.
int LEDS_ENABLED = 1;                  // Control if the LED's should display the spectrum or not.  1 is true, 0 is false.
                                       // Useful for turning the LED display on and off with commands from the serial port.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = 17;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 13; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).
const int STRIP1_PIN = STRIP1_SIG;           // Output pin for neo pixels.
const int STRIP1_COUNT = 960;         // Number of neo pixels.  You should be able to increase this without

// Song maximum trackers
int maxB = 0;
int maxM = 0;
int maxH = 0;

int bassBin;
int colorNum = 0;
double curBass;
double bassPropMax = 0;
bool wasOff;
int mode = 3;

#define maxBass 80000
#define maxMid 60000
#define maxHigh 10000

int bassEndBucket;
int bassStartBucket = 1;
int midStartBucket;
int midEndBucket;
int highStartBucket;
int highEndBucket = 255;

float bassMag;
float midMag;
float highMag;

IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(STRIP1_COUNT, STRIP1_PIN, NEO_GRB + NEO_KHZ800);

int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

void changeColor() {
  colorNum++;
  if(colorNum == 7) colorNum = 0;
}

uint32_t getColor(int brightness, int colChoice) {
  switch(colChoice) {
    case 0: return strip1.Color(brightness, 0, 0);
    case 1: return strip1.Color(0, brightness, 0);
    case 2: return strip1.Color(0, 0, brightness);
    case 3: return strip1.Color(brightness, brightness, 0);
    case 4: return strip1.Color(0, brightness, brightness);
    case 5: return strip1.Color(brightness, 0, brightness);
    case 6: return strip1.Color(brightness, brightness, brightness);
  }
  return 0;
}

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// if strip = 0, strip 1; if strip = 1, strip 2.
void blankStrip(int strip) {
  for(int i = 0; i < STRIP1_COUNT; i++) {
    strip1.setPixelColor(i, strip1.Color(0, 0, 0));
  }
}

void setRangeToColor(int start, int end, int brightness, int color, int strip) {
  blankStrip(0);
  blankStrip(1);
  
  if(brightness < MIN_BRIGHTNESS) {
    brightness = 0;
  } 

  if(strip == 0) {
    for(int i = start; i < end; i++) {
      strip1.setPixelColor(i, getColor(brightness, color));
    }
  }
  
  strip1.show();
}

void setAllToColor(int brightness, int color) {
  if(brightness < MIN_BRIGHTNESS) {
    brightness = 0;
  } 
  
  for(int i = 0; i < STRIP1_COUNT; i++) {
    strip1.setPixelColor(i, getColor(brightness, color));
  }

  strip1.show();
}

void setAllColorSwitching(int brightness) {
  if(brightness < MIN_BRIGHTNESS) {
    brightness = 0;
    wasOff = true;
  } 

  if(wasOff && brightness > MIN_BRIGHTNESS) {
    wasOff = false;
    changeColor();
  }

  setAllToColor(brightness, colorNum);
}

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

float averageRange(int buck1, int buck2) {
  float diff = buck2 - buck1;
  float sum = 0;
  for (int i = buck1; i <= buck2; i++) {
    sum += magnitudes[i];
  }
  return (sum/diff);
}

// 0 = bass, 1 = mid, 2 = high
float getRangeBrightness(int range) {
  switch(range) {
    case 0: return round(((bassMag/maxBass) * BASS_SCALER) * 255);
    case 1: return round(((midMag/maxMid) * MID_SCALER) * 255);
    case 2: return round(((highMag/maxHigh) * HIGH_SCALER) * 255);
  }
  return 0;
}

float highestInRange(int start, int end) {
  float max = 0;
  for (int i = start; i < end; i++) {
    if (magnitudes[i] > max) {
      max = magnitudes[i];
    }
  }
  return max;
}

float lowestInRange(int start, int end) {
  float min = 1000000;
  for (int i = start; i < end; i++) {
    if (magnitudes[i] < min) {
      min = magnitudes[i];
    }
  }
  return min;
}


float getBassMag() {
  //return averageRange(1, midStartBucket - 1);
  return magnitudes[frequencyToBin(60)];
}

float getMidMag() {
  //return averageRange(midStartBucket, midEndBucket);
  return magnitudes[frequencyToBin(600)];
}

float getHighMag() {
  return averageRange(highStartBucket, highEndBucket);

  //return lowestInRange(highStartBucket, highEndBucket);
  //return magnitudes[frequencyToBin(3000)];
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}

void setup() {
  analogReadResolution(13);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
  pinMode(17, INPUT);
  Serial.begin(115200);

  bassBin = frequencyToBin(60.0);
  midStartBucket = frequencyToBin(1000.0);
  midEndBucket = frequencyToBin(2000.0);
  highStartBucket = midEndBucket + 1;

  strip1.begin();
  strip1.show();
  blankStrip(0);
  blankStrip(1);

  samplingBegin();
}

// Maps each of the three frequency ranges to a third of the strip
// and lights up in proportion to frequency range magnitude of contribution
void frequencySpreadMode() {
   float bass = getRangeBrightness(0);
   float mid = getRangeBrightness(1);
   float high = getRangeBrightness(2);

   // Driver Footwell strip (71 LED's, last two are unused)
   setRangeToColor(0,22,bass,0,0); //set LED's 0-22 to RED, proportional to bass magnitude
   setRangeToColor(23,45,mid,1,0); //set LED's 0-22 to GREEN, proportional to mid magnitude
   setRangeToColor(46,68,high,2,0); //set LED's 0-22 to BLUE, proportional to bass magnitude

   // Passenger Footwell strip (71 LEDs, last two are unused)
   // Reverse order of driver footwell strip
   // Driver Footwell strip
   setRangeToColor(71,93,high,2,0); //set LED's 0-22 to RED, proportional to bass magnitude
   setRangeToColor(94,113,mid,1,0); //set LED's 0-22 to GREEN, proportional to mid magnitude
   setRangeToColor(114,136,bass,0,0); //set LED's 0-22 to BLUE, proportional to bass magnitude
}

// Assigns red to bass, green to mid, and blue to high; each frequency range has an LED every
// three LEDs, and these LED's light up in proportion to the magnitude of the frequency range
void frequencyAggregateMode() {
  float bass = getRangeBrightness(0);
  float mid = getRangeBrightness(1);
  float high = getRangeBrightness(2);

  int curBin = -1;
  for (int i = 0; i < STRIP1_COUNT; i++) {
    curBin++;
    
    if(curBin == 0) {
      strip1.setPixelColor(i, getColor(bass, 0));
    } else if (curBin == 1) {
      strip1.setPixelColor(i, getColor(mid, 1));
    } else if (curBin == 2) {
      strip1.setPixelColor(i, getColor(high, 2));
    }

    if(curBin == 3) curBin = 0;
  }
}

// Makes a "bar" where no lights are on at 0 bass, the entire strip is filled at full bass,
// and anything in the middle fills a proportional amount of the strip.
// For driver footwell, starts from left and fills rightward; for passenger, reverse.
void barMagnitudeMode(int color) {
  float prop = bassMag / maxBass;
  int numLEDs = round((prop * ((float)STRIP1_COUNT/2.0)) * 71);
  setRangeToColor(0, numLEDs, 255, color, 0); // driver footwell, starting from left
  setRangeToColor(STRIP1_COUNT-numLEDs,STRIP1_COUNT,255,color, 0); // driver footwell, starting from right
}

// Returns largest magnitude in range by preference order
// IE in order of preference assuming preferred bin doesnt meet a minimum
int getLargestMagRange() {
  float mags[] = {bassMag, midMag, highMag};
  int highestRange = 0;
  for (int i = 0; i < 3; i++) {
    if(mags[i] > mags[highestRange]) {
      highestRange = i;
    }
  }
  if(mags[0] < 3000) return highestRange;
  return 0;
}

// Determines the magnitude of each frequency range relative to eachother
void adaptiveMode() {
  int maxRange = getLargestMagRange();
  setAllToColor(getRangeBrightness(maxRange), maxRange);
}

void loop() {
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    bassMag = getBassMag();
    //midMag = getMidMag() + 2000;
    midMag = 0;
    highMag = getHighMag();

    if(bassMag > maxB) maxB = bassMag;
    if(midMag > maxM) maxM = midMag;
    if(highMag > maxH) maxH = highMag;

    Serial.println("bass,   mid,    high,     power");
    Serial.print(bassMag);
    Serial.print("     ");
    Serial.print(maxM);
    Serial.print("     ");
    Serial.println(highMag);
  
    if (LEDS_ENABLED == 1)
    {
      switch(mode) {
        case 0: setAllToColor(getRangeBrightness(0), 0); break; // red
        case 1: setAllToColor(getRangeBrightness(1), 1); break;  //green
        case 2: setAllToColor(getRangeBrightness(2), 2); break; // blue
        case 3: setAllColorSwitching(getRangeBrightness(0)); break;
        case 4: frequencySpreadMode(); break;
        case 5: barMagnitudeMode(0); break; // red
        case 6: barMagnitudeMode(1); break; // green
        case 7: barMagnitudeMode(2); break; // blue
        case 8: frequencyAggregateMode(); break;
        case 9: adaptiveMode(); break;
      }
      
    }
  
    // Restart audio sampling.
    samplingBegin();
  }
}
