
////////////////////////////////////////////////////////////////////
//                          Libraries                             //
////////////////////////////////////////////////////////////////////


#include <Adafruit_NeoPixel.h>
//Necessary Libries -> Audio.h is library containing all FFT functions.
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
//#include <bits/stdc++.h> 
#include <math.h>
#include <string.h>

////////////////////////////////////////////////////////////////////
//                       Initialization                           //
////////////////////////////////////////////////////////////////////


const int Alarm_LEDs_Pin = 35;              //The pin used for the Alarm LEDs output (Pin 3 - Digital)
const int Sampling_LED_Pin =34;
const int Power_LED_Pin = 33;
const int Alarm_LEDCount = 1;                    //Sets the number of Alarm LED's used
const int Power_LED_Count = 1;
const int Sampling_LEDCount = 1;
const int rows = 24;
const int columns = 512;
const int ALARM_TYPE_PIN = 36;


double ColumnFlatnessArray[rows];           //Initialization of 1st stage math arrays
float NewColumnFlatnessArray[rows];
int MaxIndexArray[rows];
float BinaryVal[rows];








////////////////////////////////////////////////////////////////////
//                     Internal State                             //
////////////////////////////////////////////////////////////////////

//IntervalTimer  sTimer;
int Counter = 0;
Adafruit_NeoPixel AlarmLEDs = Adafruit_NeoPixel(Alarm_LEDCount, Alarm_LEDs_Pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel SamplingLEDs = Adafruit_NeoPixel(Sampling_LEDCount, Sampling_LED_Pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel PowerLEDs = Adafruit_NeoPixel(Power_LED_Count, Power_LED_Pin, NEO_GRB + NEO_KHZ800);


//This function below is responsible for setting the sampling rate
//Visit https://forum.pjrc.com/threads/38753-Discussion-about-a-simple-way-to-change-the-sample-rate Post #19 for more info
void setI2SFreq(int freq) 
{
  typedef struct 
  {
    uint8_t mult;
    uint16_t div;
  } __attribute__((__packed__)) tmclk;
  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, 44117.64706 , 48000, 88200, 44117.64706 * 2, 96000, 176400, 44117.64706 * 4, 192000};



  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };


  for (int f = 0; f < numfreqs; f++) 
  {
    if ( freq == samplefreqs[f] ) 
    {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}

//This was configured using the online Audio System Design Tool for Teensy Audio Library GUI
//Visit https://www.pjrc.com/teensy/gui/ and import the below code to understand what each individual connection, mixer, input, output, control, and functions do
/*
AudioInputI2S            i2s1;           //xy=139,91
AudioMixer4              mixer1;         //xy=312,134
AudioOutputI2S           i2s2;           //xy=392,32
AudioAnalyzeFFT1024      fft1024;        //xy=467,147
AudioConnection          patchCord1(i2s1, 0, mixer1, 0);
AudioConnection          patchCord2(i2s1, 0, i2s2, 0);
AudioConnection          patchCord3(i2s1, 1, mixer1, 1);
AudioConnection          patchCord4(i2s1, 1, i2s2, 1);
AudioConnection          patchCord5(mixer1, fft1024);
AudioControlSGTL5000     audioShield;    //xy=366,225
*/
AudioInputI2S            i2s1;           //xy=201,234
AudioFilterStateVariable filter1;        //xy=344,275
AudioFilterStateVariable filter2;        //xy=347,328
AudioOutputI2S           i2s2;           //xy=454,175
AudioMixer4              mixer1;         //xy=506,271
AudioAnalyzeFFT1024      fft1024;        //xy=674,278
AudioConnection          patchCord1(i2s1, 0, i2s2, 0);
AudioConnection          patchCord2(i2s1, 0, filter1, 0);
AudioConnection          patchCord3(i2s1, 1, i2s2, 1);
AudioConnection          patchCord4(i2s1, 1, filter2, 0);
AudioConnection          patchCord5(filter1, 2, mixer1, 0);
AudioConnection          patchCord6(filter2, 2, mixer1, 1);
AudioConnection          patchCord7(mixer1, fft1024);
AudioControlSGTL5000     audioShield;    //xy=428,368

const int samplefreq = 8000;               //Set sampling frequency to 8000 Hz.

//Sets the audio input used for recording audio, using the teensy I2S (audio adapter board).
//const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;

// An array to hold the frequency bands
//It is described as the following arrayName[Rows]{Columns]
//It would be a good idea eventually to store 24 and 512 in seperate integer variables, and replace every instance in this entire code file with the variable names.
float FFT2D[rows][512];

float rowkfft[columns];
float rowkfftlog[columns];
bool detect = false;





////////////////////////////////////////////////////////////////////
//                       Setup / Main                             //
////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:


  Serial.begin(38400);                      //Begin using serial port

//Serial.println(F_PLL);


  //pinMode(PowerIndicatorLED, OUTPUT);       //Set the power indication LED to a digital output
  //digitalWrite(PowerIndicatorLED, HIGH);    //Provides a high value to PowerIndicatorLED (The LED is always on when device is running)

  pinMode(ALARM_TYPE_PIN, INPUT);
  
    
  PowerLEDs.begin();
  AlarmLEDs.begin();                        //Initialize the Adafruit_NeoPixel library
  SamplingLEDs.begin();

  SamplingLEDs.show();
     AlarmLEDs.show();                         //Initialize all LEDs to "off"
     PowerLEDs.show();


  
for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, PowerLEDs.Color(0, 30, 0));
    } 
  PowerLEDs.show();


  
 

  // Audio requires memory to work.
  // Might be useful to expirement with this at some point.
  AudioMemory(12);

  // Enable the audio shield and set the output volume and the sampling freq.
  audioShield.enable();
  audioShield.volume(0.5);
  audioShield.inputSelect(myInput);
  audioShield.micGain(20);
  setI2SFreq(samplefreq);

  // configure the mixer to equally add left & right
  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);



}



void loop() 
{
     

for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, 0);
    }
    AlarmLEDs.show();
for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, 0);
    }
    SamplingLEDs.show();

//  filter1.frequency(1000);
  //filter2.frequency(1000);
     
    //Serial.print("Is FFT AVAILABLE?  ");
   // Serial.print(fft1024.available());
    //Serial.println();
    if (fft1024.available()) 
  {
for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, SamplingLEDs.Color(0, 0, 30));
    } 
SamplingLEDs.show();

 
    // See this conversation to change this to more or less than 16 log-scaled bands?
    // https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands
    
    // read the 512 FFT frequencies into 512 levels
    // music is heard in octaves, but the FFT data
    // is linear, so for the higher octaves, read
    // many FFT bins together.

    // The nested for loops constructs the 2D array full of FFT value, imitating a makeshift spectrogram.
    // Currently at 512 columns by 24 rows. To change this, you must change the array size too. Which would make it a good idea to store these in variables as mentioned earlier.
    // Delay of 120ms between each sample taken and FFT'd. This gives us approx an 3 second sample time per array.
    for (int m = 0; m < rows; m++) 
    {
      for(int n = 0; n < 512; n++)
      {
        fft1024.windowFunction(AudioWindowHamming1024);
        FFT2D[m][n] = fft1024.read(n); 

        if(FFT2D[m][n] < 0.000061)
        {
          FFT2D[m][n] = 0.000061035156250;
        }

       
      }
      delay(120);    
    }

    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, 0);
    }
    SamplingLEDs.show();

 
    //Prints the 2D array of the FFT values. Might be good to turn this into a function and just call it instead of having this in the loop.
    for(int j = 0; j < rows; j++)
    {
      for(int k = 0; k < 512; k++)
      {
        Serial.print(FFT2D[j][k], 4);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

for (int k = 0; k < rows ; k++)
{
  memcpy(&rowkfft,&FFT2D[k],sizeof(FFT2D[k]));
  memcpy(&rowkfftlog,&FFT2D[k],sizeof(FFT2D[k]));

for(int j = 0; j < columns; j++)
{
rowkfftlog[j] = {log10(rowkfftlog[j])};
} 
  
/*
for(int i = 0; i < columns; i++)
{
  Serial.println(rowkfft[i]);
  }
*/

  
  float AM = ArithmeticMean(rowkfft, columns);
  ColumnFlatnessArray[k] = {AM};
    
}

  double Flatness = FindFlatness(ColumnFlatnessArray, rows);
  Serial.println();
  Serial.print("Flatness is  ");
  Serial.print(Flatness);
  Serial.println();


if (Flatness < 0.08 || Flatness > 0.55)
{
  detect = false;
  DetectionCheck(detect);
}
else

{
  for(int k = 0; k < rows; k++)
  {
    memcpy(&rowkfft,&FFT2D[k],sizeof(FFT2D[k]));
    int EMI = ExtractMaxIndex(rowkfft, columns);
    MaxIndexArray[k] = {EMI};
    
    
  }
  
  float DevMean = FindDevMean(MaxIndexArray, rows);
  Serial.print("Devmean is  ");
  Serial.print(DevMean);
  Serial.println();

  
  if (DevMean > 90 || DevMean < 2.5)
  {
    detect = false;
    DetectionCheck(detect);
  }
  else 
  {
    for(int k = 0; k < rows; k++)
    {
      memcpy(&rowkfft,&FFT2D[k],sizeof(FFT2D[k]));
      memcpy(&rowkfftlog,&FFT2D[k],sizeof(FFT2D[k]));

        for(int j = 0; j < columns; j++)
        {
        rowkfftlog[j] = {log10(rowkfftlog[j])};
        } 
  
      float NAM = NewArithmeticMean(rowkfft, columns);
      NewColumnFlatnessArray[k] = {NAM};
      
    } 
    
    float NewFlatness = FindNewFlatness(NewColumnFlatnessArray, rows);
    
    Serial.print("New Flatness is  ");
    Serial.print(NewFlatness);
    Serial.println();
  
    
    if (NewFlatness >= 0.7)
    {
      detect = false;
      DetectionCheck(detect);
    }
    else
    {
      for(int k = 0; k < rows; k++)
      {
        memcpy(&rowkfft,&FFT2D[k],sizeof(FFT2D[k]));
        int BV = BinaryValue(rowkfft, columns);
        BinaryVal[k] = {BV};
        
      }
      
      float SumBinVec = FindSumBinVec(BinaryVal, rows);
    Serial.print("SumBinVec is  ");
    Serial.print(SumBinVec);
    Serial.println();
      
      if(SumBinVec <= 12)
      {
        detect = false;
        DetectionCheck(detect);
      }
      else
      {
        detect = true;
        DetectionCheck(detect);
      }
      
    }
  }
}

}


////////////////////////////////////////////////////////////////////
//                       Mathematics                              //
////////////////////////////////////////////////////////////////////

float GeoMean(float rowkfftlog[], int columns)
{
   double SumLog  = 0;                                  //Initialize the product to 1. (Not zero, as the product will start and propegate to zero)

  for(int i = 0; i < columns; i++)
  {
    SumLog = SumLog + rowkfftlog[i];  //Compute the running product for every value of the individual fft (multiplies the 512 fft values to one another)
 
  }
  
    double GeometricMean = (1/(double) columns) * SumLog;        // computes the geometric mean (recall: the geometric mean is the "columnsth" root of the running product) 1/coulumns is typecasted to a float for a float output.



    return GeometricMean; 
}

/////////////////////////////////////////////////////////////////////

float ArithmeticMean(float rowkfft[], int columns)
{
  double MeanSum = 0;                          //Initialize the running sum to 0 to be added to.

  for(int i = 0; i < columns; i++)
  {
    MeanSum = MeanSum + rowkfft[i];                     //Compute the running sum for every value of the individual fft (adds the 512 fft values to one another
    
  }

     double Mean = MeanSum / (double) columns;               //Compute the mean by dividing the running sum by the number of columns (typecasted to a float for a float value output)
  
     double MeanLog = log10(Mean);
    
     double GM = GeoMean(rowkfftlog, columns);                 //Calls the GeoMean function and sets float "GM" equal to the output, "GeometricMean"
     double ColumnFlatnessLog = GM - MeanLog;                   //Calculates the Spectral Flatness Measure of the individual fft (SFM is the ratio of the geometric mean to the arithmetic mean)

     double ColumnFlatness = pow(10, ColumnFlatnessLog);

    return ColumnFlatness;      
}


/////////////////////////////////////////////////////////////////////

float NewGeoMean(float rowkfftlog[], int columns)
{
   double NewSumLog = 0;                                  //Initialize the product to 1  (as per logic above in function "GeoMean")

  for(int i = 63; i < 257; i++)                     //Newcolumnflatness is calculated the same way as columnflatness, except across a specific band (63 - 257) of array indices.
  {
    NewSumLog = NewSumLog + rowkfftlog[i];                 //running product
      
  }
    double NewGeometricMean = (1/ (double) 193) * NewSumLog;        // computes the new geometric mean (ie the "columnsth" root of the running product)
   
    return NewGeometricMean; 
}

/////////////////////////////////////////////////////////////////////

float NewArithmeticMean(float rowkfft[], int columns)
{
  double NewMeanSum = 0;                           //Does exactly the same as "ArithmeticMean" function above, again, from array indices 63 - 257 (with 8kHz sampling rate and 1024 point fft, approx. 500 - 2000 Hz.) 

  for(int i = 63; i < 257; i++)
  {
    NewMeanSum = NewMeanSum + rowkfft[i];
    
  }
    double NewMean = NewMeanSum / (double) 193;

    double NewMeanLog = log10(NewMean);
   
    double NewGM = NewGeoMean(rowkfftlog, columns);

    double NewColumnFlatnessLog = NewGM - NewMeanLog;

    double NewColumnFlatness = pow(10, NewColumnFlatnessLog);


    return NewColumnFlatness;
}

/////////////////////////////////////////////////////////////////////

int ExtractMaxIndex(float rowkfft[], int columns)
{


int maxIndex = 0;                             //Start by setting the FIRST (0th) fft value as the max value.
double maxValue = rowkfft[maxIndex];

for(int i = 1; i < columns; i++)
{
    if(rowkfft[i] > maxValue)                         //For all values in the fft array, check if its greater than the current max, if it is, set the new value to the current max.
    {
        maxValue = rowkfft[i];
        maxIndex = i;                           //when the current max is set, set the current INDEX, and at the end of the loop, will be the index of the maximum value in the array.
    }
    
}

return maxIndex;
}

/////////////////////////////////////////////////////////////////////

float BinaryValue(float rowkfft[], int columns)
{
int MI = ExtractMaxIndex(rowkfft, columns);                   //Call the "ExtractMaxIndex" function above and set the return value (maxIndex) to an integer value "MI."
float BinaryVal = 0;                              

if ( MI > 63 && MI < 257)                         //Decide whether the maximum index falls within the range of 500 - 2000 Hz, and return a respective binary value 
{
  BinaryVal = 1;
}
else 
{
  BinaryVal = 0;
}

return BinaryVal;

}

////////////////////////////////////////////////////////////////////
//             Detection Mathematics/Detection/Alarm              //
////////////////////////////////////////////////////////////////////

double FindFlatness(double ColumnFlatnessArray[], int rows)       //This function operates on the FULL math vector "ColumnFlatnessArray" ONLY when the counter reaches 29.
{
  double MeanSum = 0;
  long double Flatness = 0;                         //Initialize variales.

    for (int n = 0; n < rows; n++)
    { 
      MeanSum = MeanSum + ColumnFlatnessArray[n];           //Simply takes the average of all 30 (rows) values in the ColumnFlatnessArray
    }

    Flatness = MeanSum / (double) rows;            
    return Flatness;
}

/////////////////////////////////////////////////////////////////////

float FindNewFlatness(float NewColumnFlatnessArray[], int rows)
{
  float MeanSum = 0;                          //Same logic as directly above.
  float NewFlatness = 0;

    for (int n = 0; n < rows ; n++)
      {
        MeanSum = MeanSum + NewColumnFlatnessArray[n];
      }

      NewFlatness = MeanSum / (float) rows;

      return NewFlatness;
}

/////////////////////////////////////////////////////////////////////




float FindDevMean( int maxIndex[], int rows)
{
  int newrows = rows - 1;
  int Deviation [newrows] = {0};
      for (int n = 0; n < newrows; n++)
      {
        Deviation[n] = abs(maxIndex[n] - maxIndex[n+1]);
        
      }


      int MeanSum = 0;
      float DevMean = 0;

        for (int k = 0; k < newrows; k++)
        {
          MeanSum = MeanSum + Deviation[k];
        }

        DevMean = MeanSum / (float) newrows;

        return DevMean;
}




/////////////////////////////////////////////////////////////////////



float FindSumBinVec(float BinaryValue[], int rows)
{
  float SumBinVec = 0;
    for (int n = 0; n < rows ; n++)
    {
      SumBinVec = SumBinVec + BinaryValue[n];
    }
    return SumBinVec;
}



/////////////////////////////////////////////////////////////////////















////////////////////////////////////////////////////////////////////
//                     Detection/Alarm                            //
////////////////////////////////////////////////////////////////////






bool DetectionCheck(bool detect)
{
if(detect == false)
{
 NoDetectionLights();
}
else
{

bool Alarmtype = digitalRead(ALARM_TYPE_PIN);

  if(Alarmtype == LOW)
{
  DetectionAlarm1();
}

  else
  {
    DetectionAlarm2();
  }
}
}



void DetectionAlarm1(){
 
  
    int delaytime = 250;

    
  for (int i = 0; i<4; ++i){
    for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, AlarmLEDs.Color(150, 0, 0));
    } 
    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, SamplingLEDs.Color(150, 0, 0));
    } 
    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, PowerLEDs.Color(150, 0, 0));
    } 
    AlarmLEDs.show();
    SamplingLEDs.show();
    PowerLEDs.show();

    
    delay(delaytime);

    
    for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, 0);
    }
    AlarmLEDs.show();
    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, 0);
    }
    SamplingLEDs.show();
    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, 0);
    }
    PowerLEDs.show();
    delay(delaytime);
    }

    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, PowerLEDs.Color(0, 30, 0));
    } 
  PowerLEDs.show();
Serial.println("Siren Detected");
    
  }



void DetectionAlarm2(){
 
  
    int delaytime = 250;

    
  for (int i = 0; i<2; ++i){



    
    for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, AlarmLEDs.Color(150, 0, 0));
    } 
    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, SamplingLEDs.Color(0, 0, 150));
    } 
    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, PowerLEDs.Color(150, 0, 0));
    } 
    AlarmLEDs.show();
    SamplingLEDs.show();
    PowerLEDs.show();

    
    delay(delaytime);

    
    for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, 0);
    }
    AlarmLEDs.show();
    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, 0);
    }
    SamplingLEDs.show();
    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, 0);
    }
    PowerLEDs.show();
    
    delay(delaytime);

 for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, AlarmLEDs.Color(0, 0, 150));
    } 
    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, SamplingLEDs.Color(150, 0, 0));
    } 
    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, PowerLEDs.Color(0, 0, 150));
    } 
    AlarmLEDs.show();
    SamplingLEDs.show();
    PowerLEDs.show();

    
    delay(delaytime);

    
    for(int j = 0; j<Alarm_LEDCount; ++j){
      AlarmLEDs.setPixelColor(j, 0);
    }
    AlarmLEDs.show();
    for(int j = 0; j<Sampling_LEDCount; ++j){
      SamplingLEDs.setPixelColor(j, 0);
    }
    SamplingLEDs.show();
    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, 0);
    }
    PowerLEDs.show();
    
    delay(delaytime);


    }

    for(int j = 0; j<Power_LED_Count; ++j){
      PowerLEDs.setPixelColor(j, PowerLEDs.Color(0, 30, 0));
    } 
  PowerLEDs.show();
Serial.println("Siren Detected");
    
  }










void NoDetectionLights()
{

Serial.println("No Siren Detected");

  }
