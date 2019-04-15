
float SW_version = 1.01;
/*
' ######################################################################
' ###
' ###       *** Light Flicker Measure with Arduino (10Hz to 200KHz) ***
' ###       ----------------------------------------------------------
' ###       *Wolfgang Franke*
' ###       Projekt: 02.Jan.2019 - 26.Feb.2019
' ###       Software-Version 1.01    (used IDE: Arduino v1.8.8)
' ###       ----------------------------------------------------------
' ###       Hardware Setup:
' ###       - Analog:   
' ###           + BPW34 Photo Diodes (as fast Light Sensor to receive Light Frequency Pulses)
' ###           + AD823A dual OPV (to I-U convert and amplify light sensor pulses from BPW34) (TLC272 pin-compatible but much faster) 
' ###       - Digital:  
' ###           + ATMega328P with Arduino-Uno Bootloader and FDTI interface (5V, 16 MHz, 32MB Flash)
' ###             + Timer1-Counter clocked by TMR1 T1-externClock/PD5=D5 for counting Light Frequency Pulses
' ###             + ADC0 to measure OPV operation-point voltage (AD823A) and to sample waveform if flicker <10kHz
' ###           + OLED 128x64 (I2C Address: 0x3C)
' ###           + digital Poti AD5242-1M (I2C Address: 0xC2) to control OPV AD823A operation-point and turn ON/OFF 3 of the 4 BPW34 (to allow measure bright light)
' ###
' ###       Function of Light Frequency Measure Loop:
' ###       (1) adjust operating point of OPV (by BPW34 and in amplifier) to match out-signal to Comperator-Input before every Pulse Count of Ardunio CTC
' ###             Pulse high volt target: 4.0V (OPV_PulseHigh_Volt_target) as target operation point, for good signal-to-noise/sunlight ratio
' ###               light is too bright:  Pulse min voltage > 4.4V (OPV_PulseLow_Volt_saturate)
' ###               light is too dark:    Pulse max voltage < 0,1V (OPV_PulseHigh_Volt_weak)
' ###               not used: faint light flicker:  Difference of Pulse max-min voltage < 0,3V (OPV_Pulse_Volt_LH_weak)
' ###       (2) count Pulses (light flicker) on Timer1 T1-externClock-Pin (D5) using Overflow-ISR,  
' ###             1 milli = 1024µs (1000 millis = 1024ms)
' ###             1 sec   = 976,5 millis
' ###             lenght of measure 0,5s has good results for 50Hz
' ###       (3) calculate frequency from Timer1-Counter + Overflow and display on OLED nicely in Hz/kHz,
' ###             1 second counter: INT=65535 is too small for 100kHz
' ###               100Hz = 100/s     => 1s Interval = 100
' ###               1 kHz = 1000/s    => 1s Interval = 1000
' ###              10 kHz = 10.000/s  => 1s Interval = 10.000
' ###             100 kHz = 100.000/s => 1s Interval = 100.000! is>65.535
' ###       (4) measure pulse waveform (for frequency up to 10kHz, with ADC in free-run mode with 76kHz)
' ###       
' ###       known limitations:
' ###         - one analogRead() takes ≈110µs/0,1ms and the 50 loops to determine low/high peak voltage of pulses 
' ###           take ≈5ms and that mathematically works with >200Hz light pulse frequency,
' ###         - if a high or low peak voltage can not be found due to a >5ms light pulse periods (notably <80Hz)
' ###           then if max=0 a wrong flicker=100% and if min=max a wrong flicker=0% will be set,
' ###         - with the Arduino ADC sampling rate of 110µs(9,1kHz) there should be an inability to detect Light Pulses 
' ###           with exactly the same frequency and multiples of it, but this exact condition was non-reproducible,
' ###       
' ######################################################################

/*
' ######################################################################
' ###  Pinbelegungen
' ###  --------------------------------------------------------------
' ###  Arduino      ATMega328P        Flicker Measure
' ###  Uno/ProMini  DIL-28            Beschaltung_WF
' ###  -------------------------------------------------------------
' ###  V-In/RAW     -                 Arduino Power Supply (7V-12V)
' ###  3,3V         -                 Arduino Vout=3,3V (I-out<100mA?)
' ###  5V           -                 Arduino Vout=5V (IN or I-out<200mA)
' ###  
' ###  RESET        (01)                n.c.
' ###  Vcc          (07)              5V+
' ###  Vcc          (20)              5V+
' ###  ARef         (21)                n.c.
' ###  GND          (08)              GND
' ###  GND          (22)              GND
' ###  
' ###  A4/SDA       (27)PC4           I2C-SDA (external Pull-Up 10k to Vcc=5V)
' ###  A5/SCL       (28)PC5           I2C-SCL (external Pull-Up 10k to Vcc=5V)       
' ###  
' ###  D05          (11)PD5           IN TMR1-T1  :  Timer1/Counter1 External Clock Source (T1-externClock Pin)
' ###  A0           (23)A0/PC0        IN ADC0     :  measure Amplifier Operations Point
' ###  D13/SCK      (19)PB5           OUT LED13   :  used for experimantal delay-loop (LED with 330/1K to GND)
' ###  D02/INT0     (04)PD2           IN+Pullup   :  INT0 as frequency pulse counter (not longer used)
' ######################################################################
*/

    
//#########################################################################
//   ******  MCU, Hardware and compiler definitions   ******
//#########################################################################

// *** DEBUG LEVELS ON/OFF ***
#define Serial_waits_for_USB_Enumeration 0    // 0: Serial does NOT wait for USB-Enumeration after Power-ON (Serial-Prints to Terminal get lost until USB-Serial port is re-enumerated)
                                              // 1: Serial-Init waits in loop with timeout of 10 sec for USB-Enumeration after Power-ON (send first Serial.Print sent to Terminal when USB re-enumerated or timeout)
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Definition for ADC
#define ADC_Pin                 0           // Address for ADC free-running mode (is Ardunio A0)
#define ADCPin_OPV_Volt        A0           // analogRead() Address, measure Voltage of OPV operation point for BPW34 amplifier
#define ADC_URef              5.0           // ADC U-Ref in Volt (DEFAUL=5V , INTERNAL 1.1V for ATMega328)

// Declaration for DigiPoti AD5242 connected to I2C (Hardware SDA, SCL pins)
#define AD5242_I2C_Addr     0x2C            // AD5242 I2C address is 0x2C

// Declaration for OLED display with SSD1306 connected to I2C (Hardware SDA, SCL pins)
#define OLED_I2C_Address     0x3C           // Address of OLED (0x3C for China Module, 0x3D for Adafruit compatible)
#define SCREEN_WIDTH          128           // OLED display width, in pixels
#define SCREEN_HEIGHT          64           // OLED display height, in pixels
#define OLED_RESET             -1           // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//------------------------------------------------------------------------
//   ******  end MCU, Hardware and compiler definitions   ******
//-------------------------------------------------------------------------


//#########################################################################
//   ****** global variables definitions   ******
//#########################################################################

//define variables for measuring OPV-output voltage on ADC
const float OPV_PulseHigh_Volt_target    = 4.0;      // ca.3.7-4.0V - target operation point for Pulse high voltage, for good signal-to-noise/sunlight ratio
const float OPV_PulseLow_Volt_saturate   = 4.4;      // ca. 4.0-4.5V - higher is too bright (max pulse measured)
const float OPV_PulseHigh_Volt_weak      = 0.1;      // ca. 0.1-0.3V - below is too dark (max pulse measured)

float OPV_Pulse_Volt_max;                       // max Voltage of Pulse for OPV operation point for BPW34 amplifier
float OPV_Pulse_Volt_min;                       // min Voltage of Pulse for OPV operation point for BPW34 amplifier
unsigned int Flicker_Percent;                   // percentage of flicker in light (= (ADC_OPV_Volt_raw_max - ADC_OPV_Volt_raw_min) * 100 / ADC_OPV_Volt_raw_max;)

unsigned int ADC_OPV_Volt_raw;                  // ADC 10 bit value of Voltage of OPV operation point for BPW34 amplifier
unsigned int ADC_OPV_Volt_raw_max;              // stores max ADC 10 bit value of Voltage of OPV operation point for BPW34 amplifier
unsigned int ADC_OPV_Volt_raw_min;              // stores min ADC 10 bit value of Voltage of OPV operation point for BPW34 amplifier

//define variables für Regelkreis zur Einstellung des BPW34-OPV Arbeitspunktes vor jedem Mess-Intervall
unsigned int R_RAW            = 128;            // stores resistor value, starts with POR setting of resistor
const int R_min               = 2;              // min setting of elecronic resistor (0 Ohm)
const int R_max               = 255;            // max setting of elecronic resistor (1M)
boolean R_decreased_flag;                       // request reached (R <= Rmin)
boolean R_increased_flag;                       // request reached (R >= Rmax)

//define variables for counting light frequency pulses on Timer1 T1-externClock
unsigned int Millis_per_Second = 977;           // Arduino: 1 milli = 1024 µs (1 s = 976,56 millis)
unsigned int Measure_Interval_SplitSecond = 2;  // Measure Pulses for real 0,25s or 0,5s
unsigned int Millis_to_Measure;                 // calulated Millis of mesure interval
unsigned long MilliSec_IntervalStart;           // start of voltage measure interval
unsigned long Pulses_counted;                   // counts light frequency pulses on Timer1-Counter
unsigned long CTR1_frequency;                   // calculated frequency on Timer1 T1-externClock
byte CTR1_frequfract;                           // fraction of calculated frequency in KHz
int CTR1_frequency_displayed;                   // stores the decimal value of shortened value in display
byte HzKHz_flag;                                // flag to print Hz or KHz to the display
unsigned long CTR1_frequency_displayed_lastCycle; // saves frequency calculated from last cycle
byte same_frequency_displayed_counter;          // counter of consecutive cycles with same measure, to later display Pulse waveform

//Timer1-Counter
unsigned int overflowCount;                     
unsigned long overflowCopy;
unsigned int timer1CounterValue;

//ADC Pulse waveform probing and printing
unsigned int ADC_save_Interval;                 // interval of ADC measure for Pulse waveform probing
byte ADCvalues[128];                            // stores ADC values of pulse waveform measures
byte ADCvalue_Pointer;                          // pointer to array ADCvalues[]
byte ADCSRA_save;                               // saves the Register while ADC free-running mode
byte ADCSRB_save;                               // saves the Register while ADC free-running mode
byte ADMUX_save;                                // saves the Register while ADC free-running mode
unsigned int max_yValue;                        // max value of all waveform samples to display
unsigned int scaled_yValue;                     // waveform samples value scaled to display pixels

//-------------------------------------------------------------------------
//   ****** End of global variables definitions ******
//-------------------------------------------------------------------------



//#########################################################################
//    ****** main program init ******
//#########################################################################

// main setup
void setup() 
{
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // *** MCU config ***
  
  // pin for INT0
//  pinMode(2, INPUT_PULLUP);     // INT0: init as input (Arduino Digital-2 = ATMega328 PD2/INT0)
  pinMode(13, OUTPUT);            // used for the "experimental" Delay-Loop (3,5µs) when sampling ADC of pulse


  //Initialize serial port 
  Serial.begin(115200);
  if (Serial_waits_for_USB_Enumeration == 0)                  // wait for serial port if flag Serial_waits_for_USB_Enumeration is set
  {
    unsigned long MillisWait = millis() + 10000;              // timeout 10s if serial connection to PC established
    while (!Serial)                                           // wait for serial port to open (restart Terminal program)
    {  
      if (millis() > MillisWait)                              // timeout?
      {
        break;                                                // timeout, start program without Serial Communication
      }      
    }  
  } //end init serial port
  Serial.println(F("Welcome @ Arduino (115200 baud)"));        
  Serial.print(F("Light Flicker Measure - W.Franke 2019 - v"));    
  Serial.println(SW_version, 2);                               
  Serial.println(F(" "));
  
  //config ADC
  analogReference(DEFAULT);           // URef = Vcc=5V   // analogReference(INTERNAL); // URef=1,1V

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // *** external hardware config ***
  
  //config OLED Display and used Font
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_Address))      // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  { 
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.setFont();                  // set default font with size (setCursor(x,y) points to lower-left pixel of first character)
  display.setTextColor(WHITE);        // set text "white" (FYI: (BLACK, WHITE) = inverse text)
  display.setTextSize(1);             // set 1x scale text (scales bitmat, that's not a vector font)

  //init DigiPoti at 50%
  Set_DigiPoti_newValue_for_OPV(R_RAW);       // init DigiPoti R1 in feedback loop of OPV to 500k 
    
  splashscreen();                     // Show splash screen
  delay(3000); 
  display.clearDisplay();  

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // *** Software config ***
  same_frequency_displayed_counter = 1;      // reset counter of consecutive cycles with same measure, to later display Pulse waveform
  CTR1_frequency_displayed_lastCycle = 0;    // reset frequency calculated from last cycle

}//end of Arduino "Setup"
//-------------------------------------------------------------------------
//   ****** End setup ******
//-------------------------------------------------------------------------




//#########################################################################
//    ****** Arduino main program loop ******
//#########################################################################

//Arduino main loop
void loop() 
{

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (1A) read current Pulse Voltage of OPV operation point for BPW34 amplifier on output ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  Set_DigiPoti_newValue_for_BPW34(0);         // init DigiPoti R2 to turn ON all BPW34 diodes for normal/low brightness
 
  // measure pulse low and high peak voltage    
  // one analogRead() takes ≈110µs/0,1ms => 50 loops take ≈5ms and that duration mathematically will catch the low/high change of a pulse >200Hz
  // tests have shown good statistical zero crossing detection down to 100Hz (10ms pulse width)
  // but below 50Hz (>20ms pulse width) nearly no zero crossing is detected, and low/high peak will become same resulting (risk of "division by zero" for flicker-percent)
  ADC_OPV_Volt_raw_min = 1024;
  ADC_OPV_Volt_raw_max = 0;                               // reset ADC_OPV_Volt_raw_max
  for (byte i=0; i <= 50; i++)                            // try to find the peak of "alternating" voltage by statistically probing the wafeform
  {
    ADC_OPV_Volt_raw = analogRead(ADCPin_OPV_Volt);       // read ADC 10-bit raw value
    if (ADC_OPV_Volt_raw > ADC_OPV_Volt_raw_max)          // store max volt of BPW34 amplifier on output
    {
      ADC_OPV_Volt_raw_max = ADC_OPV_Volt_raw;
    }
    if (ADC_OPV_Volt_raw < ADC_OPV_Volt_raw_min)          // store min volt of BPW34 amplifier on output
    {
      ADC_OPV_Volt_raw_min = ADC_OPV_Volt_raw;
    } 
  } 
  OPV_Pulse_Volt_min = ADC_OPV_Volt_raw_min * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)
  OPV_Pulse_Volt_max = ADC_OPV_Volt_raw_max * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (1B) change electronic resistor in OPV amplifier for BPW34 light sensor, to get best voltage of OPV operation point for output signal ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
/*  starte Regelkreis zur Einstellung des BPW34-OPV Arbeitspunktes vor jedem Mess-Intervall
    Ziel: float OPV_PulseHigh_Volt_target           // target operation point for Pulse high voltage, for good signal-to-noise/sunlight ratio
    - start condition 1:  OPV_Pulse_Volt_max >= OPV_PulseHigh_Volt_target
        1) verkleinere OPV-R bis (OPV_Pulse_Volt_max <= OPV_PulseHigh_Volt_target) oder (R<=Rmin)
        2) wenn danach (R > Rmin) : Meldung "Flicker %" -ELSE (R<=Rmin):
            wenn (OPV_Pulse_Volt_min > OPV_PulseLow_Volt_saturate) : Meldung "zu hell"
    - start condition 2:  OPV_Pulse_Volt_max < OPV_PulseHigh_Volt_target    
        1) vergrößere OPV-R bis (OPV_Pulse_Volt_max >= OPV_PulseHigh_Volt_target) oder (R>=Rmax)
        2) wenn danach (R < Rmax) : Meldung "Flicker %" -ELSE (R>=Rmax):
            wenn (OPV_Pulse_Volt_max > OPV_PulseHigh_Volt_weak) : Meldung "zu dunkel".    
*/
  // change electronic resistor in OPV amplifier for BPW34 light sensor, to get best voltage of OPV operation point for output signal
  // start condition 1:  OPV_Pulse_Volt_max >= OPV_PulseHigh_Volt_target  
  if (OPV_Pulse_Volt_max >= OPV_PulseHigh_Volt_target)   
  {
    do
    {
      R_decreased_flag = decrease_R_OPV();                    // verkleinere OPV-R - return false if (R <= Rmin)
      // measure pulse low and high peak voltage
      ADC_OPV_Volt_raw_min = 1024;
      ADC_OPV_Volt_raw_max = 0;                               // reset ADC_OPV_Volt_raw_max
      for (byte i=0; i <= 50; i++)                            // try to find the peak of "alternating" voltage by statistically probing the wafeform
      {
        ADC_OPV_Volt_raw = analogRead(ADCPin_OPV_Volt);       // read ADC 10-bit raw value
        if (ADC_OPV_Volt_raw > ADC_OPV_Volt_raw_max)          // store max volt of BPW34 amplifier on output
        {
          ADC_OPV_Volt_raw_max = ADC_OPV_Volt_raw;
        }
        if (ADC_OPV_Volt_raw < ADC_OPV_Volt_raw_min)          // store min volt of BPW34 amplifier on output
        {
          ADC_OPV_Volt_raw_min = ADC_OPV_Volt_raw;
        } 
      } 
      OPV_Pulse_Volt_max = ADC_OPV_Volt_raw_max * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)
    } while ((OPV_Pulse_Volt_max >= OPV_PulseHigh_Volt_target) && (R_decreased_flag));   // verkleinere OPV-R bis (ADC_OPV_Volt_Value < OPV_PulseHigh_Volt_target) oder (R<=Rmin)


    // if too bright then turn off some BPW34 to reduce OPV input signal and test again
    if (R_RAW < 10)                                 // this low R1 means very high brightness detected, start again with some BPW34=OFF, R1=max and search OPV operation point 
    {
      Set_DigiPoti_newValue_for_OPV(127);           // re-init DigiPoti R1 in feedback loop of OPV to 1M 
      R_RAW = 127;                                  // re-init R_RAW counter
      Set_DigiPoti_newValue_for_BPW34(255);           // set DigiPoti R2 to turn OFF some BPW34 diodes for high brightness
      do
      {
        R_decreased_flag = decrease_R_OPV();                    // verkleinere OPV-R - return false if (R <= Rmin)
        // measure pulse low and high peak voltage
        ADC_OPV_Volt_raw_min = 1024;
        ADC_OPV_Volt_raw_max = 0;                               // reset ADC_OPV_Volt_raw_max
        for (byte i=0; i <= 50; i++)                            // try to find the peak of "alternating" voltage by statistically probing the wafeform
        {
          ADC_OPV_Volt_raw = analogRead(ADCPin_OPV_Volt);       // read ADC 10-bit raw value
          if (ADC_OPV_Volt_raw > ADC_OPV_Volt_raw_max)          // store max volt of BPW34 amplifier on output
          {
            ADC_OPV_Volt_raw_max = ADC_OPV_Volt_raw;
          }
          if (ADC_OPV_Volt_raw < ADC_OPV_Volt_raw_min)          // store min volt of BPW34 amplifier on output
          {
            ADC_OPV_Volt_raw_min = ADC_OPV_Volt_raw;
          } 
        } 
        OPV_Pulse_Volt_max = ADC_OPV_Volt_raw_max * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)
      } while ((OPV_Pulse_Volt_max >= OPV_PulseHigh_Volt_target) && (R_decreased_flag));   // verkleinere OPV-R bis (ADC_OPV_Volt_Value < OPV_PulseHigh_Volt_target) oder (R<=Rmin)      
    }


    // calculate max/min/delta of Pulse Voltage of OPV operation point of BPW34 amplifier on output
    OPV_Pulse_Volt_max = ADC_OPV_Volt_raw_max * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)  
    OPV_Pulse_Volt_min = ADC_OPV_Volt_raw_min * ADC_URef / 1023 ; // map ADC raw (int) to volt (float) 
    display.clearDisplay();  
    display.setTextSize(1);
    display.setCursor(5,38);
    display.print(F("Amp L-H: "));
    display.print(OPV_Pulse_Volt_min);
    display.print(F("-"));
    display.print(OPV_Pulse_Volt_max);
    display.println(F(" V"));

    // display information on light flicker quantitiy
    if (R_decreased_flag)                                               // Voltage was successfully adjusted to U_APsoll
    {
      if ((ADC_OPV_Volt_raw_max/10) > 0)                                // calculate percentage of flicker in light with preventing "division by zero"
      {
        Flicker_Percent = (abs((ADC_OPV_Volt_raw_max/10) - (ADC_OPV_Volt_raw_min/10)) * 100) / (ADC_OPV_Volt_raw_max/10);    // percentage of flicker in light
      }        
      else
      {
        Flicker_Percent = 100;
      }
      display.setCursor(5,51);
      display.print(F("Flicker: "));
      display.print(Flicker_Percent);
      display.print(F("%  ("));
      display.print(R_RAW);
      display.println(F(")"));
    }
    else                                                                // not able to adujst Voltage to U_APsoll (reached the R_min-limit before)
    {
      if (OPV_Pulse_Volt_min > OPV_PulseLow_Volt_saturate)             // voltage is higher than U_Sat: too bright light, no pulses generated to count
      {
        display.setCursor(5,51);
        display.print(F("zu hell        ("));        
        display.print(R_RAW);
        display.println(F(")"));        
        display.display();                                              // let Frequency value blink to indicate unsteady result
      }
      else
      {
        if ((ADC_OPV_Volt_raw_max/10) > 0)                              // calculate percentage of flicker in light with preventing "division by zero"
        {
          Flicker_Percent = (abs((ADC_OPV_Volt_raw_max/10) - (ADC_OPV_Volt_raw_min/10)) * 100) / (ADC_OPV_Volt_raw_max/10);    // percentage of flicker in light
        }        
        else
        {
          Flicker_Percent = 100;
        }
        display.setCursor(5,51);
        display.print(F("Flicker: "));
        display.print(Flicker_Percent);
        display.print(F("%  ("));
        display.print(R_RAW);
        display.println(F(")"));
      }
    }  
  }

  else
    
  // start condition 2:  OPV_Pulse_Volt_max < OPV_PulseHigh_Volt_target
  {
    do
    {
      R_increased_flag = increase_R_OPV();                    // vergrößere OPV-R - return false if (R >= Rmax)
      // measure pulse low and high peak voltage
      ADC_OPV_Volt_raw_min = 1024;
      ADC_OPV_Volt_raw_max = 0;                               // reset ADC_OPV_Volt_raw_max
      for (byte i=0; i <= 50; i++)                            // try to find the peak of "alternating" voltage by statistically probing the wafeform
      {
        ADC_OPV_Volt_raw = analogRead(ADCPin_OPV_Volt);       // read ADC 10-bit raw value
        if (ADC_OPV_Volt_raw > ADC_OPV_Volt_raw_max)          // store max volt of BPW34 amplifier on output
        {
          ADC_OPV_Volt_raw_max = ADC_OPV_Volt_raw;
        }
        if (ADC_OPV_Volt_raw < ADC_OPV_Volt_raw_min)          // store min volt of BPW34 amplifier on output
        {
          ADC_OPV_Volt_raw_min = ADC_OPV_Volt_raw;
        } 
      } 
      OPV_Pulse_Volt_max = ADC_OPV_Volt_raw_max * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)
    } while ((OPV_Pulse_Volt_max <= OPV_PulseHigh_Volt_target) && (R_increased_flag));   // vergrößere OPV-R bis (ADC_OPV_Volt_Value > U_APsoll) oder (R>=Rmax)

    // calculate max/min/delta of Pulse Voltage of OPV operation point of BPW34 amplifier on output
    OPV_Pulse_Volt_max = ADC_OPV_Volt_raw_max * ADC_URef / 1023 ; // map ADC raw (int) to volt (float)  
    OPV_Pulse_Volt_min = ADC_OPV_Volt_raw_min * ADC_URef / 1023 ; // map ADC raw (int) to volt (float) 
    display.clearDisplay();  
    display.setTextSize(1);
    display.setCursor(5,38);
    display.print(F("Amp L-H: "));
    display.print(OPV_Pulse_Volt_min);
    display.print(F("-"));
    display.print(OPV_Pulse_Volt_max);
    display.println(F(" V"));


    // display information on light flicker quantitiy
    if (R_increased_flag)                                               // Voltage was successfully adjusted to U_APsoll
    {
      if ((ADC_OPV_Volt_raw_max/10) > 0)                                // calculate percentage of flicker in light with preventing "division by zero"
      {
        Flicker_Percent = (abs((ADC_OPV_Volt_raw_max/10) - (ADC_OPV_Volt_raw_min/10)) * 100) / (ADC_OPV_Volt_raw_max/10);    // percentage of flicker in light
      }        
      else
      {
        Flicker_Percent = 100;
      }
      display.setCursor(5,51);
      display.print(F("Flicker: "));
      display.print(Flicker_Percent);
      display.print(F("%  ("));
      display.print(R_RAW);
      display.println(F(")"));
    }
    else                                                                // not able to adujst Voltage to U_APsoll (reached the R_max-limit before)
    {
      if (OPV_Pulse_Volt_max < OPV_PulseHigh_Volt_weak)                 // voltage is lower than U_weak: too dark light, no pulses generated to count
      {
        display.setCursor(5,51);
        display.print(F("zu dunkel      ("));        
        display.print(R_RAW);
        display.println(F(")"));        
        display.display();                                              // let Frequency value blink to indicate unsteady result
      }    
      else
      {
        if ((ADC_OPV_Volt_raw_max/10) > 0)                              // calculate percentage of flicker in light with preventing "division by zero"
        {
          Flicker_Percent = (abs((ADC_OPV_Volt_raw_max/10) - (ADC_OPV_Volt_raw_min/10)) * 100) / (ADC_OPV_Volt_raw_max/10);    // percentage of flicker in light
        }        
        else
        {
          Flicker_Percent = 100;
        }
        display.setCursor(5,51);
        display.print(F("Flicker: "));
        display.print(Flicker_Percent);
        display.print(F("%  ("));
        display.print(R_RAW);
        display.println(F(")"));
      }  
    }  
  }
  
  delay(300);     // let finish Arduino queue'd stuff before counting pulses

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (2A) init Timer1-Counter to count (rising) pulses (light flicker) on D5 ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // stop and reset Timer1-Counter
  TCCR1A = 0;     // TCCR1A – Timer/Counter1 Control Register A : Output Compare pins OC1A/OC1B disconnected / Normal mode (counter)
  TCCR1B = 0;     // TCCR1B – Timer/Counter1 Control Register B : Timer/Counter stopped - C10-C12 Clock Select No clock source  / ICNC1 Input Capture Noise Canceler OFF / ICES1 Input Capture Edge Select = FALLING
  TCNT1 = 0;      // reset Counter1 to zero  

  // enable Timer1 Overflow Interrupt  
  TIMSK1 = bit (TOIE1);   // TIMSK1 - Timer/Counter1 Interrupt Mask Register : TOIE1 Overflow Interrupt Enabled / OCIE1x Output Compare A+B Match Interrupt OFF / ICIE1 Input Capture Interrupt OFF
  overflowCount = 0;      // reset overflowCounter

  Millis_to_Measure = Millis_per_Second / Measure_Interval_SplitSecond;  // based on a real second, not 1000 Millis
  Millis_to_Measure = Millis_to_Measure + 10;                            // adjust to statistically catch last wave end ??????????????
      
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (2B) enable Timer1-Counter and start counting Pulses on D5 for 0,5s ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // save current Millis as start-time 
  MilliSec_IntervalStart = millis();                //set millis-start for next measure interval, wait for 500ms
  // start Timer1 as Counter with External clock source on T1 pin (D5), Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);   // TCCR1B – Timer/Counter1 Control Register B : External clock source on T1 pin. Clock on rising edge.
  // wait for "Millis_to_Measure" (0,5s)
  while ((unsigned long)(millis() - MilliSec_IntervalStart) < Millis_to_Measure);     // waits for 0,5s (handles roll-over at 4,294,967,295 ms correctly)

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (2C) stop and read Timer1-Counter (of events on pin D5)  ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // save counter value of Timer1
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  overflowCopy = overflowCount;  
  // test if we just missed the last overflow
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 256) // TIFR1 – Timer/Counter1 Interrupt Flag Register : TOV1 Timer/Counter1 Overflow Flag, TOV1 is automatically cleared when the Timer/Counter1 Overflow Interrupt Vector is executed.
    overflowCopy++;  
  // stop and reset Timer1-Counter
  TCCR1A = 0;   // TCCR1A – Timer/Counter1 Control Register A : Output Compare pins OC1A/OC1B disconnected / Normal mode (counter)
  TCCR1B = 0;   // TCCR1B – Timer/Counter1 Control Register B : C10-C12 Clock Select No clock source (Timer/Counter stopped) / ICNC1 Input Capture Noise Canceler OFF / ICES1 Input Capture Edge Select = FALLING
  // disable Timer1 Overflow Interrupt  
  TIMSK1 = 0;   // TIMSK1 – Timer/Counter1 Interrupt Mask Register : TOIE1 Overflow Interrupt OFF / OCIE1x Output Compare A+B Match Interrupt OFF / ICIE1 Input Capture Interrupt OFF
  // calculate total count
  Pulses_counted = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (3A) calculate freuqency from TMR1-counter, consider to display nicely in ranges ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  switch (Pulses_counted)           // counter to multiply by "Measure_Interval_SplitSecond" as CTR1-measure was only 1/Measure_Interval_SplitSecond sec.
  {  // case values and Freq-calc for 0,5s CTR1-measures
    case 0 ... 499:                                          // 0-999 Hz range
      CTR1_frequency = (Pulses_counted) * 2; // f in Hz
      if (CTR1_frequency < 10)                               // remove frequency < 10Hz
      {
        CTR1_frequency = 0;
      }      
      if ((CTR1_frequency == 98) || (CTR1_frequency == 102))    // stabilize 100Hz main power measure-jitter
      {
        CTR1_frequency = 100;
      }
      HzKHz_flag = 0;                                         // flag for Hz
    break;  // end case
    case 500 ... 4999:                                        // 1.0-9.9 kHz range
      CTR1_frequency = Pulses_counted / 500;                  // f in kHz
      CTR1_frequfract = (Pulses_counted % 500) / 50;          // calculate Modulo of frequency (erste Stelle nach Komma)
      HzKHz_flag = 1;                                         // flag for KHz+fraction
    break;  // end case
    case 5000 ... 49999:                                      // 10-99 kHz range
      CTR1_frequency = Pulses_counted / 500;                  // f in kHz
      HzKHz_flag = 2;                                         // flag for KHz only
    break;  // end case
    case 50000 ... 250000:                                    // >100-500 kHz range    
      CTR1_frequency = Pulses_counted / 500;                  // f in kHz
      HzKHz_flag = 3;                                         // flag for KHz only
    break;  // end case
  }
  

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (3B) print frequency value to display ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  display.setTextSize(3);            // set 3x scale text
  display.setCursor(3,3);            
  display.print(CTR1_frequency);
  // print frequency Hz/kHz to display
  switch (HzKHz_flag)        
  {        
    case 0:                         // 0-999 Hz range
      display.setTextSize(3);       // set 3x scale text
      display.println(F(" Hz"));
      CTR1_frequency_displayed = CTR1_frequency;                            // hash for displayed digits to count 3 same consecutive value to show waveform
      if (abs((CTR1_frequency_displayed_lastCycle - CTR1_frequency_displayed)) <= 10)
      {
        CTR1_frequency_displayed = CTR1_frequency_displayed_lastCycle;      // allow +/-10Hz to avoid alternating measures over/under decimal point
      }      
    break;  // end case
    case 1:                         // 1.0-9.9 kHz range
      display.print(F(","));
      display.print(CTR1_frequfract);
      display.setTextSize(2);       // set 2x scale text
      display.setCursor(60,8);
      display.println(F(" kHz"));    
      CTR1_frequency_displayed = ((CTR1_frequency * 1000) + (CTR1_frequfract * 100));  // hash for displayed digits to count 3 same consecutive value to show waveform
      if (abs((CTR1_frequency_displayed_lastCycle - CTR1_frequency_displayed)) <= 100)
      {
        CTR1_frequency_displayed = CTR1_frequency_displayed_lastCycle;      // allow +/-0,1kHz to avoid alternating measures over/under decimal point
      }
      
    break;  // end case
    case 2:                         // 10-99 kHz range
      display.setTextSize(2);       // set 2xscale text
      display.setCursor(43,8);
      display.println(F(" kHz"));
      CTR1_frequency_displayed = CTR1_frequency;  // save displayed digits to count 3 same consecutive value to show waveform
    break;  // end case
    case 3:                         // 100-131 kHz range
      display.setTextSize(2);       // set 2x scale text
      display.setCursor(60,8);
      display.println(F(" kHz"));
//      CTR1_frequency_displayed = CTR1_frequency;  // save displayed digits to count 3 same consecutive value to show waveform      
    break;  // end case
  }     
  display.display();            // copy buffer to display with Frequency and light flicker quantitiy


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (4a) sample waveform (frequency <10kHz, ADC free-run mode with 76kHz sampling) ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  //remember the currently measured frequency
  CTR1_frequency = (Pulses_counted) * 2;                    // f in Hz
  
  // count cycles of same consecutive measures to display Pulse waveform after 3 same values
  if ((CTR1_frequency_displayed == CTR1_frequency_displayed_lastCycle) && (CTR1_frequency >= 10) && (CTR1_frequency < 11000))   // only if frequency 10Hz-10kHz
  {
    same_frequency_displayed_counter++;                              // incr counter of consecutive cycles with same measure
  } 
  else
  {
    same_frequency_displayed_counter = 1;                            // reset counter of consecutive cycles with same measure
    CTR1_frequency_displayed_lastCycle = CTR1_frequency_displayed;      // saves current frequency calculated
  }

  // display waveform only after 3 stable consecutive frequency measures
  if ((same_frequency_displayed_counter == 3) && (CTR1_frequency >= 10) && (CTR1_frequency < 11000))    // show pulse waveform only 10Hz-10kHz
  {
    same_frequency_displayed_counter = 1;    // reset counter of stable consecutive frequency measures

    // clear screen below frequency number by drawing lines in black
    for (int i=30; i<=display.height(); i++)           // draw lines
    {
      display.drawLine(0, i, display.width(), i, BLACK);
      delay(1);
    }  
    // Draw a rectangle in white
    display.drawRect(0, 28, display.width(), display.height()-28, WHITE);
    delay(1);

    //measure wave form with ADC free-run mode with sampling rate 76kHz
    //use experimental delay-values for ADC-interval by using a loop of: (digitalWrite() taking 3,8µs + ADC-ISR 76000/s)
    //100Hz = 10ms -> 1 wave = 10ms  => 10.000µs mit 32 samples = 312µs Sample-Intervalle (3,2kHz Sampling)
    // base of calculation: 1Hz=6000 loops of DigitalWrite + ADC-INT)
      //  10Hz: delay 3120µs= (6000x DigitalWrite + ca. 70000 ADC-INT)  (0,3kHz Sampling)
      // 100Hz: delay 312µs = (60x DigitalWrite + ca. 7000 ADC-INT)     (3,2kHz Sampling)
      // 1kHz:  delay 31µs  =  (6x DigitalWrite + ca. 600 ADC-INT)      (32kHz Sampling)
      // 2kHz:  delay 15µs  =  (3x DigitalWrite + ca. 300 ADC-INT)      (64kHz Sampling)
      // 3kHz:  delay 10µs  =  (2x DigitalWrite + ca. 200 ADC-INT)      (96kHz Sampling too high or ADC free-run mode 76kSamples)
    if (CTR1_frequency <= 2500)                     // up to 2500Hz waveform is ok for 76kHz ADC sampling
    {
      ADC_save_Interval = 6000 / CTR1_frequency;    // choose experimental interval to sample waveform, works good 10Hz to 2,5kHz      
    }
    else                                            // for freq >2500Hz use ADC sampling like 2500Hz
    {
      ADC_save_Interval = 6000 / 2500;              // choose experimental interval to sample waveforms >2,5kHz
    }
    
    ADCvalue_Pointer = 0;                           // reset Pointer to array of ADC values

    // start+stop ADC in Free-running mode, 76kSamples with ADC-ready INT
    start_ADC_Free_Running();                // start ADC in Free-Running Mode and ADC conversion-ready ISR
    for (int i=0; i <= 127; i++)             // copy 128 ADC values in intervals equivalent to "Micros_ADC_Interval"
    {
      ADCvalue_Pointer = i;                   // ADC-ready ISR will copy ADC value to this point in array 
  
      // experimental delay for ADC-interval using loop of (digitalWrite() taking 3,8µs + ADC-ISR)
      for (int j=0; j < ADC_save_Interval; j++)  // delay to ADC-interval
      {
        digitalWrite(13, HIGH);               // takes Oszi-measured 3,4/3,8µs 
      }
    }   
    stop_ADC_Free_Running();                  // stop ADC in Free-Running Mode and allow Arduino Standard analogRead()
    analogReference(DEFAULT);                 // URef = Vcc=5V   // analogReference(INTERNAL); // URef=1,1V    
    int dummy = analogRead(ADCPin_OPV_Volt);  // dummy read to skip first unsafe ADC measure after re-init
    digitalWrite(13, LOW);                    // turn off LED13


//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** (4b) visualize waveform on display ***
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //physics of sampled waveform values and physical display pixels (OLED-SSD1306):
          //display.drawPixel(0, 0, WHITE);         // pixel in "Upper-Left" corner of display
          //display.width(0-127) = 128              // OLED display width, in pixels
          //display.height(0-63) = 64               // OLED display height, in pixels
    //map used diagram coordinates (x=values, y=percent) to the pixels available on screen:
          // X-Axis:  x_value(0) = x_pixel(2)   to   x_value(124) = x_pixel(126)
          // Y-Axis:  y_value(0) = y_pixel(62)  to   y_value(100) = y_pixel(30)

    //scale y_Val values of pulse waveform to y_px of display pixels
    max_yValue = 0;                             // reset max value of all waveform samples to display
    for (int x=2; x <= 125; x++)                // use X-scale as array-pointer
    {
      if (ADCvalues[x] > max_yValue)            // search max y-Value to scale up/down to max y_px of display coordinates
      {
        max_yValue = ADCvalues[x];              // waveform y-values as byte
      }
    }
    
    //print to display
    if (CTR1_frequency < 3500)                 // <3500Hz waveform print each value (x=2 ... x <= 125)
    {
      for (int x=2; x <= 125; x++)  //use X-scale as array-pointer
      {
        //scale y-Values of pulse waveform to available display pixel-lines 
            //x_val(2-126)  equals  pixel 2-126 = 124 lines
            //y_val(0-100)  equals  pixel 62-30 = 32 lines
        scaled_yValue = map(ADCvalues[x], 0, max_yValue, 61, 30);      // waveform samples value scaled to display pixels          
        display.drawPixel(x, scaled_yValue, WHITE);                    // (x=2 ... x <= 125) fit to physical screen
      }
    }
    else                                        // >=3500Hz waveform print only value (x=2 ... x <= 62) and stretch to screen
    {
      for (int x=1; x <= 62; x++)  //use X-scale as array-pointer
      {
        //scale y-Values of pulse waveform to available display pixel-lines 
            //x_val(2-62)   equals  pixel 2-126 = 124 lines
            //y_val(0-100)  equals  pixel 62-30 = 32 lines
        scaled_yValue = map(ADCvalues[x], 0, max_yValue, 61, 30);      // waveform samples value scaled to display pixels          
        display.drawPixel(x*2, scaled_yValue, WHITE);                  // (x=2 ... x <= 62) stretched to screen
      }
    }
        
    display.display();              // copy buffer to display
    delay(1000);                    // add delay to longer show waveform screen before start new measure cycle
  }

  delay(3000);                      // show screen for 3s before start new measure cycle
 
  //End of Light Frequency Measure Loop:
} // end of Arduino "Main Loop"  
//------------------------------------------------------------------------
//   ****** End main program loop ******
//-------------------------------------------------------------------------



//#########################################################################
//   ****** Subroutines ******
//#########################################################################

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: increase_R_OPV ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
boolean increase_R_OPV()   //increase DigiPoti resistor in OPV amplifier for BPW34 light sensor
{
  if (R_RAW < R_max)                  // check if the resistor value can be increased
  {
    R_RAW++;
    Set_DigiPoti_newValue_for_OPV(R_RAW);
    return true;                      // new resistor value was set
  }
  else
  {
    return false;  //if resistor is requested to go above maximum allowed value
  }
} //end of Subroutine: increase_R_OPV
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: decrease_R_OPV ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
boolean decrease_R_OPV()   //decrease DigiPoti resistor in OPV amplifier for BPW34 light sensor
{
  if (R_RAW > R_min)                  // check if the resistor value can be increased
  {
    R_RAW--;
    Set_DigiPoti_newValue_for_OPV(R_RAW);
    return true;                      // new resistor value was set
  }
  else
  {
    return false;  //if resistor is requested to go above maximum allowed value
  }
} //end of Subroutine: decrease_R_OPV
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: Set_DigiPoti_newValue_for_OPV - feedback loop of OPV ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void Set_DigiPoti_newValue_for_OPV(byte R_RAW_Value) // DigiPoti R1 in feedback loop of OPV
{
//code for DigiPoti AD5242 (I2C=0x2C, 2x1M, 256 steps)
// Write Values to R1 or R2 :
//  - send to open Slave Address (open I2C with Address as 7 bit without the R/W-bit)
//  - send Instruction byte (1 byte)
//          bit 7 : A/B = RDAC subaddress select; 0 for RDAC1 and 1 for RDAC2.
//          bit 6 : RS = Midscale reset, active high.
//          bit 5 : SD = Shutdown in active high. Same as /SHDN which is in inverse logic.
//          bit 4 : O1 = Output logic pin latched values
//          bit 3 : O2 = Output logic pin latched values
//          bit 2-0 : not used
//  - send Data byte (1 byte)
// Read Values from R1 or R2 :
//  - send Slave Address (open I2C with Address as 7 bit without the R/W-bit)
//  - read Data byte (1 byte)

  //write to the DigiPoti AD5242 (0-1MOhm) - R1 in feedback loop of OPV
  // set new value for DigiPoti R-1
  Serial.print(F("setting R1 for OPV: "));
  Serial.println(R_RAW_Value);
  Wire.beginTransmission(AD5242_I2C_Addr);   // Prepare I2C transmission with Slave Device Address
  Wire.write(0x00);               // Instruction byte with pointer to AD5242 Sub-Address for POT RDAC-1 (bit-8 = L)
  Wire.write(R_RAW_Value);        // Value for new resistance value (as byte)
  Wire.endTransmission();         // Send queue and stop I2C transmission
} //end of sub "Set_DigiPoti_newValue_for_OPV"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: Set_DigiPoti_newValue_for_BPW34 - input signal for OPV ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void Set_DigiPoti_newValue_for_BPW34(byte R_RAW_Value) // set R2 to switch BPW34 ON/OFF
{
  //DigiPoti AD5242 (R2 used as binary switch 0 or 1MOhm) to turn OFF some BPW34 diodes to reduce OPV input signal when strong brightness
  // set new value for DigiPoti R-2
  Serial.print(F("setting R2 for BPW34: "));
  Serial.println(R_RAW_Value);
  Wire.beginTransmission(AD5242_I2C_Addr);   // Prepare I2C transmission with Slave Device Address
  Wire.write(0x80);               // Instruction byte with pointer to AD5242 Sub-Address for POT RDAC-2 (bit-8 = H)
  Wire.write(R_RAW_Value);        // Value for new resistance value (as byte)
  Wire.endTransmission();         // Send queue and stop I2C transmission
} //end of sub "Set_DigiPoti_newValue_for_BPW34"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: start_ADC_Free-Running ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void start_ADC_Free_Running(void) 
{
  // config and start ADC in Free-Running Mode with 1MHz speed (76kSamples/s) and ADC conversion-ready ISR
  // FYI: Arduino default prescale is 128 => 125 KHz ADC clock ; a pure ADC conversion takes 13 ADC clocks = 104µs (9600 ADC/s)

  //save ADC registers of default Arduino settings
  ADCSRA_save = ADCSRA;                               // saves the Register while ADC free-running mode
  ADCSRB_save = ADCSRB;                               // saves the Register while ADC free-running mode
  ADMUX_save = ADMUX;                                 // saves the Register while ADC free-running mode
  
  // stop ADC by reset ADCSRA – ADC Control and Status Register A
  ADCSRA = 0;   // clear ADCSRA register
                          // Bit 7 – ADEN=1: ADC Enable
                          // Bit 6 – ADSC=1: ADC Start Conversion
                          // Bit 5 – ADATE=1: ADC Auto Trigger Enable (Source by ADTS in ADCSRB)
                          // Bit 4 – ADIF: ADC Interrupt Flag (1=INT occured, cleared by HW by ISR)
                          // Bit 3 – ADIE=1: ADC Interrupt Enable
                          // Bits 2:0 – ADPS[2:0]: ADC Prescaler Select Bits 
                            // Sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
                            // In Arduino Uno: ADC clock is 16 MHz, Pre-Scaler is 128 and a conversion takes 13 clock cycles (125kHz=9600/s)
                            //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
                            //ADCSRA |= (1 << ADPS2);                   // 16 prescaler for 76.9 KHz (1MHz=76k/s, is max ADC frequency with precise results)

  // set ADCSRB – ADC Control and Status Register B
  ADCSRB = 0;   // clear ADCSRB register
                                // sets ADTS[2:0]=000 for free-running mode (ADC Auto Trigger Source)

  // set ADMUX – ADC Multiplexer Selection Register
  //Bit 7:6 – REFS[1:0]: Reference Selection Bits
  ADMUX |= (1 << REFS0);        // REFS1=0 + REFS0=1: reference voltage: AVCC with external capacitor at AREF pin
                                // REFS1=1 + REFS0=1: Internal 1.1V Voltage Reference with external capacitor at AREF pin
                                // REFS1=0 + REFS0=0: AREF, Internal Vref turned off
  //Bit 5 – ADLAR: ADC Left Adjust Result        
  ADMUX |= (1 << ADLAR);        // left align ADC value to 8 bits from ADCH register (allows only reading a 8-bit ADC value from ADCH)              

  //MUX[3:0]: Analog Channel Selection Bits
  ADMUX |= (ADC_Pin & 0x0F);    // 0000 = A0 analog input pin 
                                // 0001 = A1 analog input pin 
                                // 0111 = A7 analog input pin
                                // 1000 = A8 = Temperature Sensor
                                // 1110 = internal Volt-Ref 1.1V (VBG)
                                // 1111 = GND
  // set ADCSRA – ADC Control and Status Register A
  ADCSRA |= (1 << ADPS2);       // 16 prescaler for 76.9 KHz
//  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);       // 32 prescaler for 38,5 KHz
  ADCSRA |= (1 << ADATE);       // enable auto trigger
  ADCSRA |= (1 << ADIE);        // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);        // enable ADC
  ADCSRA |= (1 << ADSC);        // start ADC measurements
} //end of sub "start_ADC_Free-Running"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: stop_ADC_Free-Running ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void stop_ADC_Free_Running(void) 
{
  // stop ADC (was in Free-Running Mode with ADC conversion ready ISR and at 1MHz speed (76000 ADC/s))  
  ADCSRA = 0;   // Stop ADC and ADC-INT by clearing ADCSRA register
  
  //restore ADC register of default Arduino settings
  ADCSRA = ADCSRA_save;                               // saves the Register while ADC free-running mode
  ADCSRB = ADCSRB_save;                               // saves the Register while ADC free-running mode
  ADMUX = ADMUX_save;                                 // saves the Register while ADC free-running mode
                          
} //end of sub "stop_ADC_Free-Running"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: SplashScreen ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void splashscreen(void) 
{
  // display command: usage with Adafruit GFX lib
  // Character map: github.com/cacycleworks/st7735_charmap
  // FYI: on POR, the Adfruit GFX-library initializes the buffer with an Adafruit splash screen.
  // Tip: display.print((char) 0xDA);   // Prints character from ASCII value   
  // To show the display buffer on the screen, you MUST call display.display() to copy the buffer to the screen!
  
  display.clearDisplay();  
  
  // Draw a single pixel in white just for fun
  display.drawPixel(124, 60, WHITE);
  
  // Draw a line in white
  display.drawLine(0, 35, 127, 35, WHITE);
  delay(1);

  // Draw a rectangle in white
  display.drawRect(0, 0, display.width(), display.height(), WHITE);
  delay(1);
  display.display();

  // Invert and restore display
  delay(800);
  display.invertDisplay(true);
  delay(150);
  display.invertDisplay(false);
  delay(200);
  
  //write text 
  display.setTextSize(1);               // set 1x scale text
  display.setTextColor(BLACK, WHITE);   // set 'inverse' text
  display.setCursor(5,20);              // display 0,0 (x,y) starts at left,top corner  
  display.print(F("W.Franke - SW v"));
  display.println(SW_version, 2);       // print float with 2 decimal places
  
  display.setTextSize(1);               // set 1x scale text
  display.setTextColor(WHITE);          // set white text  
  display.setCursor(5,5);               // display 0,0 (x,y) starts at left,top corner  
  display.println(F("Light Flicker Test"));

  display.display();

} //end of sub "SplashScreen"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



//#########################################################################
//   ****** Interrupt Service Routines ******
//#########################################################################

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: Timer1 Counter Overflow ISR - counter ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
ISR (TIMER1_OVF_vect)           // Timer1 Counter Overflow ISR
{
  ++overflowCount;              // count number of Counter1 overflows
}  // end of TIMER1_OVF_vect
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// *** Subroutine: ADC conversion ready ISR ***
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
ISR (ADC_vect)                  // ADC conversion ready ISR
{
  ADCvalues[ADCvalue_Pointer] = ADCH;     // read 8 bit value from ADC (left aligned) into array
}  // end of ADC conversion ready ISR
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
