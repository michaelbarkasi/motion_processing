/****************************************************************
 * 
 * 2-Sensor Motion Sonification via ttPWM and 3D position processing for multi-pivot body movements
 * 
 * by Michael Barkasi
 * copyright 2022
 * Performance Sonification Corp. 
 * performancesonification.ca
 * 
 * For use with three 20948 sensors (SPI) and SparkFun's Thing Plus (SAMD51/Cortex M4)
 * 
 * Sketch grabs raw gyro and accel data from sensors via SPI and interrupts, detects motion vs rest, learns a 
 *  path of motion from an initial position (built by integrating gyro readings and using a custom-written
 *  motion-processing alogirthm to work out position in 3D space), and based on that learned path sonifies
 *  subsequent motion. At start, sketch also grabs leftward direction (from user motion) and computes conversion from local coordinates
 *  to an external Cartesian reference frame with y pointing left, x forward, and z up. Assuming proper limb 
 *  segments entered, unit converts the local rotation readings in a position path through that external coordinate frame. 
 *  
 *  Requires the SparkFun libary ICM_20948.h and Arduino libraries for the Cortex M4. 
 * 
 * **************************************************************
 * Some code related to SPI/Interrupts for Sen20948 taken from: 
 * 
 * Example3_Interrupts.ino
 * ICM 20948 Arduino Library Demo
 * Builds on Example2_Advanced.ino to set up interrupts when data is ready
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: June 5 2019
 *
 * Some code related to the setup for Sen20948 taken from (and now modified): 
 * 
 * Example2_Advanced.ino
 * ICM 20948 Arduino Library Demo
 * Shows how to use granular configuration of the ICM 20948
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * Example7_DMP_Quat6_EulerAngles.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, April 25th, 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * Also, the redefine of "CM_20948_Status_e ICM_20948::initializeDMP(void)" in SensorSetup is from: 
 * 
 * Example10_DMP_FastMultipleSensors.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, April 25th, 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 *  From SparkFun on examples 7 and 10: 
 *  ** This example is based on InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
 *  ** We are grateful to InvenSense for sharing this with us.
 *  
 * License for this code contained in License.md. This license is the MIT License, which is found here: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/License.md
 *      and here: https://opensource.org/licenses/MIT
 *      
 *      The relevant part: "Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation 
 *                          files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, 
 *                          modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 *                          is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included 
 *                          in all copies or substantial portions of the Software."
 * 
 * ***************************************************************
 * 
 * Sound Generation: 
 * 
 * The code for the base wave form (from TCC1), contained in SetupBasePulseWave 
 *    is taken largely from a post by MartinL (Jan 2019) on the Arduino forums; 
 *    See: https://forum.arduino.cc/t/metro-m4-express-atsamd51-pwm-frequency-and-resolution/566491/2
 *  It was originally written written for the Metro M4, to run at some other frequency 
 *    (I changed the timer speeds, prescalers, and PER/CC0 values to adjust frequency to something higher, e.g. 240kHz).
 *  I modified the code to work with the SparkFun Thing Plus (SAMD51) by looking up the necessary registery values in the SAMD51 
 *    datasheet (see page numbers in the code).
 *  It's written to work with pin D11 of the SparkFun Thing Plus, which is pin PA16 on the SAMD51; which (I believe) must be used with TCC1/WO[0]
 *    (This latter part is also a modification from MartinL, who used TCC0.)
 *  Finally, to get the base wave form to actually generate a signal on D11, 
 *    I needed two more lines of code, to set DIRSET and OUTCLR, which I got from Shawn Hymel's write-up (Dec 22, 2018);
 *    See: https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
 *  Shawn notes that his write-up is also based on posts by MartinL.
 *  
 *  The code for the timer controlling the hearable oscilation (TCC2), contained in SetupHearableOscilation and the associted Handlers, is also based on  
 *    the above mentioned code from MartinL, along with typical examples of NVIC setups found online, like this one: 
 *    https://emalliab.wordpress.com/2021/04/16/comparing-timers-on-samd21-and-samd51-microcontrollers/
 *    
 *  How the sound is generated: Two timers are used. The first (TCC1) controls a base wave form of digital pulses with ultrasonic (e.g., 240 kHz) oscilation. 
 *    Since we're using TCCs, we can do PWM (pulse-width modulation). E.g., 240 kHz is too fast for a speaker, which (acting as a low-pass filter)
 *    only "sees" the average voltage, which is digital HIGH * duty cycle. (Duty cycle is pulse width, e.g. pulse on for 70% of the period and off 
 *    for 30%.) The second timer (TCC2) oscilates pulse width (duty cycle) of this wave at a hearable frequency, flipping between 0 and some second 
 *    nonzero value. This second nonzero value essentially ends up controlling volume, since it covaries with the voltage "seen" on each pulse by the speaker.
 *    The "width" of this second timer ends up being the wave width (as in, sound wave width) of the hearable oscilination, which is controlled here by 
 *    the varible "pulsefraction" (changes the quality of the sound). Finally, note that I've layered in the first harmonic: e.g., calling for a 440Hz pitch tone
 *    actually produces an 880Hz tone, where every other wave form only has some fraction (harmonic_ratio) of the amplitude of the main frequency waves. 
 *    
 *  Legal Stuff related to grabbing MartinL's code from the Arduino forums: 
 *  
 *  For more on the use of code from the forum, see: 
 *  https://www.arduino.cc/en/terms-conditions
 *  Also see the Contributor License Agreement (CLA), which I assume governs MartinL's code in that post: https://www.arduino.cc/en/Main/ContributorLicenseAgreement
 *  The relevant part of the CLA seems to be: "Subject to the terms and conditions of this Agreement, You hereby grant to Arduino and to recipients of software 
 *                                             distributed by Arduino a perpetual, worldwide, non-exclusive, no-charge, royalty-free, irrevocable copyright 
 *                                             license to reproduce, prepare derivative works of, publicly display, publicly perform, sublicense, and distribute 
 *                                             Your Contributions and such derivative works."
 *  And also this: "Subject to the terms and conditions of this Agreement, You hereby grant to Arduino and to recipients of software distributed by Arduino a 
 *                  perpetual, worldwide, non-exclusive, no-charge, royalty-free, irrevocable (except as stated in this section) patent license to make, have made, 
 *                  use, offer to sell, sell, import, and otherwise transfer the Work, where such license applies only to those patent claims licensable by You that 
 *                  are necessarily infringed by Your Contribution(s) alone or by combination of Your Contribution(s) with the Work to which such Contribution(s) 
 *                  was submitted."
 *  
 *  I assume it is fine to use small bits of code from places like https://emalliab.wordpress.com, as (1) I've substantially modified it, 
 *                                                                                  (2) it's derivative itself, (3) similar code is widely available online, 
 *                                                                                  (4) it was freely available online with no stated restrictions, and 
 *                                                                                  (5) it's a small part of my overall code. 
 *  
 * 
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// Definitions for Serial Port
#define SERIAL_PORT Serial

// Definitions for Interrupts
#define INT_PIN_myICM1 12 
//#define INT_PIN_myICM2 13
#define INT_PIN_myICM3 4

// Definitions for SPI
#define SPI_PORT SPI     
#define SPI_FREQ 4500000 // 7000000 is max for Sen20948 (see datasheet)? Sparkfun defaults to 5M; other TDK sheets suggest 2.5M
#define CS_PIN_myICM1 5 // in the PCB being made, this is changed to D10      
//#define CS_PIN_myICM2 6
#define CS_PIN_myICM3 9  
ICM_20948_SPI myICM1; // Arbitrary name for the sensor
//ICM_20948_SPI myICM2; // Arbitrary name for the sensor
ICM_20948_SPI myICM3; // Arbitrary name for the sensor

// Motion Model - Dimensions
#define limbseg1length 345.0 // Upperarm
#define limbseg2length 260.0 // Forearm
#define wristthickness 35.0 // 
#define elbowthickness 80.0 //
#define s1ratio 0.5
#define s2ratio 0.8
const bool LEFT_HANDED = false;
// Controlling Motion Model:
const float wrist_marker_height = 13.0;
const float sensorheighabovelimbthickness = 20.0; // = 10 if sensor taped on, = 20 if velcro is used
const bool t_as_wrist = true;
const bool t_as_marker = false;
const bool recompute_IP_at_rest = true;
const float seconds_to_record = 2.0;
#define left_trigger_value 1.05 // keep as high as possible; may lower to 1.025 if participant struggling
int date = 20220324;
int subject_num = 001;
int condition_code = 2;
int motion_num = 0; // to start at 1, set as 0. 
/*
 *  Condition codes: 
 *  0 = no sound
 *  1 = error sonification
 *  2 = reach space sonification
 *  3 = error sonification control 
 *  4 = reach space sonification control
 *  
 */

// Misc Motion - Sensors and labelling
#define LimbSeg1Sensor Sensor1
#define LimbSeg1SensorPrint "Sensor1"
#define LimbSeg1Sensor_GravityIP Sensor1_GravityIP
#define LimbSeg1Sensor_LEFT Sensor1_GravityLEFT
#define LimbSeg1SensorIndex 0
#define LimbSeg1SensorRot Sensor1_Rot
//#define LimbSeg1SensorSavedReadingsRot SavedReadingsRot1

#define LimbSeg2Sensor Sensor3
#define LimbSeg2SensorPrint "Sensor3"
#define LimbSeg2Sensor_GravityIP Sensor3_GravityIP
#define LimbSeg2Sensor_LEFT Sensor3_GravityLEFT
#define LimbSeg2SensorIndex 2
#define LimbSeg2SensorRot Sensor3_Rot
//#define LimbSeg2SensorSavedReadingsRot SavedReadingsRot3

#define LimbSeg1Name "upperarm"
#define LimbSeg2Name "forearm"

// Variables for interrupts (pulling sensor data)
volatile bool isrFired1 = false;
//volatile bool isrFired2 = false;
volatile bool isrFired3 = false;

// Variables and constants for sound generation: 
const int soundoutpin = 11; // What chip pin is wired to the speaker? This *must* be pin D11, which is what ttPWMSetup assumes. A lot needs to be changed to change this pin. 
const long BasePWM_Freq = 240000; // Sets frequency of the base pulse wave. Safe values: 120k, 240k, 480k.
  /* The higher BasePWM_Freq, the lower the volume resolution. 480k = 250 volume levels, 240k = 500 volume levels, and 120k = 1000 volume levels.
     But, I suspect that lower values for BasePWM_Freq lead to poor acustic wave forms and bad sound quality, so 240k is a compromise value. */
const long BasePWM_PER = ( 120000000 / BasePWM_Freq ) - 1; // Don't change; Sets the top value of the timer controlling the base pulse wave. Assumes f_GCLKTCC = 120 MHz, PRESCALER = 1 (see ttPWMSetup).
const int MaxAmplitude = (int) BasePWM_PER; // Don't change
volatile int amplitude = MaxAmplitude; // Variable controlling volume
const float harmonic_ratio = 0.15; // What proportion of the main frequency (TF) amplitude do we want the first harmonic (2 * TF) amplitude to be? 
volatile int amplitude_harmonic = (int)( MaxAmplitude * harmonic_ratio ); // variabl controlling volume of the harmonic overtone
volatile int amp = 1.0; // intermediate variable for computing amplitude (don't want processor grabbing "amplitude" with weird value via interrupt in the middle of computing it.
const int initialpitch = 440;  // Can be changed, but 440 works well
const int PitchValueMin = 200; // Can be changed
const int PitchValueMax = 6000; // Can be changed
volatile int TF = initialpitch; // Variable controlling pitch of tone
volatile long TCTOPL = 12000000L / (2 * TF) - 1; // This must be the formula, written for the ttPWM; need to double the TF ("2 *") to account for the harmonic frequency.
int TCTOP = (uint16_t)TCTOPL; // Leave as is, should be marked volatile
const float initialpulsefraction = 0.35;
volatile float pulsefraction = initialpulsefraction; // Variable controlling the width of the sound waves (as a fraction of total wave period). 
volatile bool harmonic = true; // Variable to flipping between a main frequency wave and a harmonic overtone wave
//bool pulsefraction_up = true; // uncomment for use with oscilating wave width in TC2_Handler
 /*  Explanation of these variables (and how the sound is generated): 
    amplitude: The amplitude value (essentially) controls volume or loudness (signal height). More precisely, it's a "gain" control. 
      It should range between 0 and the value in the formula in BasePWM_PER (assuming f_GCLKTCC = 120 MHz, PRESCALER = 1).
      BasePWM_PER is the TOP value of the counter controlling the base pulse wave form;
      For example, BasePWM_PER = 499 generates a 240 kHz base pulse wave. 
      duty cycle ends up being amplitude / BasePWM_PER. 
    TF: Tone frequency; Mathematically, anything from 183 to 6 million are valid 
      given the code/math/architecture. Values below 183 will overflow the 16-bit register, 
      and values over 6 million end up in nonsense like dividing by zero or negative numbers.
      Of course, since a good audible range is something like 200 to 8000, this is fine. 
    TCTOPL: Variable stores PER (TOP) value of counter TCC2, while TCTOP * pulsefraction controls the capture-compare (CC) 
      value of TCC2. TCC2 is the counter ultimately controlling the tone pitch. Every time TCC2 overflows its PER counter, an interrupt 
      flips the pulse width of the base waveform to the value of the variable "amplitude" (or "amplitude_harmonic"), 
      turning the wave on. When the CC value of TCC2 is hit, a second interrupt flips this pulse width to zero. Frequency (pitch) of the tone is 
      thus equal to oscilator speed (cycles per sec) divided by the number of times the counter overflows twice (since every other overflow is initiating
      a harmonic overtone wave). Since the oscilator used for TCC2 runs at 12 MHz, this means that TF = 12000000 / (2 * (TCTOP - 1)). We have to subtract 1 
      because the counter starts at 0, not 1. A simple rearrangement of this equation gets us the formula for TCTOP. 
      Note: We need this extra step because 12000000 is longer than 16 bit; 
      a 32 kHz oscilator would save us this step, but would offer poor control/resolution of tone pitch.
    TCTOP: Finally, convert into int16 for register */

// Core Vector Variables
struct Vectors {
  float r = 0.0; // Uncomment if we need to use quaternions, in which case this is q0, hence, it should initialize as zero, in case it doesn't get set elsewhere and we're dealing with a vector in R^3
  float x = 0.0; // q1; if ever encoding quaternions, think of x/y/z as q1/q2/q3.  
  float y = 0.0; // q2; it doesn't matter what we initialize these values as, they all get overwritten. 0/1/0/0 is a holdover from previous programs that used quaternions.
  float z = 0.0; // q3
} Sensor1_Acc, Sensor1_Gyro, // Sensor1_Mag, 
  //Sensor2_Acc, Sensor2_Gyro, // Sensor2_Mag,
  Sensor3_Acc, Sensor3_Gyro, // Sensor3_Mag,
  Sensor1_GyroSaved, Sensor3_GyroSaved, // , Sensor2_GyroSaved // these are used when integrating the gyro into the ROT vector 
  Sensor1_Rot, Sensor3_Rot, // Sensor2_Rot,
  Sensor1_GravityIP, Sensor3_GravityIP, // Sensor2_GravityIP, 
  Sensor1_GravityLEFT, Sensor3_GravityLEFT, // Sensor2_GravityLEFT,
  Sensor1_GravityBias, Sensor3_GravityBias, // Sensor2_GravityBias,
  Sensor1_GyroBias, Sensor3_GyroBias, // Sensor2_GyroBias, 
  p1,p2,t,a1x,a1y,a1z,s1,a2x,a2y,a2z,s2,
  p1_IP,p2_IP,t_IP,a1x_IP,a1y_IP,a1z_IP,s1_IP,a2x_IP,a2y_IP,a2z_IP,s2_IP,
  zerov;

// Variables related to detecting motion and setting up the unit's states once motion/rest is detected. 
#define MOTIONCHECKSENSOR_Gyro Sensor3_Gyro
bool WAITING_FOR_INITATION = true;
bool IN_MOTION = false;
bool IPfound = false;
bool WAITING_FOR_REST = true;
bool MOTION_JUST_ENDED = false;
bool MOTION_JUST_STARTED = false;
int jostle_buff_count = 0;
int jostle_buff_count_saved = 1;
int jostle_noise_filter = 100; // at 2ms sampling, 50 = 100ms to allow for sensor jostling on initiation (does not seem to noticably impact latency, and sensor is still jumpy)
int rest_buff_count = 0;
int rest_noise_filter = 100; // at 2ms sampling, 50 = 100ms to allow for sensor jostling on stop 
const float motionthreshold_gyro = (10.0 * (PI/180.0)) * (10.0 * (PI/180.0)); // How fast (in radians per second) must a sensor be moving (i.e., rotating) for it to be considered "in motion"?
                              // Number here is magnetude of the *total* rotation <x,y,z> vector, not rotation about any one axis. (3 degrees per axis is standard noise)
const float motionthreshold_gyro_high = (motionthreshold_gyro * 3.0) * (motionthreshold_gyro * 3.0);

// Variables controlling how often the sound is updated during the main loop
const int SAMPLERATE = 1001; // In Hz. How fast do we want the unit to update tone? Ideal rate depends on the sensor's update rate and how much math we're trying to do. 
const int SAMPLECOUNTER = (int) (750000 / SAMPLERATE); // This is the TOP value to which the counter controlling attempted sample reads runs. 750000 is speed of clock clocking the sample timer.
bool UPDATE_SOUND_NOW = true; // The timer (TC2) controlling sound update rate will flip this on when it's time to read. 
const float dt = 1.0 / SAMPLERATE;

// Variables related to or set during training the unit on a motion: 
bool TRAINING = true;
bool IPSAVED = false;
bool WAITING_TO_INITATE_SAMPLE_MOTION = true;
bool WAITING_FOR_SAMPLE_MOTION_END = false;
const int NUM_TRAINING_MOTIONS = 1;
const int maxsamplestorecord = (int) (SAMPLERATE * seconds_to_record); // How many samples should we try to record?
const int IPcounterMax = 1000;
int IPcounter = 0; // want to sample readings at IP a bunch and average them; variable for keeping track of samples taken
int MOTsamplecount = 0; // For keeping track (in training and live sonification) of which sample we're at in the time sequence. 
int MOTsamplecount_saved = 1;
const float IPtolerancefraction = 0.1; // This fraction used to set the above value, e.g. ".1 = within 10 percent". 
float IPtolerance; // How close to the gravity vectors recorded at the start do we need to be for the unit to think user is in the IP? 
struct Vectors p2_model[maxsamplestorecord];
struct Vectors t_model[maxsamplestorecord];

// Variables used during live sonification of motion
bool PRINTED_RUN_SUMMARY_DATA = false;
bool SONIFYING = false;
bool smoothtail = false; // for adjusting time-warping algorithm ("tail" of model)
unsigned long time1 = 0; // time1, time2, and time3 used to time how long we've been recording, during live sonification
unsigned long time2 = 0;
unsigned long time3 = 0;
const int MaxReadingsToSave = maxsamplestorecord;
int MOTsamplecount_livesaved[MaxReadingsToSave];
int tail = 1000;
int Realsamplecount = 0; // because of the compensation for different velocity, this comes apart from the MOTsamplecount
struct Vectors p2_saved[MaxReadingsToSave];
struct Vectors t_saved[MaxReadingsToSave];
float samplerate_live = 1.0; // Variable used to store/compute the sample rate of a given motion sonification (to confirm it's at whatever we've set)
float t_err;
float p2_err;
float total_error;
bool TWON = true;
int TWbegin = 1;
bool TWFIRST = true;
const float LEFTtrigger = left_trigger_value; 
const float Gyro_multiple = PI / (16.4 * 180.0); // puts gyro readings in radians
const float Acc_multiple = 9.80665 / 2048.0; // puts accel readings in m/s^2
int amp_saved = MaxAmplitude;
int amp_discontinuity_sample = 0;
bool amp_discontinuity = false;
float dist_target = 0;
float dist_travelled = 0; // reset

float avg_last_error = 0;

// Variables set during training and used to control the sonification (how the motion is mapped to sound). 
float ty_min;
float ty_max;
float tz_min;
float tz_max;

bool ready1; bool ready3;

void setup() {
  
  pinMode(soundoutpin, OUTPUT);

  SERIAL_PORT.begin(115200); // Begin Serial Port
  while (!SERIAL_PORT);

  SERIAL_PORT.println(F("Press any key to begin."));
  while (!SERIAL_PORT.available()) {
    delay(1);
  }

  // Enable SPI communication 
  SPI_PORT.begin();
  
  // Setup TCC1, which will generate the base pulse wave
  // Set up the generic clock (GCLK7) to clock timer TCC1 
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120 MHz clock source by divisor 1: 120 MHz/1 = 120 MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization  
  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC1 peripheral channel, see p. 169; 25 = GCLK_TCC1/GCLK_TCC0
                          GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC1
  // below: Port (ulPort) is PORTA; pin (ulPin) is PORT_PA16 (= D11)
  PORT->Group[g_APinDescription[11].ulPort].DIRSET.reg = g_APinDescription[11].ulPin; // Set pin as output
  PORT->Group[g_APinDescription[11].ulPort].OUTCLR.reg = g_APinDescription[11].ulPin; // Set pin as low once CC0 reached (toggles back to high once value in PER is reached?)
  // Enable the peripheral multiplexer on pin D11
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
  // Set the D11 (PORT_PA16) peripheral multiplexer to the correct peripheral (even port number)
  PORT->Group[g_APinDescription[11].ulPort].PMUX[8].reg |= 0x5; // see p. 900, 923
  TCC1->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 1, 120 MHz/1 = 120 MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 
  TCC1->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC1 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC1->SYNCBUSY.bit.WAVE)                    // Wait for synchronization
  TCC1->PER.reg = BasePWM_PER;                        // Set-up the PER (period) register;
  while (TCC1->SYNCBUSY.bit.PER);                    // Wait for synchronization
                                                     // Formula: f_PWM = f_GCLKTCC / (PRESCALER (PER + 1))
                                                     // In this case, f_GCLKTCC = 120 MHz, PRESCALER = 1;
                                                     // So, for example, to produce a 240 kHz base pulse wave: 120 x 10^6 / 500 = 240 x 10^3
  TCC1->CC[0].reg = amplitude;                    // Set-up the CC (counter compare), channel 0 register; Recall, this controls volume. 
  while (TCC1->SYNCBUSY.bit.CC0);                    // Wait for synchronization
  TCC1->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC1
  while (TCC1->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
  
  // Setup TCC2, which will generate a hearable oscilation off the base pulse wave: 
  // Set up a generic clock (GCLK8) to clock timer TCC2 
  GCLK->GENCTRL[8].reg = GCLK_GENCTRL_DIV(4) |       // Divide the 48MHz clock source by divisor 4: 48MHz/1 = 12MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK8
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL8);               // Wait for synchronization  
  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC2 peripheral channel, see p. 169; 29 = GCLK_TCC2/GCLK_TCC3
                          GCLK_PCHCTRL_GEN_GCLK8;    // Connect generic clock 8 to TCC2
  TCC2->CTRLA.bit.ENABLE = 0;            // Disable TCC2
  while (TCC2->SYNCBUSY.bit.ENABLE);     // Wait for sync
  TCC2->CTRLA.reg = TC_CTRLA_SWRST;      // Reset TCC2
  while (TCC2->SYNCBUSY.bit.SWRST);      // Wait for sync
  TCC2->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;   // Set TCC2 wave generation mode to Match Frequency Generation
  TCC2->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE; // No prescaler (Divide by 1)
  TCC2->PER.reg = TCTOP;
  TCC2->CC[0].reg = (int) TCTOP * pulsefraction;                 // Write the counter value controlling wave form frequency
  while (TCC2->SYNCBUSY.reg > 0);          // Wait for sync
  // Setup the interrupts used to adjust pulse width of the base 240 kHz wave form
  // Basic idea: MC0 is triggered on a CC0 match, OVF when the counter hits top. See pp. 1766, 1976, 1984
  NVIC_DisableIRQ(TCC2_1_IRQn); // TCC2_1_IRQn is 98, the number for the MC0 channel (see p. 73)
  NVIC_ClearPendingIRQ(TCC2_1_IRQn);
  NVIC_SetPriority(TCC2_1_IRQn, 0);
  NVIC_EnableIRQ(TCC2_1_IRQn);
  // Set interrupt register
  TCC2->INTENSET.bit.MC0 = 1;
  while (TCC2->SYNCBUSY.reg > 0);
  // Setup the interrupts used to adjust pulse width of the base 240 kHz wave form
  NVIC_DisableIRQ(TCC2_0_IRQn); // TCC2_0_IRQn is 97, the number for the overflow (OVF) channel (see p. 73)
  NVIC_ClearPendingIRQ(TCC2_0_IRQn);
  NVIC_SetPriority(TCC2_0_IRQn, 0);
  NVIC_EnableIRQ(TCC2_0_IRQn);
  // Set interrupt register
  TCC2->INTENSET.bit.OVF = 1;
  while (TCC2->SYNCBUSY.reg > 0);

  // Setup Interrupts
  // myICM1
  pinMode(INT_PIN_myICM1, INPUT_PULLUP); // Using a pullup b/c ICM-20948 Breakout board has an onboard pullup as well and we don't want them to compete
  attachInterrupt(digitalPinToInterrupt(INT_PIN_myICM1), icmISR1, FALLING); // Set up a falling interrupt; // interrupt 6?
  // myICM2
  //pinMode(INT_PIN_myICM2, INPUT_PULLUP);                                   
  //attachInterrupt(digitalPinToInterrupt(INT_PIN_myICM2), icmISR2, FALLING); // interrupt 3?
  // myICM3
  pinMode(INT_PIN_myICM3, INPUT_PULLUP);                                   
  attachInterrupt(digitalPinToInterrupt(INT_PIN_myICM3), icmISR3, FALLING); // interrupt 1?

  //myICM1.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial about ICM1
  //myICM2.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial about ICM2
  //myICM3.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial about ICM3

  // Initialize the sensors over SPI
  initializeSensor_SPI (myICM1, CS_PIN_myICM1, 1);
  //initializeSensor_SPI (myICM2, CS_PIN_myICM2, 2);
  initializeSensor_SPI (myICM3, CS_PIN_myICM3, 3);

  // Configure the DMP for each sensor
  // configureDMP (myICM1);
  // configureDMP (myICM2);
  // configureDMP (myICM3);

  // Configure sensors for raw data pulls
  configureSensor (myICM1);
  //configureSensor (myICM2);
  configureSensor (myICM3);

  // Configure interrupts for each sensor
  configureInterrupts (myICM1);
  //configureInterrupts (myICM2);
  configureInterrupts (myICM3);

  // Setup TC2, which will control when samples are (attempted to be) read and the sound is adjusted: 
  // Set up a generic clock (GCLK9) to clock timer TC2
  GCLK->GENCTRL[9].reg = GCLK_GENCTRL_DIV(8) |       // Divide the 48MHz clock source by divisor 8: 48MHz/8 = 6MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK8
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL9);               // Wait for synchronization  

  GCLK->PCHCTRL[26].reg = GCLK_PCHCTRL_CHEN |        // Enable the TC2 peripheral channel, see p. 169; 26 = GCLK_TC2
                          GCLK_PCHCTRL_GEN_GCLK9;    // Connect generic clock 9 to TC2

  TC2->COUNT16.CTRLA.bit.ENABLE = 0;            // Disable TC2
  while (TC2->COUNT16.SYNCBUSY.bit.ENABLE);     // Wait for sync
  TC2->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;      // Reset TC2
  while (TC2->COUNT16.SYNCBUSY.bit.SWRST);      // Wait for sync

  TC2->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16; // Set TC2 for 16 bit mode? (Isn't that default?)
  TC2->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;   // Set TC2 wave generation mode to Match Frequency Generation
  TC2->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_ENABLE; // Prescaler (Divide by 8)
  TC2->COUNT16.CC[0].reg = SAMPLECOUNTER;                 // Write the counter value controlling wave form frequency
  while (TC2->COUNT16.SYNCBUSY.reg > 0);          // Wait for sync

  // Setup the interrupts
  NVIC_DisableIRQ(TC2_IRQn);
  NVIC_ClearPendingIRQ(TC2_IRQn);
  NVIC_SetPriority(TC2_IRQn, 1);
  NVIC_EnableIRQ(TC2_IRQn);

  // Set interrupt register
  TC2->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC2->COUNT16.SYNCBUSY.reg > 0);

  // Set gravity biases: 
  Sensor1_GravityBias.x = -0.05; 
  Sensor1_GravityBias.y = 0.075; 
  Sensor1_GravityBias.z = 0.06; 
  //Sensor2_GravityBias.x = 0.04; 
  //Sensor2_GravityBias.y = 0.0; 
  //Sensor2_GravityBias.z = 0.2; 
  Sensor3_GravityBias.x = 0.09; 
  Sensor3_GravityBias.y = 0.11; 
  Sensor3_GravityBias.z = -0.08; 
  // Make sure we set values for the saved gyro vectors
  Sensor1_GyroSaved = zerov;
  //Sensor2_GyroSaved = zerov;
  Sensor3_GyroSaved = zerov;

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));
  amplitude_update(0); 

  delaywithFIFOreset(1000);

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Please be still and in the initial position."));
  SERIAL_PORT.println();
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
    delay(1);
  }
  SERIAL_PORT.println(F("Press any key when in the initial position."));
  while (!SERIAL_PORT.available()) {
    delay(1);
  }
  SERIAL_PORT.println();
  SERIAL_PORT.println(F("At the start of the tone, shift arm to the left smoothly."));

  delaywithFIFOreset(2500);

  // Need to find the left direction from user's motion;
  // Want to subtract off directional changes due to rotation about gravity vector, since the upper arm will 
  // rotate appreciably in this way, giving a LEFT vector that points in a different direction than the one from the forearm. 
  Vectors G_start1;
  Vectors G_start3;
  Vectors xaxis; xaxis.x = 1.0, xaxis.y = 0, xaxis.z = 0;
  Vectors yaxis; yaxis.x = 0, yaxis.y = 1.0, yaxis.z = 0;
  Vectors zaxis; zaxis.x = 0, zaxis.y = 0, zaxis.z = 1.0;
  UpdateTF(initialpitch + 220);
  amplitude_update(MaxAmplitude);
  bool NEEDLEFT = true;
  int LEFTcounter = 1;
  int LEFTcounterMax = 100;
  int LEFTtimer1 = micros();
  int LEFTtimer2;
  float LEFTdt;
  float s1adj = 0; float s3adj = 0;
  float G_start3_mag;
  bool ready1; bool ready3;
  while(NEEDLEFT) {
    // Always fetch readings as soon as interrupts are noticed
    if (isrFired1) { // If our isr flag is set then clear the interrupts on the ICM
      isrFired1 = false;
      FetchReadingsSensor (myICM1, Sensor1_Acc, Sensor1_Gyro); //, Sensor1_Mag); // Not pulling Mag data
      Sensor1_Acc = vadd(Sensor1_Acc,Sensor1_GravityBias); // compensate for the accel bias; gyro bias handled in integratesensordata_advanced
      ready1 = true;
    }
//    if (isrFired2) { // If our isr flag is set then clear the interrupts on the ICM
//      isrFired2 = false;
//      FetchReadingsSensor (myICM2, Sensor2_Acc, Sensor2_Gyro); //, Sensor2_Mag); 
//      Sensor2_Acc = vadd(Sensor2_Acc,Sensor2_GravityBias);
//    }
    if (isrFired3) { // If our isr flag is set then clear the interrupts on the ICM
      isrFired3 = false;
      FetchReadingsSensor (myICM3, Sensor3_Acc, Sensor3_Gyro); //, Sensor3_Mag);
      Sensor3_Acc = vadd(Sensor3_Acc,Sensor3_GravityBias);
      ready3 = true;
    }
    if (ready1 && ready3 && LEFTcounter == 1 && vmagSqrd(MOTIONCHECKSENSOR_Gyro) < motionthreshold_gyro) {
      // Subject should be at rest, so Acc should be the gravity vector pointing up; grab as reference point
      Sensor1_GravityLEFT = Sensor1_Acc; 
      //Sensor2_GravityLEFT = Sensor2_Acc;
      Sensor3_GravityLEFT = Sensor3_Acc;
      G_start1 = Sensor1_Acc;
      G_start3 = Sensor3_Acc;
      G_start3_mag = vmag(G_start3);
      ready1 = false;
      ready3 = false;
      LEFTcounter++;
    } else if (ready1 && ready3 && (vmag(Sensor3_Acc) < LEFTtrigger * G_start3_mag)) { // if not moving significantly, just keep track of gravity
      // For first 250 ms or so, just keep track of gravity vector
      LEFTtimer2 = micros(); 
      LEFTdt = (((float)LEFTtimer2 - (float)LEFTtimer1) / 1000000.0);
      LEFTtimer1 = micros(); 
      Vectors rot1 = vscaler_mult(Sensor1_Gyro,LEFTdt); 
      Vectors rot3 = vscaler_mult(Sensor3_Gyro,LEFTdt);
      Vectors q1 = rot_quat(rot1,xaxis,yaxis,zaxis,zerov);
      Vectors q3 = rot_quat(rot3,xaxis,yaxis,zaxis,zerov);
      // If we rotated G_start with q, we'd be rotating it with the physical axis system (but G_start is fixed). 
      // The below equations are in the coordinates of that moving physical axis system
      // In that coordinate system, G_start appears to rotate in the direction opposite the physical axes' rotation
      // So, need to rotate with the conj of q.
      G_start1 = qvq(quat_conj(q1),G_start1);
      G_start3 = qvq(quat_conj(q3),G_start3); 
      ready1 = false;
      ready3 = false;
    } else if (ready1 && ready3) { // if moving, grab samples
      // As before, start by keeping track of gravity vector in our local physical axis system
      LEFTtimer2 = micros(); 
      LEFTdt = (((float)LEFTtimer2 - (float)LEFTtimer1) / 1000000.0);
      LEFTtimer1 = micros(); 
      Vectors rot1 = vscaler_mult(Sensor1_Gyro,LEFTdt); 
      Vectors rot3 = vscaler_mult(Sensor3_Gyro,LEFTdt);
      Vectors q1 = rot_quat(rot1,xaxis,yaxis,zaxis,zerov);
      Vectors q3 = rot_quat(rot3,xaxis,yaxis,zaxis,zerov);
      G_start1 = qvq(quat_conj(q1),G_start1);
      G_start3 = qvq(quat_conj(q3),G_start3); 
      // Now find quat reaigning the local coordinate system with gravity vector as zaxis
      Vectors qa1 = alignment_quat(G_start1,zaxis);
      Vectors qa3 = alignment_quat(G_start3,zaxis);
      // Transform coordinates using this and recompute q, the quat giving the felt rotation in these new coordinates
      q1 = rot_quat(rot1,qvq(qa1,xaxis),qvq(qa1,yaxis),qvq(qa1,zaxis),zerov);
      q3 = rot_quat(rot3,qvq(qa3,xaxis),qvq(qa3,yaxis),qvq(qa3,zaxis),zerov);
      // Extract the rotation felt about the zaxis = gravity, using atan2 to preserve the direction
      float phi1 = atan2(q1.z,q1.r); 
      float phi3 = atan2(q3.z,q3.r); 
      s1adj += phi1;
      s3adj += phi3;
      // Rotate the acceleration vectors about gravity opposite this direction
      Sensor1_Acc = qvq(formquat(-1*phi1,G_start1),Sensor1_Acc); // These equations are still in the local coordinates of the physical system
      Sensor3_Acc = qvq(formquat(-1*phi3,G_start3),Sensor3_Acc);
      // Finally save this under our running average
      Sensor1_GravityLEFT = vrunning_avg(Sensor1_GravityLEFT,Sensor1_Acc,LEFTcounter);
      //Sensor2_GravityLEFT = vrunning_avg(Sensor2_GravityLEFT,Sensor2_Acc,LEFTcounter);
      Sensor3_GravityLEFT = vrunning_avg(Sensor3_GravityLEFT,Sensor3_Acc,LEFTcounter);
      // Advance counter
      LEFTcounter++;
      // Reset for next pass
      ready1 = false;
      ready3 = false;
    }
    if (LEFTcounter == LEFTcounterMax) {
      NEEDLEFT = false;
      SERIAL_PORT.println(F("Left-pointing gravity vectors saved! Vectors:"));
      SERIAL_PORT.print(Sensor1_GravityLEFT.x,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(Sensor1_GravityLEFT.y,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(Sensor1_GravityLEFT.z,3);
//      SERIAL_PORT.print(Sensor2_GravityLEFT.x,3);
//      SERIAL_PORT.print(F(","));
//      SERIAL_PORT.print(Sensor2_GravityLEFT.y,3);
//      SERIAL_PORT.print(F(","));
//      SERIAL_PORT.println(Sensor2_GravityLEFT.z,3);
      SERIAL_PORT.print(Sensor3_GravityLEFT.x,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(Sensor3_GravityLEFT.y,3);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(Sensor3_GravityLEFT.z,3);
      SERIAL_PORT.println();
      s1adj = s1adj * (180.0/PI);
      s3adj = s3adj * (180.0/PI);
      SERIAL_PORT.println(F("Compensated angular displacement about Z/up (deg): "));
      SERIAL_PORT.print(F("Sensor 1: "));
      SERIAL_PORT.println(s1adj,1);
      SERIAL_PORT.print(F("Sensor 3: "));
      SERIAL_PORT.println(s3adj,1);
      SERIAL_PORT.println();
      amplitude_update(0); 
    }
    myICM1.clearInterrupts();
    //myICM2.clearInterrupts();
    myICM3.clearInterrupts();
  }

  delaywithFIFOreset(500);
  
  SERIAL_PORT.println(F("Training beginning now."));
  SERIAL_PORT.println();
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
    delay(1);
  }
  SERIAL_PORT.println(F("Press any key when back in the initial position."));
  while (!SERIAL_PORT.available()) {
    delay(1);
  }
  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Please stay still and wait for IP capture."));
  SERIAL_PORT.println(F("... IP recorded during sound, followed by IP-capture beep."));

  delaywithFIFOreset(5000); 

  amplitude_update(0); // Whenever we do an amplitude update we run this function, as it also updates amplitude_harmonic
}

void loop() {
  
  // Always fetch readings as soon as interrupts are noticed
  if (isrFired1) { // If our isr flag is set then clear the interrupts on the ICM
    isrFired1 = false;
    FetchReadingsSensor (myICM1, Sensor1_Acc, Sensor1_Gyro); //, Sensor1_Mag); // Not pulling Mag data
  }
//  if (isrFired2) { // If our isr flag is set then clear the interrupts on the ICM
//    isrFired2 = false;
//    FetchReadingsSensor (myICM2, Sensor2_Acc, Sensor2_Gyro); //, Sensor2_Mag); 
//    Sensor2_Acc = vadd(Sensor2_Acc,Sensor2_GravityBias);
//  }
  if (isrFired3) { // If our isr flag is set then clear the interrupts on the ICM
    isrFired3 = false;
    FetchReadingsSensor (myICM3, Sensor3_Acc, Sensor3_Gyro); //, Sensor3_Mag);
  }

  //  Run Motion Check (always keep tabs on if we're in motion or still) 
  float vmag_gyro = vmagSqrd(MOTIONCHECKSENSOR_Gyro);
  if (vmag_gyro < motionthreshold_gyro) { //make sure to select a sensor whose movement correlates well with the whole system
    if (jostle_buff_count_saved == jostle_buff_count) { // don't wait around if jostling isn't being detected
      rest_buff_count += 6;
    } else {
      rest_buff_count++;
    }
    if (rest_buff_count > rest_noise_filter) { // if not moving 
      IN_MOTION = false; // Signal we're at rest     
      if (WAITING_FOR_REST) MOTION_JUST_ENDED = true; // signal that motion has just ended
    }
    jostle_buff_count_saved = jostle_buff_count;
  } else { // else the gyro readings indicate movement
    if (vmag_gyro > motionthreshold_gyro_high) { // don't wait around if we're definitely moving
      jostle_buff_count += 11; 
    } else {
      jostle_buff_count++;
    }
    if (jostle_buff_count > jostle_noise_filter) { // If we're moving
      IN_MOTION = true; // Signal we're in motion
      if (WAITING_FOR_INITATION) MOTION_JUST_STARTED = true; // signal that we have initiated movement
    }
  }

  // There are certain routines that should run only once, as soon as the unit decides it's moving
  if (MOTION_JUST_STARTED) {
    Realsamplecount = 0; // Reset the two variables keeping track of the number of samples since motion has initiated
    MOTsamplecount = 0;
    rest_buff_count = 0; // Reset the rest buffer
    //if (!TRAINING) IPfound = false; // Signal we're now out of the IP (but not if we're training) // once thought this was needed here, doesn't seem to be; don't turn on unless there's a problem it might fix.
    WAITING_FOR_INITATION = false; // By flipping to false, unit is remembering (for next time through loop) that the motion hasn't just started
    MOTION_JUST_STARTED = false; // This status should turn itself off once it's run
    WAITING_FOR_REST = true;
    PRINTED_RUN_SUMMARY_DATA = false; 
  }

  // There are certain routines that should run only once, as soon as the unit decides its still
  if (MOTION_JUST_ENDED) {
    amplitude_update(0); // End to motion should always turn the sound off
    jostle_buff_count = 0; // Reset the jostle buffer
    IPfound = false; // Signal we're now out of the IP
    WAITING_FOR_REST = false; // By flipping to false, unit is remember (for next time through loop) that the motion hasn't just ended
    MOTION_JUST_ENDED = false; // This status should turn itself off once it's run 
    WAITING_FOR_INITATION = true;
  }

  // If needed and appropriate, test if we're in the IP
  if (!IN_MOTION) {
    if (!IPfound) {
      if (IPSAVED) { // If initial position (IP) has been found and saved, check to see if we're in it
        
        if ( vdst(Sensor3_GravityIP,Sensor3_Acc) < IPtolerance ){ // We are near the IP
          //if ( vdst(Sensor2_GravityIP,Sensor2_Acc) < IPtolerance ) {
            if ( vdst(Sensor1_GravityIP,Sensor1_Acc) < IPtolerance ) {

              // if not training, Grab latest gravity vector to reset motion model IP
              if (!TRAINING) {
                if (recompute_IP_at_rest) {
                  Sensor1_Acc = vadd(Sensor1_Acc,Sensor1_GravityBias); // compensate for the accel bias; gyro bias handled in integratesensordata_advanced
                  //Sensor2_Acc = vadd(Sensor2_Acc,Sensor2_GravityBias);
                  Sensor3_Acc = vadd(Sensor3_Acc,Sensor3_GravityBias);
                  Vectors recal_q1 = alignment_quat(Sensor1_GravityIP,Sensor1_Acc);
                  //Vectors recal_q2 = alignment_quat(Sensor2_GravityIP,Sensor2_Acc); 
                  Vectors recal_q3 = alignment_quat(Sensor3_GravityIP,Sensor3_Acc); 
                  Sensor1_GravityIP = Sensor1_Acc; // Grab the acceleration vector as the gravity vector at IP
                  //Sensor2_GravityIP = Sensor2_Acc;
                  Sensor3_GravityIP = Sensor3_Acc;
                  Sensor1_GravityLEFT = qvq(recal_q1,Sensor1_GravityLEFT);
                  //Sensor2_GravityLEFT = qvq(recal_q2,Sensor2_GravityLEFT);
                  Sensor3_GravityLEFT = qvq(recal_q3,Sensor3_GravityLEFT);
                  compute_motionmodel_IP(limbseg1length,limbseg2length);
                  //SERIAL_PORT.println();
                  //SERIAL_PORT.println(F("In IP! (Recomputed IP variables)"));
                  //SERIAL_PORT.println();
                } else {
                  reset_motionmodelIP();
                  //SERIAL_PORT.println();
                  //SERIAL_PORT.println(F("In IP! (IP variables pulled from memory)"));
                  //SERIAL_PORT.println();
                }
              } else {
                SERIAL_PORT.println();
                SERIAL_PORT.println(F("In IP!"));
                SERIAL_PORT.println();
              }

              IPfound = true;
              rest_buff_count = 0; // Reset the rest buffer so the unit response doesn't bounce around too much
              jostle_buff_count = 0; // Reset the jostle buffer so the unit response doesn't bounce around too much
              playIPbeep(); // Alert user that they're still and in the IP (ready to sonify).
              //UpdateTF(initialpitch); // Turn on sound, signaling to user that we're ready for movement
              //amplitude_update(MaxAmplitude);  
            }
          //}
        }
      }
    }
  }
  
  if (UPDATE_SOUND_NOW) {
    
    if (TRAINING) {

      if (!IPSAVED) { // The first step in training the unit is to grab the IP
        if (!IN_MOTION) { // If we're still, assume the user is in the IP and grab it.
          if (IPcounter == 0) {
            Sensor1_Acc = vadd(Sensor1_Acc,Sensor1_GravityBias); // compensate for the accel bias
            Sensor3_Acc = vadd(Sensor3_Acc,Sensor3_GravityBias);
            Sensor1_GravityIP = Sensor1_Acc; // Grab the acceleration vector as the gravity vector at IP
            //Sensor2_GravityIP = Sensor2_Acc;
            Sensor3_GravityIP = Sensor3_Acc;
            Sensor1_GyroBias = Sensor1_Gyro; // the gyro should be at rest (0,0,0), so whatever readings we get here need to be subtracted as we integrate
            //Sensor2_GyroBias = Sensor2_Gyro;
            Sensor3_GyroBias = Sensor3_Gyro;
          }
          if (IPcounter > 0) {
            UpdateTF(initialpitch);
            amplitude_update(MaxAmplitude);
            Sensor1_Acc = vadd(Sensor1_Acc,Sensor1_GravityBias); // compensate for the accel bias
            Sensor3_Acc = vadd(Sensor3_Acc,Sensor3_GravityBias);
            Sensor1_GravityIP = vrunning_avg(Sensor1_GravityIP,Sensor1_Acc,IPcounter); // Now take the vectors a bunch more and average them
            //Sensor2_GravityIP = vrunning_avg(Sensor2_GravityIP,Sensor2_Acc,IPcounter);
            Sensor3_GravityIP = vrunning_avg(Sensor3_GravityIP,Sensor3_Acc,IPcounter);
            Sensor1_GyroBias = vrunning_avg(Sensor1_GyroBias,Sensor1_Gyro,IPcounter);
            //Sensor2_GyroBias = vrunning_avg(Sensor2_GyroBias,Sensor2_Gyro,IPcounter);
            Sensor3_GyroBias = vrunning_avg(Sensor3_GyroBias,Sensor3_Gyro,IPcounter);
          }
          if (IPcounter == IPcounterMax) {
            amplitude_update(0);
            IPSAVED = true;
            IPtolerance = IPtolerancefraction * vmag(Sensor1_GravityIP);           
            SERIAL_PORT.println();
            SERIAL_PORT.println(F("IP Saved! Gravity Vectors:"));
            SERIAL_PORT.print(Sensor1_GravityIP.x,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.print(Sensor1_GravityIP.y,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.println(Sensor1_GravityIP.z,3);
//            SERIAL_PORT.print(Sensor2_GravityIP.x,3);
//            SERIAL_PORT.print(F(","));
//            SERIAL_PORT.print(Sensor2_GravityIP.y,3);
//            SERIAL_PORT.print(F(","));
//            SERIAL_PORT.println(Sensor2_GravityIP.z,3);
            SERIAL_PORT.print(Sensor3_GravityIP.x,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.print(Sensor3_GravityIP.y,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.println(Sensor3_GravityIP.z,3);
            SERIAL_PORT.println();
            float GMag1 = vmag (Sensor1_GravityIP);
            //float GMag2 = vmag (Sensor2_GravityIP);
            float GMag3 = vmag (Sensor3_GravityIP);
            SERIAL_PORT.println(F("Gravity Vector Magnitudes:"));
            SERIAL_PORT.println(GMag1,5);
            //SERIAL_PORT.println(GMag2,5);
            SERIAL_PORT.println(GMag3,5);
            SERIAL_PORT.println();
            SERIAL_PORT.println(F("Gyro Bias Vectors:"));
            SERIAL_PORT.print(Sensor1_GyroBias.x,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.print(Sensor1_GyroBias.y,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.println(Sensor1_GyroBias.z,3);
//            SERIAL_PORT.print(Sensor2_GyroBias.x,3);
//            SERIAL_PORT.print(F(","));
//            SERIAL_PORT.print(Sensor2_GyroBias.y,3);
//            SERIAL_PORT.print(F(","));
//            SERIAL_PORT.println(Sensor2_GyroBias.z,3);
            SERIAL_PORT.print(Sensor3_GyroBias.x,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.print(Sensor3_GyroBias.y,3);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.println(Sensor3_GyroBias.z,3);  
            Sensor1_GyroBias = vscaler_mult(Sensor1_GyroBias,dt);
            //Sensor2_GyroBias = vscaler_mult(Sensor2_GyroBias,dt);
            Sensor3_GyroBias = vscaler_mult(Sensor3_GyroBias,dt);
            SERIAL_PORT.println();
            SERIAL_PORT.println(F("Gyro Bias / ROT drift Vectors: (How many degrees will the ROT drift per sample at rest?)"));
            SERIAL_PORT.print(Sensor1_GyroBias.x,6);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.print(Sensor1_GyroBias.y,6);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.println(Sensor1_GyroBias.z,6);
//            SERIAL_PORT.print(Sensor2_GyroBias.x,6);
//            SERIAL_PORT.print(F(","));
//            SERIAL_PORT.print(Sensor2_GyroBias.y,6);
//            SERIAL_PORT.print(F(","));
//            SERIAL_PORT.println(Sensor2_GyroBias.z,6);
            SERIAL_PORT.print(Sensor3_GyroBias.x,6);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.print(Sensor3_GyroBias.y,6);
            SERIAL_PORT.print(F(","));
            SERIAL_PORT.println(Sensor3_GyroBias.z,6);  
            compute_motionmodel_IP(limbseg1length,limbseg2length); // set IPs for rot-to-pos motion model
            SERIAL_PORT.println();
            SERIAL_PORT.println(F("Motion model initial position computed."));
            SERIAL_PORT.println();
            delaywithFIFOreset(1000);
            // Now turn on sound, signaling to user that we're ready for movement
            UpdateTF(initialpitch);
          }
          IPcounter++;
        }
      }
      else { // else the IP has been saved, so we should be learning the training motion 
        if (WAITING_TO_INITATE_SAMPLE_MOTION) {
          if (IPfound && IN_MOTION) {
            WAITING_TO_INITATE_SAMPLE_MOTION = false; // this status should turn itself off when tripped
            WAITING_FOR_SAMPLE_MOTION_END = true;
            //IPfound = false; // once thought this was needed here, doesn't seem to be; don't turn on unless there's a problem it might fix.
            SERIAL_PORT.println(); // Now alert the user
            SERIAL_PORT.println(F("Motion detected! Recording."));
            // Grab first model sample
            integratesensordata();
            compute_motionmodel(LimbSeg1SensorRot,LimbSeg2SensorRot);
            // save sample motion
            p2_model[MOTsamplecount] = p2;
            t_model[MOTsamplecount] = t;
            MOTsamplecount++; // note that the first sample gets saved in MOTsamplecount = 0
            UpdateTF(initialpitch + 120); // raise pitch to signal that recording has begun
            time1 = micros();
          }
        }
        else { // else we're collecting a sample motion
          if (IN_MOTION) {
            if (MOTsamplecount < maxsamplestorecord) {
              integratesensordata();
              compute_motionmodel(LimbSeg1SensorRot,LimbSeg2SensorRot);
              // save sample motion
              p2_model[MOTsamplecount] = p2;
              t_model[MOTsamplecount] = t;
              MOTsamplecount++; // note that the first sample gets saved in MOTsamplecount = 0
            }
          }
          else { // else we're come to a stop;

            if (WAITING_FOR_SAMPLE_MOTION_END) {
              time2 = micros();
              WAITING_FOR_SAMPLE_MOTION_END = false; // this status should turn itself off when tripped
              WAITING_TO_INITATE_SAMPLE_MOTION = true; 
              amplitude_update(0); // turn sound off to notify that motion was captured
              if (MOTsamplecount > 250) {
                MOTsamplecount_saved = MOTsamplecount; // save how many samples were taken, if less than before
                SERIAL_PORT.println(F("Motion captured!"));
                SERIAL_PORT.println();
                SERIAL_PORT.print(F("Number of samples saved: "));
                SERIAL_PORT.println(MOTsamplecount_saved,1); // technically it adds one at the end that isn't saved, but since it starts its count at zero, this works out. 
                SERIAL_PORT.println();
                // Find y min and max for t
                ty_min = t_model[0].y;
                for ( int i = 1; i < MOTsamplecount; i++ ) {
                  if (t_model[i].y < ty_min) ty_min = t_model[i].y;
                }
                ty_max = t_model[0].y;
                for ( int i = 1; i < MOTsamplecount; i++ ) {
                  if (t_model[i].y > ty_max) ty_max = t_model[i].y;
                }
                tz_min = t_model[0].z;
                for ( int i = 1; i < MOTsamplecount; i++ ) {
                  if (t_model[i].z < tz_min) tz_min = t_model[i].z;
                }
                tz_max = t_model[0].z;
                for ( int i = 1; i < MOTsamplecount; i++ ) {
                  if (t_model[i].z > tz_max) tz_max = t_model[i].z;
                }
                SERIAL_PORT.print(F("t_y Min: "));
                SERIAL_PORT.print(ty_min,3);
                SERIAL_PORT.print(F(", t_y Max: "));
                SERIAL_PORT.println(ty_max,3);
                SERIAL_PORT.print(F("t_z Min: "));
                SERIAL_PORT.print(tz_min,3);
                SERIAL_PORT.print(F(", t_z Max: "));
                SERIAL_PORT.println(tz_max,3);
                SERIAL_PORT.println();
                //reset_motionmodelIP(); // set IPs for rot-to-pos motion model
                delaywithFIFOreset(500); 
                Playback_model();
                SERIAL_PORT.println(F("Recorded model motion:"));
                SERIAL_PORT.println();
                SERIAL_PORT.println(F("Model_Num, p2x, p2y, p2z, tx, ty, tz"));
                for ( int i = 0; i < MOTsamplecount_saved; i++ ) {
                  SERIAL_PORT.print(i,1);
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(p2_model[i].x,5);
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(p2_model[i].y,5);
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(p2_model[i].z,5);
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(t_model[i].x,5);
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(t_model[i].y,5);
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.println(t_model[i].z,5);
                }
                time3 = time2 - time1;
                float samplingtime = (float)( (float)time3 / (float)1000000 );
                samplerate_live = MOTsamplecount_saved / samplingtime; 
                delaywithFIFOreset(2000);
                TRAINING = false;
                SERIAL_PORT.print(F("Sample Rate: "));
                SERIAL_PORT.print(samplerate_live,1);
                SERIAL_PORT.println(F(" (Hz)"));
                SERIAL_PORT.println();
                SERIAL_PORT.println(F("Training Complete!"));
                SERIAL_PORT.println();
                SERIAL_PORT.println(F("Sonifying:"));
                SERIAL_PORT.println();
                SERIAL_PORT.print(F("Date_(YYYYMMDD), Condition_code, Participant_Num, Motion_Num, Sample_Num, Model_Num, Pitch_(Hz), Amplitude_(/"));
                SERIAL_PORT.print(MaxAmplitude,1);
                SERIAL_PORT.println(F("), p2_error_(mm), t_error_(mm), dist_to_target_(mm), total_distance_travelled_by_t_(mm), p2x, p2y, p2z, tx, ty, tz, TW_code, TW_discontinuity, Sampling/Motion_Time_(s), Samples_Taken, Sample_Rate_(Hz), Final_Distance_of_t_from_Target_(mm), Avg_Error, Max_Allowed_Error"));
                MOTsamplecount = 0;
              }
              else {
                rest_buff_count = 0; // Reset the rest buffer so the unit response doesn't bounce around too much
                jostle_buff_count = 0; // Reset the jostle buffer so the unit response doesn't bounce around too much
                SERIAL_PORT.println();
                SERIAL_PORT.println(F("Bad Sample Run! Try Again"));
                SERIAL_PORT.println(); 
              }
            }
          }
        }
      }
  }
    else { // else (we're not in training, so) we're in live sonification
      if (IN_MOTION && IPfound) { 
        if (MOTsamplecount == 0) {
          time1 = micros();
          SONIFYING = true;
          avg_last_error = 0;
        }
        // Integrate gyro data and compute position in 3D space
        integratesensordata();
        compute_motionmodel(LimbSeg1SensorRot,LimbSeg2SensorRot);
          
          /*
          error1_Pos = vdst(Sensor1_Pos,SavedPos[MOTsamplecount][0]); // start by finding the distance (error) between the current POS vectors and the trained POS vectors
          error2_Pos = vdst(Sensor2_Pos,SavedPos[MOTsamplecount][1]); // Recall arrays start at 0, so the sensor 1 array is 0, sensor 2 is 1, sensor 3 is 2. 
          error3_Pos = vdst(Sensor3_Pos,SavedPos[MOTsamplecount][2]);
          error_total_Pos = (error1_Pos + error2_Pos + error3_Pos) - normal_motion_deviation_Pos[MOTsamplecount];
          if (error_total_Pos < 0) error_total_Pos = 0;
          error_weber_Pos = WeberError_Amplitude(error_total_Pos);
          amp = (int) ( MaxAmplitude - error_weber_Pos ); 
          amplitude_update(amp); 
          */

          /*
          // compute ROT error and new amplitude:
          error1_Rot = vdst(Sensor1_Rot,SavedRot[MOTsamplecount][0]); // start by finding the distance (error) between the current ROT vectors and the trained ROT vectors
          error2_Rot = vdst(Sensor2_Rot,SavedRot[MOTsamplecount][1]); // Recall arrays start at 0, so the sensor 1 array is 0, sensor 2 is 1, sensor 3 is 2. 
          error3_Rot = vdst(Sensor3_Rot,SavedRot[MOTsamplecount][2]);
          error_total_Rot = error1_Rot + error3_Rot; // + error2_Rot 
          error_weber_Rot = WeberError_Amplitude(error_total_Rot);
          */

          // Compute sound update
          amp = (int) ( constrain( son_map( t.y, ty_min, ty_max, 50, MaxAmplitude ), 0, MaxAmplitude ) ); 
          TF = (int) ( constrain( son_map( t.z, tz_min, tz_max, PitchValueMin, PitchValueMax  ), PitchValueMin, PitchValueMax) ); 
          amplitude_update(amp);
          UpdateTF(TF);

        // Save readings for printing later: 
        if (Realsamplecount < MaxReadingsToSave) {
          MOTsamplecount_livesaved[Realsamplecount] = MOTsamplecount;
          p2_saved[Realsamplecount] = p2;
          t_saved[Realsamplecount] = t;
        }

        // Now determine what MOTsample we should be at, based on current POS vectors
        if (MOTsamplecount < TWbegin) { // first MOTsamplecount is a special case; just advance once
          MOTsamplecount++;
        } else if (smoothtail && MOTsamplecount > tail) {
          MOTsamplecount++;
        }
        else {
          // To find our place on the model, find the MOTsamplecount to which we're closest. 
          int prev = MOTsamplecount - 1;
          int next = MOTsamplecount + 1;
          float total_error_next;
          float total_error_prev;
          // sqrt function takes way too long to compute, and we don't actually need to know "real" distance; so, just compare squares
          total_error = vdstSqrd(t,t_model[MOTsamplecount]) + vdstSqrd(p2,p2_model[MOTsamplecount]); // Find (squared) distance from the current MOTsamplecount
          total_error_prev = vdstSqrd(t,t_model[prev]) + vdstSqrd(p2,p2_model[prev]); // Find (squared) distance from the previous MOTsamplecount
          total_error_next = vdstSqrd(t,t_model[next]) + vdstSqrd(p2,p2_model[next]); // Find (squared) distance from the next MOTsamplecount
          if (total_error < total_error_prev) { // if closer to current saved sample than previous saved sample 
            if (total_error < total_error_next) { // and if closer to current saved sample than next saved sample 
              MOTsamplecount = next; // then we're right on track; so advance one sample to hit on the next update
            }
            else { // else we're closer to the next saved sample than the current one, so we need to jump ahead
              // basic idea: Jump ahead step-wise as far as will continue to reduce error
              float min_error = total_error_next;
              next++;
              total_error_next = vdstSqrd(t,t_model[next]) + vdstSqrd(p2,p2_model[next]);
              while (min_error > total_error_next && next < MOTsamplecount_saved) {
                min_error = total_error_next;
                next++;
                total_error_next = vdstSqrd(t,t_model[next]) + vdstSqrd(p2,p2_model[next]);
              }
              MOTsamplecount = next; // and set the MOTsamplecount there.
              // While loop will advance "next" once too far, taking us one beyond where distance is minimized, but we want to add one anyway, so leave "next" as-is (don't use --next).
            }
          }
          else { // else closer to previous saved sample than current saved sample, so may need to jump back
            // basic idea: Jump gack step-wise as far as will continue to reduce error
            float min_error = total_error_prev;
            prev--;
            total_error_prev = vdstSqrd(t,t_model[prev]) + vdstSqrd(p2,p2_model[prev]);
            while (min_error > total_error_prev && prev > 0) {
              min_error = total_error_prev;
              prev--;
              total_error_prev = vdstSqrd(t,t_model[prev]) + vdstSqrd(p2,p2_model[prev]);
            }
            MOTsamplecount = prev; // and set the MOTsamplecount there.
            // While loop will turn "prev" back once too far, taking us one beyond where distance is minimized, but we want to subtract one anyway, so leave "prev" as-is (don't use ++prev).
          }
        }
        if (MOTsamplecount < 1) MOTsamplecount = 1; // don't let it take us back to zero; messes up sample rate counter
        else if (MOTsamplecount == MOTsamplecount_saved) MOTsamplecount--; // don't let us hit MOTsamplecount_saved (returns gibberish for error next update)

        // no matter what, keep track of actual number of updates
        Realsamplecount++;

      }
      else if (!IN_MOTION && SONIFYING) { // else we're in live sonification, but not moving (motion has come to an end)
        //amplitude_update(0); // a bit of a hack, but make sure the sound cuts as soon as we enter this print / hit the end of the recorded motion
        
        if ( Realsamplecount > 499 ) { // don't print jerks

          motion_num += 1;
        
          time2 = micros(); 
          time3 = time2 - time1;
          float samplingtime = (float)( (float)time3 / (float)1000000 );
          samplerate_live = Realsamplecount / samplingtime; 
          
          delaywithFIFOreset(500); 
          Playback_model();
          
          
          for ( int i = 0; i < MaxReadingsToSave; i++ ) {
            if ( i < Realsamplecount ) {
              SERIAL_PORT.print(date,1); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(condition_code,1);
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(subject_num,1);
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(motion_num,1); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(i,1); // Sample_Num
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(MOTsamplecount_livesaved[i],1); // Model num
              SERIAL_PORT.print(F(", "));
              // Recompute pitch and volume from saved position data
              // Now compute distance from model in 3D space   
              amp = (int) ( constrain( son_map( t_saved[i].y, ty_min, ty_max, 50, MaxAmplitude ), 0, MaxAmplitude ) ); 
              TF = (int) ( constrain( son_map( t_saved[i].z, tz_min, tz_max, PitchValueMin, PitchValueMax  ), PitchValueMin, PitchValueMax) );  
              SERIAL_PORT.print(TF,1); // Pitch
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(amp,1); // Volume
              // check for amp discontinuity 
              if ( (abs( amp_saved - amp ) / amp_saved) > 0.1 ) {
                amp_discontinuity = true;
                amp_discontinuity_sample = i-1;
              }
              t_err = sqrt(vdstSqrd( t_saved[i], t_model[ MOTsamplecount_livesaved[i] ] ));
              p2_err = sqrt(vdstSqrd( p2_saved[i], p2_model[ MOTsamplecount_livesaved[i] ] ));
              avg_last_error += sqrt(vdstSqrd( t, t_model[ MOTsamplecount ] ) + vdstSqrd( p2, p2_model[ MOTsamplecount ] ));
              amp_saved = amp;
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(p2_err,5); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(t_err,5);
              SERIAL_PORT.print(F(", "));
              dist_target = vdst( t_saved[i], t_model[ MOTsamplecount_saved - 1 ] );
              SERIAL_PORT.print(dist_target,5);
              SERIAL_PORT.print(F(", "));
              if ( i == 0 ) {
                dist_travelled += vdst( t_saved[i], t_IP );
              } else {
                dist_travelled += vdst( t_saved[i], t_saved[i-1] );
              }
              SERIAL_PORT.print(dist_travelled,5);
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(p2_saved[i].x,5); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(p2_saved[i].y,5); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(p2_saved[i].z,5); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(t_saved[i].x,5); 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(t_saved[i].y,5); 
              SERIAL_PORT.print(F(", "));
              if ( i == Realsamplecount - 1 || i == MaxReadingsToSave - 1 ) {
                SERIAL_PORT.print(t_saved[i].z,5); 
              } else {
                SERIAL_PORT.print(t_saved[i].z,5); 
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.println(F(", "));
              }
              
            }
          }
          
          // Check if time-warping is failing
          /*
            Time warp check codes: 
              0 = Passed; no issues detected. 
              1 = Time-warp algorithm failure detected (trapped low on start), attempting to compensate (first try).
              2 = Second time-warp algorithm failure detected (trapped low on start), shutting off time-warping.
              3 = Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, small or no impact on measured error; Taking no action.
              4 = Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, attempting to compensate (first try).
              5 = Time-warp algorithm failure detected at indicated sample, large impact on measured error in second half of motion; attempting to smooth tail.
              6 = Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, attempting to compensate. 
              7 = Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, could not compensate; shutting off time-warping. 
           */
          if ( (MOTsamplecount_livesaved[150] < 51 || MOTsamplecount_livesaved[500] < 101) && TWFIRST && TWON && Realsamplecount > 500) { // if model number at sample 150 is 50 or less
            TWbegin = 100;
            SERIAL_PORT.print(F(", "));
            SERIAL_PORT.print(1,1); // Time-warp algorithm failure detected (trapped low on start), attempting to compensate (first try).
            SERIAL_PORT.print(F(", "));
            SERIAL_PORT.print(F(", "));
            TWFIRST = false;
          } else if ( (MOTsamplecount_livesaved[150] < 51 || MOTsamplecount_livesaved[500] < 101) && !TWFIRST && TWON && Realsamplecount > 500) {
            TWbegin = MaxReadingsToSave + 1;
            SERIAL_PORT.print(F(", "));
            SERIAL_PORT.print(2,1); // Second time-warp algorithm failure detected (trapped low on start), shutting off time-warping.
            SERIAL_PORT.print(F(", "));
            SERIAL_PORT.print(F(", "));
            TWON = false;
          } else if (TWON && Realsamplecount > 500) {
            bool TWdiscontinuity = false;
            int discon = 0;
            for ( int i = 0; i < MaxReadingsToSave; i++ ) {
              if ( i < (Realsamplecount - 1) ) {
                if (abs(MOTsamplecount_livesaved[i] - MOTsamplecount_livesaved[i+1]) > 50) {
                  TWdiscontinuity = true;
                  discon = i;
                }
              }
            }
            if (TWdiscontinuity && TWFIRST) {
              if ( !amp_discontinuity || (amp_discontinuity && amp_discontinuity_sample != discon) ) {
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(3,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, small or no impact on measured error; Taking no action.
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(discon,1);
                SERIAL_PORT.print(F(", "));
              } else if ( discon < MOTsamplecount_saved * 0.5) {
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(4,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, attempting to compensate (first try).
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(discon,1);
                SERIAL_PORT.print(F(", "));
                TWbegin = 100;
                TWFIRST = false;
              } else {
                smoothtail = true;
                tail = discon - 20;
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(5,1); // Time-warp algorithm failure detected at indicated sample, large impact on measured error in second half of motion; attempting to smooth tail.
                SERIAL_PORT.print(F(", "));
                SERIAL_PORT.print(discon,1);
                SERIAL_PORT.print(F(", "));
              }
            } else if (TWdiscontinuity && !TWFIRST) {
              if (discon == TWbegin && TWbegin < MOTsamplecount_saved) {
                if ( !amp_discontinuity || (amp_discontinuity && amp_discontinuity_sample != discon) ) {
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(3,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, small or no impact on measured error; Taking no action.
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(discon,1);
                  SERIAL_PORT.print(F(", "));
                } else {
                  TWbegin += 100;
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(6,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, attempting to compensate.
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(discon,1);
                  SERIAL_PORT.print(F(", "));
                }
              } else if (discon < MOTsamplecount_saved * 0.5) {
                if ( !amp_discontinuity || (amp_discontinuity && amp_discontinuity_sample != discon) ) {
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(3,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, small or no impact on measured error; Taking no action.
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(discon,1);
                  SERIAL_PORT.print(F(", "));
                } else {
                  TWbegin = MaxReadingsToSave + 1;
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(7,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, could not compensate; shutting off time-warping. 
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(discon,1);
                  SERIAL_PORT.print(F(", "));
                  TWON = false;
                }
              } else {
                if ( !amp_discontinuity || (amp_discontinuity && amp_discontinuity_sample != discon) ) {
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(3,1); // Time-warp algorithm failure detected (large discontinuity after start) at indicated sample, small or no impact on measured error; Taking no action.
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(discon,1);
                  SERIAL_PORT.print(F(", "));
                } else {
                  smoothtail = true;
                  tail = discon - 20;
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(5,1); // Time-warp algorithm failure detected at indicated sample, large impact on measured error in second half of motion; attempting to smooth tail.
                  SERIAL_PORT.print(F(", "));
                  SERIAL_PORT.print(discon,1);
                  SERIAL_PORT.print(F(", "));
                  }
              }
            } else {
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(0,1); // Passed; no issues detected. 
              SERIAL_PORT.print(F(", "));
              SERIAL_PORT.print(F(", ")); 
            }
          } else {
            SERIAL_PORT.print(F(", "));
            SERIAL_PORT.print(0,1); // Passed; no issues detected. 
            SERIAL_PORT.print(F(", "));
            SERIAL_PORT.print(F(", "));
          }
          
          SERIAL_PORT.print(samplingtime,3);
          SERIAL_PORT.print(F(", "));
          SERIAL_PORT.print(Realsamplecount,1);
          SERIAL_PORT.print(F(", "));
          SERIAL_PORT.print(samplerate_live,1);
          SERIAL_PORT.print(F(", "));
          float final_disttarget = vdst( t, t_model[ MOTsamplecount_saved - 1 ] );
          SERIAL_PORT.print(final_disttarget,1);
          SERIAL_PORT.print(F(", "));
          avg_last_error = (avg_last_error / Realsamplecount); 
          SERIAL_PORT.print(avg_last_error,2);
          SERIAL_PORT.print(F(", "));
          SERIAL_PORT.println(F("NA"));

        }
      
        SONIFYING = false;
        amp_saved = MaxAmplitude;
        amp_discontinuity_sample = 0;
        amp_discontinuity = false;
        dist_target = 0;
        dist_travelled = 0; 
        delaywithFIFOreset(1); // It takes awhile to print, so we want the FIFO to reset
      }
    }
    UPDATE_SOUND_NOW = false;
  }

myICM1.clearInterrupts();
//myICM2.clearInterrupts();
myICM3.clearInterrupts();
  
}

void integratesensordata(void) {
  // find the ROT vectors
  Sensor1_Rot = vintegrate_gyro(Sensor1_Gyro,Sensor1_GyroSaved);
  //Sensor2_Rot = vintegrate_gyro(Sensor2_Gyro,Sensor2_GyroSaved); 
  Sensor3_Rot = vintegrate_gyro(Sensor3_Gyro,Sensor3_GyroSaved); 
  
  // subtract away the drift in the ROT vectors from gyro bias
  Sensor1_Rot = vsubtract(Sensor1_Rot,Sensor1_GyroBias); // recall that the GyroBias vectors have already had each component multipled by dt
  //Sensor2_Rot = vsubtract(Sensor2_Rot,Sensor2_GyroBias);
  Sensor3_Rot = vsubtract(Sensor3_Rot,Sensor3_GyroBias);
}

void amplitude_update(int thisamplitude){
  thisamplitude = constrain(thisamplitude,0,MaxAmplitude);
  amplitude = thisamplitude;
  amplitude_harmonic = (int)( amplitude * harmonic_ratio );
}

void playIPbeep(void) {
  int SavedTF = TF;
  int Savedamplitude = amplitude;
  amplitude_update(0);
  delay(200);
  UpdateTF(660);
  amplitude_update(MaxAmplitude); 
  delay(500); 
  amplitude_update(0);
  delay(200);
  amplitude_update(Savedamplitude);
  UpdateTF(SavedTF);
  myICM1.resetFIFO(); // Need to keep FIFO current?
  //myICM2.resetFIFO();
  myICM3.resetFIFO();
}

void delaywithFIFOreset(int milli) {
  delay(milli);
  myICM1.resetFIFO(); // Need to keep FIFO current?
  //myICM2.resetFIFO();
  myICM3.resetFIFO();
}

void icmISR1(void) {
  isrFired1 = true; // Can't use I2C within ISR on 328p, so just set a flag to know that data is available
}
//void icmISR2(void) {
//  isrFired2 = true; // Can't use I2C within ISR on 328p, so just set a flag to know that data is available
//}
void icmISR3(void) {
  isrFired3 = true; // Can't use I2C within ISR on 328p, so just set a flag to know that data is available
}

void UpdateTF (uint16_t newTF) {
  TCTOPL = 12000000L / (2 * newTF) - 1;  // Do the needed math to get new TOP value for TC0; need to double the TF ("2 *") to account for the harmonic frequency.
  TCTOP = (uint16_t)TCTOPL; // Just double check and make sure we're passing the right data type to the register
    // Note that TF (or "newTF") really is the hearable frequency, e.g. if it's 440, it will be a 440 Hz tone.
  // Pitch needs to be updated now: 
  TCC2->PER.reg = TCTOP; // Set-up the PER (period) register, controlling wave form frequency (here, controls frequency of hearable oscilation);
  TCC2->CC[0].reg = (int)( TCTOP * pulsefraction ); // Set-up the CC (counter compare), channel 0 register; this controls duty cycle (wave width) of the hearable frequency. 
  while (TCC2->SYNCBUSY.reg > 0); // Wait for sync
}

void Playback_model (void) {
  int SavedTF = TF;
  int Savedamplitude = amplitude;
  for ( int i = 0 ; i < MOTsamplecount_saved ; i++ ) {
    amp = (int) ( constrain( son_map( t_model[i].y, ty_min, ty_max, 50, MaxAmplitude ), 0, MaxAmplitude)); 
    TF = (int) ( constrain( son_map( t_model[i].z, tz_min, tz_max, PitchValueMin, PitchValueMax  ), PitchValueMin, PitchValueMax) ); // WeberError_Pitch(MOTsamplecount)
    amplitude_update(amp);
    UpdateTF(TF);
    delayMicroseconds(950);
  }
  amplitude_update(Savedamplitude);
  UpdateTF(SavedTF);
  myICM1.resetFIFO(); // Need to keep FIFO current?
  //myICM2.resetFIFO();
  myICM3.resetFIFO();
}

// This handler is what actually uses the second timer (TCC2) to generate the hearable oscilation of the pulse wave generated by the first timer (TCC1).
// The names "TCC2_0_Handler" and "TCC2_1_Handler" are defined in a library; so, defined elsewhere as functions to execute on the above interrupts. 
void TCC2_0_Handler(void) {
  if (TCC2->INTFLAG.bit.OVF == 1) //Test if an OVF-Interrupt has occured
  {
    TCC2->INTFLAG.bit.OVF = 1;  //Clear the Interrupt-Flag
    if (harmonic) {
      TCC1->CC[0].reg = amplitude_harmonic;
      while (TCC1->SYNCBUSY.bit.CC0); 
      harmonic = !harmonic;
    }
    else {
      TCC1->CC[0].reg = amplitude;
      while (TCC1->SYNCBUSY.bit.CC0); 
      harmonic = !harmonic;
    }   
  }
}
void TCC2_1_Handler(void) {
  if (TCC2->INTFLAG.bit.MC0 == 1)  //Test if an MC0-Interrupt has occured
  {
    TCC2->INTFLAG.bit.MC0 = 1;  //Clear the Interrupt-Flag
    TCC1->CC[0].reg = 0;
    while (TCC1->SYNCBUSY.bit.CC0);
  }
}

void TC2_Handler(void) { // This handler uses timer TC2 to trigger a sound update. 
  // If this interrupt is due to the compare register matching the timer count
  if (TC2->COUNT16.INTFLAG.bit.MC0 == 1) { // p. 64
    TC2->COUNT16.INTFLAG.bit.MC0 = 1;
    if ( UPDATE_SOUND_NOW == false ) 
      UPDATE_SOUND_NOW = true;    
    /* uncomment these lines to give sound oscilating wave width; gives sound richer texture (mostly works without clogging processor; maybe a tiny buggy)
    if (pulsefraction >= 0.795) pulsefraction_up = false;
    if (pulsefraction <= 0.3) pulsefraction_up = true;
    if (pulsefraction_up) pulsefraction = pulsefraction + 0.001;
    else pulsefraction = pulsefraction - 0.001;
    TCC2->CC[0].reg = (int)( TCTOP * pulsefraction ); // Set-up the CC (counter compare), channel 0 register; this controls duty cycle (wave width) of the hearable frequency. 
    while (TCC2->SYNCBUSY.reg > 0); // Wait for sync */
  }
}

float son_map ( float pos, float pos_min, float pos_max, float sound_min, float sound_max ) {
  // Want vertex (h,k) at (pos_min - 100,sound_min) and intersection at (pos_maxt,sound_max); use y = a * (x - h)^2 + k
  float h = pos_min - 100;
  float pos_maxt = pos_max + 100; 
  pos_maxt = pos_maxt * pos_maxt;
  float son = ((sound_max - sound_min) / pos_maxt) * ( ( pos - h ) * ( pos - h) ) + sound_min;
  return(son);
}

void FetchReadingsSensor (ICM_20948_SPI &myICM, Vectors &storeAccReadingInThisVector, Vectors &storeGyroReadingInThisVector) { 
  myICM.getAGMT();            // get the A, G, M, and T readings
  // Scaled data (2000dps gyro, 16g accel range; for 4g, use 8192)
  storeAccReadingInThisVector.x = myICM.agmt.acc.axes.x * Acc_multiple; // convert first to g by dividing by 8192, then to m/s^2 by multiplying by 9.80665, i.e. (myICM.agmt.acc.axes.x / 2048.0) * 9.80665
  storeAccReadingInThisVector.y = myICM.agmt.acc.axes.y * Acc_multiple;
  storeAccReadingInThisVector.z = myICM.agmt.acc.axes.z * Acc_multiple;
  storeGyroReadingInThisVector.x = myICM.agmt.gyr.axes.x * Gyro_multiple; // convert to degrees/s by dividing by 16.4, then to radians by multiplying by * (PI/180.0), i.e. (myICM.agmt.gyr.axes.x / 16.4) * (PI/180.0)
  storeGyroReadingInThisVector.y = myICM.agmt.gyr.axes.y * Gyro_multiple;
  storeGyroReadingInThisVector.z = myICM.agmt.gyr.axes.z * Gyro_multiple; 
  /* storeMagReadingInThisVector.x = myICM.agmt.mag.axes.x;
  storeMagReadingInThisVector.y = myICM.agmt.mag.axes.y;
  storeMagReadingInThisVector.z = myICM.agmt.mag.axes.z; */
}

void FetchReadingsSensor_Mag (ICM_20948_SPI &myICM, Vectors &storeAccReadingInThisVector, Vectors &storeGyroReadingInThisVector, Vectors &storeMagReadingInThisVector) {
  myICM.getAGMT();            // get the A, G, M, and T readings
  // Scaled data (2000dps gyro, 16g accel range; for 4g, use 8192)
  storeAccReadingInThisVector.x = (myICM.agmt.acc.axes.x / 2048.0) * 9.80665; // convert first to g by dividing by 8192, then to m/s^2 by multiplying by 9.80665
  storeAccReadingInThisVector.y = (myICM.agmt.acc.axes.y / 2048.0) * 9.80665;
  storeAccReadingInThisVector.z = (myICM.agmt.acc.axes.z / 2048.0) * 9.80665;
  storeGyroReadingInThisVector.x = myICM.agmt.gyr.axes.x / 16.4; // convert to degrees/s by dividing by 16.4
  storeGyroReadingInThisVector.y = myICM.agmt.gyr.axes.y / 16.4;
  storeGyroReadingInThisVector.z = myICM.agmt.gyr.axes.z / 16.4;
  storeMagReadingInThisVector.x = myICM.agmt.mag.axes.x;
  storeMagReadingInThisVector.y = myICM.agmt.mag.axes.y;
  storeMagReadingInThisVector.z = myICM.agmt.mag.axes.z; 
}

void compute_motionmodel_IP (float limb1length, float limb2length) {
  // Must find the coordinates of the sensor axes (a1x,a1y,a1z,a2x,a2y,a2z) in a coordinate system with z pointing up, y to the left, and x straight ahead.
  
  // Begin with each sensor's axes in the local coordinate frame (defined by those axes) of the sensor
  a1x.x = 1.0; a1x.y = 0; a1x.z = 0; // a1x, a1y, a1z are the x, y, and z axes of the sensor on the first limb segment
  a1y.x = 0; a1y.y = 1.0; a1y.z = 0;
  a1z.x = 0; a1z.y = 0; a1z.z = 1.0;
  a2x.x = 1.0; a2x.y = 0; a2x.z = 0; // a2x, a2y, a2z are the x, y, and z axes of the sensor on the second limb segment
  a2y.x = 0; a2y.y = 1.0; a2y.z = 0;
  a2z.x = 0; a2z.y = 0; a2z.z = 1.0;
  //Vectors yaxis = a1y; Vectors xaxis = a1x; Vectors zaxis = a1z; // just saving values for use later

  // For procedure below, we'll be using following idea: If quat q rotates v into w (v,w defined in frame fixed by v), then conj q takes vectors in the v coordinates and spits out their position in the coordinates of a frame fixed by w
  
  // For each sensor, generate a quaternion that rotates the sensor's z-axis into the IP gravity vector (which points up opposite the ground) and align's its y-axis (or negative y-axis, for left-handed people) with the LEFT vector's projection onto the plane perpendicular to the gravity vector.
  // By the above idea, the conjugate of the resulting quaternions will be a coordinate transform that puts the z-axis onto the gravity vector and x axis onto the mag vector's projection into the plane perpendicular to gravity -- call this the "geo-grav" frame
  // So, for example, assuming a2y started in its local coordinates (= 0,1,0), qvq(quat_conj(qct_s2),a2y) gives the physical y-axis in coordinates of the geo-grav frame
  Vectors qct_s1;
  Vectors qct_s2;
  if (!LEFT_HANDED) { // if right-handed
    qct_s1 = generate_coordinatetransform_wPlaneProj( a1z, LimbSeg1Sensor_GravityIP, a1y, LimbSeg1Sensor_LEFT );
    qct_s2 = generate_coordinatetransform_wPlaneProj( a2z, LimbSeg2Sensor_GravityIP, a2y, LimbSeg2Sensor_LEFT ); 
  } else { // else, if left-handed
    qct_s1 = generate_coordinatetransform_wPlaneProj( a1z, LimbSeg1Sensor_GravityIP, vscaler_mult(a1y,-1.0), LimbSeg1Sensor_LEFT ); // when left-handed, LimbSeg1Sensor_LEFT actually points right
    qct_s2 = generate_coordinatetransform_wPlaneProj( a2z, LimbSeg2Sensor_GravityIP, vscaler_mult(a2y,-1.0), LimbSeg2Sensor_LEFT ); // when left-handed, LimbSeg2Sensor_LEFT actually points right
  }
  
  // Next, find the quat that rotates the y-axis of the geo-grav coordinate frame around that frame's z-axis into alignment with the subject's left direction
  // We assume here that the limb segment 2 sensor's y-axis points opposite the limb segment 2's direction (forearm direction); we assume the forearm is pointing left, so the sensor axis is pointing right.
  // //Vectors qo = alignment_quat(yaxis, vscaler_mult( planeproj( qvq(quat_conj(qct_s2),a2y), xaxis, yaxis ), -1.0) ); // This equation is in the coordinates of the geo-grav frame; planeproj might be unnecessary?
  
  // Now want quaternions which will convert the local coordinates of each sensor into a coordinate frame w/ y-axis pointing left, x-axis pointing ahead, and z-axis pointing up
  // //Vectors q2 = quat_mult(quat_conj(qo),quat_conj(qct_s2)); // quat converting the coordinate system of the sensor on limb segment 2 into the desired external coordinate frame
  // //Vectors q1 = quat_mult(quat_conj(qo),quat_conj(qct_s1)); // quat converting the coordinate system of the sensor on limb segment 1 into the desired external coordinate frame

  // Do the actual conversion: 
  a1x = qvq(quat_conj(qct_s1),a1x); // these now are the initial-position coordinates of the sensor axes in an external reference frame w/ y-axis pointing left, x-axis pointing ahead, and z-axis pointing up
  a1y = qvq(quat_conj(qct_s1),a1y); // Notice that these are all "unshifted" (untranslated), meaning, for each sensor, coordinates assume that the "external" frame is centered on the sensor
  a1z = qvq(quat_conj(qct_s1),a1z); // So, we will "merge" the two systems by first setting the coordinates of the limbs and sensors, then shifting the axes with those 
  a2x = qvq(quat_conj(qct_s2),a2x); 
  a2y = qvq(quat_conj(qct_s2),a2y);
  a2z = qvq(quat_conj(qct_s2),a2z);

  // Print results (only if training)
  if (TRAINING) {
  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Computing initial positions ... "));
  SERIAL_PORT.println(F("Assuming " LimbSeg2SensorPrint " is on the " LimbSeg2Name ", which points exactly left parallel to the ground"));
  SERIAL_PORT.println(F("... assuming " LimbSeg1SensorPrint " in on the " LimbSeg1Name "."));
  SERIAL_PORT.println();
  SERIAL_PORT.print(F("Entered " LimbSeg2Name " length: "));
  SERIAL_PORT.println(limb2length,1);
  SERIAL_PORT.print(F("Entered " LimbSeg1Name " length: "));
  SERIAL_PORT.println(limb1length,1);
  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Unshifted Sensor Axes: "));
  SERIAL_PORT.print(F(LimbSeg1Name " " LimbSeg1SensorPrint " x (x,y,z): "));
  SERIAL_PORT.print(a1x.x,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(a1x.y,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(a1x.z,3);
  SERIAL_PORT.print(F(LimbSeg1Name " " LimbSeg1SensorPrint " y (x,y,z): "));
  SERIAL_PORT.print(a1y.x,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(a1y.y,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(a1y.z,3);
  SERIAL_PORT.print(F(LimbSeg1Name " " LimbSeg1SensorPrint " z (x,y,z): "));
  SERIAL_PORT.print(a1z.x,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(a1z.y,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(a1z.z,3);
  SERIAL_PORT.print(F(LimbSeg2Name " " LimbSeg2SensorPrint " x (x,y,z): "));
  SERIAL_PORT.print(a2x.x,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(a2x.y,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(a2x.z,3);
  SERIAL_PORT.print(F(LimbSeg2Name " " LimbSeg2SensorPrint " y (x,y,z): "));
  SERIAL_PORT.print(a2y.x,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(a2y.y,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(a2y.z,3);
  SERIAL_PORT.print(F(LimbSeg2Name " " LimbSeg2SensorPrint " z (x,y,z): "));
  SERIAL_PORT.print(a2z.x,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(a2z.y,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(a2z.z,3);
  SERIAL_PORT.println();
  SERIAL_PORT.print(F("Angles: a1x/a1y, a1y/a1z, a1z/a1x: "));
  float angle1xy = vangle_rad(a1x,a1y) * (180.0/PI);
  float angle1yz = vangle_rad(a1y,a1z) * (180.0/PI);
  float angle1zx = vangle_rad(a1z,a1x) * (180.0/PI);
  SERIAL_PORT.print(angle1xy,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(angle1yz,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(angle1zx,3);
  SERIAL_PORT.print(F("Angles: a2x/a2y, a2y/a2z, a2z/a2x: "));
  float angle2xy = vangle_rad(a2x,a2y) * (180.0/PI);
  float angle2yz = vangle_rad(a2y,a2z) * (180.0/PI);
  float angle2zx = vangle_rad(a2z,a2x) * (180.0/PI);
  SERIAL_PORT.print(angle2xy,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.print(angle2yz,3);
  SERIAL_PORT.print(F(","));
  SERIAL_PORT.println(angle2zx,3);
  }
  
  // Find coordinates in our external system of limb segment 2 (term and origin, p2) and its sensor (s1). place origin of this external sensor at p2. 
  p2 = zerov; // assume the elbow rests at the origin of our external coordinate system
  p2.z = elbowthickness * 0.375; // midpoint of the elbow joint 
  t = zerov;
  if (!LEFT_HANDED) {
    t.y = limb2length; // assume the forearm points along the y-axis (left direction) of our external coordinate system
  } else { // else, if left-handed
    t.y = limb2length * -1.0; // assume the forearm points along the negative y-axis (right direction) of our external coordinate system
  }
  if (t_as_wrist) {
    t.z = wristthickness * 0.5; // the midpoint of the wrist 
  } else if (t_as_marker) {
    t.z = wristthickness + wrist_marker_height; // the optical marker
  } else {
    t.z = wristthickness * 0.5;
  }
  s2 = vscaler_mult(t,s2ratio); // assume the limb2 sensor is 80% of the way down the limb (near the wrist)
  s2.z = wristthickness + sensorheighabovelimbthickness;
  // Assume limb segment 1 points in the opposite direction of its sensor's y-axis
  p1 = vscaler_mult(a1y,-1.0 * limb1length); 
  s1 = vadd(vscaler_mult(p1,s2ratio), vscaler_mult(a1z,(elbowthickness * 0.5) + sensorheighabovelimbthickness)); // assume the limb1 sensor is 50% of the way up the limb, and out in the direction of sensor 1's z-axis by half the elbow thickness plus 10mm
  
  // Finally, use the sensor coordinates to shift the axes out: 
  a1x = vadd(a1x,s1); 
  a1y = vadd(a1y,s1); 
  a1z = vadd(a1z,s1); 
  a2x = vadd(a2x,s2); 
  a2y = vadd(a2y,s2); 
  a2z = vadd(a2z,s2); 
  
  // Save with fast reset later: 
  p1_IP = p1; p2_IP = p2;
  t_IP = t;
  a1x_IP = a1x; a1y_IP = a1y; a1z_IP = a1z;
  s1_IP = s1;
  a2x_IP = a2x; a2y_IP = a2y; a2z_IP = a2z;
  s2_IP = s2;
}

void reset_motionmodelIP() {
  p1 = p1_IP; p2 = p2_IP;
  t = t_IP;
  a1x = a1x_IP; a1y = a1y_IP; a1z = a1z_IP;
  s1 = s1_IP;
  a2x = a2x_IP; a2y = a2y_IP; a2z = a2z_IP;
  s2 = s2_IP;
}

void compute_motionmodel (Vectors rotd1, Vectors rotd2) {

  // Rotate points on limb segment 1 as if pivoting about s1
  Vectors q1 = rot_quat(rotd1,a1x,a1y,a1z,s1);
  Vectors p1_new = quatrot(q1,p1,s1); // p1 (shoulder joint) not actually rotating, but need to compute this to find translation for system
  Vectors p2_new = quatrot(q1,p2,s1);
  Vectors s1_new = s1; // quatrot(q1,s1,s1); // unecessary, as s1 is translated into origin
  Vectors a1x_new = quatrot(q1,a1x,s1);
  Vectors a1y_new = quatrot(q1,a1y,s1);
  Vectors a1z_new = quatrot(q1,a1z,s1);

  // This would have moved p1, which is actually stationary (shoulder joint in reach), so, translate the L1 system back to p1
  Vectors trans1 = vsubtract(p1,p1_new);
  p2_new = vadd(p2_new, trans1); // because p1 is stationary, we want to add the translation, i.e. p1_new + trans1 = p1
  s1_new = vadd(s1_new, trans1);
  a1x_new = vadd(a1x_new, trans1);
  a1y_new = vadd(a1y_new, trans1);
  a1z_new = vadd(a1z_new, trans1);

  // Rotate points on limb segment 2 as if pivoting about s2
  Vectors q2 = rot_quat(rotd2,a2x,a2y,a2z,s2);
  Vectors p2o_new = quatrot(q2,p2,s2); // The origin of limb segment 2; the first time (when computing p2_new) we treated p2 as the termination of LS1, now treating it as origin of LS2
  Vectors t_new = quatrot(q2,t,s2);
  Vectors s2_new = s2; // quatrot(q2,s2,s2); // unecessary, as s2 is translated into origin
  Vectors a2x_new = quatrot(q2,a2x,s2);
  Vectors a2y_new = quatrot(q2,a2y,s2);
  Vectors a2z_new = quatrot(q2,a2z,s2);

  // Must reattach the origin of limb segment 2 to the termination of limb segment 1
  Vectors trans2 = vsubtract(p2_new, p2o_new);
  t_new = vadd(t_new, trans2);
  s2_new = vadd(s2_new, trans2);
  a2x_new = vadd(a2x_new, trans2);
  a2y_new = vadd(a2y_new, trans2);
  a2z_new = vadd(a2z_new, trans2);
  
  // Finally, update the positions
  p2 = p2_new; // Remember, p1 should *nod* be updated to p1 new!
  t = t_new;
  a1x = a1x_new; a1y = a1y_new; a1z = a1z_new;
  s1 = s1_new;
  a2x = a2x_new; a2y = a2y_new; a2z = a2z_new;
  s2 = s2_new;
  
}

Vectors rot_quat ( Vectors r, Vectors a1, Vectors a2, Vectors a3, Vectors u ) {
  Vectors a1shift = vsubtract(a1,u); // It might be prodent to normalize these three shifted vectors, but given their construction when building the IP, they *should* be normalized already
  Vectors a2shift = vsubtract(a2,u); 
  Vectors a3shift = vsubtract(a3,u);
  Vectors qx = formquat(r.x,a1shift);
  Vectors qy = formquat(r.y,a2shift);
  Vectors qz = formquat(r.z,a3shift);
  Vectors q = quat_mult( qx, quat_mult( qy, qz) );
  q = normalize4(q); // we must ensure this is normalized, although it should be as well (if increased speed needed, drop this step).
  return(q);
}

Vectors quatrot ( Vectors q, Vectors v, Vectors p) {
  Vectors vshift = vsubtract(v,p);
  Vectors rotatedposition = qvq(q,vshift);
  rotatedposition = vadd(rotatedposition,p);
  return(rotatedposition);
}

Vectors quatrot_full ( Vectors r, Vectors a1, Vectors a2, Vectors a3, Vectors v, Vectors u, Vectors p) {
  Vectors a1shift = vsubtract(a1,u); 
  Vectors a2shift = vsubtract(a2,u); 
  Vectors a3shift = vsubtract(a3,u);
  a1shift = normalize(a1shift);
  a2shift = normalize(a2shift);
  a3shift = normalize(a3shift);
  Vectors qx = formquat(r.x,a1shift);
  Vectors qy = formquat(r.y,a2shift);
  Vectors qz = formquat(r.z,a3shift);
  qx = normalize4(qx);
  qy = normalize4(qy);
  qz = normalize4(qz);
  Vectors vshift = vsubtract(v,p);
  Vectors rotatedposition = qvq(qx,qvq(qy,qvq(qz,vshift)));
  rotatedposition = vadd(rotatedposition,p);
  return(rotatedposition);
}

//Vectors rotdelta_rad ( Vectors f, Vectors s) {
//  // return being feed into quatrot, which assumes input angles (r) are in radians
//  Vectors rotd;
//  rotd.x = (s.x - f.x) * (PI/180.0);
//  rotd.y = (s.y - f.y) * (PI/180.0);
//  rotd.z = (s.z - f.z) * (PI/180.0);
//  return(rotd); // returns radians
//}

bool vsame ( Vectors v, Vectors w ) {
  if ( v.x == w.x && ( v.y == w.y && v.z == w.z ) ) {
    return(true);
  } else {
    return(false);
  }
}

float vangle_rad ( Vectors v, Vectors w ) {
  float phi;
  if ( vsame(v,w) ) {
    phi = 0;
  } else if ( vsame(v,vscaler_mult(w,-1.0)) ) {
    phi = PI;
  } else {
    Vectors vn = normalize(v);
    Vectors wn = normalize(w);
    float x = dotp(vn,wn); // point of normalizing is so that these magnitudes are 1;  / ( vmag(vn) * vmag(wn) );
    if ( x > 1 ) x = 1;
    if ( x < -1 ) x = -1;
    phi = acosf(x);
  }
  return(phi); // returns radians
}

Vectors planeproj ( Vectors u, Vectors n1, Vectors n2 ) { 
  // Assumes n1 and n2 are orthogonal ???
  Vectors projection;
  if ( vsame(n1,n2) ) { // if n1 and n2 are the same (and so don't define a plane), then take vector projection
    float phi = vangle_rad(u,n1);
    float scalarproj = vmag(u) * cosf(phi); 
    projection = vscaler_mult( normalize(n1) , scalarproj );
  } else { // find the projection of vector u onto the plane formed by othogonal vectors n1 and n2
    Vectors n = crossp(n1,n2);
    Vectors normproj = vscaler_mult( n, dotp(u,n) / ( vmag(n) * vmag(n) ) ); 
    projection = vsubtract(u,normproj); 
  }
  return(projection); 
}

Vectors quat_mult ( Vectors q1, Vectors q2 ) {
  Vectors q = vadd( vscaler_mult(q2,q1.r) , vadd( vscaler_mult(q1,q2.r) , crossp(q1,q2) ) );
  q.r = q1.r * q2.r - dotp(q1,q2); 
  return(q);
}

Vectors generate_coordinatetransform_wPlaneProj( Vectors v1, Vectors v1rotated, Vectors v2, Vectors v2rotated ) {
  // when called to compute axes IP, v1rotated and v2rotated will be the IP gravity and mag vectors, which should be normalized
  Vectors v1rotated_norm = normalize(v1rotated);
  Vectors v2rotated_norm = normalize(v2rotated);
  Vectors q_ct1 = alignment_quat(v1,v1rotated_norm); 
  Vectors v2_new = qvq(q_ct1,v2);
  Vectors v1_new = qvq(q_ct1,v1);
  Vectors v2rotated_proj = planeproj ( v2rotated_norm , v2_new , crossp(v1_new,v2_new) ); // in the case for which function is written (compute axes IP), v1 = Zaxis, v2 = Xaxis, and so cross(v1,v2) = Yaxis, meaning we're projecting v2rotated (mag vector) onto x-y plane
  Vectors q_ct2 = alignment_quat(v2_new,v2rotated_proj);
  Vectors q_ct = quat_mult(q_ct2,q_ct1);
  return(q_ct); // this quat rotates v1 to v1rotated and rotates v2 to v2rotated
}

Vectors alignment_quat ( Vectors v, Vectors w ) {
  float phi = vangle_rad(v,w);
  float sphi = sinf(phi/2.0);
  Vectors crossvw = crossp(v,w);
  float crossvw_m = vmag(crossvw);
  if ( crossvw_m != 0 ) crossvw = vdivide(crossvw,crossvw_m);
  Vectors q;
  q.r = cosf(phi/2.0);
  q.x = sphi * crossvw.x;
  q.y = sphi * crossvw.y;
  q.z = sphi * crossvw.z;
  q = normalize4(q);
  return(q); // this quat will rotate v into w
}

Vectors quat_conj ( Vectors q ) {
  Vectors qc;
  qc.r = q.r;
  qc.x = -1 * q.x;
  qc.y = -1 * q.y;
  qc.z = -1 * q.z;
  return(qc); 
}

Vectors formquat ( float theta, Vectors v ) {
  Vectors q;
  float cos2theta = cosf(theta/2.0);
  float sin2theta = sinf(theta/2.0);
  q.r = cos2theta;
  q.x = sin2theta * v.x;
  q.y = sin2theta * v.y;
  q.z = sin2theta * v.z;
  return (q); 
}

Vectors normalize ( Vectors v ) {
  Vectors w;
  w = vdivide(v,vmag(v));
  return(w); 
}

Vectors normalize4 ( Vectors v ) {
  Vectors w;
  w = vdivide4(v,vmag4(v));
  return(w); 
}

float vmag ( Vectors thisVector ) {
  float MagV;
  MagV = sqrtf ( thisVector.x * thisVector.x + thisVector.y * thisVector.y + thisVector.z * thisVector.z );
  return MagV;
}

float vmag4 ( Vectors thisVector ) {
  float MagV;
  MagV = sqrtf ( thisVector.r * thisVector.r + thisVector.x * thisVector.x + thisVector.y * thisVector.y + thisVector.z * thisVector.z );
  return MagV;
}

float vmagSqrd ( Vectors thisVector ) {
  float MagV;
  MagV = thisVector.x * thisVector.x + thisVector.y * thisVector.y + thisVector.z * thisVector.z ;
  return MagV;
}

Vectors vdivide ( Vectors v, float r ) {
  Vectors w;
  w.x = v.x /r;
  w.y = v.y /r;
  w.z = v.z /r;
  return(w);
}

Vectors vdivide4 ( Vectors v, float r ) {
  Vectors w;
  w.r = v.r /r;
  w.x = v.x /r;
  w.y = v.y /r;
  w.z = v.z /r;
  return(w);
}

Vectors vsubtract ( Vectors v1, Vectors v2 ) {
  Vectors output;
  output.x = v1.x - v2.x;
  output.y = v1.y - v2.y;
  output.z = v1.z - v2.z;
  return output;
}

Vectors vadd ( Vectors v1, Vectors v2 ) {
  Vectors output;
  output.x = v1.x + v2.x;
  output.y = v1.y + v2.y;
  output.z = v1.z + v2.z;
  return output;
}

Vectors qvq ( Vectors thisQuat, Vectors thisPosition ) {
  // Fewer computations than the other definition
  // Also, note that we're assuming our quaternions are unit quaternions (they should be, or should be very close). 

  thisPosition.r = 0;
  float qv_r = thisQuat.r * thisPosition.r - thisQuat.x * thisPosition.x - thisQuat.y * thisPosition.y - thisQuat.z * thisPosition.z ;
  float qv_x = thisQuat.r * thisPosition.x + thisQuat.x * thisPosition.r + thisQuat.y * thisPosition.z - thisQuat.z * thisPosition.y ; 
  float qv_y = thisQuat.r * thisPosition.y - thisQuat.x * thisPosition.z + thisQuat.y * thisPosition.r + thisQuat.z * thisPosition.x ;
  float qv_z = thisQuat.r * thisPosition.z + thisQuat.x * thisPosition.y - thisQuat.y * thisPosition.x + thisQuat.z * thisPosition.r ; 

  // This is the same operation as above, expect the sign is flipped everywhere we have a non-real quat component (since the second q in qvq is the inverse of q). 
  //float qvq_r = qv_r * thisQuat.r + qv_x * thisQuat.x + qv_y * thisQuat.y + qv_z * thisQuat.z ; // no point in computing r value of the output
  float qvq_x = qv_r * thisQuat.x * -1.0 + qv_x * thisQuat.r - qv_y * thisQuat.z + qv_z * thisQuat.y ;
  float qvq_y = qv_r * thisQuat.y * -1.0 + qv_x * thisQuat.z + qv_y * thisQuat.r - qv_z * thisQuat.x ;
  float qvq_z = qv_r * thisQuat.z * -1.0 - qv_x * thisQuat.y + qv_y * thisQuat.x + qv_z * thisQuat.r ;

  Vectors thisFinalPosition;
  thisFinalPosition.r = 0; // qvq_r ; // physically meaningful vectors have r = 0
  thisFinalPosition.x = qvq_x ;
  thisFinalPosition.y = qvq_y ;
  thisFinalPosition.z = qvq_z ;

  return(thisFinalPosition);
}

Vectors vintegrate_gyro ( Vectors thisValue, Vectors &thisSavedValue ) {
  Vectors thisResult;
  thisResult = vscaler_mult(vdivide(vadd(thisValue,thisSavedValue),2.0),dt); // find average rate of rotation during dt and multiply that rate by dt and add to the old sum
  thisSavedValue = thisValue; // save the current reading for averaging the next time this function is called
  return(thisResult);
}

Vectors vrunning_avg ( Vectors thisVector1, Vectors thisVector2, int thisNum ) {
  Vectors ravg;
  ravg.x = (thisVector1.x * thisNum + thisVector2.x) / (1 + thisNum);
  ravg.y = (thisVector1.y * thisNum + thisVector2.y) / (1 + thisNum);
  ravg.z = (thisVector1.z * thisNum + thisVector2.z) / (1 + thisNum);
  return(ravg);
}

Vectors vscaler_mult ( Vectors thisVector1, float thisfloat ) {
  Vectors output;
  output.x = thisVector1.x * thisfloat;
  output.y = thisVector1.y * thisfloat;
  output.z = thisVector1.z * thisfloat;
  return(output);
}

float vdstSqrd ( Vectors thisVector1, Vectors thisVector2 ) {
  float x = thisVector1.x - thisVector2.x;
  float y = thisVector1.y - thisVector2.y;
  float z = thisVector1.z - thisVector2.z;
  float distance = x*x + y*y + z*z ;
  return distance;
}

float vdst ( Vectors thisVector1, Vectors thisVector2 ) {
  float x = thisVector1.x - thisVector2.x;
  float y = thisVector1.y - thisVector2.y;
  float z = thisVector1.z - thisVector2.z;
  float distance = sqrtf ( x*x + y*y + z*z );
  return distance;
}

float dotp ( Vectors thisVector1, Vectors thisVector2 ) {
  float dotproduct;
  dotproduct = thisVector1.x * thisVector2.x + thisVector1.y * thisVector2.y + thisVector1.z * thisVector2.z ;
  return dotproduct;
}

Vectors crossp ( Vectors thisVector1, Vectors thisVector2 ) {
  Vectors crossproduct;
  crossproduct.x = thisVector1.y * thisVector2.z - thisVector1.z * thisVector2.y ;
  crossproduct.y = thisVector1.z * thisVector2.x - thisVector1.x * thisVector2.z ;
  crossproduct.z = thisVector1.x * thisVector2.y - thisVector1.y * thisVector2.x ;
  return(crossproduct);
}

/* 

Some quaternion functions. 
Not currently used, but can use these if working with quaternions. 

void quat_multiplication ( Vectors &thisQuat1, Vectors &thisQuat2, Vectors &thisQuatProduct ) {
  thisQuatProduct.r = ( thisQuat1.r * thisQuat2.r ) - dotp ( thisQuat1, thisQuat2 ); 
  thisQuatProduct.x = VectorCrossProduct_x ( thisQuat1, thisQuat2 ) + thisQuat1.r * thisQuat2.x + thisQuat2.r * thisQuat1.x;
  thisQuatProduct.y = VectorCrossProduct_y ( thisQuat1, thisQuat2 ) + thisQuat1.r * thisQuat2.y + thisQuat2.r * thisQuat1.y;
  thisQuatProduct.z = VectorCrossProduct_z ( thisQuat1, thisQuat2 ) + thisQuat1.r * thisQuat2.z + thisQuat2.r * thisQuat1.z;
}

void quat_conjugate ( Vectors &thisQuat, Vectors &thisQuatConjugate ) {
  thisQuatConjugate.r = thisQuat.r; 
  thisQuatConjugate.x = -1 * thisQuat.x;
  thisQuatConjugate.y = -1 * thisQuat.y;
  thisQuatConjugate.z = -1 * thisQuat.z;
}

void quat_rotation_slow ( Vectors &thisQuat, Vectors &thisPosition, Vectors &thisFinalPosition ) {
  // The multiplicative inverse of a quaternion q is q^-1 = q'/(q*q'), for q' the conjugate of q. 
  // But, we can assume our quaternions are unit quaternions (they should be, or very close), in which case q^-1 = q'.
  // So, quat_rotation is qvq^-1, but we'll treat it as qvq'. 
  quat_multiplication ( thisQuat, thisPosition, VectorForStoringQuatMultiplication1 );
  quat_conjugate ( thisQuat, VectorForStoringQuatConjugates1 ); 
  quat_multiplication ( VectorForStoringQuatMultiplication1, VectorForStoringQuatConjugates1, thisFinalPosition ); 
}
*/

/*
// I have confirmed that quat_Rotation_slow and quat_rotation outputs are the same; 
// Also have confirmed that the answer is correct: multiplying a quat by its conjugate yields the multiplicative identity element <1,0,0,0>
void quat_rotation ( Vectors &thisQuat, Vectors &thisPosition, Vectors &thisFinalPosition ) {
  // Fewer computations than the other definition
  // Also, note that we're assuming our quaternions are unit quaternions (they should be, or should be very close). 

  float qv_r = thisQuat.r * thisPosition.r - thisQuat.x * thisPosition.x - thisQuat.y * thisPosition.y - thisQuat.z * thisPosition.z ;
  float qv_x = thisQuat.r * thisPosition.x + thisQuat.x * thisPosition.r + thisQuat.y * thisPosition.z - thisQuat.z * thisPosition.y ; 
  float qv_y = thisQuat.r * thisPosition.y - thisQuat.x * thisPosition.z + thisQuat.y * thisPosition.r + thisQuat.z * thisPosition.x ;
  float qv_z = thisQuat.r * thisPosition.z + thisQuat.x * thisPosition.y - thisQuat.y * thisPosition.x + thisQuat.z * thisPosition.r ; 

  // This is the same operation as above, expect the sign is flipped everywhere we have a non-real quat component (since the second q in qvq is the inverse of q). 
  float qvq_r = qv_r * thisQuat.r + qv_x * thisQuat.x + qv_y * thisQuat.y + qv_z * thisQuat.z ; 
  float qvq_x = qv_r * thisQuat.x * -1.0 + qv_x * thisQuat.r - qv_y * thisQuat.z + qv_z * thisQuat.y ;
  float qvq_y = qv_r * thisQuat.y * -1.0 + qv_x * thisQuat.z + qv_y * thisQuat.r - qv_z * thisQuat.x ;
  float qvq_z = qv_r * thisQuat.z * -1.0 - qv_x * thisQuat.y + qv_y * thisQuat.x + qv_z * thisQuat.r ;

  thisFinalPosition.r = qvq_r ;
  thisFinalPosition.x = qvq_x ;
  thisFinalPosition.y = qvq_y ;
  thisFinalPosition.z = qvq_z ;

}

void set_gyro_quat ( Vectors &thisGyro, Vectors & thisGyroQuat ) {
  float dt = 1.0 / SAMPLERATE; 
  // Change Gyro readings to radians: 
  float j = PI / 180.0;
  thisGyro.x = thisGyro.x * j; 
  thisGyro.y = thisGyro.y * j; 
  thisGyro.z = thisGyro.z * j; 
  thisGyroQuat.r = 1.0; 
  thisGyroQuat.x = dt * thisGyro.x; // often this formula would have * 0.5, but I've accounted for this already in the gyro integrating
  thisGyroQuat.y = dt * thisGyro.y;
  thisGyroQuat.z = dt * thisGyro.z;
}

void quat_normalize ( Vectors &thisQuat ) {
  float MagQ;
  MagQ = sqrt ( thisQuat.r * thisQuat.r + thisQuat.x * thisQuat.x + thisQuat.y * thisQuat.y + thisQuat.z * thisQuat.z );
  thisQuat.r = thisQuat.r / MagQ;
  thisQuat.x = thisQuat.x / MagQ;
  thisQuat.y = thisQuat.y / MagQ;
  thisQuat.z = thisQuat.z / MagQ;
}
*/

/*
void GenerateQuatToAlignVectors ( Vectors &thisVector1, Vectors &thisVector2, Vectors &thisQuat, float thisRotationScaler ) {
  float RotationAngleToAlign = thisRotationScaler * AngleBetweenVectors_Rad ( thisVector1, thisVector2 );
  float sinRATA = sin ( RotationAngleToAlign / 2 );
  thisQuat.r = cos ( RotationAngleToAlign / 2 ); 
  thisQuat.x = sinRATA * VectorCrossProduct_x ( thisVector1, thisVector2 );
  thisQuat.y = sinRATA * VectorCrossProduct_y ( thisVector1, thisVector2 );
  thisQuat.z = sinRATA * VectorCrossProduct_z ( thisVector1, thisVector2 );
}

void GenerateQuatToRotateAroundVector ( Vectors &thisRotationAxis, float thisRadian, Vectors &thisQuat ) {
  float sinRA = sin ( thisRadian / 2 );
  thisQuat.r = cos ( thisRadian / 2 ); 
  thisQuat.x = sinRA * thisRotationAxis.x;
  thisQuat.y = sinRA * thisRotationAxis.y;
  thisQuat.z = sinRA * thisRotationAxis.z;
}
*/

/* 

Not currently using (trig is too slow), but here in case we need later.

float AngleBetweenVectors ( Vectors &thisVector1, Vectors &thisVector2 ) {
  float PHI = AngleBetweenVectors_Rad (thisVector1, thisVector2);
  PHI = PHI * ( 180.0 / PI ) ; // PI = 3.14159
  return PHI;
}

float AngleBetweenVectors_Rad ( Vectors &thisVector1, Vectors &thisVector2 ) {
  float Mag1 = vmag ( thisVector1 );
  float Mag2 = vmag ( thisVector2 ); 
  float dotproduct = dotp ( thisVector1, thisVector2 );
  float COSPHI = dotproduct / ( Mag1 * Mag2 ) ; 
  float PHI = acos( COSPHI ) ; 
  if ( !( PHI > 0.0 || PHI <= 0.0 ) ) PHI = 0.0;
  return PHI;
}

void AddVectors ( Vectors &thisVector1, Vectors &thisVector2, Vectors &thisTotalVector ) {
  thisTotalVector.x = thisVector1.x + thisVector2.x;
  thisTotalVector.y = thisVector1.y + thisVector2.y;
  thisTotalVector.z = thisVector1.z + thisVector2.z;
}

*/
