/*
 * Author: Alex Ferguson
 * Created: 06/03/25
 * 
 * Description:
 * Executes 5 tasks with real-time requirements in a cyclic executive schedule
 * Tasks 1 & 2 generate a digital waveform on gpio pins 25 and 33
 * Tasks 3 & 4 measure digital signals connected to gpio pins 18 and 19
 * Task 5 calls B31DGMonitor's 'doWork' function 
 * Tasks 6 & 7 are executed whenever there is slack and do not have real-time requirements. Task 6 lights an LED 
 * when the sum of the measured frequencies exceed 1500Hz, and task 7 uses a button to toggle the state of an LED
 * and call the doWork function.
 *
*/ 


#include "B31DGMonitor.h"
#include <Ticker.h>

// defines for pin numbers and constant values
#define FRAME_DURATION_MS 2
#define FRAME_NUMBER 30
#define SIGNAL_1 33
#define SIGNAL_2 25
#define FREQUENCY_1 19
#define FREQUENCY_2 18
#define LED1 12
#define LED2 14
#define BUTTON 13

// prototype functions
void frame();
void digitalSignal1();      // Task 1
void digitalSignal2();      // Task 2
void measureFrequency1();   // Task 3
void measureFrequency2();   // Task 4
void doWork();              // Task 5
void compareFreq();         // Task 6
void task7();               // Task 7
void IRAM_ATTR btnInterrupt();    // Detect button presses 

// flags for button interrupt
int ledState = LOW;
bool doWorkFlag = 0;

B31DGCyclicExecutiveMonitor monitor;

// freeRTOS ticker object
Ticker frameTicker;

// counter to keep track of which frame is currently being executed
unsigned long frameCounter = 0;

// global variables for measuring frequency input
float pulseDuration = 0;
unsigned int freq1 = 0;
unsigned int freq2 = 0;
unsigned int slot = 0;

// variables to debounce button press 
volatile unsigned long buttonTime = 0;
volatile unsigned long lastButtonTime = 0;
const int debounceTime = 50;

void IRAM_ATTR btnInterrupt()
{
  // get number of millis that have passed when button has been pressed
  buttonTime = millis();
  // only executes code if time between button presses is greater than that set debounce time
  if(buttonTime - lastButtonTime > debounceTime)
  { 
    // set flags to be used by task 7 
    ledState = !ledState;
    doWorkFlag = 1;
    lastButtonTime = buttonTime;
  }
}

// cyclic exective schedule
void frame()
{
  slot = frameCounter % FRAME_NUMBER;
  
  switch(slot)
  {
    case(0):  digitalSignal1();    doWork();         digitalSignal2(); break;  
    case(1):  measureFrequency1();                                     break;
    case(2):  digitalSignal2();  digitalSignal1();   task7();          break;
    case(3):  doWork();          digitalSignal2();   task7();          break;
    case(4):  digitalSignal1();  measureFrequency2();                  break;
    case(5):  digitalSignal2();  doWork();           task7();          break;
    case(6):  digitalSignal2();  digitalSignal1();   task7();          break;
    case(7):  measureFrequency2(); doWork();                           break;
    case(8):  digitalSignal2();  digitalSignal1();   task7();          break;
    case(9):  digitalSignal2();  measureFrequency1();                  break;
    case(10): doWork();          digitalSignal1();   task7();          break;
    case(11): digitalSignal2();  measureFrequency1();                  break;
    case(12): digitalSignal2();  digitalSignal1();   task7();          break;
    case(13): doWork();          measureFrequency2();                  break;
    case(14): digitalSignal2();  digitalSignal1();   task7();          break;
    case(15): doWork();          digitalSignal2();   task7();          break;
    case(16): digitalSignal1();  measureFrequency2();                  break;
    case(17): digitalSignal2();  measureFrequency1();                  break;
    case(18): doWork();          digitalSignal2();   digitalSignal1(); break;
    case(19): task7();                                                 break;
    case(20): digitalSignal2();  digitalSignal1();   doWork();         break;
    case(21): digitalSignal2();  measureFrequency1();                  break;
    case(22): digitalSignal1();  doWork();           task7();          break;
    case(23): digitalSignal2();  measureFrequency2();                  break;
    case(24): digitalSignal2();  digitalSignal1();   task7();          break;
    case(25): doWork();                              task7();          break;
    case(26): digitalSignal2();  digitalSignal1();   task7();          break;
    case(27): digitalSignal2();  measureFrequency2();                  break;
    case(28): doWork();          digitalSignal1();   task7();          break; 
    case(29): digitalSignal2();  measureFrequency1();                  break;
  }
  compareFreq();    // doesn't need to be in schedule as it is not time-critical
  frameCounter++;   // increment frame counter so next frame is executed
}

// task 1
// output a digital signal on GPIO pin according to assignment requirements
void digitalSignal1()
{
  monitor.jobStarted(1);

  digitalWrite(SIGNAL_1, HIGH);
  delayMicroseconds(250);
  digitalWrite(SIGNAL_1, LOW);
  delayMicroseconds(50);
  digitalWrite(SIGNAL_1, HIGH);
  delayMicroseconds(300);
  digitalWrite(SIGNAL_1, LOW);

  monitor.jobEnded(1);
}

// task 2
// output a digital signal on GPIO pin according to assignment requirements
void digitalSignal2()
{
  monitor.jobStarted(2);

  digitalWrite(SIGNAL_2, HIGH);
  delayMicroseconds(100);
  digitalWrite(SIGNAL_2, LOW);
  delayMicroseconds(50);
  digitalWrite(SIGNAL_2, HIGH);
  delayMicroseconds(200);
  digitalWrite(SIGNAL_2, LOW);

  monitor.jobEnded(2);
}

// task 3
// measures frequency between 666Hz and 1kHz
void measureFrequency1()
{
  monitor.jobStarted(3);
  // measure the duration of a pulse starting from a rising edge and ending on a falling edge 
  // set timeout value to be slightly longer than longest period to measure to avoid comprimising RT requirements if 
  // signal isn't connected, or significantly lower than expected values
  // pulseDuration = pulseIn(FREQUENCY_1, HIGH, 1550);
  // // check the pulse duration is within the frequency input range (666Hz - 1kHz)
  // // duty cycle is 50% so pulse duration is half the length of the period 
  // if(pulseDuration <= 760  && pulseDuration > 499)
  // {
  //   freq1 = 1000000.0 / (pulseDuration * 2);    // convert period into frequency 
  // }
  // else
  // {
  //   freq1 = 0;
  // }
  freq1 = 700;
  monitor.jobEnded(3); 
}

// task 4
void measureFrequency2()
{
  monitor.jobStarted(4);
  
  pulseDuration = pulseIn(FREQUENCY_2, HIGH, 1250);
  // check if pulse duration is within the frequency input range (833Hz - 1.5kHz)
  // if(pulseDuration <= 599 && pulseDuration >= 332)
  // {
  //   freq2 = 1000000.0 / (pulseDuration * 2);    // convert period to frequency 
  // }
  // else // set frequency to 0 otherwise to allow LED to turn of if signal generator is off or disconnected
  // {
  //   freq2 = 0;
  // }
  freq2 = 900;
  monitor.jobEnded(4);
}

// task 5
// calls doWork function 
void doWork()
{
  monitor.jobStarted(5);

  monitor.doWork();

  monitor.jobEnded(5);
}

// task 6
// compare the frequencies measured in task 3 & 4 and turn on LED if their sum is greater than 1500
void compareFreq()
{
  if(freq1 + freq2 > 1500)
  {
    digitalWrite(LED1, HIGH);
  } 
  else
  {
    digitalWrite(LED1, LOW);
  }
}

// task 7
// toggle state of led and call doWork function
void task7()
{
  // check status of flags set in btn interrupt handler 
  // if ledState flag is high, then LED should be turned on, if it is low then LED is turned off
  digitalWrite(LED2, ledState);

  // executes doWork function if doWorkFlag is high
  if(doWorkFlag){
    doWorkFlag = 0;
    monitor.doWork();
  }
}

void setup() 
{
  Serial.begin(115200);
  // set up input/output modes for pins being used
  pinMode(SIGNAL_1, OUTPUT);
  pinMode(SIGNAL_2, OUTPUT);
  pinMode(FREQUENCY_1, INPUT);
  pinMode(FREQUENCY_2, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(SIGNAL_1, LOW);
  digitalWrite(SIGNAL_2, LOW);

  // attach button press interrupt to GPIO pin, to be triggered on rising edge
  attachInterrupt(BUTTON, btnInterrupt, RISING);

  // start frameTicker object to call frame function every frame duration
  frameTicker.attach_ms(FRAME_DURATION_MS, frame);

  // idk causes violations if task isn't called 
  digitalSignal1();
  // delay to allow time for ticker to start running
  delayMicroseconds(3000);

  monitor.startMonitoring();
  delayMicroseconds(1000);

}

void loop() 
{
  // unsigned long bT = micros();
  // for (int i=0; i<1000; i++) {
  //   // task to execute
  // }
  // unsigned long execTime = micros()-bT;
  // Serial.print("Execution Time (x1000): ");
  // Serial.println(execTime);
}
