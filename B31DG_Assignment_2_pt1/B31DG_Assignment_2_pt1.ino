#include "B31DGMonitor.h"
#include <Ticker.h>

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
void digitalSignalOne();
void digitalSignalTwo();
void measureFrequency1();
void measureFrequency2();
void doWork();

// flags for button interrupt
bool ledState = 0;
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

// cyclic exective schedule
void frame()
{
  slot = frameCounter % FRAME_NUMBER;
  
  switch(slot)
  {
    case(0):  digitalSignalTwo();  doWork();           digitalSignalOne(); break;  
    case(1):  measureFrequency1();                                         break;
    case(2):  digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(3):  doWork();            digitalSignalTwo();   task7();          break;
    case(4):  digitalSignalOne();  measureFrequency2();                    break;
    case(5):  digitalSignalTwo();  doWork();             task7();          break;
    case(6):  digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(7):  measureFrequency2(); doWork();                    break;
    case(8):  digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(9):  digitalSignalTwo();  measureFrequency1();                    break;
    case(10): doWork();            digitalSignalOne();   task7();          break;
    case(11): digitalSignalTwo();  measureFrequency1();                    break;
    case(12): digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(13): doWork();            measureFrequency2();                    break;
    case(14): digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(15): doWork();            digitalSignalTwo();   task7();          break;
    case(16): digitalSignalOne();  measureFrequency2();                    break;
    case(17): digitalSignalTwo();  measureFrequency1();                    break;
    case(18): doWork(); digitalSignalTwo();  digitalSignalOne();           break;
    case(19): task7();                                                     break;
    case(20): digitalSignalTwo();  digitalSignalOne();   doWork();         break;
    case(21): digitalSignalTwo();  measureFrequency1();                    break;
    case(22): digitalSignalOne();  doWork();             task7();          break;
    case(23): digitalSignalTwo();  measureFrequency2();                    break;
    case(24): digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(25): doWork();                                  task7();          break;
    case(26): digitalSignalTwo();  digitalSignalOne();   task7();          break;
    case(27): digitalSignalTwo();  measureFrequency2();                    break;
    case(28): doWork();            digitalSignalOne();   task7();          break; 
    case(29): digitalSignalTwo();  measureFrequency1();                    break;
  }
  compareFreq();
  frameCounter++;
}

void IRAM_ATTR btnInterrupt()
{
  buttonTime = millis();
  if(buttonTime - lastButtonTime > debounceTime)
  {
    ledState = !ledState;
    doWorkFlag = 1;
    lastButtonTime = buttonTime;
  }

}

void setup() 
{
  Serial.begin(115200);
  pinMode(SIGNAL_1, OUTPUT);
  pinMode(SIGNAL_2, OUTPUT);
  pinMode(FREQUENCY_1, INPUT);
  pinMode(FREQUENCY_2, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(SIGNAL_1, LOW);
  digitalWrite(SIGNAL_2, LOW);

  attachInterrupt(BUTTON, btnInterrupt, RISING);

  frameTicker.attach_ms(FRAME_DURATION_MS, frame);

  digitalSignalOne();
  // delay to allow time for ticker to start running
  delayMicroseconds(3000);

  monitor.startMonitoring();

}

// task 1
// output a digital signal 
void digitalSignalOne()
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
// output a digital signal
void digitalSignalTwo()
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
void measureFrequency1()
{
  monitor.jobStarted(3);
  // measure the duration of a pulse starting from a rising edge and ending on a falling edge 
  pulseDuration = pulseIn(FREQUENCY_1, HIGH, 1550);
  // check the pulse duration is within the frequency input range (666Hz - 1kHz)
  // duty cycle is 50% so pulse duration is half the length of the period 
  if(pulseDuration <= 760  && pulseDuration > 499)
  {
    freq1 = 1000000.0 / (pulseDuration * 2);    // convert period into frequency 
  }
  else
  {
    freq1 = 0;
  }
  monitor.jobEnded(3); 
}

// task 4
void measureFrequency2()
{
  monitor.jobStarted(4);

  pulseDuration = pulseIn(FREQUENCY_2, HIGH, 1250);
  // check if pulse duration is within the frequency input range (833Hz - 1.5kHz)
  if(pulseDuration <= 599 && pulseDuration >= 332)
  {
    freq2 = 1000000.0 / (pulseDuration * 2);    // convert period to frequency 
  }
  else
  {
    freq2 = 0;
  }
  monitor.jobEnded(4);
}

// task 5
void doWork()
{
  monitor.jobStarted(5);
  // only run do work function every 25 frames i.e. every 50ms / 20Hz
  if(slot == 25)
  {
    monitor.doWork();
  }
  monitor.jobEnded(5);
}

void task7()
{
  // check status of flags set in btn interrupt handler and execute if flags are high 
  if(ledState)
  {
    digitalWrite(LED2, HIGH);
  } 
  else
  {
    digitalWrite(LED2, LOW);
  }

  if(doWorkFlag){
    doWorkFlag = 0;
    monitor.doWork();
  }
}

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

void loop() 
{

}
