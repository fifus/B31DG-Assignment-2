/*
 * Author: Alex Ferguson
 * Created: 27/04/25
 * 
 * Description:
 * Executes 5 tasks with real-time requirements using the freeRTOS scheduler
 * Tasks 1 & 2 generate a digital waveform on gpio pins 25 and 33
 * Tasks 3 & 4 measure digital signals connected to gpio pins 18 and 19
 * Task 5 calls B31DGMonitor's 'doWork' function 
 * Tasks 6 & 7 do not have real-time requirements and are executed when no higher priority tasks are available. 
 * Task 6 lights an LED  when the sum of the measured frequencies exceed 1500Hz, and task 7 uses a button 
 * to toggle the state of an LED and call the doWork function.
 *
*/ 

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "B31DGMonitor.h"

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
void digitalSignal1(void *pvParameters);
void digitalSignal2(void *pvParameters);
void measureFrequency1(void *pvParameters);
void measureFrequency2(void *pvParameters);
void doWork(void *pvParameters);
void compareFreq(void *pvParameters);
void task7(void *pvParameters);
void IRAM_ATTR btnInterrupt();


// task handlers
TaskHandle_t taskOneHandler;
TaskHandle_t taskTwoHandler;
TaskHandle_t taskThreeHandler;
TaskHandle_t taskFourHandler;
TaskHandle_t taskFiveHandler;
TaskHandle_t taskSixHandler;
TaskHandle_t taskSevenHandler;

// holds the state to toggle the led for task 7
int ledState = LOW;

// variables to hold values for measuring frequencies
float pulseDuration = 0;
float freq1 = 0;
float freq2 = 0;

// debounce variables for button press
const int debounceTime = 10;
volatile unsigned long buttonTime = 0;
volatile unsigned long lastButtonTime = 0;

B31DGCyclicExecutiveMonitor monitor(100);

// frequency measurement semaphores and button press semaphore
SemaphoreHandle_t freqSem1;
SemaphoreHandle_t freqSem2;
SemaphoreHandle_t btnSem;

// period (ms) for tasks 1 - 5
const unsigned int task1P = 4;
const unsigned int task2P = 3;
const unsigned int task3P = 10;
const unsigned int task4P = 10;
const unsigned int task5P = 5;

void IRAM_ATTR btnInterrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  buttonTime = millis();
  if(buttonTime - lastButtonTime > debounceTime)
  {
    xSemaphoreGiveFromISR(btnSem, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// task 1
void digitalSignal1(void *pvParameters)
{
  (void) pvParameters;
  //delayMicroseconds(500);
  TickType_t xlastWakeTime = xTaskGetTickCount();

  for(;;)
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

    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task1P));
  }
}
// task 2
void digitalSignal2(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xlastWakeTime = xTaskGetTickCount();

  for(;;)
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

    //delayMicroseconds(750);
    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task2P));

  }
}

// task 3
void measureFrequency1(void *pvParameters)
{
  (void) pvParameters;
  //delayMicroseconds(100);
  TickType_t xlastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    monitor.jobStarted(3);

    pulseDuration = pulseIn(FREQUENCY_1, HIGH, 1550);

    xSemaphoreTake(freqSem1, portMAX_DELAY);
    if(pulseDuration <= 760  && pulseDuration > 499)
    {
      freq1 = 1000000.0 / (pulseDuration * 2); 
    }
    else
    {
      freq1 = 0;
    }  
    xSemaphoreGive(freqSem1);

    monitor.jobEnded(3); 
    //vTaskDelay(pdMS_TO_TICKS(task3P));
    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task3P));
  }
}

// task 4
void measureFrequency2(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xlastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    monitor.jobStarted(4);
    // set pulseIn function to timeout after half the period so it doesn't 
    // affect execution time if there's no signal input 
    pulseDuration = pulseIn(FREQUENCY_2, HIGH, 1250);
    
    xSemaphoreTake(freqSem2, portMAX_DELAY);
    // only update frequency if it falls within the expected range of frequencies 
    if(pulseDuration <= 599 && pulseDuration >= 332)
    {
      freq2 = 1000000.0 / (pulseDuration * 2);
    }
    else  // set to zero otherwise, as LED won't turn off if signal gen is turned off
    {
      freq2 = 0;
    }
    xSemaphoreGive(freqSem2);
    
    monitor.jobEnded(4);
    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task4P));
  }
}

// task 5
// calls the monitor's doWork function
void doWork(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xlastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    monitor.jobStarted(5);

    monitor.doWork();

    monitor.jobEnded(5);

    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task5P));
  }
}

// task 6
void compareFreq(void *pvParameters)
{
  // local variables to store frequencies to avoid semaphores from being held longer than is neccessary
  // (help prevent deadlocks)
  float locFreq1 = 0;
  float locFreq2 = 0;
  for(;;)
  {
    // wait for freq1 semaphore to be released
    if(xSemaphoreTake(freqSem1, portMAX_DELAY)==pdTRUE)
    {
      locFreq1 = freq1;
      xSemaphoreGive(freqSem1);
    }

    // wait for freq2 semaphore to be released
    if(xSemaphoreTake(freqSem2, portMAX_DELAY)==pdTRUE)
    {
      locFreq2 = freq2;
      xSemaphoreGive(freqSem2);
    }

    if(locFreq1 + locFreq2 >= 1500)
    {
      digitalWrite(LED1, HIGH);
    } 
    else
    {
      digitalWrite(LED1, LOW);
    } 
  }
}

void task7(void *pvParameters)
{
  (void) pvParameters;
  for(;;)
  {
    // wait for semaphore to be given from ISR
    if(xSemaphoreTake(btnSem, portMAX_DELAY) == pdTRUE)
    { 
      // toggle the state of the LED, then call the doWork function
      ledState = !ledState;
      digitalWrite(LED2, ledState);
      monitor.doWork();
    }
  }
}


void setup() {
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

  // create semaphores for frequency measurements and push button
  freqSem1 = xSemaphoreCreateMutex();
  freqSem2 = xSemaphoreCreateMutex();
  btnSem = xSemaphoreCreateBinary();

  // create free rtos tasks on core 0
  xTaskCreatePinnedToCore(digitalSignal1, "task 1", 2048, NULL, 4, &taskOneHandler, 0);
  xTaskCreatePinnedToCore(digitalSignal2, "task 2", 2048, NULL, 4, &taskTwoHandler, 0);
  xTaskCreatePinnedToCore(measureFrequency1, "task 3", 2048, NULL, 1, &taskThreeHandler, 0);
  xTaskCreatePinnedToCore(measureFrequency2, "task 4", 2048, NULL, 1, &taskFourHandler, 0);
  xTaskCreatePinnedToCore(doWork, "task 5", 1024, NULL, 2, &taskFiveHandler, 0);
  xTaskCreatePinnedToCore(compareFreq, "task 6", 2048, NULL, 0, &taskSixHandler, 0);
  xTaskCreatePinnedToCore(task7, "task 7", 1024, NULL, 0, &taskSevenHandler, 0);

  delay(60);

  monitor.startMonitoring();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.printf("Task 2 stack: %d bytes remaining", uxTaskGetStackHighWaterMark(taskTwoHandler));
}
