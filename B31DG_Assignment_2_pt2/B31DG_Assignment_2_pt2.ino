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
void digitalSignalOne(void *pvParameters);
void digitalSignalTwo(void *pvParameters);
void measureFrequencyOne(void *pvParameters);
void measureFrequencyTwo(void *pvParameters);
void doWork(void *pvParameters);
//void compareFreq(void *pvParameters);
void task7(void *pvParameters);


// task handlers
TaskHandle_t taskOneHandler;
TaskHandle_t taskTwoHandler;
TaskHandle_t taskThreeHandler;
TaskHandle_t taskFourHandler;
TaskHandle_t taskFiveHandler;
TaskHandle_t taskSixHandler;
TaskHandle_t taskSevenHandler;

int buttonState;
int lastButtonState = LOW;

float pulseDuration = 0;
float freq1 = 0;
float freq2 = 0;

const int debounceTime = 10;
unsigned long lastDebounceTime = 0;

B31DGCyclicExecutiveMonitor monitor(100);

SemaphoreHandle_t freqSem;

// period (ms) for tasks 1 - 5
const unsigned int task1P = 4;
const unsigned int task2P = 3;
const unsigned int task3P = 10;
const unsigned int task4P = 10;
const unsigned int task5P = 5;

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

  freqSem = xSemaphoreCreateMutex();

  // create free rtos tasks on core 0
  xTaskCreatePinnedToCore(digitalSignalOne, "task 1", 2048, NULL, 4, &taskOneHandler, 0);
  xTaskCreatePinnedToCore(digitalSignalTwo, "task 2", 2048, NULL, 4, &taskTwoHandler, 0);
  xTaskCreatePinnedToCore(measureFrequencyOne, "task 3", 2048, NULL, 1, &taskThreeHandler, 0);
  xTaskCreatePinnedToCore(measureFrequencyTwo, "task 4", 2048, NULL, 1, &taskFourHandler, 0);
  xTaskCreatePinnedToCore(doWork, "task 5", 1024, NULL, 2, &taskFiveHandler, 0);
  //xTaskCreatePinnedToCore(compareFreq, "task 6", 2048, NULL, 2, &taskSixHandler, 0);
  xTaskCreatePinnedToCore(task7, "task 7", 1024, NULL, 0, &taskSevenHandler, 0);

  delay(60);

  monitor.startMonitoring();
}

// task 1
void digitalSignalOne(void *pvParameters)
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
void digitalSignalTwo(void *pvParameters)
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
void measureFrequencyOne(void *pvParameters)
{
  (void) pvParameters;
  //delayMicroseconds(100);
  TickType_t xlastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    monitor.jobStarted(3);

    if(xSemaphoreTake(freqSem, portMAX_DELAY) == pdTRUE)
    {
      pulseDuration = pulseIn(FREQUENCY_1, HIGH, 1550);

      if(pulseDuration <= 760  && pulseDuration > 499)
      {
        freq1 = 1000000.0 / (pulseDuration * 2); 
      }
      else
      {
        freq1 = 0;
      }  

      if(freq1 + freq2 >= 1500)
      {
        digitalWrite(LED1, HIGH);
      } 
      else
      {
        digitalWrite(LED1, LOW);
      }
      xSemaphoreGive(freqSem);
    } 

    monitor.jobEnded(3); 
    //vTaskDelay(pdMS_TO_TICKS(task3P));
    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task3P));
  }
}

// task 4
void measureFrequencyTwo(void *pvParameters)
{
  (void) pvParameters;
  TickType_t xlastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    monitor.jobStarted(4);
  // set pulseIn function to timeout after half the period so it doesn't affect execution time if there's no signal input 

    if(xSemaphoreTake(freqSem, portMAX_DELAY) == pdTRUE)
    {
      pulseDuration = pulseIn(FREQUENCY_2, HIGH, 1250);
      if(pulseDuration <= 599 && pulseDuration >= 332)
      {
        freq2 = 1000000.0 / (pulseDuration * 2);
      }
      else
      {
        freq2 = 0;
      }
      xSemaphoreGive(freqSem);
    }
    
    monitor.jobEnded(4);
    vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(task4P));
  }
}

// task 5
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

void task7(void *pvParameters)
{
  (void) pvParameters;
  for(;;)
  {
    digitalWrite(LED2, LOW);

    TickType_t lastDebounceTime = xTaskGetTickCount();
    for (;;) {
      // read the state of the button pin 
        int reading = digitalRead(BUTTON);
        // check if the current reading is not equal to the previous reading
        if (reading != lastButtonState) {
            // get tick count for current button press
            lastDebounceTime = xTaskGetTickCount();
        }
        // debounce, check if difference between current tick count and tick count of previous reading 
        // is greater than debounce threshold
        if ((xTaskGetTickCount() - lastDebounceTime) > debounceTime) {
            if (reading != buttonState) {
                // set current button state to debounced reading
                buttonState = reading;
                // on falling edge of button press, call doWork function and write led HIGH
                if (buttonState == LOW) {
                    digitalWrite(LED2, !digitalRead(LED2));
                    monitor.doWork();
                }
            }
        }
        lastButtonState = reading;
    }
  }
} 

void compareFreq(void *pvParameters)
{
  if(xSemaphoreTake(freqSem, portMAX_DELAY)==pdTRUE)

  {
    if(freq1 + freq2 >= 1500)
    {
      digitalWrite(LED1, HIGH);
    } 
    else
    {
      digitalWrite(LED1, LOW);
    }
    xSemaphoreGive(freqSem);
  }

}


void loop() {
  // put your main code here, to run repeatedly:

}
