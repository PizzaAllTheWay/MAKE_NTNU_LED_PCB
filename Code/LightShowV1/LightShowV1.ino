// Include libraries ----------
#include <Arduino.h>
#include <FastLED.h> 
#include <Arduino_FreeRTOS.h>
#include <semphr.h> 


// Global variables ----------
#define NUM_LEDS 1
#define DATA_PIN 11
#define CLOCK_PIN 13

#define buttonR A3
#define buttonG A2
#define buttonB A1
#define buttonFancy A0

#define semaphoreWaitT 100
#define CRITICAL_TIME_DELAY 100

int globalR = 0;
int globalG = 0;
int globalB = 0;


// Define objects ----------
CRGB leds[NUM_LEDS];

// Create semaphore for critical recources ----------
SemaphoreHandle_t semaphoreR = NULL;
SemaphoreHandle_t semaphoreG = NULL;
SemaphoreHandle_t semaphoreB = NULL;

// Define task prototypes that will be used in multithreading ----------
void monitorInputButtonR (void *pvParameters);
void ledShow (void *pvParameters);


// Setup for the program ----------
void setup() {
  // Pause for readjusting everything

  // Debuging begin
  Serial.begin(9600);
  Serial.println("errrmm........????");

  // Configure led settings
  LEDS.addLeds<APA102,DATA_PIN, CLOCK_PIN,RBG>(leds,NUM_LEDS);
  FastLED.show(); // Turn off all leds

  // Configure buttons
  pinMode(buttonR, INPUT);
  pinMode(buttonG, INPUT);
  pinMode(buttonB, INPUT);
  pinMode(buttonFancy, INPUT);

  // Initialize semaphore for critical recources
  semaphoreR = xSemaphoreCreateMutex();
  semaphoreG = xSemaphoreCreateMutex();
  semaphoreB = xSemaphoreCreateMutex();

  /*
   * Create multithreads from prototype functions
   * xTaskCreate(<function>, <name>, <Stack size>, <Parameters>, <Priority>, <Task handle>);
   */
  xTaskCreate(monitorInputButtonR, "ButtonR", 128, NULL, 1, NULL);
  xTaskCreate(ledShow, "ledShow", 128, NULL, 1, NULL);

  Serial.println("help");
}


// Loop ----------
void loop() {
  // Empty
}


// Multithreading functions ----------
void monitorInputButtonR (void *pvParameters) {
  // Local variables
  int localR = 0;
  int positive = true;

  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  Serial.println("running loop");

  while(true) {
    // Check if RED button was pressed
    if (digitalRead(buttonR)) {
      // Take acurate value of current RED collor
      if (xSemaphoreTake(semaphoreR, (TickType_t) semaphoreWaitT) == pdTRUE) {
        localR = globalR;
        xSemaphoreGive(semaphoreR);
      }

      // If button is pressed collor value goes upp
      // When max is reached start counting down
      if ((positive) && (localR < 255)) {
        localR++;
      }
      else if ((!positive) && (localR > 0)) {
        localR--;
      }
      else if (((positive) && (localR >= 255)) || ((!positive) && (localR <= 0))) {
        positive = !positive;
      }
      
      // Rewrite global value with the local value
      if (xSemaphoreTake(semaphoreR, (TickType_t) semaphoreWaitT) == pdTRUE) {
        globalR = localR;
        xSemaphoreGive(semaphoreR);
      }

      // Debug
      Serial.println("Button R pressed!!!");

      // Accurate realtime delay
      xTaskDelayUntil(&xLastWakeTime, CRITICAL_TIME_DELAY/portTICK_PERIOD_MS);
    }

    Serial.println("running loop");
  }
}


void ledShow (void *pvParameters) {
  // Local variables
  int localR = 0;
  int localG = 0;
  int localB = 0;
  
  while(true) {
    // Take acurate value of current RED collor
    if (xSemaphoreTake(semaphoreR, (TickType_t) semaphoreWaitT) == pdTRUE) {
      localR = globalR;
      xSemaphoreGive(semaphoreR);
    }

    // Take acurate value of current GREEN collor
    if (xSemaphoreTake(semaphoreG, (TickType_t) semaphoreWaitT) == pdTRUE) {
      localG = globalG;
      xSemaphoreGive(semaphoreG);
    }

    // Take acurate value of current BLUE collor
    if (xSemaphoreTake(semaphoreB, (TickType_t) semaphoreWaitT) == pdTRUE) {
      localB = globalB;
      xSemaphoreGive(semaphoreB);
    }

    // Light upp all leds with real time colors
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(localR, localG, localB);
    }

    FastLED.show();
  }
}
