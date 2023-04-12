// Include libraries ----------
#include <Arduino.h>
#include <FastLED.h> 
#include <Arduino_FreeRTOS.h>
#include <semphr.h> 


// Global variables ----------
#define NUM_LEDS 9
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
int globalFancy = 0;


// Define objects ----------
CRGB leds[NUM_LEDS];


// Create semaphore for critical recources ----------
SemaphoreHandle_t semaphoreRGB = NULL;
SemaphoreHandle_t semaphoreFancy = NULL;


// Define task prototypes that will be used in multithreading ----------
void monitorInputButtonsRGB (void *pvParameters);
void monitorInputButtonFancy (void *pvParameters);
void ledShow (void *pvParameters);


// Define methods that will be used in various ways ----------
void ledCollors (int R, int G, int B);
void led (int numLED, int R, int G, int B);

byte *Wheel(byte WheelPosition);


// Setup for the program ----------
void setup() {
  // Debuging begin
  Serial.begin(9600);

  // Configure led settings
  LEDS.addLeds<APA102,DATA_PIN, CLOCK_PIN,RBG>(leds,NUM_LEDS); // Initialize led strip
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1500); // Set power limit of LED strip to 5V, 1500mA
  FastLED.clear(); // Turn off all leds

  // Configure buttons
  pinMode(buttonR, INPUT);
  pinMode(buttonG, INPUT);
  pinMode(buttonB, INPUT);
  pinMode(buttonFancy, INPUT);

  // Initialize semaphore for critical recources
  semaphoreRGB = xSemaphoreCreateMutex();
  semaphoreFancy = xSemaphoreCreateMutex();

  /*
   * Create multithreads from prototype functions
   * xTaskCreate(<function>, <name>, <Stack size>, <Parameters>, <Priority>, <Task handle>);
   */
  xTaskCreate(monitorInputButtonsRGB, "ButtonsRGB", 64, NULL, 1, NULL);
  xTaskCreate(monitorInputButtonFancy, "ButtonFancy", 64, NULL, 1, NULL);
  xTaskCreate(ledShow, "ledShow", 512, NULL, 1, NULL);
}


// Loop ----------
void loop() {
  // Empty
}


// Multithreading functions ----------
void monitorInputButtonsRGB (void *pvParameters) {
  // Local variables
  int localR = 0;
  int localG = 0;
  int localB = 0;
  int holdRoundsR = 0;
  int holdRoundsG = 0;
  int holdRoundsB = 0;

  int maxPower = 150;

  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  while(true) {
    // Take acurate value of current collors
    if (xSemaphoreTake(semaphoreRGB, (TickType_t) semaphoreWaitT) == pdTRUE) {
      localR = globalR;
      localG = globalG;
      localB = globalB;
      xSemaphoreGive(semaphoreRGB);
    }

    // Check if RED button was pressed
    if (digitalRead(buttonR)) {
      // If button is pressed collor value goes upp
      // When max is reached start from scrach and give some time to react
      if ((localR <= 0) && (holdRoundsR < 0)) {
        holdRoundsR++;
      }
      else if ((localR < maxPower) && (holdRoundsR >= 0)) {
        localR += pow(localR, 2) * 0.001 + 1;
      }
      else if (localR >= maxPower) {
        localR = maxPower;
        holdRoundsR++;
      }

      if (holdRoundsR >= 20) {
        holdRoundsR = -20;
        localR = 0;
      }
    }

    // Check if GREEN button was pressed
    if (digitalRead(buttonG)) {
      // If button is pressed collor value goes upp
      // When max is reached start from scrach and give some time to react
      if ((localG <= 0) && (holdRoundsG < 0)) {
        holdRoundsG++;
      }
      else if ((localG < maxPower) && (holdRoundsG >= 0)) {
        localG += pow(localG, 2) * 0.001 + 1;
      }
      else if (localG >= maxPower) {
        localG = maxPower;
        holdRoundsG++;
      }

      if (holdRoundsG >= 20) {
        holdRoundsG = -20;
        localG = 0;
      }
    }

    // Check if BLUE button was pressed
    if (digitalRead(buttonB)) {
      // If button is pressed collor value goes upp
      // When max is reached start from scrach and give some time to react
      if ((localB <= 0) && (holdRoundsB < 0)) {
        holdRoundsB++;
      }
      else if ((localB < maxPower) && (holdRoundsB >= 0)) {
        localB += pow(localB, 2) * 0.001 + 1;
      }
      else if (localB >= maxPower) {
        localB = maxPower;
        holdRoundsB++;
      }

      if (holdRoundsB >= 20) {
        holdRoundsB = -20;
        localB = 0;
      }
    }

    // Rewrite global values with the local values
    if (xSemaphoreTake(semaphoreRGB, (TickType_t) semaphoreWaitT) == pdTRUE) {
      globalR = localR;
      globalG = localG;
      globalB = localB;
      xSemaphoreGive(semaphoreRGB);
    }

    // Accurate real time delay
    xTaskDelayUntil(&xLastWakeTime, CRITICAL_TIME_DELAY/portTICK_PERIOD_MS);
  }
}

void monitorInputButtonFancy (void *pvParameters) {
  // Local variables
  int localFancy = 0;
  bool pressed = false;

  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  while(true) {
    
    // Take acurate value of current BLUE collor
    if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
      localFancy = globalFancy;
      xSemaphoreGive(semaphoreFancy);
    }

    /*
     * If presed for long time, only detect it as one press to stop program from incrementing to fast
     * Once the button is released, reset the state and the button can be pressed again
     */
    if (digitalRead(buttonFancy) && !pressed) {
      pressed = true;
      localFancy++;
    }
    else if (!digitalRead(buttonFancy)) {
      pressed = false;
    }

    // Check if the maximum fancy modes reached, if so reset
    if (localFancy > 4) {
      localFancy = 0;
    }

    // Rewrite global value with the local value
    if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
      globalFancy = localFancy;
      xSemaphoreGive(semaphoreFancy);
    }

    // Accurate real time delay
    xTaskDelayUntil(&xLastWakeTime, CRITICAL_TIME_DELAY/portTICK_PERIOD_MS);
  }
}

void ledShow (void *pvParameters) {
  // Local variables
  int localR = 0;
  int localG = 0;
  int localB = 0;
  int localFancy = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount(); 

  int breathingCycles = 25;
  int breathingPower = 2;
  int breathingDamper = 5000;
  int BREATHING_TIME_DELAY = 100;

  int blinkTimeDelay = 50;

  int seizuteMaxPower = 200; // Max brightness
  int seizurePWeight = 40; // Contrast, the lower the number, the more contrast
  int seizureLEDOffPWeight = 9; // 0-10 (0: always leave leds ON / 10: always turn OFF leds)
  int seizureTimeDelay = 15;

  float RainbowLightMax = 0.5; // 0 - 1 (Min - Max)
  int RAINBOW_TIME_DELAY = 25;

  // Get current Fancy button state
  if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
    localFancy = globalFancy;
    xSemaphoreGive(semaphoreFancy);
  }
  
  while(true) {
    // Custom colors ----------
    while (localFancy == 0) {
      // Get current Fancy button state
      if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
        localFancy = globalFancy;
        xSemaphoreGive(semaphoreFancy);
      }

      // Take acurate value of current collors
      if (xSemaphoreTake(semaphoreRGB, (TickType_t) semaphoreWaitT) == pdTRUE) {
        localR = globalR;
        localG = globalG;
        localB = globalB;
        xSemaphoreGive(semaphoreRGB);
      }

      // Light upp all leds
      ledCollors(localR, localG, localB);
    }
    
    // Breathing ----------
    // Original colours incase algorithm goes overboard
    int originalR = localR;
    int originalG = localG;
    int originalB = localB;

    breathing: // <= Goto loop referance so that we can break out of the loop any time for maximum responsines for person when Fancy button is clicked

    while (localFancy == 1) {
      // Collors DOWN
      for (int i = 0; i < breathingCycles; i++) {
        // Get current Fancy button state and check if Fancy button has been trigered to exit the loop
        if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
          if (localFancy != globalFancy) {
            localFancy = globalFancy;
            xSemaphoreGive(semaphoreFancy);
            goto breathing;
          }
          xSemaphoreGive(semaphoreFancy);
        }

        // Calculatiuon for good breathing animation compared to colour brightnes
        if (originalR == 0) {localR = 0;}
        else {
          localR -= 1/(pow(localR, breathingPower)/breathingDamper + 1);
          if (localR < 1) {
            localR = 1;
          }
        }

        if (originalG == 0) {localG = 0;}
        else {
          localG -= 1/(pow(localG, breathingPower)/breathingDamper + 1);
          if (localG < 1) {
            localG = 1;
          }
        }

        if (originalB == 0) {localB = 0;}
        else {
          localB -= 1/(pow(localB, breathingPower)/breathingDamper + 1);
          if (localB < 1) {
            localB = 1;
          }
        }

        // Display animation
        ledCollors(localR, localG, localB);

        // Accurate real time delay
        xTaskDelayUntil(&xLastWakeTime, BREATHING_TIME_DELAY/portTICK_PERIOD_MS);
      }

      // Collors UPP
      for (int i = 0; i < breathingCycles; i++) {
        // Get current Fancy button state and check if Fancy button has been trigered to exit the loop
        if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
          if (localFancy != globalFancy) {
            localFancy = globalFancy;
            xSemaphoreGive(semaphoreFancy);
            goto breathing;
          }
          xSemaphoreGive(semaphoreFancy);
        }

        // Calculatiuon for good breathing animation compared to colour brightnes
        localR += pow(localR, breathingPower)/breathingDamper + 1;
        localG += pow(localG, breathingPower)/breathingDamper + 1;
        localB += pow(localB, breathingPower)/breathingDamper + 1;

        if (localR > originalR) {localR = originalR;}
        if (localG > originalG) {localG = originalG;}
        if (localB > originalB) {localB = originalB;}

        // Display animation
        ledCollors(localR, localG, localB);

        // Accurate real time delay
        xTaskDelayUntil(&xLastWakeTime, BREATHING_TIME_DELAY/portTICK_PERIOD_MS);
      }
    }

    // Blinking ----------
    // Take acurate value of current collors
    if (xSemaphoreTake(semaphoreRGB, (TickType_t) semaphoreWaitT) == pdTRUE) {
      localR = globalR;
      localG = globalG;
      localB = globalB;
      xSemaphoreGive(semaphoreRGB);
    }

    // Clear all leds
    ledCollors(0, 0, 0);

    while (localFancy == 2) {
      // Get current Fancy button state
      if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
        localFancy = globalFancy;
        xSemaphoreGive(semaphoreFancy);
      }

      // Leds turn ON/OFF by probaility
      led(random(NUM_LEDS), localR, localG, localB); // Turn ON a random LED
      led(random(NUM_LEDS), 0, 0, 0); // Turn OFF a random LED

      // Accurate real time delay
      xTaskDelayUntil(&xLastWakeTime, random(1, blinkTimeDelay)/portTICK_PERIOD_MS);
    }

    // Seizure ----------
    // Temporary array for random RGB values
    int randomRGB[3];
    int indexMaxVal = 0;

    // Clear all leds
    ledCollors(0, 0, 0);

    while (localFancy == 3) {
      // Get current Fancy button state
      if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
        localFancy = globalFancy;
        xSemaphoreGive(semaphoreFancy);
      }

      // Weighted probability
      randomRGB[0] = random(seizuteMaxPower); // R
      randomRGB[1] = random(seizuteMaxPower); // G
      randomRGB[2] = random(seizuteMaxPower); // B

      int maxVal = 0;

      for (int i = 0; i < (int)(sizeof(randomRGB)/sizeof(randomRGB[0])); i++) {
        if (randomRGB[i] > maxVal) {
          maxVal = randomRGB[i];
          indexMaxVal = i;
        }
      }

      if (indexMaxVal != 0) {localR = random(0, seizurePWeight);} // Weigh down R even lower if NOT maximum value
      else {localR = randomRGB[indexMaxVal];}
      if (indexMaxVal != 1) {localG = random(0, seizurePWeight);} // Weigh down G even lower if NOT maximum value
      else {localG = randomRGB[indexMaxVal];}
      if (indexMaxVal != 2) {localB = random(0, seizurePWeight);} // Weigh down B even lower if NOT maximum value
      else {localB = randomRGB[indexMaxVal];}

      // Turn ON a random LED
      led(random(NUM_LEDS), localR, localG, localB); 

      // Turn OFF a random LED
      if (seizureLEDOffPWeight > random(10)) {
        led(random(NUM_LEDS), 0, 0, 0); 
      }

      // Accurate real time delay
      xTaskDelayUntil(&xLastWakeTime, random(1, seizureTimeDelay)/portTICK_PERIOD_MS);
    }
    
    // Rainbow ----------
    // Variables
    byte *c;
    uint16_t i, j;

    Rainbow: // <= Goto command for fast responsive exiting of the wile loop when new fancy status is active

    while (localFancy == 4) {
      /*
       * |==================== (START) ====================|
       * 
       * Coloor mixing code in this section is made by:
       * Author: Electriangle
       * Source: https://github.com/Electriangle/RainbowCycle_Main/blob/main/RainbowCycle_Animation.ino
       */

      for(j=0; j < 256; j++) {
        // Get current Fancy button state and check if Fancy button has been trigered to exit the loop
        if (xSemaphoreTake(semaphoreFancy, (TickType_t) semaphoreWaitT) == pdTRUE) {
          if (localFancy != globalFancy) {
            localFancy = globalFancy;
            xSemaphoreGive(semaphoreFancy);
            goto Rainbow;
          }
          xSemaphoreGive(semaphoreFancy);
        }

        for(i=0; i < NUM_LEDS; i++) {
          c = Wheel(((i * 255 /NUM_LEDS) + j) & (255));

          localR = (int)((float)(*c) * RainbowLightMax);
          localG = (int)((float)(*(c+1)) * RainbowLightMax);
          localB = (int)((float)(*(c+2)) * RainbowLightMax);

          leds[NUM_LEDS - 1 - i].setRGB(localR, localG, localB);
        }
        FastLED.show();

        // Accurate real time delay
        xTaskDelayUntil(&xLastWakeTime, RAINBOW_TIME_DELAY/portTICK_PERIOD_MS);
      }

      /*
       * |==================== (STOP) ====================|
       * 
       * Coloor mixing code in this section is made by:
       * Author: Electriangle
       * Source: https://github.com/Electriangle/RainbowCycle_Main/blob/main/RainbowCycle_Animation.ino
       */    
    }
  }
}

// Other functions ----------
void ledCollors (int R, int G, int B) {
  // Light upp all leds with real time colors
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(R, G, B);
  }

  FastLED.show();
}

void led (int numLED, int R, int G, int B) {
  // Light upp a single LED
  leds[numLED] = CRGB(R, G, B);

  FastLED.show();
}

/*
 * |==================== (START) ====================|
 * 
 * Coloor mixing code in this section is made by:
 * Author: Electriangle
 * Source: https://github.com/Electriangle/RainbowCycle_Main/blob/main/RainbowCycle_Animation.ino
 */
byte *Wheel(byte WheelPosition) {
  static byte c[3];
 
  if(WheelPosition < 85) {
   c[0] = WheelPosition * 3;
   c[1] = 255 - WheelPosition * 3;
   c[2] = 0;
  }
  else if(WheelPosition < 170) {
   WheelPosition -= 85;
   c[0] = 255 - WheelPosition * 3;
   c[1] = 0;
   c[2] = WheelPosition * 3;
  }
  else {
   WheelPosition -= 170;
   c[0] = 0;
   c[1] = WheelPosition * 3;
   c[2] = 255 - WheelPosition * 3;
  }

  return c;
}
/*
 * |==================== (STOP) ====================|
 * 
 * Coloor mixing code in this section is made by:
 * Author: Electriangle
 * Source: https://github.com/Electriangle/RainbowCycle_Main/blob/main/RainbowCycle_Animation.ino
 */
