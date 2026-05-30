/*
  ESP32 Tourelle - rotation + buzzer actif
  - ROT:+010 / ROT:-010 : rotation tourelle
  - BUZ:0800             : buzzer pendant N ms (perte de cible)

  Buzzer actif rond noir 2 pattes :
    + (patte longue) → GPIO 22
    - (patte courte) → GND
*/

#include <Arduino.h>
#include <math.h>

#define PIN_STEP   18
#define PIN_DIR    19
#define PIN_EN     21
#define PIN_BUZZER 22

#define QUEUE_SIZE    20
#define STEPS_PER_DEG 8.8889
#define STEP_DELAY_US 400

const bool INVERT_DIR = false;

QueueHandle_t rotQueue;
QueueHandle_t buzQueue;

void buzzerBeep(int duration_ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(duration_ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void stepMotorDegrees(float deg) {
  if (deg == 0.0 || isnan(deg)) return;

  bool dir = deg >= 0;
  if (INVERT_DIR) dir = !dir;

  digitalWrite(PIN_DIR, dir ? HIGH : LOW);

  int steps = abs((int)round(deg * STEPS_PER_DEG));
  if (steps <= 0) return;

  int ramp = min(steps / 2, 15);

  for (int i = 0; i < steps; i++) {
    int delay_us;
    if (i < ramp) {
      delay_us = STEP_DELAY_US + (ramp - i) * 80;
    } else if (i >= steps - ramp) {
      delay_us = STEP_DELAY_US + (i - (steps - ramp)) * 80;
    } else {
      delay_us = STEP_DELAY_US;
    }

    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(delay_us);
  }
}

void taskSerialCom(void* param) {
  String cmd;

  while (true) {
    if (Serial.available()) {
      cmd = Serial.readStringUntil('\n');
      cmd.trim();

      if (cmd.length() == 0) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
        continue;
      }

      if (cmd.startsWith("ROT:")) {
        float deg = cmd.substring(4).toFloat();
        xQueueSend(rotQueue, &deg, 0);
      } else if (cmd.startsWith("BUZ:")) {
        int dur = cmd.substring(4).toInt();
        if (dur > 0 && dur <= 5000) {
          xQueueSend(buzQueue, &dur, 0);
        }
      }
    }

    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void taskMoteur(void* param) {
  while (true) {
    float deg;
    if (xQueueReceive(rotQueue, &deg, portMAX_DELAY) == pdTRUE) {
      stepMotorDegrees(deg);
    }
  }
}

void taskBuzzer(void* param) {
  while (true) {
    int dur;
    if (xQueueReceive(buzQueue, &dur, portMAX_DELAY) == pdTRUE) {
      buzzerBeep(dur);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP,   OUTPUT);
  pinMode(PIN_DIR,    OUTPUT);
  pinMode(PIN_EN,     OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_EN,     LOW);
  digitalWrite(PIN_STEP,   LOW);
  digitalWrite(PIN_DIR,    LOW);
  digitalWrite(PIN_BUZZER, LOW);

  rotQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
  buzQueue = xQueueCreate(4,          sizeof(int));

  xTaskCreatePinnedToCore(taskSerialCom, "SerialCom", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskMoteur,    "Moteur",    4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskBuzzer,    "Buzzer",    2048, NULL, 1, NULL, 0);

  Serial.println("READY");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
