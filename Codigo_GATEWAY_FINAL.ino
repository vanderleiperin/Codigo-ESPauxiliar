#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL2kh-iwOf9"
#define BLYNK_TEMPLATE_NAME "monitoramento de motores elétricos "
#define BLYNK_AUTH_TOKEN "15jbQe4pak5p5P3hG0VZBhFBWsPZov2v" 

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <BlynkSimpleEsp32.h>

char auth[] = "15jbQe4pak5p5P3hG0VZBhFBWsPZov2v";
char ssid[] = "POCO";
char pass[] = "82639vanderlei";

typedef struct {
  char id[30];
  float current; 
  float temperature;
} SensorData;

SensorData receivedData;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  Serial.println("Dados recebidos.");
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  Serial.print("ID: ");
  Serial.println(receivedData.id);
  Serial.print("Corrente (A): ");
  Serial.println(receivedData.current);
  Serial.print("Temperatura (°C): ");
  Serial.println(receivedData.temperature);

  // Envia os dados recebidos para diferentes widgets no Blynk com base no ID
  if (strcmp(receivedData.id, "sensor1") == 0) {
    // Dados do microcontrolador 1
    Blynk.virtualWrite(V1, receivedData.current); // Corrente do sensor 1
    Blynk.virtualWrite(V2, receivedData.temperature); // Temperatura do sensor 1
  } else if (strcmp(receivedData.id, "sensor2") == 0) {
    // Dados do microcontrolador 2
    Blynk.virtualWrite(V3, receivedData.current); // Corrente do sensor 2
    Blynk.virtualWrite(V4, receivedData.temperature); // Temperatura do sensor 2
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando setup...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  Serial.println("WiFi iniciado no canal 6.");

  connectToWiFi();
  Blynk.begin(auth, ssid, pass);

  esp_err_t espNowStatus = esp_now_init();
  if (espNowStatus == ESP_OK) {
    Serial.println("ESPNow Init Success");
    esp_err_t cbStatus = esp_now_register_recv_cb(onDataRecv);
    if (cbStatus == ESP_OK) {
      Serial.println("Callback de recepção registrado com sucesso.");
    } else {
      Serial.print("Falha ao registrar callback: ");
      Serial.println(cbStatus);
    }
  } else {
    Serial.print("ESPNow Init Failed: ");
    Serial.println(espNowStatus);
    ESP.restart();
  }
}

void loop() {
  Blynk.run();
  delay(500);
}

void connectToWiFi() {
  Serial.print("Conectando-se à rede Wi-Fi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("\nConectado à rede Wi-Fi.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
