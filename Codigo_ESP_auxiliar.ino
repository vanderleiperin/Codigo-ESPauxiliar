#include <esp_now.h>
#include <WiFi.h>

#define ID "sensor1"
#define CHANNEL 6

#define ACS712_PIN 34
#define PT100_PIN 35
#define INTERVAL 5000
#define SOFTWARE_INTERVAL 1000
#define NUM_SAMPLES 100  // Número de amostras para média

uint8_t gatewayMacAdress[6] = {0x8C, 0x4B, 0x14, 0x9C, 0xD0, 0x9C};
esp_now_peer_info_t gateway;

typedef struct {
  char id[30];
  float current;
  float temperature;
} SensorData;

SensorData sensorData;

hw_timer_t *timer = NULL;
int contador_espnow = 0, flag_espnow = 0;
double valor_corrente = 0;
int valor_corrente_adc = 0;

double filtro_a(double x_n, double Ts, double wn) {
  static double y_n = 0, y_nm1 = 0, x_nm1 = 0;
  double alpha = Ts * wn;
  y_n = ((2 - alpha) / (alpha + 2)) * y_nm1 + (alpha / (alpha + 2)) * (x_n + x_nm1);
  y_nm1 = y_n;
  x_nm1 = x_n;
  return y_n;
}

double filtro_b(double x_n, double Ts, double wn) {
  static double y_n = 0, y_nm1 = 0, x_nm1 = 0;
  double alpha = Ts * wn;
  y_n = ((2 - alpha) / (alpha + 2)) * y_nm1 + (alpha / (alpha + 2)) * (x_n + x_nm1);
  y_nm1 = y_n;
  x_nm1 = x_n;
  return y_n;
}

void IRAM_ATTR ISR_Timer() {
  valor_corrente_adc = analogRead(ACS712_PIN);
  valor_corrente = ((valor_corrente_adc * 3.3) / 4095.0); 
  valor_corrente = (valor_corrente) / (1.5 * 0.1); // 100 mv/A equivale ao ganho do sensor e 1.5 é a VRef/2
  valor_corrente = valor_corrente * valor_corrente;
  valor_corrente = filtro_a(valor_corrente, 0.001, 2 * 3.1415 * 5);
  valor_corrente = sqrt(valor_corrente);
  valor_corrente = filtro_b(valor_corrente, 0.001, 2 * 3.1415 * 1);

  contador_espnow++;
  if (contador_espnow >= 3000)// ajusta o tempo de leitura do timer
  {
    contador_espnow = 0;
    flag_espnow = 1;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Aguarda a inicialização do monitor serial

  Serial.println("Iniciando setup...");

  pinMode(ACS712_PIN, INPUT);
  pinMode(PT100_PIN, INPUT);

  setupWiFi();
  setupEspNow();
  addGatewayAsPeer();

  timer = timerBegin(1000000);             
  timerAttachInterrupt(timer, &ISR_Timer);   
  timerAlarm(timer, 1000, true, 0); 

  Serial.println("Setup concluído.");
}

void loop() {
  if (flag_espnow == 1) {
    Serial.println("Flag de envio detectada, lendo sensor...");
    readSensor();
    sendSensorData();
    flag_espnow = 0;
    Serial.println("Valor Analogico");
    Serial.println(analogRead(ACS712_PIN));
  }
}

void readSensor() {
  Serial.println("Lendo o sensor...");

  sensorData.current = valor_corrente;
    strcpy(sensorData.id, ID);
    Serial.print("Corrente: ");
    Serial.println(sensorData.current);
    Serial.print("Temperatura: ");
    Serial.println(sensorData.temperature);

  float voltage = readVoltage();  // Leitura média da tensão
  float temperature = convertVoltageToTemperature(voltage);  // Conversão da tensão para temperatura
  
  // Limitação da temperatura entre 0 e 110 graus Celsius
  if (temperature < 0) {
    temperature = 0;
  } else if (temperature > 110) {
    temperature = 110;
  }
  sensorData.temperature = temperature;

  Serial.print("Tensão medida (V): ");
  Serial.print(voltage, 2);  // Imprime a tensão com 2 casas decimais
  Serial.print(" - Temperatura calculada (°C): ");
  Serial.println(temperature, 2);  // Imprime a temperatura com 2 casas decimais

  delay(1000);  // Aguarda 1 segundo antes da próxima leitura
}

float readVoltage() {
  long sum = 0;  // Variável para armazenar a soma das leituras

  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(PT100_PIN);  // Leitura do ADC
    delay(10);  // Pequena pausa entre as leituras para estabilidade
  }

  float average = sum / NUM_SAMPLES;  // Calcula a média
  float voltage = (average / 4095.0) * 3.3;  // Converte a média para tensão (considerando ADC de 12 bits e Vref de 3.3V)

  return voltage;
}

float convertVoltageToTemperature(float voltage) {
  // Coeficientes baseados no ajuste polinomial
  float a = -50.6;  // Coeficiente quadrático
  float b = 51.9;  // Coeficiente linear
  float c = 86.5;  // Termo constante

  // Fórmula do ajuste polinomial
  float temperature = a * voltage * voltage + b * voltage + c;

  return temperature;
}

void sendSensorData() {
  esp_err_t result = esp_now_send(gatewayMacAdress, (uint8_t*)&sensorData, sizeof(sensorData));
  Serial.print("Status do envio: ");
  if (result == ESP_OK) {
    Serial.println("Sucesso");
  } else {
    Serial.println("Erro no envio");
  }
}

void setupWiFi() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  bool hidden = true;
  if (WiFi.softAP(ID, "12345678", CHANNEL, hidden)) {
    Serial.println("WiFi configurado como AP");
  } else {
    Serial.println("Falha ao configurar WiFi como AP");
  }
}

void setupEspNow() {
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Inicializado com Sucesso");
    esp_now_register_send_cb(onDataSent);
  } else {
    Serial.println("ESPNow Falha ao inicializar");
    ESP.restart();
  }
}

void addGatewayAsPeer() {
  gateway.channel = CHANNEL;
  gateway.encrypt = 0;
  gateway.ifidx = WIFI_IF_AP;
  memcpy(gateway.peer_addr, gatewayMacAdress, sizeof(gatewayMacAdress));
  if (esp_now_add_peer(&gateway) == ESP_OK) {
    Serial.println("Peer adicionado com sucesso");
  } else {
    Serial.println("Falha ao adicionar peer");
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Dados enviados?: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sim" : "Falhou");
}
