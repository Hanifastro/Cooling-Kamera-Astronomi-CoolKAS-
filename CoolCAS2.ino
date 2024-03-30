#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <FS.h>
#include <LITTLEFS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_TSL2591.h>


// Konfigurasi WiFi
const char *ssid = "CoolCAS";
const char *password = "bimasakti";

// Konfigurasi IP
IPAddress local_IP(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// Objek server
AsyncWebServer server(80);
// Objek WebSocket
AsyncWebSocket ws("/ws");

// Konfigurasi sensor suhu DS18B20
const int DS18B20Pin = 33;
OneWire oneWire(DS18B20Pin);
DallasTemperature tempSensor(&oneWire);


// Konfigurasi PID
double Setpoint, Input, Output;
double Kp = 140, Ki = 120, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Konfigurasi pengatur suhu TEC
const int TECControlPin = 4;

int tecOutput = 0;


// Konfigurasi INA219
Adafruit_INA219 ina219;

// Konfigurasi baterai
const float BatteryCapacity = 5000.0; // Kapasitas baterai dalam mAh
const float BatteryVoltage = 12; // Tegangan baterai dalam volt
const int BatteryPin = 35;



float readTemperature() {
  tempSensor.requestTemperatures();
  float temperature = tempSensor.getTempCByIndex(0);
  return temperature;
}

// Konfigurasi SQM
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
void configureSensorTSL2591() {
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
}

// Konstanta kalibrasi A
const float calibrationConstantA = 14.82; // Gantilah dengan nilai kalibrasi yang sesuai

float calculateSQM(float frequency) {
  return calibrationConstantA - 2.5 * log10(frequency);
}

float readTSL2591() {
  sensors_event_t event;
  tsl.getEvent(&event);
  
  if (event.light) {
    float sqm = calculateSQM(event.light);
    return sqm;
  } else {
    return -1;
  }
}


int readBatteryLevel() {
float batteryVoltage = ina219.getBusVoltage_V();

if (batteryVoltage >= 14.4) {
return 100;
} else if (batteryVoltage >= 13.6) {
return 100;
} else if (batteryVoltage >= 13.3) {
return map(batteryVoltage, 13.3, 13.6, 90, 99);
} else if (batteryVoltage >= 13.1) {
return map(batteryVoltage, 13.1, 13.3, 40, 89);
} else if (batteryVoltage >= 12.9) {
return map(batteryVoltage, 12.9, 13.1, 20, 39);
} else if (batteryVoltage >= 12.5) {
return map(batteryVoltage, 12.5, 12.9, 14, 19);
} else if (batteryVoltage >= 10.3) {
return map(batteryVoltage, 10.3, 12.5, 10, 13);
} else {
return 0; // Baterai kosong
}
}

void setup() {
  Serial.begin(115200);

  // Konfigurasi WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

 
// Inisialisasi sensor suhu DS18B20
tempSensor.begin();

// Inisialisasi PID
myPID.SetMode(AUTOMATIC);


  // Inisialisasi pengatur suhu TEC
  pinMode(TECControlPin, OUTPUT);

  // Inisialisasi INA219
  ina219.begin();

  // Inisialisasi baterai
  pinMode(BatteryPin, INPUT);

if (tsl.begin()) {
  configureSensorTSL2591();
} else {
  Serial.println("TSL2591 not found");
}

  // Inisialisasi LittleFS
  if (!LITTLEFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  // Inisialisasi SPIFFS Editor
  server.addHandler(new SPIFFSEditor(SPIFFS, "admin", "admin"));

  // Konfigurasi WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Konfigurasi halaman HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/index.html", "text/html");
  });

  // Konfigurasi file JavaScript
  server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/chart.js", "text/javascript");
  });
// Konfigurasi file JavaScript
  server.on("/bootstrap.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/bootstrap.min.js", "text/javascript");
  });

  // Konfigurasi file JavaScript
  server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/jquery.min.js", "text/javascript");
  });

    // Konfigurasi file JavaScript
  server.on("/popper.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/popper.min.js", "text/javascript");
  });

   // Konfigurasi file JavaScript
  server.on("/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LITTLEFS, "/bootstrap.min.css", "text/css");
  });
  // Mulai server
  server.begin();
}

void loop() {
  // Baca suhu dari sensor DS18B20
  float ds18b20Temperature = readTemperature();

  // Baca arus dan tegangan dari INA219
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();

  float sqm = readTSL2591();

  // Hitung kapasitas baterai yang tersisa
  float batteryLevel = readBatteryLevel();

  // Kirim data ke halaman HTML melalui WebSocket
  DynamicJsonDocument doc(256);
  doc["lm35Temperature"] = ds18b20Temperature;

  // Menggunakan Output dari PID sebagai suhu TEC
  

  doc["shuntvoltage"] = shuntvoltage;
  doc["busvoltage"] = busvoltage;
  doc["current_mA"] = current_mA;
  doc["power_mW"] = power_mW;
  doc["batteryLevel"] = batteryLevel;
  doc["targetTemperature"] = Setpoint;
  doc["sqm"] = sqm;
  String data;
  serializeJson(doc, data);
  ws.textAll(data);

  // Baca nilai suhu saat ini dan kirim ke PID untuk diolah
  Input = ds18b20Temperature;
  myPID.Compute();

  // Cek selisih set point terhadap suhu TEC
float setpointDiff = Setpoint - ds18b20Temperature;

if (setpointDiff < 0) {
    // Ketika selisih set point terhadap suhu TEC negatif
    // Atur nilai PWM sesuai dengan kebutuhan Anda (misalnya, antara 0 hingga 1023)
    tecOutput = map(Output, -255, 255, 150, 1023);
  } else {
    // Ketika selisih set point terhadap suhu TEC positif
    // Atur nilai PWM sesuai dengan kebutuhan Anda (misalnya, 0 hingga 40)
    tecOutput = map(Output, -255, 255, 350, 90);
  }

  
  // Terapkan keluaran PWM ke pengatur suhu TEC
  analogWrite(TECControlPin, tecOutput);

  // Tunggu sebentar sebelum iterasi berikutnya
  delay(500);
}



/// Fungsi untuk mengirim data dari halaman HTML ke ESP32 melalui WebSocket
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo info = *((AwsFrameInfo*)arg);
    if (info.final && info.index == 0 && info.len == len && info.opcode == WS_TEXT) {
      // Parsing data JSON dari string yang diterima
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, data, len);
      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }

      // Mengambil jenis pesan dari data JSON
      String messageType = doc["type"];

      if (messageType == "setpoint") {
        // Mengambil nilai suhu target dari data JSON
        float targetTemperature = doc["targetTemperature"];

        // Mengubah nilai suhu target dan mengirim ke PID untuk diolah
        Setpoint = targetTemperature;
        myPID.SetTunings(Kp, Ki, Kd);
        myPID.SetSampleTime(500); // Interval waktu untuk menghitung nilai Output (dalam milidetik)
      }
    }
  }
}
