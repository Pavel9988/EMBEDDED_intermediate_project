#include "esp_camera.h"   // Libreria per la gestione della camera ESP32
#include <WiFi.h>         // Libreria per la connessione WiFi

// ===================
// Selezione modello camera
// ===================
// Decommenta SOLO la riga che corrisponde al tuo modello di camera.
// Qui è selezionato il modello AI Thinker:
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"  // File con la mappatura dei pin per il modello scelto

// ===========================
// Credenziali WiFi
// ===========================
const char *ssid = "HUAWEI P10 lite";      // Nome della rete WiFi
const char *password = "1234567890123";       // Password della rete WiFi

// ===========================
// Configurazione IP statica (compatibile con rete hotspot iPhone)
// ===========================
// Configura l'indirizzo IP statico per la ESP32-CAM sulla rete Huawei
IPAddress local_IP(10, 157, 116, 171);       // IP statico desiderato (assicurati che sia libero!)
IPAddress gateway(10, 157, 116, 1);          // Gateway della rete Huawei (tipicamente l’indirizzo del router)
IPAddress subnet(255, 255, 255, 0);          // Subnet mask della rete Huawei
IPAddress primaryDNS(8, 8, 8, 8);            // DNS primario (facoltativo)
IPAddress secondaryDNS(8, 8, 4, 4);          // DNS secondario (facoltativo)




// ===========================
// Variabili globali
// ===========================
camera_config_t config;     // Struttura di configurazione della camera
esp_err_t err;              // Variabile per gestire eventuali errori
bool cameraActive = true;   // Stato della camera: attiva/disattiva

// ===========================
// Dichiarazione funzioni
// ===========================
void startCameraServer();           // Avvia il server web per lo streaming
void setupLedFlash(int pin);        // Gestisce il LED flash, se presente

// ===========================
// SETUP: viene eseguito una sola volta all'avvio
// ===========================
void setup() {
  Serial.begin(115200);             // Inizializza la comunicazione seriale per debug
  Serial.setDebugOutput(true);      // Abilita messaggi di debug sulla seriale
  Serial.println();

  // Configura l'IP statico PRIMA di connettersi al WiFi
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Errore nella configurazione IP statica");
  }

  // ===========================
  // Configurazione dei parametri della camera
  // ===========================
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;           // Frequenza clock per la camera
  config.frame_size   = FRAMESIZE_UXGA;     // Risoluzione iniziale (UXGA = 1600x1200)
  config.pixel_format = PIXFORMAT_JPEG;     // Formato pixel JPEG per streaming
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;                 // Qualità JPEG (più basso = migliore qualità)
  config.fb_count     = 1;                  // Numero di frame buffer

  // Se la memoria PSRAM è presente, migliora qualità e frame buffer
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Se PSRAM non è presente, riduci risoluzione e cambia posizione buffer
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Impostazioni per face detection/recognition (non usate qui)
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP); // Solo per alcuni modelli
  pinMode(14, INPUT_PULLUP);
#endif

  // ===========================
  // Inizializza la camera
  // ===========================
  err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return; // Se fallisce, esci dal setup
  }

  // Ottieni il puntatore al sensore della camera per configurazioni avanzate
  sensor_t *s = esp_camera_sensor_get();
  // Alcuni sensori hanno colori invertiti e saturazione alta: correggi
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // Ribalta l'immagine verticalmente
    s->set_brightness(s, 1);   // Aumenta leggermente la luminosità
    s->set_saturation(s, -2);  // Riduci la saturazione dei colori
  }
  // Riduci la risoluzione per aumentare il frame rate iniziale
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA); // 320x240
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  // Se presente, configura il LED flash
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  // ===========================
  // Connessione alla rete WiFi
  // ===========================
  WiFi.begin(ssid, password);    // Avvia la connessione WiFi
  WiFi.setSleep(false);          // Disabilita la modalità sleep per evitare disconnessioni

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // ===========================
  // Avvia il server web della camera
  // ===========================
  startCameraServer();

  Serial.print("Camera Ready! Usa 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' per connetterti");
}

// ===========================
// LOOP: viene eseguito continuamente dopo il setup
// ===========================
void loop() {
  // Controlla se è stato inviato un comando via seriale
/*  if (Serial.available()) {
    char comando = Serial.read(); // Legge il carattere inviato

    if (comando == 'G') {  // GO: riavvia completamente il dispositivo ESP32
      Serial.println("Riavvio completo ESP32...");
      delay(100);  
      ESP.restart();    // Comando di riavvio dell’ESP32
    }
    
    else if (comando == 'S') { // STOP: spegne la camera se è accesa
      if (cameraActive) {
        Serial.println("Arresto camera...");
        err = esp_camera_deinit();
        if (err == ESP_OK) {
          cameraActive = false;
          Serial.println("Camera arrestata con successo.");
        } else {
          Serial.printf("Errore nello stop della camera: 0x%x\n", err);
        }
      } else {
        Serial.println("La camera è già disattivata.");
      }
    }

    else if (comando == 'E') { // ERRORE: mostra l'ultimo codice errore
      Serial.printf("Ultimo codice errore: 0x%x\n", err);
    }

    else {
      Serial.println("Comando non riconosciuto. Usa G (Go), S (Stop), E (Errore)");
    }
  }  */

  delay(100); // Attende 100ms per evitare letture multiple ravvicinate (debounce)
}
