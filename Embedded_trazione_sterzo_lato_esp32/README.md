# Embedded_trazione_sterzo_lato_esp32

# Come visualizzare la Cam sul server tramite l'utlizzo del wi-fi

Per prima cosa su Arduino_ide è stato scaricato il pacchetto Esp32_cam versione 3.2.0

Successivamente sono stati aggiunti i seguenti contenuti URLs: 
http://arduino.esp8266.com/stable/package_esp8266com_index.json
https://dl.espressif.com/dl/package_esp32_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

questi pacchetti servono a dire all'IDE dove trovare e scaricare i pacchetti necessari per programmare le schede ESP8266 e ESP32.

Nel file CameraWebServer:
1. é importante selezionare solamente il seguente modello di Camera: 
   #define CAMERA_MODEL_AI_THINKER // Has PSRAM
2. selezionare il wi-fi e la propria password

Dopo Aver attaccato ESP32-CAM al computer tramite cavo USB e aver selezionato la porta relativa corretta 
(COM 4 o COM 5), si può procedere a caricare lo sketch per poter visualizzare la cam sul server.
Come Board Manager va impostato "AI Thinker ESP32-CAM".

IMPORTANTE!!!!
Prima di caricare lo sketch bisogna seguire i seguenti passi:
1. tenere premuto IO0 sulla ESP32-CAM-MB
2. inseme a IO0 premere il tasto RST(reset) sulla ESP32-CAM e poi rilasciarlo 
3. Rilasciare IO0

A questo punto sul Serial Monitor si dovrebbe tenere il seguente risultato: 
WiFi connected
Camera Ready! Use 'http://192.168.1.105' to connect

Se per caso il wi-fi non viene rilevato, o escono dei problemi, bisogna ripremere il tasto RST 
della ESP32-CAM e il problema dovrebbe risolversi.

Infine bisogna copiare il link URL e incollarlo sul server web. A quel punto si apre un interfaccia dove 
si può visualizzare la Camera.



