# HTPA32x32dR2_ESP32
/* HTPA32x32 ESP32 (idf 4.4) example
 Unless required by applicable law or agreed to in writing, this software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.*/


This is a WIP app to capture thermal images from Heimann sensor (HTPA32x32dR2) using an esp32s3 device connected directly to the sensor and communicating via I2C. based on freeRTOS implementation

 features:
- eap32 wifi server (AP and Station modes are available) switching to AP can be done through a web page or esp configurations
- HTML page to display, capture and download thermal images
- an option to connect a reset button to ESP to reset wifi configuration default settings



HTTP API:
- "/api/get": trigger sensor capture command and return string of 1024 (32x32) comma seperated values (coverted to °C celsius)
- "/": return html page (home page) to capture sensor data or set esp32 wifi AP 


Important Configuration:
- make sure to select the correct device lookup table in sensordef_32x32.h 
- the default setting is using 8MB flash, however, this can be reduced if plotly-2.9.0.min.js (inhtml/index.html) is loaded from CDN (https://cdn.plot.ly/plotly-2.9.0.min.js)
- displayed thermal image at index.html is upside down, will be fixed later
- web capturing interval can be changed in function htpaCatpure, currently set to 3 seconds 