# ESP_Thermal_Camera_WebServer

Welcome to see the micro project based on ESP32. <br>
Thermal Camera based on: <br>
1) ESP32 (Wrover)
2) MLX 90640 Thermal Camera 32x24 pixels
3) 0.95' OLED (SD1331)
4) ESPAsyncWebServer
5) SPIFFS - file system 
<br>

Important features - TO DO LIST:
- [x] Grabbing thermal image from MLX90640
- [x] Test image on OLED
- [x] Show thermal image on OLED
- [x] Build WebServer with ESPAsyncWebServer
- [x] Automatic update of variables (e.g. MaxTemp) on Website
- [x] Save thermal image as picture (BMP) in SPIFFS
- [x] Show Thermal Image (BMP) on Website
- [x] Automatic update of BMP (suitable SetInterval in <script>)
- [x] Case - 3D Model and print it on FDM 3D Printer
- [ ] Tweak updating Thermal Image (now SetInterval set to 1 sec - buggy image if set <1sec or there is more Clients)
- [ ] Reducing overall latency (Issue #2)
- [ ] Maybe Stream thermal image to Website (because why not) - faster updating
- [ ] Add Switch on Website to On/Off OLED display


Note: 10/10/2019<br>
I have MLX90640 files that are 6 months old. The latest files from Melexis contain a bug in the MLX90640_API.cpp file.
