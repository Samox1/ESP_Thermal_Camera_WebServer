/*
Thermal Image Camera - Project using ESP8266 or ESP32 and MLX90640 sensor (32x24 px)
Design with 0.95' OLED (SD1331) and WebServer to see images (with interpolation) on any other device
Project based on MLX data sheet and examples
Author: Szymon Baczyński
Date: April 2019
Version: V1 
*/


#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include "index.h"                //Our HTML webpage contents with javascripts
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

float mlx90640To[768];
paramsMLX90640 mlx90640;

// You can use any (4 or) 5 pins
#define sclk 18
#define mosi 23
#define cs   17
#define rst  5
#define dc   16


// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//the colors we will be using
const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };

WebServer server(80);

//Enter your SSID and PASSWORD
const char* ssid = "ESP";
const char* password = "camera";

float p = 3.1415926;
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);

float MaxTemp = 0;
float MinTemp = 0;
float CenterTemp = 0;

// SETUP
//==========================================================================

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  Serial.begin(115200);while (!Serial); //Wait for user to open terminal

  //ESP32 As access point
  WiFi.mode(WIFI_AP); //Access Point mode
  WiFi.softAP(ssid, password);
  
  Serial.print("SDA pin: "); Serial.println(SDA);
  Serial.print("SCL pin: ");Serial.println(SCL);
  
  Serial.println("MLX90640 IR Array Example");

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status=0;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");
  
  int SetRefreshRate;
  SetRefreshRate = MLX90640_SetRefreshRate(0x33,0x03);

// --- Part Display OLED --- //

  display.begin();

  Serial.println("init");
  uint16_t time = millis();
  display.fillScreen(BLACK);
  time = millis() - time;
  Serial.println(time, DEC);
  delay(500);


  display.fillScreen(BLACK);
  display.setCursor(0,0);
  display.print("Welcome!\nThis is example of Thermal Image Camera based on MLX90640 sensor.\n by SamoX");
  delay(2000);
  display.fillScreen(BLACK);
  lcdTestThermalImage();
  delay(1000);
  //display.fillScreen(BLACK);
  //Once params are extracted, we can release eeMLX90640 array

// --- Part WebServer ESP --- //
  
  IPAddress ServerIP = WiFi.softAPIP(); // Obtain the IP of the Serve
  Serial.print("IP address: ");
  Serial.println(ServerIP);   //IP address assigned to your ESP
  display.setCursor(0,49);
  display.print(ServerIP);    // IP address on Display
//----------------------------------------------------------------
 
  server.on("/", handleRoot);      //This is display page
  server.on("/readADC", handleADC);//To get update of ADC Value only
 
  server.begin();                  //Start server
  Serial.println("HTTP server started");
}


// LOOP
//===========================================================================

void loop()
{
  server.handleClient();

  
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }
  
    // Calculate difference between Subpages (chess-mode)
/*    int pa = 0;
    int niepa = 0;
    float sumpa = 0;
    float sumniepa = 0;

    int w = 32;
    int h = 24;

    for(int i=0; i<h; i++) {
      for(int j=0; j<w; j++) {
        if((i+j)%2 == 0){
          //Serial.println(j+(w*i));
          sumpa = mlx90640To[j+(w*i)];
          pa++;
        }else{
          //Serial.print("*"); Serial.println(j+(w*i));
          sumniepa = mlx90640To[j+(w*i)];
          niepa++;
        }
      }
    }

    sumpa = sumpa / (float)pa;
    sumniepa = sumniepa / (float)niepa;
    float diff = sumpa - sumniepa;          // Difference between even and odd 
    
    if(diff < 0.0){
      for(int i=0; i<h; i++) {
        for(int j=0; j<w; j++) {
          if((i+j)%2 == 0){
            mlx90640To[j+(w*i)] += abs(diff); 
          }else{
            //mlx90640To[j+(w*i)] += abs(diff);
          }
        }
      }
    }else{
       for(int i=0; i<h; i++) {
        for(int j=0; j<w; j++) {
          if((i+j)%2 == 0){
            //mlx90640To[j+(w*i)] += abs(diff); 
          }else{
            mlx90640To[j+(w*i)] += abs(diff);
          }
        }
      }
    }
*/
    //float MaxTemp = 0;                // Variables as global
    //float MinTemp = 0;
    CenterTemp = (mlx90640To[165]+mlx90640To[180]+mlx90640To[176]+mlx90640To[192]) / 4.0;  // Temp in Center - based on 4 pixels

    MaxTemp = mlx90640To[0];            // Get first data to find Max and Min Temperature
    MinTemp = mlx90640To[0];
    
    for (int x = 0 ; x < 768 ; x++)     // Find Maximum and Minimum Temperature
    {
      if (mlx90640To[x] > MaxTemp){
        MaxTemp = mlx90640To[x];
      }
      if (mlx90640To[x] < MinTemp){
        MinTemp = mlx90640To[x];
      }
    }

    //MaxTemp = 40.0;
    //MinTemp = 0.0;
    
    //display.fillRect(0, 0, 96, 48, BLACK);    // Black important sector - image and text on right side
    lcdThermalImage(mlx90640To, MinTemp, MaxTemp);    // Function to draw Thermal Image on OLED 

    display.fillRect(66, 0, 30, 48, BLACK);     // Black only text with Max, Center and Min temperature
    
    display.setCursor(66,0);                    // Text with Max, Center and Min Temperature on right side
    display.setTextColor(RED);
    display.print(MaxTemp,2);

    display.setCursor(66,18);
    display.setTextColor(WHITE);
    display.print(CenterTemp,2);
    
    display.setCursor(66,36);
    display.setTextColor(BLUE);
    display.print(MinTemp,2);
  

  //MLX_to_Serial(mlx90640To);

  delay(100);
  //display.fillScreen(BLACK);
  
  }

// ----------- FUNCTION ----- FUNCTION ----- FUNCTION ----- //

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
 String s = MAIN_page; //Read HTML contents
 server.send(200, "text/html", s); //Send web page
}

void handleADC() {
  extern float CenterTemp;

  server.send(200, "text/plane", String(CenterTemp)); //Send ADC value only to client ajax request
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}


void ThermalImageToWeb(float mlx90640To[], float MinTemp, float MaxTemp)
{
  const char *filename = "thermal.pgm";
  uint8_t w,h;
  uint8_t x = 32;
  uint8_t y = 24;
  unsigned char pic[y][x];
  const int MaxColorComponentValue = 255;
  const char *comment = "# this is my new binary pgm file";
  FILE * fp;
  
  for (h = 0; h < y; h++) {
    for (w = 0; w < x; w++) {
      uint8_t colorIndex = map(mlx90640To[w+(x*h)], MinTemp-5.0, MaxTemp+5.0, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      pic[y][x] = colorIndex;
    }  
  }
  
  fp = fopen(filename, "wb");
  fprintf(fp, "P5\n %s\n %d\n %d\n %d\n", comment, x, y, MaxColorComponentValue);   /* write header to the file */
  fwrite(pic, sizeof(pic), 1, fp);   /* write image data bytes to the file */
  fclose(fp);
}


void lcdThermalImage(float mlx90640To[], float MinTemp, float MaxTemp)
{
  uint8_t w,h;
  uint8_t box = 2;
  display.setAddrWindow(0, 0, 96, 64);
  

  for (h = 0; h < 24; h++) {
    for (w = 0; w < 32; w++) {
      uint8_t colorIndex = map(mlx90640To[w+(32*h)], MinTemp-5.0, MaxTemp+5.0, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      
      display.fillRect(box * w, box * h, box, box, camColors[colorIndex]);
      //display.writePixel(w, h, camColors[colorIndex]);
    }  
  }
  display.endWrite();
}


void lcdTestThermalImage(void)
{
  uint8_t w,h;
  display.setAddrWindow(0, 0, 96, 64);

  for (h = 0; h < 48; h++) {
    for (w = 0; w < 64; w++) {
      if (h*w <= 255) {
        display.writePixel(w, h, camColors[h*w]);
      } else{
        display.writePixel(w, h, RED);
      }
    }
  }
  display.endWrite();
}

void MLX_to_Serial(float mlx90640To[])
{
  for (int x = 0 ; x < 768 ; x++)
  {
    //Serial.print("Pixel ");
    Serial.print(x);
    Serial.print(": ");
    Serial.print(mlx90640To[x], 2);
    //Serial.print("C");
    Serial.println();
  }
}

