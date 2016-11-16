
#include <DHT.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_NRF24.h>
test


//Constants
#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define RAIN_GAUGE_PIN 2 //Rain gauge on pin 2
#define RAIN_GAUGE_INT 0 //Interrupt 0 tied to D2
#define windPin 3
#define wind_int 1 //Interrupt 1 tied to D3
#define MSECS_CALC_RAIN_FALL  50000
#define ulong unsigned long
//#define RAIN_FACTOR 0.2794

DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
float val = 0;                    // variable for reading the pin status

int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value
float temperature; //Float for Fahrenheit
float pressure_TS; //Float for BMP180 val



// Defines
#define SPEED_UPDATE_PERIOD  60000
#define DIRECTION_UPDATE_PERIOD 60000
#define UI_UPDATE_PERIOD 60000

// Variables
volatile uint16_t numRevs = 0;
uint8_t windOffset = 0;
uint32_t time, updateSpeed, updateDirection, updateUI;
String direction = "ERROR IN ADC!";
float speed;
// dirOffset is offset in degrees.
//uint16_t dirOffset = 0;
float anemometerFixFactor = 0.099;
//int windPin = 2;
ulong nextCalcRain; 
volatile int numBuckettip = 0;

// Implement all 16 directions anyways, because they can be triggered.  Likelihood is low, but better to have accuracy
//const uint16_t DIR_LUT[16] = { 79, 138, 198, 240, 324, 397, 428, 567, 623, 744, 787, 847, 906, 940, 949, 967 };
// Mapping between DIR_LUT and cardinal directions for offset
//const uint8_t DIR_OFF_LUT[16] = { 12, 14, 13, 0, 15, 10, 11, 2, 1, 8, 9, 6, 7, 4, 3, 5 };
//const String CARD_LUT[16] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };

//NRF24 setup
// Singleton instance of the radio driver
RH_NRF24 nrf24;
// RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini


typedef struct package
{
  int32_t temperature;
  int32_t humidity;
  int32_t pressure_TS;
  int32_t speed;
  int32_t vane;
  int32_t numBuckettip;
};

//typedef struct package Package;
package data;

//End NRF24 setup

//BMP180 Setup
SFE_BMP180 pressure;
#define ALTITUDE 179.0 //Altitude for my house
double T, P, p0, a;


void countAnemometer() {
  numRevs++;
}

void setup() {
  Serial.begin(115200);
  //setup_wifi();
  dht.begin(); //Setting up DHT22
  //Setting up NRF24 module
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(6))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  delay(1500);
  pinMode(windPin, INPUT); //Setting up wind sensors
  digitalWrite(windPin, HIGH);
  attachInterrupt(wind_int, countAnemometer, FALLING);
  pinMode(RAIN_GAUGE_PIN, INPUT);
  digitalWrite(RAIN_GAUGE_PIN, HIGH);
  attachInterrupt(RAIN_GAUGE_INT, countRainmeter, FALLING);
  updateSpeed = millis() + SPEED_UPDATE_PERIOD;
  updateDirection = millis() + DIRECTION_UPDATE_PERIOD;
  updateUI = millis() + UI_UPDATE_PERIOD;

  if (pressure.begin()) //Setting up BMP pressure sensor
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while (1); // Pause forever.
  }
}


void publish_temp() {
  hum = dht.readHumidity();
  temp = dht.readTemperature();
  temperature = (temp * 9.0) / 5.0 + 32.0;
  pressure_TS = (p0*0.0295333727);
  Serial.print("DHT22 Temperature: ");
  Serial.print(temperature);
  Serial.println("F");
  Serial.print("DHT22 Humidity: ");
  Serial.print(hum);
  Serial.println("%");
}

void push_nrf24() {
  Serial.println("Sending to nrf24_server");
  // Send a message to nrf24_server
  //uint8_t data[] = "Hello World!";
  data.temperature = temperature * 100;
  data.humidity = hum * 100;
  data.pressure_TS = pressure_TS * 100;
  data.speed = speed * 100;
  data.vane = analogRead(A0);
  data.numBuckettip = numBuckettip;
  //Debug info
  Serial.println("Debug data");
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(hum);
  Serial.print("Pressure :");
  Serial.println(pressure_TS);
  Serial.print("Wind speed: ");
  Serial.println(speed);
  Serial.print("Bucket tips: ");
  Serial.println(numBuckettip);
  
  nrf24.send((uint8_t*)&data, sizeof(data));

  nrf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (nrf24.waitAvailableTimeout(500))
  {
    // Should be a reply message for us now   
    if (nrf24.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is nrf24_server running?");
  }

}

void BMP180() {
  // Begin BMP180 stuff
  char status;
// double T,P,p0,a; moved to top of file

// Loop here getting pressure readings every 10 seconds.

// If you want sea-level-compensated pressure, as used in weather reports,
// you will need to know the altitude at which your measurements are taken.
// We're using a constant called ALTITUDE in this sketch:

Serial.println();
Serial.print("provided altitude: ");
Serial.print(ALTITUDE, 0);
Serial.print(" meters, ");
Serial.print(ALTITUDE*3.28084, 0);
Serial.println(" feet");

// If you want to measure altitude, and not pressure, you will instead need
// to provide a known baseline pressure. This is shown at the end of the sketch.

// You must first get a temperature measurement to perform a pressure reading.

// Start a temperature measurement:
// If request is successful, the number of ms to wait is returned.
// If request is unsuccessful, 0 is returned.

status = pressure.startTemperature();
if (status != 0)
{
  // Wait for the measurement to complete:
  delay(status);

  // Retrieve the completed temperature measurement:
  // Note that the measurement is stored in the variable T.
  // Function returns 1 if successful, 0 if failure.

  status = pressure.getTemperature(T);
  if (status != 0)
  {
    // Print out the measurement:
    Serial.print("temperature: ");
    Serial.print(T, 2);
    Serial.print(" deg C, ");
    Serial.print((9.0 / 5.0)*T + 32.0, 2);
    Serial.println(" deg F");

    // Start a pressure measurement:
    // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    status = pressure.startPressure(3);
    if (status != 0)
    {
      // Wait for the measurement to complete:
      delay(status);

      // Retrieve the completed pressure measurement:
      // Note that the measurement is stored in the variable P.
      // Note also that the function requires the previous temperature measurement (T).
      // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
      // Function returns 1 if successful, 0 if failure.

      status = pressure.getPressure(P, T);
      if (status != 0)
      {
        // Print out the measurement:
        Serial.print("absolute pressure: ");
        Serial.print(P, 2);
        Serial.print(" mb, ");
        Serial.print(P*0.0295333727, 2);
        Serial.println(" inHg");

        // The pressure sensor returns abolute pressure, which varies with altitude.
        // To remove the effects of altitude, use the sealevel function and your current altitude.
        // This number is commonly used in weather reports.
        // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
        // Result: p0 = sea-level compensated pressure in mb

        p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
        Serial.print("relative (sea-level) pressure: ");
        Serial.print(p0, 2);
        Serial.print(" mb, ");
        Serial.print(p0*0.0295333727, 2);
        Serial.println(" inHg");

        // On the other hand, if you want to determine your altitude from the pressure reading,
        // use the altitude function along with a baseline pressure (sea-level or other).
        // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
        // Result: a = altitude in m.

        a = pressure.altitude(P, p0);
        Serial.print("computed altitude: ");
        Serial.print(a, 0);
        Serial.print(" meters, ");
        Serial.print(a*3.28084, 0);
        Serial.println(" feet");
      }
      else Serial.println("error retrieving pressure measurement\n");
    }
    else Serial.println("error starting pressure measurement\n");
  }
  else Serial.println("error retrieving temperature measurement\n");
}
else Serial.println("error starting temperature measurement\n");


}

void loop() {
  //Anemometer + wind vane code from e4_weather
  time = millis();
  /*
  if (time >= updateDirection) {
    for (int i = 0; i < 16; i++) {
      if (DIR_LUT[i] >= analogRead(A0)) {
        direction = CARD_LUT[DIR_OFF_LUT[(i + (int)(dirOffset / 22.5)) % 16]];
        break;
      }
    }
    updateDirection += DIRECTION_UPDATE_PERIOD;
  }
 */
 
  if (time >= updateSpeed) {
    speed = 4793.333 / SPEED_UPDATE_PERIOD * numRevs * anemometerFixFactor;
    numRevs = 0;
    updateSpeed += SPEED_UPDATE_PERIOD;
  }
 
//  if (time >= nextCalcRain) {
//    getUnitRain();
//    nextCalcRain = time + MSECS_CALC_RAIN_FALL;
//  }
  
  if (time >= updateUI) {
    Serial.print("Direction: ");
    Serial.print(direction);
    Serial.print(" degrees\tSpeed: ");
    Serial.print(speed);
    Serial.println("MPH");
    BMP180();
    publish_temp();
    push_nrf24();
    updateUI += UI_UPDATE_PERIOD;
    numBuckettip = 0; //Clear out bucket tips at end of update
  }

}
void countRainmeter()
{
  static unsigned long last_millis = 0;
  unsigned long m = millis();
  if (m - last_millis < 200) {
    // ignore interrupt: probably a bounce problem
  } else {
    numBuckettip++;
    Serial.print(numBuckettip); //Check and print the count of the rain gauge
  }
  last_millis = m;
}

