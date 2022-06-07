#include <Arduino.h>

// Include libraries for DS18B20 temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to port 2
#define ONE_WIRE_BUS 6

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress gsTemperatureSensor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  sensors.begin();

  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if (!sensors.getAddress(gsTemperatureSensor, 0)) Serial.println("Unable to find address for Device 0");

  sensors.setResolution(gsTemperatureSensor, 9);
}

void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
}

void loop() {
  // put your main code here, to run repeatedly: 
  Serial.print("\nRequesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  printTemperature(gsTemperatureSensor); // Use a simple function to print out the data
  delay(1000);
}