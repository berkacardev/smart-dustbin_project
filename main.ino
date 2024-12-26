#include <DHT.h>
#include <HX711_ADC.h>

#define TRIG_PIN 25  
#define ECHO_PIN 26  

#define DHT_PIN 14
#define DHT_TYPE DHT11

#define DOUT  4 
#define CLK   5 

#define MQ4_AO 33
#define BUZZER 23
#define WARNING_LED 21

DHT dht(DHT_PIN, DHT_TYPE);

HX711_ADC LoadCell(DOUT, CLK);

void setup() {
  Serial.begin(9600); 
  //LoadCell.begin();
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT); 
  pinMode(BUZZER,OUTPUT);
  pinMode(WARNING_LED,OUTPUT);
}

void loop() {
  //processOfDistance();
  //processOfHeatAndHumidity();
  //processOfWeight();
  processOfGas();
  delay(250);
}

void processOfDistance(){
  long duration; 
  float distance; 
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Uzaklık: ");
  Serial.print(distance);
  Serial.println("Cm");
}

void processOfHeatAndHumidity(){
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  Serial.print("Sıcaklık: ");
  if (isnan(humidity) || isnan(temperature) || humidity > 100) {
    Serial.println("DHT11 okuma hatası!");
  } else {
    Serial.print("Sıcaklık: ");
    Serial.print(temperature);
    Serial.print(" °C");
    Serial.print("  Nem: ");
    Serial.print(humidity);
    Serial.println(" %");
  }
}

void processOfWeight(){
  float weight = LoadCell.getData();
  Serial.print("Ağırlık: ");
  Serial.print(weight); 
  Serial.println(" gram");

}

void processOfGas(){
  int measurmentForGas = analogRead(MQ4_AO);
  int dangerGasDensityReferancePoint = 2000;
  Serial.println(measurmentForGas);
  if(measurmentForGas>dangerGasDensityReferancePoint){
    processOfWarningSystem();
  }
}
void processOfWarningSystem(){
  digitalWrite(BUZZER,HIGH);
  digitalWrite(WARNING_LED,HIGH);
  delay(250);
  digitalWrite(BUZZER,LOW);
  digitalWrite(WARNING_LED,LOW);
  delay(300);
}
