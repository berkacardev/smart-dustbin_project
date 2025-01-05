#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <HX711_ADC.h>

#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#define TRIG_PIN 25  
#define ECHO_PIN 26  
#define DHT_PIN 14
#define DHT_TYPE DHT11
#define MQ4_AO 33
#define BUZZER_LED 23
#define MOTOR 4
#define SDA_PIN 16
#define SCL_PIN 17


#define MAX_SIZE 30    
#define THRESHOLD 50
#define REQUIRED_COUNT 15 

const int HX711_dout = 18;
const int HX711_sck = 19;
const int calVal_eepromAdress = 0;
unsigned long t = 0;
         
float DISTANCE = 0;
float GAS = 0;
float WEIGHT = 0;
float TEMPREATURE = 0;
float HUMIDITY = 0;

float REAL_DISTANCE = 0;
float REAL_GAS = 0;
float REAL_WEIGHT = 0;
float REAL_TEMPREATURE = 0;
float REAL_HUMIDITY = 0;

HX711_ADC LoadCell(HX711_dout, HX711_sck);
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4); 

void setup() {
  Serial.begin(9600); 
  _lcdInit();
  _pinInit();
  _loadCellInit();
}

void loop() {
  processOfFullness();
  processOfTemperatureAndHumidity();
  processOfGas();
  String serailInfo = "F:";
  serailInfo+= "Mesafe(Cm): ";
  serailInfo+= DISTANCE;
  serailInfo+=" Sıcaklık(C): ";
  serailInfo+=TEMPREATURE;
  serailInfo+=" Nem(%): ";
  serailInfo+=HUMIDITY;
  serailInfo+=" Gaz: ";
  serailInfo+=GAS;
  serailInfo+=" Agirlik: ";
  serailInfo+=WEIGHT;
  Serial.println(serailInfo);

  String serailInfoForReal = "R:";
  serailInfoForReal+= "Mesafe(Cm): ";
  serailInfoForReal+= REAL_DISTANCE;
  serailInfoForReal+=" Sıcaklık(C): ";
  serailInfoForReal+=REAL_TEMPREATURE;
  serailInfoForReal+=" Nem(%): ";
  serailInfoForReal+=REAL_HUMIDITY;
  serailInfoForReal+=" Gaz: ";
  serailInfoForReal+=REAL_GAS;
  serailInfoForReal+=" Agirlik: ";
  serailInfoForReal+=REAL_WEIGHT;
  Serial.println(serailInfoForReal);
}
void processOfFullness(){
  processOfDistance();
  processOfWeight();
  float distanceFullnessAsPercent = (int)(100-(DISTANCE/60)*100);
  float weightFullnessAsPercent = (int)((WEIGHT/400)*100);
  String strFullness = "DM:";
  strFullness+=distanceFullnessAsPercent;
  strFullness+=" DA: ";
  strFullness+=weightFullnessAsPercent;
  lcd.setCursor(0,0);
  lcd.print(strFullness);

  if(_filterDistanceData(distanceFullnessAsPercent) || _filterWeightData(weightFullnessAsPercent)){
        processOfMotor();
  }
}
void processOfDistance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = kalmanFilter(duration * 0.034 / 2);
  REAL_DISTANCE = distance;
  if(distance<0){
    DISTANCE =0;
  }
  else if(distance>60){
    DISTANCE = 60;
  }
  if(distance>0 && distance<60){
        DISTANCE = distance;
  }

}

void processOfWeight(){
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; 
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData()*3;
      REAL_WEIGHT = i;
      if(i<0){
        i = 0;
      }
      else if(i>1000){
        i=1000;
      }
      WEIGHT = i;
      newDataReady = 0;
      t = millis();
    }
  }
}

void processOfMotor(){
  digitalWrite(MOTOR,HIGH);
  delay(5000);
  digitalWrite(MOTOR,LOW);
}
void processOfTemperatureAndHumidity(){
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  humidity = kalmanFilter(humidity);
  temperature = kalmanFilter(temperature);
  if(isnan(humidity)){
    humidity = 0;
  }
  if(isnan(temperature)){
    temperature = 0;
  }
  _filterTemperatureAndHumidity(temperature,humidity);
  String strTempreature = "Sicaklik:";
  strTempreature+=TEMPREATURE;
  lcd.setCursor(0,1);
  lcd.print(strTempreature);
  String strHumidity = "Nem(%):";
  strHumidity+=HUMIDITY;
  lcd.setCursor(0,2);
  lcd.print(strHumidity);
}


void processOfGas(){
  int measurmentForGas = analogRead(MQ4_AO);
  int dangerGasDensityReferancePoint = 600;
  REAL_GAS = measurmentForGas;
  GAS = kalmanFilter(measurmentForGas);
  if(measurmentForGas>dangerGasDensityReferancePoint){
    processOfWarningSystem();
  }
  String strGas = "Gaz:";
  strGas+=GAS;
  lcd.setCursor(0,3);
  lcd.print(strGas);
  delay(500);
}
void processOfWarningSystem(){
  digitalWrite(BUZZER_LED,HIGH);
  delay(250);
  digitalWrite(BUZZER_LED,LOW);
  delay(300);
}

float distanceData[MAX_SIZE];
int currentIndexForDıstanceData = 0;         
int countForDıstanceData = 0;      
bool _filterDistanceData(float _distance) {
  distanceData[currentIndexForDıstanceData] = _distance;
  currentIndexForDıstanceData = (currentIndexForDıstanceData + 1) % MAX_SIZE;
  if (countForDıstanceData < MAX_SIZE) {
    countForDıstanceData++;
  }
  if (countForDıstanceData >= MAX_SIZE) {
    int belowThresholdCount = 0;
    for (int i = 0; i < countForDıstanceData; i++) {
      if (distanceData[i] > THRESHOLD) {
        belowThresholdCount++;
      }
    }
    if (belowThresholdCount >= REQUIRED_COUNT) {
      countForDıstanceData = 0;
      currentIndexForDıstanceData = 0;
      return true;
    }
    countForDıstanceData = 0;
    currentIndexForDıstanceData = 0;
  }
  return false;
}


float weightData[MAX_SIZE];
int currentIndexForWeightData = 0;         
int countForWeightData = 0;      
bool _filterWeightData(float _weight) {
  weightData[currentIndexForWeightData] = _weight;
  currentIndexForWeightData = (currentIndexForWeightData + 1) % MAX_SIZE; // Döngüel olarak diziyi güncelle
  if (countForWeightData < MAX_SIZE) {
    countForWeightData++;
  }
  if (countForWeightData >= MAX_SIZE) {
    int belowThresholdCount = 0;
    for (int i = 0; i < countForWeightData; i++) {
      if (weightData[i] > THRESHOLD) {
        belowThresholdCount++;
      }
    }
    if (belowThresholdCount >= REQUIRED_COUNT) {
      countForWeightData = 0; // Dizi sıfırlanır
      currentIndexForWeightData = 0;
      return true;
    }
    countForWeightData = 0;
    currentIndexForWeightData = 0;
  }
  return false;
}

float temperaturePastData = 0;
float temperatureNowData = 0;
float humidityPastData = 0;
float humidityNowData = 0;
void _filterTemperatureAndHumidity(float temperature, float humidity) {
    if(temperaturePastData ==0 && temperatureNowData == 0){
      temperatureNowData = temperature;
    }
    else{
      temperaturePastData = temperatureNowData;
      temperatureNowData = temperature;
    }
    if(temperatureNowData<=0){
      temperatureNowData = -1*temperatureNowData;
    }
    if(temperatureNowData/temperaturePastData<1.2 || temperatureNowData/temperaturePastData>0.80){
        TEMPREATURE = temperatureNowData;
    }
    else{
        TEMPREATURE = temperaturePastData;
    }

  if(humidityPastData ==0 && humidityNowData == 0){
      humidityNowData = humidity;
    }
    else{
      humidityPastData = humidityNowData;
      humidityNowData = humidity;
    }
    if(humidityNowData<=0){
      humidityNowData = -1*humidityNowData;
    }
    if(humidityNowData/humidityPastData<1.2 || humidityNowData/humidityPastData>0.80){
        HUMIDITY = humidityNowData;
    }
    else{
        HUMIDITY = humidityPastData;
    }
}

void _loadCellInit(){
  LoadCell.begin();
  float calibrationValue;
  calibrationValue = 696.0;
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom
  unsigned long stabilizingtime = 2000; 
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); 
    Serial.println("Startup is complete");
  }
}
void _lcdInit(){
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();    
  lcd.backlight();
}
void _pinInit(){
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT); 
  pinMode(BUZZER_LED,OUTPUT);
  pinMode(MOTOR,OUTPUT);
}
float kalmanFilter(float measurement) {
  float Q = 0.01;
  float R = 0.1;
  float P = 1;
  float K = 0;
  float X = 0;
  float Z = 0;
  P = P + Q;
  K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;
  return X;
}