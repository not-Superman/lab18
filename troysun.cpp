#include "Adafruit_DHT_Particle.h"
#include "DS18B20.h"
#include "MQ135.h"
#include "cellular_hal.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(cellular_credentials_set("internet", "", "", NULL));
SerialLogHandler logHandler; // логирование через serial-port

#define KEEPALIVE_PERIOD 60 // задержка между проверками сети
bool hasSetKeepAlive = false; // TODO обход бага в прошивках 0.6.2 - 0.8.0-rc.3

#define TIME_SYNC_PERIOD (24 * 60 * 60 * 1000) // 24часа
unsigned long lastSync = millis(); // запись времени последней синхронизации

// AMP-B025 (влажность почвы) (3.3..5v 50mA)
//-------------------------------------------------------
#define MOISTURE_PIN A0 // пин для подключения сигнального
#define HUMIDITY_MIN 400 // мин. уровень влажности
bool amp_b025_enable = true; // замеры влажности почвы
int humGround; // влажность земли
//-------------------------------------------------------
// DHT //TODO
//-------------------------------------------------------
#define SensorPin A2 // pH
#define Offset -0  // калибровка pH 
#define samplingInterval 20
#define printInterval 800 // ?
#define ArrayLenth  40
#define DHTTYPE DHT11
#define DHTPIN RX
DHT dht(DHTPIN, DHTTYPE);
double ph;
int pHArray[ArrayLenth];
int pHArrayIndex;
static unsigned long printTime = millis();
static unsigned long samplingTime = millis();
static float pHValue, voltage2;
//-------------------------------------------------------
// DS18B20 (3.3..5v 1mA) (−55…+125 °C)
//-------------------------------------------------------
#define ONE_WIRE_PIN D4 // пин для подключения
#define DS18B20_PRECISION 10 // точность (9-12)
#define DS18B20_MAXRETRY 3 // кол-во попыток опроса датчика при неудачном предыдущем
#define nSENSORS 1 // общее кол-во датчиков
DS18B20 ds18b20(ONE_WIRE_PIN); // настройка DS18B20-библиотеки
bool ds18b20_enable = true; // замеры температуры почвы
uint8_t ds18b20Addr[nSENSORS][8]; // массив для хранеия адресов датчиков
// const uint8_t DS18B20_GND[8]; // имя датчика температуры почвы// TODO добавить имя
double tempGround; // температура почвы
//-------------------------------------------------------
// MQ135 (5v 150mA)
//-------------------------------------------------------
#define PIN_MQ135         B4 // сигнальный пин
#define PIN_MQ135_HEATER  B5 // нагреватель
MQ135 mq135(PIN_MQ135); // доступ к библиотеке
bool mq135_enable = true; // вкл/выкл замеров CO2 (ppm)
bool mq135_calibration_enable = false; // вкл/выкл калибровки
bool mq135_calibration_ok = true; // откалиброван ли датчик
unsigned long tBeginHeatMQ135; // время начала нагрева MQ135
#define MQ135_SAMPLES (30*60)/10 // кол-во записей для калибровки
unsigned int rzero_w; // кол-во имеющихся записей для калибровки
unsigned int rzero[MQ135_SAMPLES]; // массив для хранения калибровочных данных
double ppm; // концентрация в воздухе CO2
//-------------------------------------------------------
// LED
//-------------------------------------------------------
#define ALL_LED 3 // кол-во LED-лент для изменения яркости
#define PIN_RED     D1 // красный // TODO замена пинов для подключения
#define PIN_GREEN   D2 // зеленый
#define PIN_BLUE    D3 // синий
#define PIN_DEMO    TX // отдельная светодиодная лента
#define RED 0 //указатель в массиве //красный
#define GREEN 1 //указатель в массиве //зеленый
#define BLUE 2 //указатель в массиве //синий
int BRI_LEVEL[ALL_LED][24]; // двумерный массив [цвет ленты][час (0-23)
int BRI_RED;// красная
int BRI_GREEN; // зеленая
int BRI_BLUE; // синяя
int BRI_DEMO; // демонстрационная лента
//-------------------------------------------------------
// FAN
//-------------------------------------------------------
#define PIN_FAN A1 // пин управления вентилятором
#define FAN_MIN_SPEED 30 // минимальный уровень вращения вентилятора (в %)
#define FAN_MAX_SPEED 80 // максимальный уровень вращения вентилятора (в %)
byte fanSpeed = 0; // текущая скорость вращения
//-------------------------------------------------------
// DEBUG (управление удаленной отладкой)
//-------------------------------------------------------
bool debug = false; // включение debug-режима
#define DEBUG_MIN_DELAY 5 // минимальная задержка между отправкой debug-значений
unsigned int debugDelay = 0; // текущая задержка

//-------------------------------------------------------

// пин подключения насоса
#define PIN_PUMP    D5 // ?убираем в дальнейшем]

// ?насос
int avtoP = 1;
int avto = 1;

int water = 0;
double tempC = 0;
double vlaga = 0;
String now_time;

// включение определенных функций
//-------------------------------------------------
bool enabledPH = false; // замеры PH
bool enabledDEMO = false; // LED-demo 
//-------------------------------------------------

// функция подсчета среднего pH за интервал времени 
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    return 0;
  }
  if(number<5){
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }
    }
    avg = (double)amount/(number-2);
  }
  return avg;
}

// mq135_calibration()
// калибровка MQ135 (замеры сопротивления на свежем воздухе)
void mq135_calibration () {
    rzero[rzero_w] = (unsigned int)(mq135.getRZero() * 100);
    Serial.print("MQ135_RZERO_SAMPLE: "); // вывод в консоль
    Serial.print(rzero[rzero_w]);
    Serial.print(" (");
    Serial.print(rzero_w + 1); // отсчет в массиве идет с нуля,
    Serial.print(" of "); // а для сэмплов с единицы
    Serial.print(MQ135_SAMPLES);
    Serial.println(")");
    rzero_w++;
    
    if ( rzero_w == MQ135_SAMPLES ) { // накопилось достаточно материала
        unsigned long mq135_avg = 0;
        unsigned int f = 0;
        do {
            mq135_avg += rzero[f]; // складываем все замеры
            f++;
        } while (f < MQ135_SAMPLES);

        mq135_avg /= MQ135_SAMPLES; // делим на кол-во образцов
        mq135_calibration_ok = true; // калибровка произведена

        // получившимся (float) значением RZERO нужно заменить
        // то, что записано в библиотеке MQ135.h
        float mq135_RZERO = (float)mq135_avg / 100.0;
        Serial.print("MQ135_RZERO: "); // вывод в консоль
        Serial.println(mq135_RZERO);
        Particle.publish("MQ135_RZERO", String(mq135_RZERO), PRIVATE); // публикуем на сервер
    }
}

// DS18B20_getTemp
// получение температурных данных от датчиков DS18B20
double DS18B20_getTemp(uint8_t addr[8]) {
    int retry = 0;
    double _temp;
    do {
        _temp = ds18b20.getTemperature(addr); // запрашиваем данные с датчика
        if ( ds18b20.crcCheck() ) {
            break; // останавливаем цикл, т.к данные в целостности
        } else retry++; // нифига не целые, повторяем попытку
    } while (retry < DS18B20_MAXRETRY);
    
    if ( retry > DS18B20_MAXRETRY ) { // попытки израсходованы
        _temp = NAN;
        Serial.println("temp: N/A"); // ошибка при получении данных
    }
    return _temp;
}

// setBrightnessLED
// установка яроктси LED-лент по значениям из заданного массива
void setBrightnessLED () {
    // установка яровня яркости LED-лент
    for (int i = 0; i < ALL_LED; i++) { // для всех LED-лент
        for (int h = 0; h < 24; h++) { // для каждого часа
            if ( h != Time.hour() ) { // проверяем какой час
                continue; // не наш час, пропускаем
            } else { // оп-па, наш часик
                int CURRENT_PIN; // пин для регулировки
                int CURRENT_LED = i; // лента для регулировки
                
                // выбор пина для регулировки:
                if ( CURRENT_LED == RED ) CURRENT_PIN = PIN_RED;
                if ( CURRENT_LED == GREEN ) CURRENT_PIN = PIN_GREEN;
                if ( CURRENT_LED == BLUE ) CURRENT_PIN = PIN_BLUE;

                // значение для установки берем из массива
                analogWrite(CURRENT_PIN, BRI_LEVEL[CURRENT_LED][h]); // установка яркости
            }
        }
    }
}

// setDebugDelay()
// включаем debug для отправки значений с датчиков.
// переданное значение равняется таймингу между
// отправками данных в секундах
int setDebugDelay (String seconds) {
    unsigned int _timeout = seconds.toInt();
    unsigned int _minDelay = DEBUG_MIN_DELAY;
    if ( _timeout == 0 ) {
        Serial.println("debug:off");
        debugDelay = 0;
        debug = false;
        return 0;
    } else if ( _timeout < _minDelay ) {
        // меньше минимального возможного интервала
        Serial.println("debug:off (DEBUG_MIN_DELAY)");
        debug = false;
        return -1;
    } else if ( _timeout > 0 ) {
        Serial.print("debug:on (");
        Serial.print(_timeout);
        Serial.println("sec)");
        debugDelay = _timeout;
        debug = true;
        return _timeout;
    }
}

// setFanspeed()
// установка скорости вентилятора в процентах (0-100)
int setFanSpeed(String value) {
  unsigned int _value = value.toInt();
  uint8_t _valueToPin = 0;
  if ( _value < FAN_MIN_SPEED ) { _value = FAN_MIN_SPEED; }
  else if ( _value > FAN_MAX_SPEED ) { _value = FAN_MAX_SPEED; }

  // TODO добавить коэффициент расчетный
  _valueToPin = map(_value, 0, 100, 0, 255) * 1;
  analogWrite(PIN_FAN,_valueToPin);
  fanSpeed = _value;
  Serial.println(_valueToPin);
  Serial.print("Fan speed: ");
  Serial.print(_value);
  Serial.println("%");
  return _value;
}

// sendToServer()
// отправляем какую-то информацию на сервер
void sendToserver(String data, String value) {
  Serial.print("data = ");
  Serial.print(data);
  Serial.print(" | ");
  Serial.print("value = ");
  Serial.println(value);


}

// GetRSSI()
// получаем уровень сигнала оператора
int GetRSSI (String command) {
    if (Cellular.ready()) {
        CellularSignal sig = Cellular.RSSI(); // структура для хранения ответа
        Log.info("RSSI: %d", sig.rssi);
        return sig.rssi;
    }
}

int GetFreeMemory (String command) { // свободная память в уст-ве
    Log.info("Free mem: %lukB", System.freeMemory());
    return (uint32_t)System.freeMemory();
}

// changeBri() (цвет, значение)
// изменение яркости для определенной ленты
int changeBri (char color, int bri) {

  int CURRENT_BRI; // текущая яркость
  int CURRENT_PIN; // пин для регулировки
  int DELAY = 10; // регулировка плавности
  
  switch (color)
  {
    case 'R':
      CURRENT_BRI = BRI_RED;
      CURRENT_PIN = PIN_RED; break;
    case 'G':
      CURRENT_BRI = BRI_GREEN;
      CURRENT_PIN = PIN_GREEN; break;
    case 'B':
      CURRENT_BRI = BRI_BLUE;
      CURRENT_PIN = PIN_BLUE; break;
    case 'D':
      CURRENT_BRI = BRI_DEMO;
      CURRENT_PIN = PIN_DEMO; break;
  }

  if ( CURRENT_BRI < bri ) { // набор яркости
      for (int x = CURRENT_BRI; x <= bri; x++) {
          analogWrite(CURRENT_PIN,x);
          delay(DELAY);
      }
  } else if ( CURRENT_BRI > bri ) { // снижение яркости
      for (int x = CURRENT_BRI; x >= bri; x--) {
          analogWrite(CURRENT_PIN,x);
          delay(DELAY);
      }
  }
  if ( color != 'D' ) {
    Log.info("Set [%c] LED to %d", color, bri);
  }
  return bri;
}

int ledRed(String bri) {
    char color = 'R';
    BRI_RED = changeBri(color, bri.toInt());
    return BRI_RED;
}

int ledGreen(String bri) {
    char color = 'G';
    BRI_GREEN = changeBri(color, bri.toInt());
    return BRI_GREEN;
}

int ledBlue(String bri) {
    char color = 'B';
    BRI_BLUE = changeBri(color, bri.toInt());
    return BRI_BLUE;
}

int ledDemo(String bri) {
    char demo = 'D';
    enabledDEMO = true;
    Log.info("Start demo LED-mode");
    BRI_DEMO = changeBri(demo, bri.toInt());
    return BRI_DEMO;
}

int pump(String state) {
    if ( state.toInt() == 1 ) digitalWrite(PIN_PUMP, HIGH);
    else digitalWrite(PIN_PUMP, LOW);
    return state.toInt();
}
int avtoPump(String command) { // работа по расписанию насоса
    avtoP = command.toInt();
    return command.toInt();
}
int isAvto(String command) { // функция на вкл автомат вкл лент
    avto = command.toInt();
    return command.toInt();
}

int isVisual(String command){
    for (int a = 1; a <= 255; a++) {
         analogWrite(PIN_DEMO, a);
         delay(10);
       }
    for (int b = 1; b <= 255; b++) {
         analogWrite(PIN_DEMO, 255-b);
         delay(10);
      }
    return command.toInt();
}
int isVisual2(String command){ // для полного цикла демо для лент всех
  //  int z = command.toInt();
  //  for (int j=1; j <= z; j++){
  avto = 0;
  if (BRI_BLUE>=1){
        for(int l = BRI_BLUE; l >= 0; l--){
            analogWrite(PIN_BLUE,l);
            delay(10);
            BRI_BLUE--;
        }
  }
  if (BRI_GREEN>=1){
        for(int l = BRI_GREEN; l >= 0; l--){
            analogWrite(PIN_GREEN,l);
            delay(30);
            BRI_GREEN--;
        }
  }
  if (BRI_RED>=1){
        for(int l = BRI_RED; l >= 0; l--){
            analogWrite(PIN_RED,l);
            delay(10);
            BRI_RED--;
        }
  }
  if (BRI_RED<=254){
      for(int l = BRI_RED; l <=255; l++){
          analogWrite(PIN_RED,l);
          delay(10);
          BRI_RED++;
      }
  }
  if (BRI_GREEN<=101){
      for(int l = BRI_GREEN; l<=101; l++){
          analogWrite(PIN_GREEN,l);
          delay(50);
          BRI_GREEN++;
      }
  }
  if (BRI_BLUE<=254){
      for(int l = BRI_BLUE; l<=255; l++){
          analogWrite(PIN_BLUE,l);
          delay(10);
          BRI_BLUE++;
      }
  }
  if (BRI_BLUE>=1){
      for(int l = BRI_BLUE; l >= 0; l--){
          analogWrite(PIN_BLUE,l);
          delay(10);
          BRI_BLUE--;
      }
  }
  if (BRI_GREEN>=1){
      for(int l = BRI_GREEN; l>=0; l--){
          analogWrite(PIN_GREEN,l);
          delay(50);
          BRI_GREEN--;
      }
  }
  if (BRI_RED>=1){
      for(int l = BRI_RED; l>=0; l--){
          analogWrite(PIN_RED,l);
          delay(10);
          BRI_RED--;
      }
  }
   if (BRI_RED<=254){
      for(int l = BRI_RED; l <=255; l++){
          analogWrite(PIN_RED,l);
          delay(10);
          BRI_RED++;
      }
  }
  if (BRI_GREEN<=101){
      for(int l = BRI_GREEN; l<=101; l++){
          analogWrite(PIN_GREEN,l);
          delay(50);
          BRI_GREEN++;
      }
  }
  if (BRI_BLUE<=254){
      for(int l = BRI_BLUE; l<=255; l++){
          analogWrite(PIN_BLUE,l);
          delay(10);
          BRI_BLUE++;
      }
  }
    return command.toInt();
}

void setup()
{
    Serial.begin(9600);
  
    pinMode(PIN_RED, OUTPUT); // красная LED
    pinMode(PIN_GREEN, OUTPUT); // зеленая LED
    pinMode(PIN_BLUE, OUTPUT); // синяя LED
    pinMode(PIN_DEMO, OUTPUT); // demo-лента
    
    pinMode(PIN_PUMP, OUTPUT); // насос TODO(уберется)

    pinMode(PIN_FAN, OUTPUT); // управление вентилятором
    
    // установка яркости определенной ленты:
    Particle.function("setRed", ledRed); // TODO замена на одну функцию
    Particle.function("setGreen", ledGreen);
    Particle.function("setBlue", ledBlue);
    Particle.function("demotest", ledDemo);
    //

    Particle.function("debug", setDebugDelay); // включение debug-режима через облако
    Particle.function("setFanSpeed", setFanSpeed);
    Particle.function("rssi", GetRSSI);
    Particle.function("mem", GetFreeMemory);
  
  // установква уровня яркости в определенные часы
  // заполняем массивы для дальнейшего доступа
  for ( int h = 0; h < 24; h++ ) {
    if ( ( h >= 0 ) && ( h < 4 ) ) { // 00:00 - 03:59
          BRI_LEVEL[RED][h] = 0;
          BRI_LEVEL[GREEN][h] = 0;
          BRI_LEVEL[BLUE][h] = 0;
    }
    if ( ( h >= 4 ) && ( h < 5 ) ) { // 04:00 - 04:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 0;
    }
    if ( ( h >= 5 ) && ( h < 6 ) ) { // 05:00 - 05:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 128;
          BRI_LEVEL[BLUE][h] = 0;
      }
    if ( ( h >= 6 ) && ( h < 7 ) ) { // 06:00 - 06:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 0;
      }
    if ( ( h >= 7 ) && ( h < 9 ) ) { // 07:00 - 08:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 70;
      }
    if ( ( h >= 7 ) && ( h < 9 ) ) { // 09:00 - 10:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 128;
      }
    if ( ( h >= 11 ) && ( h < 15 ) ) { // 11:00 - 14:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 255;
      }
    if ( ( h >= 15 ) && ( h < 17 ) ) { // 15:00 - 16:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 128;
      }
    if ( ( h >= 17 ) && ( h < 19 ) ) { // 17:00 - 18:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 70;
      }
    if ( ( h >= 19 ) && ( h < 22 ) ) { // 19:00 - 21:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 0;
      }
    if ( ( h >= 22 ) && ( h < 23 ) ) { // 22:00 - 22:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 128;
          BRI_LEVEL[BLUE][h] = 0;
      }
    if ( h >= 23 ) { // 23:00 - 23:59
          BRI_LEVEL[RED][h] = 255;
          BRI_LEVEL[GREEN][h] = 255;
          BRI_LEVEL[BLUE][h] = 70;
      }
    }

    if ( ds18b20_enable == true ) {
        ds18b20.resetsearch(); // запускаем DS18B20
        for (int i = 0; i < nSENSORS; i++) {   // try to read the sensor addresses
            Serial.print("Locating DS18B20 devices...");
            ds18b20.search(ds18b20Addr[i]); // and if available store
            
            // Serial.print("Device address: "); //TODO
            // Serial.print(ds18b20Addr[i]); // выводим имя найденного уст-ва в консоль
            // Serial.println();
            
            // установим выбранное разрешение датчика
            ds18b20.setResolution(ds18b20Addr[i], DS18B20_PRECISION);
        }
        // создаем переменную для запроса  температуры почвы с сервера
        Particle.variable("tempGround", tempGround);
    }

    if (mq135_enable) {
        digitalWrite(PIN_MQ135_HEATER, HIGH); // включаем нагреватель MQ135
        tBeginHeatMQ135 = millis(); // время начала нагрева
        Particle.variable("ppm", ppm);
    }
    
    if (amp_b025_enable) { // влажность почвы
        Particle.variable("humGround", humGround);
    }
  
  Particle.variable("temp", tempC); 
  Particle.variable("vlaga", vlaga);
//   Particle.variablef("ph", ph);
  Particle.variable("time", now_time);
  Particle.variable("avto", avto);
  Particle.variable("water", water);

  pinMode(A2, INPUT);
  pinMode(A5, INPUT);
  pinMode(D0, OUTPUT);
  pinMode(RX, INPUT);

  Particle.function("pump",pump);
  Particle.function("Avtopump",avtoPump);
  Particle.function("avto",isAvto);
  Particle.function("isVisual",isVisual);
  Particle.function("isVisual2",isVisual2);
  
  dht.begin();
  
  Time.zone(+5); // часовой пояс
  lastSync = millis(); // запоминаем время первоначальной синхронизации
  Particle.connect(); // обязательно оставить
}

void loop()
{
    if (Particle.connected()) { // TODO обход бага в прошивках 0.6.2 - 0.8.0-rc.3
        if (!hasSetKeepAlive) {
            hasSetKeepAlive = true;
            Particle.keepAlive(KEEPALIVE_PERIOD); // период для проверки на "жив ли"
        }
    } else hasSetKeepAlive = false;

    if ( millis() - lastSync > TIME_SYNC_PERIOD ) { // синхронизация времени
        Particle.syncTime();
        lastSync = millis();
    }

    // получение температуры почвы
    if ( (millis() % (7 * 1000) ) < 10 ) { // каждые 7 секунд с точностью в 10мс
        tempGround = DS18B20_getTemp(ds18b20Addr[0]); //TODO добавить указатель на има датчика
    } 

    if ( (millis() % (10 * 1000) ) < 10 ) { // каждые 10 сек с точностью в 10мс
        Log.info("+10 sec");
        
        if ( enabledDEMO != true ) { // TODO ?в demo-режиме отключено
            setBrightnessLED(); // установка яркости LED-лент
        }
        if (mq135_enable) {
            // запускаем запись калибровочных данных для mq135
            if ( (mq135_calibration_enable == true) && (mq135_calibration_ok != true) ) {
                // if ( millis() - tBeginHeatMQ135 > ) // TODO калибровка только после прогрева?
                mq135_calibration();
            }
            // mq135 работает только после выполненной калибровки
            if ( mq135_calibration_ok == true) {
                ppm = mq135.getPPM(); // получаем данные по содержанию CO2
            }
        }
        if (amp_b025_enable) { // замеры влажности почвы
            humGround = analogRead(MOISTURE_PIN);
            Serial.print("AMPB025 moisture: ");
            Serial.println(humGround);
        }
    }
    
    // отправка показателей на облако
    if ( (millis() % (30 * 60 * 1000) ) < 10 ) { // каждые полчаса с точностью в 10мс
        Log.info("Send some values to server...");
        
        // ttl (time to live, 0–16777215 seconds, default 60) !!
        // NOTE: The user-specified ttl value is not yet implemented,
        // so changing this property will not currently have any impact.
        // For the time being there exists no way to access a previously published but TTL-unexpired event.
        // NOTE: Currently, a device can publish at rate of about 1 event/sec,
        // with bursts of up to 4 allowed in 1 second. Back to back burst of 4 messages will take 4 seconds to recover.
        if (Particle.connected()) {
           Particle.publish("tempGround", String(tempGround, 2), PRIVATE); // температура почвы
           Particle.publish("CO2(ppm)", String(ppm, 2), PRIVATE); // CO2 ppm, два знака после запятой
           Particle.publish("humGround", String(humGround), PRIVATE); // влажность почвы
           Particle.publish("tmpAir", String(tempC, 2), PRIVATE); // температура воздуха
           Particle.publish("water", String(water), PRIVATE);
           Particle.publish("pH", String(ph), PRIVATE); // pH почвы
        }
    }

    // вывод в консоль
    if ( (millis() % (60 * 1000) ) < 10 ) {
        if ( ds18b20_enable == true ) {
            Serial.print("DS18B20 tempGND: ");
            Serial.println(tempGround);
        }
        
        if ( (mq135_calibration_ok == true) && (mq135_enable == true) ) {
            Serial.print("MQ135 CO2: ");
            Serial.print(ppm);
            Serial.println("ppm");
        }
    }
    
    now_time = Time.format(Time.now(), "%H:%M:%S"); // ?убрать

    // debug-режим
    // включен ли debug, и прошло ли время между отправками:
    if ( (debug == true) && (millis() % (debugDelay * 1000) < 10) ) {
        if ( debugDelay < DEBUG_MIN_DELAY ) {
          Serial.println("debug:off (DEBUG_MIN_DELAY)"); // проверка на миним. значение
          debug = false;
        } else {
          // начинаем отправку данных с датчиков:
          sendToserver("tempGround", (String)tempGround);
          sendToserver("CO2(ppm)", (String)ppm);
          sendToserver("humGround", (String)humGround);
        }
    }

    if (avtoP == 1) {
      if ( (Time.hour() > 9) && (Time.minute() < 30) && (water == 1) ) { // в определенное время, при наличии воды
        digitalWrite(PIN_PUMP,HIGH);
      } else digitalWrite(PIN_PUMP,LOW);
    }
  
//   vlaga2 = (0.01*dht.getHumidity()*60+20); //Измеряем влажность // убрать из loop
//   tempC = dht.getTempCelcius();//Измеряем температуру 
//   vlaga = analogRead(A1)/40.95;

//   analogWrite(D0,255); // ?
//   if (analogRead(WKP) > 400 ){ // ?
//       water = 0;
//   }
//   if (analogRead(WKP) <= 400 ){
//       water = 1;
//   }

  if (enabledPH) {  // расчет pH
    if (millis() - samplingTime > samplingInterval) // не будет в дальнейшем (для расчетов сейчас)
    {
      pHArray[pHArrayIndex++] = analogRead(SensorPin);
      
      if (pHArrayIndex == ArrayLenth) {
        pHArrayIndex = 0;
        voltage2 = avergearray(pHArray, ArrayLenth) * 3.3 / 4095;
        pHValue = 5.58 * voltage2 + Offset;
        samplingTime = millis();
      }
      
      if (millis() - printTime > printInterval) // ?отправка показаний с определеннвм интервалом 
      {
        printTime = millis();
        ph = pHValue;
      }
    }
  }
}