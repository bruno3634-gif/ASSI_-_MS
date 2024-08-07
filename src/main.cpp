#include <ESP32_CAN.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/queue.h>

#define QUEUE_LENGTH 5
#define QUEUE_ITEM_SIZE sizeof(int)
QueueHandle_t xQueue;

#define BTN 34

#define Accelaration 4
#define Skidpad 27
#define Autocross 25
#define Trackdrive 17
#define EBS_test 15
#define Inspection 33
#define Manual_driving 23
volatile int contador = 1;

BluetoothSerial bl;




hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int flag_heart = 0;
TWAI_Interface CAN1(1000, 21, 22);



void colorWipe(uint32_t color, int wait);
void Task1code(void *pvParameters);
void leds();
#define PIN 16
#define NUM_LEDS 48
volatile boolean send = true;

volatile boolean task = true;
TaskHandle_t Task1;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

volatile unsigned long tim = 0;
volatile unsigned long protect = 0;
int flag = 0;
volatile int flag_mission_select = 0;
int emergencia = 0;
volatile uint32_t CAN_ID = 0;

uint8_t var = 0;
unsigned long waitMillis = 0;
volatile int flag_core = 0;

void IRAM_ATTR mission_select()
{
    if(millis()>= protect+600)
    {
        contador++;
        protect = millis();
    }
    
    
    
}


void IRAM_ATTR onTimer()
{
    flag_heart = 1;  
    Serial.println("tmr"); 
}


void setup()
{

    bl.begin("ALC");
    //esp_bt_gap_set_pin(pin_type, 4, pin_code);
    Serial.begin(115200);
    Serial.println("Here");
    strip.show();
    strip.clear();
    //colorWipe(strip.Color(0, 255, 0), 0);
    pinMode(BTN, INPUT);
    attachInterrupt(BTN, mission_select, FALLING);
    pinMode(Accelaration, OUTPUT);
    pinMode(Skidpad, OUTPUT);
    pinMode(Autocross, OUTPUT);
    pinMode(Trackdrive, OUTPUT);
    pinMode(EBS_test, OUTPUT);
    pinMode(Inspection, OUTPUT);
    pinMode(Manual_driving, OUTPUT);

    xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (xQueue == NULL)
    {
        Serial.println("Failed to create queue");
        while (1);
    }

    xTaskCreatePinnedToCore(
        Task1code,
        "Task1",
        10000,
        NULL,
        1,
        &Task1,
        0);
    Serial.println("Setup");

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000, true);
    timerAlarmEnable(timer);

    //esp_task_wdt_init(1,false);
   
 
}

void loop()
{
    leds();
       
    CAN_ID = CAN1.RXpacketBegin();
    int id = CAN_ID;

    if(id == 0x502)
    {
        uint8_t CAN_DLC = CAN1.RXgetDLC();
        flag = CAN1.RXpacketRead(0);
        Serial.println(flag);
        if (flag_core == 0)
        {
            task = true;
            flag_core = 1;
        }
        if (flag == 4)
        {
            emergencia = 1;
        }
        else
        {
            emergencia = 0;
        }

        // Send estado to the queue
        if (xQueueSend(xQueue, &flag, portMAX_DELAY) != pdPASS)
        {
            Serial.println("Failed to send to the queue");
        }

    }  

    if (flag_heart == 1)
    {
        CAN1.TXpacketBegin(0x50, 0);
        CAN1.TXpacketLoad(emergencia);
        CAN1.TXpacketLoad(contador);
        CAN1.TXpackettransmit();
        flag_heart = 0; 
        Serial.println("transmitted");
    }
}
   

    


void colorWipe(uint32_t color, int wait)
{
    for (int i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, color);
        strip.show();
        delay(wait);
    }
}

void Task1code(void *pvParameters)
{


    if (task == true)
    {
        int estado = 0;
        int receivedValue;
        while (1)
        {          
            if (xQueueReceive(xQueue, &receivedValue, 100 / portTICK_PERIOD_MS) == pdPASS)
            {
                 estado = receivedValue;
            }
                switch (estado)
                {
                case 1:
                    Serial.println("AS - OFF");
                    //colorWipe(strip.Color(0, 0, 0), 0);
                    strip.fill(strip.Color(0, 0, 0), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    send = true;
                    break;
                case 2:
                    Serial.println("AS - Ready");
                    //colorWipe(strip.Color(255, 255, 0), 0);
                    strip.fill(strip.Color(255, 255, 0), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    send = true;
                    break;
                case 3:
                    
                    //colorWipe(strip.Color(255, 255, 0), 0);
                    strip.fill(strip.Color(255, 255, 0), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    tim = millis();
                    while (millis() < tim + 1000){
                        if (xQueueReceive(xQueue, &receivedValue, 100 / portTICK_PERIOD_MS) == pdPASS){
                            estado = receivedValue;
                        }
                        if(estado != 3)
                        {
                            break;
                        }
                    }
                    //colorWipe(strip.Color(0, 0, 0), 0);
                    strip.fill(strip.Color(0, 0, 0), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    tim = millis();
                    while (millis() < tim + 1000){
                        if (xQueueReceive(xQueue, &receivedValue, 100 / portTICK_PERIOD_MS) == pdPASS)
                        {
                            estado = receivedValue;
                        }
                        if(estado != 3)
                        {
                            break;
                        }
                    }
                    
                    send = true;
                    Serial.println("AS - Driving");
                    break;
                case 4:
                    Serial.println("AS - Emergency");
                    //colorWipe(strip.Color(0, 0, 255), 0);
                    strip.fill(strip.Color(0, 0, 255), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    tim = millis();
                    while (millis() < tim + 1000)
                    {
                        if (xQueueReceive(xQueue, &receivedValue, 100 / portTICK_PERIOD_MS) == pdPASS)
                        {
                            estado = receivedValue;
                        }
                        if(estado != 4)
                        {
                            break;
                        }
                    }
                    
                    //colorWipe(strip.Color(0, 0, 0), 0);
                    strip.fill(strip.Color(0, 0, 0), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    send = false;
                    tim = millis();
                    while (millis() < tim + 1000)
                        if (xQueueReceive(xQueue, &receivedValue, 100 / portTICK_PERIOD_MS) == pdPASS)
                        {
                            estado = receivedValue;
                        }
                        if(estado != 4)
                        {
                            break;
                        }
                    break;
                case 5:
                    Serial.println("AS - Finished");
                    //colorWipe(strip.Color(0, 0, 255), 0);
                    strip.fill(strip.Color(0, 0, 255), 0, strip.numPixels());
                    strip.show(); // Atualiza o strip com a nova cor
                    send = true;
                    break;
                default:
                    colorWipe(strip.Color(0, 0, 0), 0);
                break;
                }
                
            
            
        }
    }
}



void leds()
{
    switch (contador)
        {
        case 1:
            digitalWrite(Manual_driving, HIGH);
            digitalWrite(Accelaration, LOW);
            digitalWrite(Skidpad, LOW);
            digitalWrite(Autocross, LOW);
            digitalWrite(Trackdrive, LOW);
            digitalWrite(EBS_test, LOW);
            digitalWrite(Inspection, LOW);
            if(bl.available())
            {
                bl.println("Manual driving");
            }
            break;
        case 2:
            digitalWrite(Manual_driving, LOW);
            digitalWrite(Accelaration, HIGH);
            digitalWrite(Skidpad, LOW);
            digitalWrite(Autocross, LOW);
            digitalWrite(Trackdrive, LOW);
            digitalWrite(EBS_test, LOW);
            digitalWrite(Inspection, LOW);
            if(bl.available())
            {
                bl.println("Accelaration");
            }
            break;
        case 3:
            digitalWrite(Manual_driving, LOW);
            digitalWrite(Accelaration, LOW);
            digitalWrite(Skidpad, HIGH);
            digitalWrite(Autocross, LOW);
            digitalWrite(Trackdrive, LOW);
            digitalWrite(EBS_test, LOW);
            digitalWrite(Inspection, LOW);
            if(bl.available())
            {
                bl.println("Skidpad");
            }
            break;
        case 4:
            digitalWrite(Manual_driving, LOW);
            digitalWrite(Accelaration, LOW);
            digitalWrite(Skidpad, LOW);
            digitalWrite(Autocross, HIGH);
            digitalWrite(Trackdrive, LOW);
            digitalWrite(EBS_test, LOW);
            digitalWrite(Inspection, LOW);
            if(bl.available())
            {
                bl.println("Autocross");
            }
            break;
        case 5:
            digitalWrite(Manual_driving, LOW);
            digitalWrite(Accelaration, LOW);
            digitalWrite(Skidpad, LOW);
            digitalWrite(Autocross, LOW);
            digitalWrite(Trackdrive, HIGH);
            digitalWrite(EBS_test, LOW);
            digitalWrite(Inspection, LOW);
            if(bl.available())
            {
                bl.println("Trackdrive");
            }
            break;
        case 6:
            digitalWrite(Manual_driving, LOW);
            digitalWrite(Accelaration, LOW);
            digitalWrite(Skidpad, LOW);
            digitalWrite(Autocross, LOW);
            digitalWrite(Trackdrive, LOW);
            digitalWrite(EBS_test, HIGH);
            digitalWrite(Inspection, LOW);
            if(bl.available())
            {
                bl.println("EBS Test");
            }
            break;
        case 7:
            digitalWrite(Manual_driving, LOW);
            digitalWrite(Accelaration, LOW);
            digitalWrite(Skidpad, LOW);
            digitalWrite(Autocross, LOW);
            digitalWrite(Trackdrive, LOW);
            digitalWrite(EBS_test, LOW);
            digitalWrite(Inspection, HIGH);
            if(bl.available())
            {
                bl.println("Inspection");
            }
            break;
        default:
            contador = 1;
            break;
        }

        flag_mission_select = 0;
}
