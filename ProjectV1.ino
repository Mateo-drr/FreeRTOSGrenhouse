#include "Arduino.h"
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"

Servo myservo; // Biblioteca Servo

//-------- Ecrã LCD -------

TFT_eSPI tft = TFT_eSPI();       // Biblioteca TFT_eSPI

uint32_t targetTime = 0;         // timeout de 1 segundo

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x

uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

byte omm = 99, oss = 99;
byte xcolon = 0, xsecs = 0;
unsigned int colour = 0;
//------------------------

//------- Estruturas de dados ---------
typedef struct temphumlumraw{
	uint32_t rawtemp = 0;
	uint32_t rawhum =0;
	uint32_t rawlum1 =0;
	uint32_t rawlum2= 0;
}thlr;

typedef struct temphumlum{
	float temp =0.0;
	float hum =0.0;
	float lum =0.0;
}thl;
//---------------------------------------

bool check, manual = false; // flags
int SensorLum = 0x29; // Endereço Sensor de luminosidade I2c.
int SensorTemp_Hum = 0x40; // Endereço Sensor de temperatura e humidade I2c.
int atuador_check = 0; // flag dos atuadores

//-- variáveis globais servomotor
int pos = 0;
bool on = false;

volatile unsigned long ulIdleCycleCount = 0UL; // variável para a função IdleHook

//-- Tasks
void vTask0(void * pvParameters); // BRAIN
void vTask1(void * pvParameters); // READ TEMP
void vTask2(void * pvParameters); // READ HUM LUM
void vTask3(void * pvParameters); // LCC
void vTask4(void * pvParameters); // BLE Tx
void vTask5(void * pvParameters); // SERVO

#define Measure_temp 0xF3 // Endereço interno do registo da temperatura.
#define Measure_hum 0xF5 // Endereço interno do registo da humidade.

//-- Definir pinos I/O
#define I2C_SDA 21
#define I2C_SCL 22
#define LED_REGA 14
#define LED_RESISTENCIA 12
#define LED_LUZ 2
#define SERVO 25

//------- Queue / Semaphore----------
QueueHandle_t xQueue;
QueueHandle_t xQueue2;
QueueHandle_t xStringQueue;
SemaphoreHandle_t xBinarySemaphore;

TaskHandle_t xTask2Handle, xTaskAtuadoresHandle, xTaskServoHandle;


//---------BLE------------------------------------------------
#define SERVICE_UUID "36353433-3231-3039-3837-363534333231"           //"6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID "36353433-3231-3039-3837-363534336261" //"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
bool deviceConnected;
char dataTxBLE[8];
BLECharacteristic *pCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.print("Conectado!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      manual = false;
      Serial.print("Desconectado!");
    }
};
class MyCallbacks: public BLECharacteristicCallbacks {

	void onRead(BLECharacteristic *pCharacteristic) {
		pCharacteristic->setValue(dataTxBLE); // Recebe a informação e coloca na variável dataTxBLE
	}

	void onNotify(BLECharacteristic *pCharacteristic) {
			pCharacteristic->setValue(dataTxBLE);
		}

    void onWrite(BLECharacteristic *pCharacteristic) {
    	static portBASE_TYPE xHigherPriorityTaskWoken;
       	xHigherPriorityTaskWoken = pdFALSE;

    	std::string rxValue = pCharacteristic->getValue();
    	char dataRx [5];
    	strcpy(dataRx, rxValue.c_str());

    	xQueueSendToBackFromISR( xStringQueue, &dataRx, (portBASE_TYPE*)&xHigherPriorityTaskWoken );

    	/* 'Give' the semaphore to unblock the task. */
    	xSemaphoreGiveFromISR( xBinarySemaphore,(portBASE_TYPE*)&xHigherPriorityTaskWoken );

    	if( xHigherPriorityTaskWoken == pdTRUE ){

    		vPortYield();
    	}
    }

};



void setup(){
	Serial.begin(115200);
	//-------- Ecrã LCD --------

	  tft.init(); // inicialização
	  tft.setRotation(0);
	  tft.fillScreen(TFT_BLACK);
	  //--------- info do projeto
	  tft.setTextSize(1);
	  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

	  tft.drawString("Realizado por:", 70, 100,2);
	  tft.drawString("Goncalo Lopes", 70, 120,2);
	  tft.drawString("Mateo Rodriguez", 70, 140,2);

	  delay(1000); // espera 1 segundo para ver
	  //-------- escreve e desenha informação fixa
	  tft.fillScreen(TFT_BLACK);
	  tft.drawString("Projeto SCE", 85, 0,2);
	  tft.drawLine(0,20,240,20,TFT_YELLOW);

	  tft.fillRect(0, 25, 240, 20, TFT_BLUE);
	  tft.drawRect(0, 25, 240, 115, TFT_BLUE);
	  tft.setTextColor(TFT_WHITE, TFT_BLUE);
	  tft.drawString("Valor de Sensores", 60, 28,2);

	  tft.setTextColor(TFT_WHITE, TFT_BLACK);
	  tft.drawString("Temperatura: ", 10, 55,2);
	  tft.drawString("Humidade: ", 10, 83,2);
	  tft.drawString("Luminosidade: ", 10, 110,2);

	  tft.drawString("C", 180, 55,2);
	  tft.drawString("RH%", 180, 83,2);
	  tft.drawString("lux", 180, 110,2);

	  tft.fillRect(0, 145, 240, 20, TFT_RED);
	  tft.drawRect(0, 145, 240, 155, TFT_RED);
	  tft.setTextColor(TFT_WHITE, TFT_RED);
	  tft.drawString("Estado dos Atuadores", 55, 148,2);

	  tft.setTextColor(TFT_WHITE, TFT_BLACK);
	  tft.drawString("Janela: ", 10, 185,2);
	  tft.drawString("Resistência: ", 10, 225,2);
	  tft.drawString("Rega: ", 10, 265,2);
	  tft.drawString("Modo: ", 5, 300,2);
	  tft.drawString("BLE: ", 125, 300,2);
	  tft.setTextColor(TFT_GREEN, TFT_BLACK);
	  tft.drawString("Automatico", 45, 300,2);
	  tft.setTextColor(TFT_RED, TFT_BLACK);
	  tft.drawString("Disconnected", 155, 300,2);

	  tft.drawCircle(180, 193, 10, TFT_WHITE);
	  tft.drawCircle(180, 233, 10, TFT_WHITE);
	  tft.drawCircle(180, 273, 10, TFT_WHITE);

	  targetTime = millis() + 1000;

	  //------ Wire Setup (I2C)
	  Wire.begin(I2C_SDA, I2C_SCL, 100000);
	  Wire.setClock(400000);

	  //------- Criar dos Queues
	  xQueue = xQueueCreate(5, sizeof(thlr)); // Data dos sensores
	  xQueue2 = xQueueCreate(5, sizeof(thl)); // Valores Convertidos para Celsius, RH e lux
	  xStringQueue = xQueueCreate(5, sizeof(char[4])); // Transmissão BLE

	  //--------- BLE
	  BLEDevice::init("LoPy"); // Give it a name // Create the BLE Device
	  BLEServer *pServer = BLEDevice::createServer(); // Configura o dispositivo como Servidor BLE
	  BLEService *pService = pServer->createService(SERVICE_UUID);  // Cria o serviço
	  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                                   CHARACTERISTIC_UUID,
                                                   BLECharacteristic::PROPERTY_READ |
                                                   BLECharacteristic::PROPERTY_WRITE |
                                                   BLECharacteristic::PROPERTY_NOTIFY); // Caracteristicas BLE
	  pCharacteristic->addDescriptor(new BLE2902());
	  pServer->setCallbacks(new MyServerCallbacks());
	  pCharacteristic->setCallbacks(new MyCallbacks());
	  pService->start();// Inicia o serviço BLE
	  pServer->getAdvertising()->start();// Inicia a descoberta do ESP32
	  Serial.println("À espera que um cliente se conecte...");
	  pCharacteristic->setValue("on | off | t | h | l");

	  // -------- Tarefas
	  esp_register_freertos_idle_hook(my_vApplicationIdleHook);

	  //xCountingSemaphore = xSemaphoreCreateCounting( 5, 0 ); //from 5 to 0
	  vSemaphoreCreateBinary( xBinarySemaphore );
	  if(xBinarySemaphore != NULL){
		  xTaskCreatePinnedToCore(vTask4, "BLE", 1024, NULL, 7, NULL, 1);
	  }
	  xTaskCreatePinnedToCore(vTask0, "LCD", 1024, NULL, 1, NULL, 1);
	  xTaskCreatePinnedToCore(vTask1, "RAW DATA", 1024, NULL, 5, NULL, 1);
	  xTaskCreatePinnedToCore(vTask2, "CALC DATA", 1024, NULL, 4, &xTask2Handle, 1);
	  // --------- I/O
	  pinMode(LED_LUZ, OUTPUT);
	  pinMode(LED_REGA, OUTPUT);
	  pinMode(LED_RESISTENCIA, OUTPUT);
	  myservo.attach(SERVO);
}

void vTask0(void * pvParameters) {
    //TickType_t xLastWakeTime;
    //const char * pcTaskName = "--------------------------LCD/BRAIN-------------------------------\r\n";
    //xLastWakeTime = xTaskGetTickCount();
    thl thlRx; //inicializa estrutura thlRx

    portBASE_TYPE xStatus;
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    for (;;) {
        //Serial.print(pcTaskName);

    	//-------- Relógio canto superior esquerdo do ecrã
    	tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    	if (targetTime < millis()) {
    		// Set next update for 1 second later
    		targetTime = millis() + 1000;

    		// Adjust the time values by adding 1 second
    		ss++;              // Advance second
    		if (ss == 60) {    // Check for roll-over
    			ss = 0;          // Reset seconds to zero
    			omm = mm;        // Save last minute time for display update
    			mm++;            // Advance minute
    			if (mm > 59) {   // Check for roll-over
    				mm = 0;
    				hh++;          // Advance hour
    				if (hh > 23) { // Check for 24hr roll-over (could roll-over on 13)
    					hh = 0;      // 0 for 24 hour clock, set to 1 for 12 hour clock
    				}
    			}
    		}


    		// Update digital time
    		int xpos = 0;
    		int ypos = 0; // Top left corner ot clock text, about half way down

    		if (omm != mm) { // Redraw hours and minutes time every minute
    			omm = mm;
    			// Draw hours and minutes
    			if (hh < 10) xpos += tft.drawChar('0', xpos, ypos, 2); // Add hours leading zero for 24 hr clock
    			xpos += tft.drawNumber(hh, xpos, ypos, 2);             // Draw hours
    			xcolon = xpos; // Save colon coord for later to flash on/off
    			xpos += tft.drawChar(':', xpos, ypos - 8, 2);
    			if (mm < 10) xpos += tft.drawChar('0', xpos, ypos, 2); // Add minutes leading zero
    			xpos += tft.drawNumber(mm, xpos, ypos, 2);             // Draw minutes
    			xsecs = xpos; // Save seconds 'x' position for later display updates
    		}
    		if (oss != ss) { // Redraw seconds time every second
    			oss = ss;
    			xpos = xsecs;

    			if (ss % 2) { // Flash the colons on/off
    				tft.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
    				tft.drawChar(':', xcolon, ypos, 2);     // Hour:minute colon
    				xpos += tft.drawChar(':', xpos, ypos, 2); // Seconds colon
    				tft.setTextColor(TFT_YELLOW, TFT_BLACK);    // Set colour back to yellow
    			}
    			else {
    				tft.drawChar(':', xcolon, ypos, 2);     // Hour:minute colon
    				xpos += tft.drawChar(':', xpos, ypos, 2); // Seconds colon
    			}

    			//Draw seconds
    			if (ss < 10) xpos += tft.drawChar('0', xpos, ypos, 2); // Add leading zero
    			tft.drawNumber(ss, xpos, ypos, 2);                     // Draw seconds
    		}
    	}


    	if(uxQueueMessagesWaiting(xQueue2) <= 1){ //Se existirem valores na queue2 entra aqui
    		xStatus = xQueuePeek( xQueue2, &thlRx, 0);	//Vai buscar valores do Queue2 (Queue dos dados calculados)
    		if( xStatus == pdPASS ){
    			if (((thlRx.temp > 29  && atuador_check!=1) || (thlRx.hum < 50 && thlRx.lum < 400 && atuador_check!=4) || ((thlRx.hum > 75 || thlRx.lum > 400) && atuador_check!=5) ||		// Se os valores dos sensores forem maior ou
    			     (thlRx.temp < 20  && atuador_check!=2) || (thlRx.temp <= 28 && thlRx.temp >= 25  && atuador_check!=3)) && !manual && !check){  	// menores que o desejado entram aqui.
					xTaskCreatePinnedToCore( vTask3, "Task Atuadores", 1024, NULL, 7, &xTaskAtuadoresHandle, 1); // Tarefa dos atuadores criada
					check = true; //flag que previne com que a vTask3 seja despoltada e leia valores antigos da queue2
				}
        	}else{
        		Serial.print( "LCD: Could not PEEK from the queue2.\r\n" );
        	}
        }else{
        	check = false; // Valores da Queue2 atualizados
        	taskENTER_CRITICAL(&myMutex);{ // entra numa zona crítica

				xStatus = xQueueReceive( xQueue2, &thlRx, 0);	//Recebe e remove valores da Queue2 (Queue dos dados calculados)
				if( xStatus == pdPASS ){
					Serial.print("LCD: Received = ");

					// ----- Converte float para string
					char txString_temp[8], txString_hum[8], txString_lum[8];
					dtostrf(thlRx.temp, 1, 1, txString_temp);
					dtostrf(thlRx.hum, 1, 1, txString_hum);
					dtostrf(thlRx.lum, 1, 0, txString_lum);

					// ----- Imprime valores no LCD
					tft.fillRect(110, 55, 50, 75, TFT_BLACK);
					tft.setTextColor(TFT_WHITE, TFT_BLACK);
					tft.drawString(txString_temp, 110, 55,2);
					tft.drawString(txString_hum, 110, 83,2);
					tft.drawString(txString_lum, 110, 110,2);

					if (deviceConnected == true){ // verifica se há algum dispositivo ligado via BLE
						tft.setTextColor(TFT_GREEN, TFT_BLACK);
						tft.drawString("Connected    ", 155, 300,2);
					}else{
						tft.setTextColor(TFT_RED, TFT_BLACK);
						tft.drawString("Disconnected", 155, 300,2);
						tft.setTextColor(TFT_GREEN, TFT_BLACK);
						tft.drawString("Automatico", 45, 300,2);
					}


					Serial.print(thlRx.temp); Serial.print(thlRx.hum); Serial.println(thlRx.lum);
				}else{
					Serial.print( "LCD: Could not receive from the queue2.\r\n" );
				}
        	}taskEXIT_CRITICAL(&myMutex); // Sai da zona crítica
        }
    	vTaskDelay( 1 ); // fica bloqueado por um curto periodo de tempo para permitir a IdleTask correr.
    }
}
void vTask1(void * pvParameters) {
    TickType_t xLastWakeTime;
    const char * pcTaskName = "-------------------------------RAW DATA------------------------------\r\n";
    thlr thlrTx; // inicializa estrutura
    portBASE_TYPE xStatus;
    unsigned portBASE_TYPE uxPriority;
    xLastWakeTime = xTaskGetTickCount();
    int  data[4];
    for (;;) {
        Serial.print(pcTaskName);

        // ----------------- TEMPERATURA ---------------
		 Wire.beginTransmission(SensorTemp_Hum); // Inicia comunicação com o sensor Si7006-A20
		 Wire.write(Measure_temp); // Indica endereço da temperatura
		 Wire.requestFrom(SensorTemp_Hum,2); // Pede para devolver 2 bytes
		 int i=0;
		 while(Wire.available()) {  //
			 data[i] = Wire.read();
			 i ++;
		 }
		 Wire.endTransmission(); // Acaba transmissão I2C

		 thlrTx.rawtemp = float(((data[0])<<8) + (data[1])); // guarda valor na estrutura
	  // ----------------- HUMIDADE ---------------
		Wire.beginTransmission(SensorTemp_Hum);
		Wire.write(Measure_hum);
		Wire.requestFrom(SensorTemp_Hum,2);

		i=0;
		while(Wire.available()) {  //
		  data[i] = Wire.read();
		  i ++;
		 }
		Wire.endTransmission();

		thlrTx.rawhum = float(((data[0])<<8) + (data[1]));
	  // ----------------- LUMINOSIDADE ---------------
		i=0;
		Wire.beginTransmission(SensorLum);
	  	Wire.write(0x80);
	  	Wire.write(0x85);
	  	Wire.write(0x89);
	  	Wire.endTransmission();
	  	Wire.beginTransmission(SensorLum);
	  	Wire.write(0x88);
		Wire.endTransmission();
		Wire.requestFrom(SensorLum,4);
		while(Wire.available()) {  //
		  data[i] = Wire.read();
		  i ++;
		 }

		thlrTx.rawlum1 = float(((data[1])<<8) + (data[0])); // guarda valor do canal 1 do sensor na estrutura
		thlrTx.rawlum2 = float(((data[3])<<8) + (data[2])); // guarda valor do canal 2 do sensor na estrutura

		xStatus = xQueueSendToBack( xQueue, &thlrTx, 0); // coloca a estrutura no inicio da Queue

        if( xStatus != pdPASS ){
        	Serial.print( ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Could not send to the queue1.\r\n" );
        }

        if(uxQueueMessagesWaiting(xQueue2) <= 1){ // Se a queue 2 estiver vazia entra
			uxPriority = uxTaskPriorityGet( NULL); // Recebe prioridade da tarefa
			Serial.print("Raise the Task2 priority to "); Serial.println(uxPriority+1);
			vTaskPrioritySet(xTask2Handle, (uxPriority + 1)); //Aumenta a prioridade da tarefa dos cálculos para esvaziar a queue e colocar na queue2
        }
        vTaskDelayUntil( & xLastWakeTime, (4000 / portTICK_PERIOD_MS)); // Insere um temporizador para voltar a correr esta tarefa ao fim de 4 seundos
    }
}
void vTask2(void * pvParameters) {
    TickType_t xLastWakeTime;
    const char * pcTaskName = "-----------------------------CALCULOS--------------------------\r\n";
    xLastWakeTime = xTaskGetTickCount();
    thlr thlrRx; // inicializa estrutura dos dados não calculados
    thl thlTx;	// inicializa estrutura dos dados calculados
    unsigned portBASE_TYPE uxPriority;
    portBASE_TYPE xStatus;
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

    for (;;) {
        Serial.print(pcTaskName);
        xStatus = xQueueReceive( xQueue, &thlrRx, 0);	//recebe e elimina valores da queue
        if( xStatus == pdPASS ){
        	Serial.print("T2: Received = ");
        	Serial.print(thlrRx.rawtemp); Serial.print(thlrRx.rawhum); Serial.print(thlrRx.rawlum1); Serial.println(thlrRx.rawlum2);

        	taskENTER_CRITICAL(&myMutex);{  //Entra zona crítica

        	thlTx.temp= ((175.72*(thlrRx.rawtemp))/65536.0)-46.85; //Calcula valor da temperatura e insere na estrutura
        	thlTx.hum = ((125*(thlrRx.rawhum))/65536.0)-6;	//Calcula valor da humidade e insere na estrutura
        	thlTx.lum = ((thlrRx.rawlum1)+(thlrRx.rawlum2))/2;	///Calcula valor da luminosidade e insere na estrutura
        	Serial.print("T2: Will Send = ");
        	Serial.print(thlTx.temp); Serial.print(thlTx.hum); Serial.println(thlTx.lum);
        	xStatus = xQueueSendToBack( xQueue2, &thlTx, 0 ); // Insere estrutura na queue 2

        	}taskEXIT_CRITICAL(&myMutex); // Sai da zona critica

        	if( xStatus == pdPASS ){
        		Serial.println("Sent to queue2");
        	}else{
        		Serial.print( ">>>>>>>>>>>>>>>>>>>>>>>>T2: Could not send to the queue2.\r\n" );
        	}
        }else{
        	Serial.print( "T2: Could not receive from the queue1.\r\n" );
        }
        if(uxQueueMessagesWaiting(xQueue) <= 1){ // Se a queue (valores dos sensores) estiver vazia entra
			uxPriority = uxTaskPriorityGet( NULL); // Valor da prioridade da tarefa 2
			Serial.print("lower T2 priority "); Serial.println(uxPriority-2);
			vTaskPrioritySet( NULL, (uxPriority - 2)); // diminui a prioridade da tarefa 2 para a original, assim a tarefa 1 tem mais prioridade e enche a queue
        }
        vTaskDelayUntil( & xLastWakeTime, (4000 / portTICK_PERIOD_MS)); // Insere um temporizador para voltar a correr esta tarefa ao fim de 4 seundos
    }
}
void vTask3(void * pvParameters) {
    const char * pcTaskName = "----------------------------ATUADORES-----------------------\r\n";
    thl thlRx; // inicializa estrutura dos dados calculados
    portBASE_TYPE xStatus;
    for (;;) {
        Serial.print(pcTaskName);
        xStatus = xQueuePeek( xQueue2, &thlRx, 0);	//recebe dados da queue2
        if( xStatus == pdPASS ){
			//-------------- Atuar Sistema de Rega  -------------
			if (thlRx.hum < 50 && thlRx.lum < 400){
				atuador_check= 4;
				digitalWrite(LED_REGA, HIGH);
				tft.fillCircle(180, 273, 10, TFT_GREEN);
			}
			if(thlRx.hum > 75 || thlRx.lum > 400){
				atuador_check= 5;
				digitalWrite(LED_REGA, LOW);
				tft.fillCircle(180, 273, 10, TFT_BLACK);
				tft.drawCircle(180, 273, 10, TFT_WHITE);
			}
			//
			//-------------- Atuar Sistema de Temperatura -------------
			if (thlRx.temp > 29){
				atuador_check= 1;
				on= false;
				digitalWrite(LED_RESISTENCIA, LOW);
				tft.fillCircle(180, 233, 10, TFT_BLACK);
				tft.drawCircle(180, 233, 10, TFT_WHITE);
				xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);
				tft.fillCircle(180, 193, 10, TFT_GREEN);
			}else if(thlRx.temp < 20){
				atuador_check= 2;
				digitalWrite(LED_RESISTENCIA, HIGH);
				tft.fillCircle(180, 233, 10, TFT_GREEN);
			}else if(thlRx.temp <= 28 && thlRx.temp >= 25){
				atuador_check= 3;
				digitalWrite(LED_RESISTENCIA, LOW);
				on= true;
				xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);
				tft.fillCircle(180, 193, 10, TFT_BLACK);
				tft.drawCircle(180, 193, 10, TFT_WHITE);
				tft.fillCircle(180, 233, 10, TFT_BLACK);
				tft.drawCircle(180, 233, 10, TFT_WHITE);
			}

			//
        }else{
        	Serial.print( "T3: Could not receive from the queue2.\r\n" );
        }
        vTaskDelete( xTaskAtuadoresHandle );
    }
}
void vTask4(void * pvParameters) {

    const char * pcTaskName = "--------------------------BLE-------------------------\r\n";
    thl thlRx;
    portBASE_TYPE xStatus, xStatus2;
    char txString_temp[8], txString_hum[8], txString_lum[8];
    char rxValue[5];
    xSemaphoreTake( xBinarySemaphore, 0);
    for (;;) {
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
        Serial.print(pcTaskName);
        xStatus = xQueuePeek( xQueue2, &thlRx, 0);
        xStatus2 = xQueueReceive( xStringQueue, &rxValue, 0);
        if( xStatus == pdPASS ){
            Serial.print("T4: read = ");
            Serial.print(thlRx.temp); Serial.print(thlRx.hum); Serial.println(thlRx.lum);
            if( xStatus2 == pdPASS){
                dtostrf(thlRx.temp, 1, 1, txString_temp);
                dtostrf(thlRx.hum, 1, 1, txString_hum);
                dtostrf(thlRx.lum, 1, 0, txString_lum);
                if (sizeof(rxValue) > 0) {
                    Serial.println("*********");
                    Serial.print("Received Value: ");
                    for (int i = 0; i < sizeof(rxValue); i++) {
                        Serial.print(rxValue[i]);
                    }
                    Serial.println();
                    Serial.println("*********");
                }

                     //SEND DATA
                    if (rxValue[0] == 't') {
                        Serial.println("TEMP");
                        strcpy(dataTxBLE, txString_temp);
                    }
                    else if (rxValue[0] == 'h') {
                        Serial.println("HUM");
                        strcpy(dataTxBLE, txString_hum);
                    }
                    else if (rxValue[0] == 'l') {
                        Serial.println("LUM");
                        strcpy(dataTxBLE, txString_lum);
                    }
                    else if (rxValue[1] == 'n') { //on
                    	Serial.println("Turning ON!");
						digitalWrite(LED_LUZ, HIGH);
					}
                    else if (rxValue[1] == 'f') { //off
						Serial.println("Turning OFF!");
						digitalWrite(LED_LUZ, LOW);
                    }

                    //MANUAL CONTROL
                    if (rxValue[0] == 'm' && !manual) {
                        Serial.println("Modo manual ativado");
                        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
                        tft.drawString("Manual    ", 45, 300,2);
                        manual = true;
                    }else if(rxValue[0] == 'm' && manual){
                        Serial.println("Modo manual desativado");
                        tft.setTextColor(TFT_GREEN, TFT_BLACK);
                        tft.drawString("Automatico", 45, 300,2);
                        atuador_check = 0;
                        manual = false;
                    }

                    if (manual){
                    	if (rxValue[0] == 'r' && rxValue[2] == 'n') { //ron -> rega
                    	    digitalWrite(LED_REGA, HIGH);
                    	    tft.fillCircle(180, 273, 10, TFT_GREEN);
                    	}
                    	else if (rxValue[0] == 'r' && rxValue[2] == 'f') { //roff
                    	    digitalWrite(LED_REGA, LOW);
                    	    tft.fillCircle(180, 273, 10, TFT_BLACK);
                    	    tft.drawCircle(180, 273, 10, TFT_WHITE);
                    	}
                    	else if (rxValue[0] == 't' && rxValue[2] == 'n') { //ton -> servo
                    		on= false;
                    		xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);
                    		tft.fillCircle(180, 193, 10, TFT_GREEN);
                    	}
                    	else if (rxValue[0] == 't' && rxValue[2] == 'f') { //toff
                    		on= true;
                    		xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);
                    		tft.fillCircle(180, 193, 10, TFT_BLACK);
                    		tft.drawCircle(180, 193, 10, TFT_WHITE);

                    	}
                    	else if (rxValue[0] == 'a' && rxValue[2] == 'n') { //aon -> resistencia
                    	    digitalWrite(LED_RESISTENCIA, HIGH);
                    		tft.fillCircle(180, 233, 10, TFT_GREEN);
                    	}
                    	else if (rxValue[0] == 'a' && rxValue[2] == 'f') { //aoff
                    		digitalWrite(LED_RESISTENCIA, LOW);
                    		tft.fillCircle(180, 233, 10, TFT_BLACK);
                    		tft.drawCircle(180, 233, 10, TFT_WHITE);
                    	}
                    }
			}else{
				Serial.print( "T4: Could not receive from the queue2.\r\n" );
			}
		}
	}
}
void vTask5(void * pvParameters) {
    const TickType_t xDelay15ms = 15 / portTICK_PERIOD_MS;
    const char * pcTaskName = "----------------------------SERVO-----------------------\r\n";
    for (;;) {
        Serial.print(pcTaskName);
        if(!on){
            do{// goes from 0 degrees to 180 degrees
                // in steps of 1 degree
                myservo.write(pos);              // tell servo to go to position in variable 'pos'
                pos++;
                Serial.print(pos);
                vTaskDelay( xDelay15ms );        // waits 15ms for the servo to reach the position
              }while(pos < 180);
            on = true;
            pos = 180;
            vTaskDelete( xTaskServoHandle );
        }else{
            do{// goes from 180 degrees to 0 degrees
                myservo.write(pos);              // tell servo to go to position in variable 'pos'
                pos--;
                Serial.print(pos);
                vTaskDelay( xDelay15ms );        // waits 15ms for the servo to reach the position
             }while(pos > 0);
            on = false;
            pos = 0;
            vTaskDelete( xTaskServoHandle );
        }
        if(pos<0){
        	on = false;
        	pos = 0;
        }else if(pos > 180){
        	on = true;
        	pos = 180;
        }
        //vTaskDelay( xDelay15ms*4 );
    }
}

//-- Função IdleHook : evita processador correr em vazio
bool my_vApplicationIdleHook( void )
  {
    /* This hook function does nothing but increment a counter. */
    ulIdleCycleCount++;
    taskYIELD();
    return true;
  }

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
}

// Function to extract numbers from compile time string
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}
