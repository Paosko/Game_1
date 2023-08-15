#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
//#include <esp_task_wdt.h>
//#include <BLEDevice.h>
#define TaskStackSize   5120
#include "BLEDevice.h"

static SemaphoreHandle_t mutex;
static portMUX_TYPE CryticalMutex = portMUX_INITIALIZER_UNLOCKED;



/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;

    bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
    }
}
void lv_timer_han(void *param)
{
    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );


    ui_init();
    int cnt=0;
    for(;;)
    {
        //taskYIELD();
        if(xSemaphoreTake(mutex,0)==pdTRUE)
        {
            portENTER_CRITICAL(&CryticalMutex);
                lv_timer_handler();
            portEXIT_CRITICAL(&CryticalMutex);
            xSemaphoreGive(mutex);
            vTaskDelay(50);
        }
        else
        {
            Serial.print("Wait for Mutex LV_up");
        }
    }

}

void lv_exec(void *param)
{
 //   static int cnt=0;
    Serial.print("I am in the function");
    int cnt=0;
    for(;;)
    {
        
        vTaskDelay(1000);
        Serial.println(xPortGetFreeHeapSize());
        if(cnt==0 && millis()>5000)
        {   cnt++;
            char str[20];
            sprintf(str,"xTaskGetTickCount:%d\n",xTaskGetTickCount());
            Serial.print(str);
            lv_event_t *ev=new lv_event_t();
            ev->code=LV_EVENT_PRESSED;
            ev->target=ui_Clock;

            ui_event_Clock(ev);
            Serial.print("Clicked 1");

            sprintf(str, "Task is running on the core: %d, \n\0",xPortGetCoreID());
            Serial.print(str);

            sprintf(str,"xTaskGetTickCount:%i \n\0",xTaskGetTickCount());
            Serial.print(str);

        }

        if(cnt==1 && millis()>10000)
        {
            lv_event_t *ev=new lv_event_t();
            ev->code=LV_EVENT_PRESSED;
            ev->target=ui_Call;
            Serial.print("Clicked 2");
            if(xSemaphoreTake(mutex,0)==pdTRUE)
            {
                ui_event_Call(ev);
                cnt++;
                xSemaphoreGive(mutex);
            }
            else
            {
                Serial.print("Wait for Mutex");
            }
        }

        if(cnt==2 && millis()>15000)
        {
            lv_event_t *ev=new lv_event_t();
            ev->code=LV_EVENT_LEAVE;
            ev->target=ui_Call;
            Serial.print("Clicked 3");
            if(xSemaphoreTake(mutex,0)==pdTRUE)
            {
                ui_event_Call(ev);
                cnt++;
                xSemaphoreGive(mutex);
            }
            else
            {
                Serial.print("Wait for Mutex");
            }
        }
    }
}






//////////////////////////////////////////////////////////////BLE START
//------ VR Box Definitions -----
enum
{
  VB_TRIGGERS = 0,
  VB_JOYX,
  VB_JOYY,
  VB_BTNAB,
  VB_BTNCD,
  VB_NUMBYTES
};

// ===== VR Box Button Masks =====
#define VB_LOW_TRIGGER    0x40
#define VB_UPR_TRIGGER    0x80
#define VB_BUTTON_A       0x10
#define VB_BUTTON_B       0x01
#define VB_BUTTON_C       0x08
#define VB_BUTTON_D       0x02
#define FRESHFLAG         0x00

#define JOYTIMEOUT        30      // joystick no activity timeout in mS

#define JoyStickDeadZone  0

#define ServerName  "Utopia 360 Remote"      // change this if your server uses a different name

//----- ESP32 Definitions ------------------------------------------------------
//----- ESP32 GPIO Allocation -----
#define SCL         22      // GPIO 22 -> I2C SCL
#define SDA         21      // GPIO 21 -> I2C SDA

#define BUILTINLED  2       // GPIO 2  -> built-in blue LED, on ->high, off -> low
#define GREENLED    15      // GPIO 15 -> Green LED - lit -> scanning
#define BLUELED     2       // GPIO 2  -> Blue LED  - lit -> connected
#define REDLED      4       // GPIO 4  -> Red LED

// these values are for GPIO driven LEDs. BUILTIN Blue LED is opposite.
#define LEDOFF    HIGH
#define LEDON     LOW

// ===== VRBOX Modes =====
// This code assumes you are using the Mouse Mode
// @ + A -> Music & Video mode
// @ + B -> Horizontal Game mode
// @ + C -> Vertical Game mode
// @ + D -> Mouse Mode  // use this mode
//  4 byte notification, Trigger Sws, Joystick X, Joystick Y, 0x00

// All four modes send data. However each mode uses different byte positions and
// values for each of the switches. The joystick acts like a 'D' pad when not in
// Mouse Mode (no analog value).
 
typedef void (*NotifyCallback)(BLERemoteCharacteristic*, uint8_t*, size_t, bool);

// this is the service UUID of the VR Control handheld mouse/joystick device (HID)
static BLEUUID serviceUUID("00001812-0000-1000-8000-00805f9b34fb");

// Battery Service UUID
static BLEUUID BatteryServiceUUID("0000180F-0000-1000-8000-00805f9b34fb");

// this characteristic UUID works for joystick & triggers (report)
static BLEUUID ReportCharUUID("00002A4D-0000-1000-8000-00805f9b34fb"); // report


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

static BLERemoteCharacteristic* pBatRemoteCharacteristic;


// pointer to a list of characteristics of the active service,
// sorted by characteristic UUID
std::map<std::string, BLERemoteCharacteristic*> *pmap;
std::map<std::string, BLERemoteCharacteristic*> :: iterator itr;

// pointer to a list of characteristics of the active service,
// sorted by characteristic handle
std::map<std::uint16_t, BLERemoteCharacteristic*> *pmapbh;
std::map<std::uint16_t, BLERemoteCharacteristic*> :: iterator itrbh;

// storage for pointers to characteristics we want to work with
// to do: change to linked list ?
BLERemoteCharacteristic *bleRcs[4];

// This is where we store the data from the buttons and joystick
volatile byte   VrBoxData[VB_NUMBYTES];
volatile bool   flag = false;         // indicates new data to process

// joyTimer is a 30 millisecond re-triggerable timer that sets the joystick 
// back to center if no activity on the joystick or trigger buttons. 
volatile uint32_t joyTimer = millis();

// task handles  
TaskHandle_t HandleJS = NULL;   // handle of the joystick task
TaskHandle_t HandleAB = NULL;   // handle of the A/B button task
TaskHandle_t HandleCD = NULL;   // handle of the C/D button task

char bfr[80];

//******************************************************************************
// HID notification callback handler.
//******************************************************************************
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify)
{
  Serial.print("Notify callback for characteristic ");

  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");

  for (int i = 0; i < length; i++)
    Serial.printf("%02X ", pData[i]);
  Serial.println();
 
  // we are getting the two trigger buttons in the first byte, joyX & joyY in 2nd & 3rd bytes
  // A four byte report is the joystick/trigger buttons.
  // A two byte report is either the A/B buttons or the C/D buttons
  // Low nibble equal to 0x05 indicates A/B buttons.
  // A/B buttons auto-repeat if held. No other buttons do this.
  if (4 == length)
  {
    // copy data to VrBoxData
    for (int i = VB_TRIGGERS; i < VB_BTNAB; i++)
      VrBoxData[i] = pData[i];

    // wake up the joystick/trigger buttons handler task
    if (HandleJS)
      vTaskResume(HandleJS);
      
    // restart the joystick timer
    joyTimer = millis() + JOYTIMEOUT;
  }
  else if (2 == length)
  {
    // show the received data
    if (0x0 < (pData[0] & 0x11))
    {
      // A/B button report, wake the A/B button handler task
      VrBoxData[VB_BTNAB] = pData[0] & 0x11;
      if (HandleAB)
        vTaskResume(HandleAB);
    }
    else if(0x0 < (pData[0] & 0xA))
    {
      // C/D button report, wake the C/D button handler task
      VrBoxData[VB_BTNCD] = pData[0] & 0xa;
      if (HandleCD)
        vTaskResume(HandleCD);
    }
    else
    {
      Serial.println("Other Case, released joystick or Trigger pins?");
    }
  }
} //  notifyCallback

//******************************************************************************
// Battery notification callback handler.
//******************************************************************************
static void BatteryNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify)
{
  Serial.println("Battery Notification Callback Event");
  Serial.print("Data length ");
  Serial.println(length);
  Serial.print("data: ");

  for (int i = 0; i < length; i++)
    Serial.printf("%02X ", pData[i]);
  Serial.println();
} //  BatteryNotifyCallback

//******************************************************************************
// Connection state change event callback handler.
//******************************************************************************
class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient* pclient)
  {
    Serial.println("onConnect event");
    
  }

  void onDisconnect(BLEClient* pclient)
  {
    Serial.println("onDisconnect event");
    connected = false;
  }


};

//******************************************************************************
// Connect to a service, register for notifications from Report Characteristics.
//******************************************************************************
bool setupCharacteristics(BLERemoteService* pRemoteService, NotifyCallback pNotifyCallback)
{
  // get all the characteristics of the service using the handle as the key
  pmapbh = pRemoteService->getCharacteristicsByHandle();
  
  // only interested in report characteristics that have the notify capability
  for (itrbh = pmapbh->begin(); itrbh != pmapbh->end(); itrbh++)
  {
    BLEUUID x = itrbh->second->getUUID();
    Serial.print("Characteristic UUID: ");
    Serial.println(x.toString().c_str());
    // the uuid must match the report uuid

    if (ReportCharUUID.equals(itrbh->second->getUUID()))
    {
      // found a report characteristic
      Serial.println("Found a report characteristic");

      if (itrbh->second->canNotify())
      {
        Serial.println("Can notify");
        // register for notifications from this characteristic
        itrbh->second->registerForNotify(pNotifyCallback);

        sprintf(bfr, "Callback registered for: Handle: 0x%08X, %d", itrbh->first, itrbh->first);
        Serial.println(bfr);
      }
      else
      {
        Serial.println("No notification");
      }
    }
    else
    {
        sprintf(bfr, "Found Characteristic UUID: %s\n", itrbh->second->getUUID().toString().c_str()); 
        Serial.println(bfr);
    }
  } //  for
  return true;
} // setupCharacteristics
 
//******************************************************************************
// Validate the server has the correct name and services we are looking for.
// The server must have the HID service, the Battery Service is optional.
//******************************************************************************
bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  
  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());
  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // BLE servers may offer several services, each with unique characteristics
  // we can identify the type of service by using the service UUID
  
  // Obtain a reference to the service we are after in the remote BLE server.
  // this will return a pointer to the remote service if it has a matching service UUID
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find HID service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  Serial.println(" - Found HID service");
  setupCharacteristics(pRemoteService, notifyCallback);
  pRemoteService = pClient->getService(BatteryServiceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find battery service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
  }
  else
  {
    Serial.println(" - Found battery service");
    setupCharacteristics(pRemoteService, BatteryNotifyCallback);
  }

  connected = true;
  return true;
} //  connectToServer

//******************************************************************************
// Scan for BLE servers and find the first one that advertises the service we are looking for.
//******************************************************************************
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
  // Called for each advertising BLE server.
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // we have found a server, see if it has the name we are looking for
    if (advertisedDevice.haveName())
    {
      if (0 == strcmp(ServerName, advertisedDevice.getName().c_str()))
      {
        Serial.println("Found VRBOX Server");

        // we found a server with the correct name, see if it has the service we are
        // interested in (HID)
  
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
        {
          Serial.println("Server has HID service");
          
          BLEDevice::getScan()->stop();
          
    
          myDevice = new BLEAdvertisedDevice(advertisedDevice);
          doConnect = true;
          doScan = true;
        } // Found our server
        else
        {
          Serial.println("Server does not have an HID service, not our server");
        }
      }
    }
    else
    {
      Serial.println("Server name does not match, not our server");
    }
  } // onResult
}; // MyAdvertisedDeviceCallbacks


// All of these tasks are designed to run forever. The tasks are resumed when
// a notification message is received with new data.
//******************************************************************************
// Joystick handler Task.
// Moving the joystick off center causes this task to be resumed about every
// 15ms. Press or release of either trigger button will also resume this task.
// If this task does not complete in less than 15mS you will lose joystick
// movement data !!!
// Holding the lower button will prevent the server from detecting that the 
// upper button has been pressed. 
// Holding the upper trigger and pressing the lower trigger results in the server
// sending a notification that the lower trigger is pressed (the upper trigger
// will be zero!). Releasing the lower trigger will cause the server to send a
// notification that the upper trigger is pressed and the lower trigger is
// released.
//******************************************************************************
void taskJoyStick(void *parameter)
{
  int8_t  x;
  int8_t  y;
  uint8_t triggers;
  
  //===== if the task requires any one time initialization, put it here =====

  // forever loop
  while(true)
  {
    // give up the CPU, wait for new data
    vTaskSuspend(NULL);
    vTaskDelay(50);
    // we just woke up, new data is available, convert joystick data to
    // signed 8 bit integers
    x = (int8_t)VrBoxData[VB_JOYX];
    y = (int8_t)VrBoxData[VB_JOYY];
    triggers = VrBoxData[VB_TRIGGERS];
    
    Serial.printf("Joystick X: %d, Y: %d Triggers: %02X\n", x, y, triggers);

    if (y < -JoyStickDeadZone)
    {
      // move forward
      Serial.println("Forward");
      
      //===== add your code here =====
      
    }
    else if (y > JoyStickDeadZone)
    {
      // move backward
      Serial.println("Backward");

      //===== add your code here =====
      
    }
        
    if (x > JoyStickDeadZone)
    {
      // turn right
      Serial.println("Turn Right");

      //===== add your code here =====
      
    }
    else if (x < -JoyStickDeadZone)
    {
      // turn left
      Serial.println("Turn Left");

      //===== add your code here =====
      
    }

    if (triggers & VB_LOW_TRIGGER)
    {
      // the lower trigger button is pressed
      Serial.println("Low Trigger Pressed");

      //===== add your code here =====
    }

    if (triggers & VB_UPR_TRIGGER)
    {
      // the upper trigger button is pressed
      Serial.println("Upper Trigger Pressed");

      //===== add your code here =====
      
    }
  } //  for
} //  taskJoyStick

//******************************************************************************
// A & B Buttons handler Task.
// Holding the A or B button down will cause this task to be invoked about every
// 15ms. If this task does not complete within 15mS you will lose button events.
// The AB buttons work similar to the trigger buttons in that the A button will
// prevent the B button from being detected and will override the B button when
// pressed while the B button is held down.
//******************************************************************************
void taskButtonAB(void *parameter)
{
  uint8_t buttons;

  //===== if the task requires any one time initialization, put it here =====
  
  while(true)
  {
    // give up the CPU, wait for new data
    vTaskSuspend(NULL);
    vTaskDelay(50);
    // we just woke up, new data is available
    buttons = VrBoxData[VB_BTNAB];
    Serial.printf("A/B Buttons: %02X\n", buttons);
       
    if (buttons & VB_BUTTON_A)
    {
      // button A pressed or is being held down
      Serial.println("Button A");
      
      //===== add your code here =====
      
    }

    if (buttons & VB_BUTTON_B)
    {
      // button B pressed or is being held down
      Serial.println("Button B");

      //===== add your code here =====
      
    }
  } //  for
} //  taskButtonAB

//******************************************************************************
// C & D Buttons handler Task. 
// Press or release of either the C or D button will resume this task. Holding
// one button down blocks the Server from detecting the other button being 
// pressed.
//******************************************************************************
void taskButtonCD(void *parameter)
{
  uint8_t buttons;
  
  //===== if the task requires any one time initialization, put it here =====
  
  while(true)
  {
    vTaskDelay(50);
    // give up the CPU
    vTaskSuspend(NULL);

    // we just woke up, new data is available
    buttons = VrBoxData[VB_BTNCD];
    Serial.printf("C/D Buttons: %02X\n", buttons);
    
    if (buttons & VB_BUTTON_C)
    {
      // button C pressed
      Serial.println("Button C");
      
      //===== add your code here =====
      
    }

    if (buttons & VB_BUTTON_D)
    {
      // button D pressed
      Serial.println("Button D");

      //===== add your code here =====
      
    }
  } //  for
} //  taskButtonCD
/////////////////////////////////////////////////////BLE STOP



void setup()
{
    BaseType_t xReturned;
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );


    mutex=xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(lv_timer_han,"lv_timer_handler",10024,NULL,23,NULL,1);
    xTaskCreatePinnedToCore(lv_exec,"lv_exec",5024,NULL,3,NULL,1);
    
    Serial.println( "Setup done" );



     xReturned = xTaskCreatePinnedToCore(taskJoyStick,             // task to handle activity on the joystick.
                          "Joystick",               // String with name of task.
                          TaskStackSize,            // Stack size in 32 bit words.
                          NULL,                     // Parameter passed as input of the task
                          1,                        // Priority of the task.
                          &HandleJS,0);               // Task handle.
  if (pdPASS == xReturned)
  {
    Serial.println("Joystick Task Created");
  }
 
  xReturned = xTaskCreatePinnedToCore(taskButtonAB,             // task to handle activity on the A & B buttons.
                          "ButtonsAB",              // String with name of task.
                          TaskStackSize,            // Stack size in 32 bit words.
                          NULL,                     // Parameter passed as input of the task
                          1,                        // Priority of the task.
                          &HandleAB,0);               // Task handle.
  if (pdPASS == xReturned)
  {
    Serial.println("AB Button Task Created");
    
  }
 
  xReturned = xTaskCreatePinnedToCore(taskButtonCD,             // task to handle activity on the C & D buttons.
                          "ButtonsCD",              // String with name of task.
                          TaskStackSize,            // Stack size in 32 bit words.
                          NULL,                     // Parameter passed as input of the task
                          1,                        // Priority of the task.
                          &HandleCD,0);               // Task handle.
  if (pdPASS == xReturned)
  {
    Serial.println("CD Button Task Created");
    
  }
  Serial.println("Starting ESP32 BLE Client...");
  BLEDevice::init("");

  // Retrieve a GATT Scanner and set the callback we want to use to be informed 
  // when we have detected a new device.  Specify that we want active scanning
  // and start the scan to run for 5 seconds.
  
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(2000);
  pBLEScan->setWindow(50);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);      // scan for 5 seconds

  if (!connected)
  {
    doScan = true;
    Serial.println("Offline, start a scan");
  }


}

void loop()
{
    //vPortCPUInitializeMutex(&CryticalMutex);

    //lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(10);
    //Serial.print(uxTaskPriorityGet(NULL));

     if (doConnect == true) 
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    } 
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
  if (connected)
  {
    // joystick no activity detector
    if (joyTimer && (joyTimer < millis()))
    {
//      Serial.println("Timeout");
      // no joystick notification for 30mS, center the joystik
      VrBoxData[VB_JOYX] = VrBoxData[VB_JOYY] = 0;

      // wake up the joystick task
      vTaskResume(HandleJS);
      
      joyTimer = 0;
    }
  }
  else if (doScan)
  {
    Serial.println("Start scanning after disconnect");
    // this is just example to start scan after disconnect, most likely there is
    // a better way to do it in Arduino
    BLEDevice::getScan()->start(0);
  }

}

