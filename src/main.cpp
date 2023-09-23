#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <esp_task_wdt.h>
#include <BLEDevice.h>
#include <soc/rtc_wdt.h>
#include <math.h>


static SemaphoreHandle_t mutex;
TaskHandle_t LCDTask;
TaskHandle_t BleTask;
String MySerInput;

#define bleServerName "Utopia 360 Remote"



static BLEUUID bmeServiceUUID("00001812-0000-1000-8000-00805f9b34fb");

// BLE Characteristics
static BLEUUID JoystickCharacteristicUUID("00002a4d-0000-1000-8000-00805f9b34fb");

// Battery Characteristic
static BLEUUID BatteryCharacteristicUUID("0000180F-0000-1000-8000-00805f9b34fb");

//Flags stating if should begin connecting and if the connection is up
static bool doConnect = false;
static bool connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* JoystickCharacteristic;
static BLERemoteCharacteristic* BatteryCharacteristic;
BLEClient* pClient;
std::map<std::uint16_t, BLERemoteCharacteristic*> *pRemoteCharacteristic;
std::map<std::uint16_t, BLERemoteCharacteristic*> :: iterator itrbh;


//Variables to store Joystick and Battery
char* JoystickChar;
char* BatteryChar;
uint8_t JoystickValue=0;
uint8_t ButtonsValue=0;

//Flags to check whether new Joystick and Battery readings are available
bool newJoystick = false;

class GameBall 
{
  public:
  struct Ball
  {
    float BallActualPositionX=0;
    float BallActualPositionY=0;
    uint angle=45;
    float speed=0.5;
  };

   struct HraciePole
  {
    lv_coord_t sirka=470/2;
    lv_coord_t sirkaPadding=0;
    lv_coord_t vyska=290/2;
    lv_coord_t vyskaPadding=10;
    uint8_t numOfBalls;
    Ball Lopty[5];
  };
  
  
  HraciePole HP=HraciePole();
  
  GameBall()
  {
    Ball lopta=Ball();
    HP.Lopty[0]=lopta;
    HP.numOfBalls=1;
  }
  void BallComputeAngle (Ball *lopta)
{

  float x=cos(lopta->angle)*lopta->speed;
  float y=sin(lopta->angle)*lopta->speed;
 // Serial.printf("x:%.2f, y:%.2f, InitX:%.2f, InitY:%.2f ",x,y,lopta->BallActualPositionX,lopta->BallActualPositionY);
  lopta->BallActualPositionX+=x;
  lopta->BallActualPositionY+=y;
  //Serial.printf(" FinishX:%.2f, FinishY:%.2f \n",x,y,lopta->BallActualPositionX,lopta->BallActualPositionY);
}

};


uint8_t MyCommandCompare(String CompCommand)
{
  if(!MySerInput.isEmpty())
  {
    if(MySerInput.substring(0,CompCommand.length()-1).compareTo(CompCommand))
    {
      Serial.print("MySerInput length: ");
      Serial.println(MySerInput.length());
      {if(MySerInput.length()-2>CompCommand.length()) 
        {
          Serial.println("Je wite");
          return CompCommand.length(); //Return Write 
        }
        else
        {
          Serial.println("Je read");
          return 1; //Return read
        }
      }
    }
    Serial.println("Nerovna sa");
    MySerInput.clear();
    return 0; //Not much
  }
  return 0; //Not much
}

GameBall MojaPrvaHra;
void BallMove(void *p)
{
  
  for(;;)
  {
    uint8_t comp=MyCommandCompare("b.s");
    if(comp>1)
    {
      Serial.println("Nastavujem rychlost lopty");
      MojaPrvaHra.HP.Lopty[0].speed=MySerInput.substring(comp,MySerInput.length()-2).toFloat();
      MySerInput.clear();
    }
    if(comp==1)
    {
      Serial.print("Rychlost lopty");
     Serial.println( MojaPrvaHra.HP.Lopty[0].speed);
     MySerInput.clear();
    }


    MojaPrvaHra.BallComputeAngle(&MojaPrvaHra.HP.Lopty[0]);
    Serial.print("X:");
    Serial.print((lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionX);
    Serial.print("Y:");
    Serial.print((lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionY);
    vTaskDelay(30);
  }
  
}
//When the BLE Server sends a new Joystick reading with the notify property
static void JoystickNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
                                          
  //store Joystick value
  //JoystickChar = (char*)pData;
  newJoystick = true;
   for (int i = 0; i < length; i++)
    Serial.print(pData[i]);
    
  Serial.println();
  JoystickValue=pData[1];
  ButtonsValue=pData[0];
  
}

//When the BLE Server sends a new Battery reading with the notify property
static void BatteryNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  //store Battery value
  BatteryChar = (char*)pData;

}



//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
  pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
  
  std::map<std::string, BLERemoteService*> *pRemoteServices = pClient->getServices();
  Serial.print("Nb of services: "); Serial.println(pRemoteServices->size());

  for(auto it = pRemoteServices->cbegin(); it != pRemoteServices->cend(); ++it)
{

  //Serial.printf("first: %s second.getUUID:%s, second.tostring:%s\n",it->first.c_str(),it->second->getUUID().toString().c_str(),it->second->toString().c_str());

  //  std::cout << it->first << " " << it->second.first << " " << it->second.second << "\n";
}
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }

  pRemoteCharacteristic=pRemoteService->getCharacteristicsByHandle();

  for( itrbh = pRemoteCharacteristic->begin(); itrbh != pRemoteCharacteristic->end(); ++itrbh)
  {
    
    //erial.printf("pRemoteCharacteristic: first: %d second.getUUID:%s, second.tostring:%s\n",itrbh->first,itrbh->second->getUUID().toString().c_str(),itrbh->second->toString().c_str());
    if (JoystickCharacteristicUUID.equals (itrbh->second->getUUID()))
    {
      if (itrbh->second->canNotify())
      {
         itrbh->second->registerForNotify(JoystickNotifyCallback);
         Serial.println("Registered notifyCallback from itrbh");
      }
    }
  }
  
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  //JoystickCharacteristic = pRemoteService->getCharacteristic(JoystickCharacteristicUUID);
  //BatteryCharacteristic = pRemoteService->getCharacteristic(BatteryCharacteristicUUID);
/*
  if (JoystickCharacteristic == nullptr) {
    Serial.print("Failed to find JoystickCharacteristic UUID");
    return false;
  }
/*
  if ( BatteryCharacteristic == nullptr) {
    Serial.print("Failed to find BatteryCharacteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 *\/
  //Assign callback functions for the Characteristics
  Serial.printf("JoystickCharacteristic can notify:%d\n",JoystickCharacteristic->canNotify());
  
  JoystickCharacteristic->registerForNotify(JoystickNotifyCallback);
  Serial.printf("Joystick ReadRawData:%02X\n",JoystickCharacteristic->readRawData());
  Serial.printf("Joystick Handle:%d\n",JoystickCharacteristic->getHandle());
    Serial.printf("Joystick Client :%d\n",JoystickCharacteristic->getRemoteService()->getClient()->toString().c_str());
  //BatteryCharacteristic->registerForNotify(BatteryNotifyCallback);
  */
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("AdvertisedDevice:%s");
    Serial.println(advertisedDevice.getName().c_str());
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */


static lv_indev_drv_t indev_drv;
static lv_disp_drv_t disp_drv;

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
    
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );


    ui_init();
    int cnt=0;
    for(;;)
    {
        //taskYIELD();

            //Serial.printf("LV time handler needs to be called:%d\n",);
            lv_timer_handler_run_in_period(5);
            
            vTaskDelay(5);

    }

}

void Wdt_reset(void *param)
{
    for(;;)
    {
        esp_task_wdt_reset();
        vTaskDelay(50);
    }
}

void lv_exec(void *param)
{
 //   static int cnt=0;
  Serial.print("Init used Stack in task lv_exec:");
  Serial.println(uxTaskGetStackHighWaterMark(NULL)); 
  int cnt=0;
  for(;;)
  {          
    vTaskDelay(10);
    if(!connected)
    {
      vTaskDelay(10000);
      lv_textarea_set_text(ui_TextArea2,"moj iny text");
    }

    if(connected && cnt==0)
    {
      cnt++;
      Serial.println("Utopia connected");
      lv_event_t *ev=new lv_event_t();
        ev->code=LV_EVENT_CLICKED;
        ui_event_Screen1(ev);
        GameBall myGame;
    }

    if(connected)
    {
        xTaskCreate(BallMove,"BallMove",5000,NULL,1,NULL);
        lv_obj_set_x(ui_Button3,(lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionX);
        lv_obj_set_y(ui_Button3,(lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionY);
      
    }
  }
}



void MyBluetooth(void * param)
{   

  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
  for(;;)
  {
      vTaskDelay(500);
      if (doConnect == true) {
      //Serial.printf("Start connecting to %s\n",pServerAddress->toString());
      if (connectToServer(*pServerAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        //Activate the Notify property of each Characteristic
        //JoystickCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        //BatteryCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      doConnect = false;
    }

    if(!connected)
    {
      pBLEScan->start(30);
    }
  }
}


void MySerialDebug(void * param)
{
  for(;;)
  {
    
    if(Serial.available())
    {
      
      MySerInput=Serial.readString();
      Serial.print("Command:");
      Serial.println(MySerInput);
    }
    vTaskDelay(200);
  }
}


void setup()
{
    pinMode(TFT_BL,OUTPUT);
    digitalWrite(TFT_BL,HIGH);
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    Serial.print("CPU FREQUENCY:");
    Serial.println(getCpuFrequencyMhz());
    Serial.print("init Heap:");
    Serial.println(xPortGetFreeHeapSize());
    Serial.printf("1. Mun of tasks:%d\n",uxTaskGetNumberOfTasks());
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );


    //
    xTaskCreate(MySerialDebug,"MySerialDebug",1024,NULL,1,NULL);
    mutex=xSemaphoreCreateMutex();
    Serial.printf("2. Mun of tasks:%d\n",uxTaskGetNumberOfTasks());
    xTaskCreatePinnedToCore(lv_timer_han,"lv_timer_handler",6000,NULL,3,&LCDTask,0);
    Serial.printf("3. Mun of tasks:%d\n",uxTaskGetNumberOfTasks());
    xTaskCreatePinnedToCore(lv_exec,"lv_exec",4000,NULL,3,NULL,0);
    Serial.printf("4. Mun of tasks:%d\n",uxTaskGetNumberOfTasks());
    xTaskCreatePinnedToCore(MyBluetooth,"MyBluetooth",9000,NULL,1,&BleTask,1);
    Serial.printf("5. Mun of tasks:%d\n",uxTaskGetNumberOfTasks());

    Serial.println( "Setup done" );


}

void loop()
{
    //lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(5000);
    //Serial.print(uxTaskPriorityGet(NULL));

}

