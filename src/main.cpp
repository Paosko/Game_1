#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui/ui.h"
#include <esp_task_wdt.h>
#include <BLEDevice.h>
#include <soc/rtc_wdt.h>
#include <math.h>
#include <EEPROM.h>

#define collaps 1 // len aby som vedel collapsnut nejaku cast kodu :D
#define reprakPin 22

//////////EEPROM mapa
#define MemHlasitost 100
#define MemAmiMaxScore 120
#define MemBrickMaxScore 140

//FreeRtos Config
static SemaphoreHandle_t MutexScore;
static SemaphoreHandle_t xGuiSemaphore;
static SemaphoreHandle_t MutexTasky;


#define maxPocetKosticiek 20
TaskHandle_t taskHandles[10];
TaskHandle_t taskAmiGame;
TaskHandle_t taskKosticky [maxPocetKosticiek];
TaskHandle_t taskSettings;

//Debug part
String MySerInput;    //Debug output nedorieseny

//Funkcne premenne
static QueueHandle_t BleKeyboardQueue; // Queue pre Utopiu
static QueueHandle_t reprakQueue; // Queue pre reprak

lv_indev_t * KeyDriver; // BLE Driver
lv_indev_t * TouchDriver; // dotykovka

static int8_t hlasitost=254;
static int AmiMaxScore=0;
static int BrickMaxScore=0;


#if collaps ///bluetooth
  #define bleServerName "Utopia 360 Remote"
  static BLEUUID bmeServiceUUID("00001812-0000-1000-8000-00805f9b34fb");
  // BLE Characteristics
  static BLEUUID JoystickCharacteristicUUID("00002a4d-0000-1000-8000-00805f9b34fb");
  // Battery Characteristic
  static BLEUUID BatteryCharacteristicUUID("0000180F-0000-1000-8000-00805f9b34fb");
  //Flags stating if should begin connecting and if the connection is up
  static bool doConnect = false; // či vykonat pripojenie
  static bool connected = false; // status napriec celemu kodu o pripojeni

  //Address of the peripheral device. Address will be found during scanning...
  static BLEAddress *pServerAddress;
  //Characteristicd that we want to read
  static BLERemoteCharacteristic* JoystickCharacteristic;
  static BLERemoteCharacteristic* BatteryCharacteristic;
  BLEClient* pClient=BLEDevice::createClient();
  std::map<std::uint16_t, BLERemoteCharacteristic*> *pRemoteCharacteristic;
  std::map<std::uint16_t, BLERemoteCharacteristic*> :: iterator itrbh;




  struct BleKey
  {
    uint32_t Key;
    bool State;
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


  uint32_t JoystickMap(uint32_t Joystick)
  {
    if(Joystick==144 || Joystick==160)
    {return LV_KEY_RIGHT;}

    if(Joystick==16 || Joystick==0)
    {return LV_KEY_LEFT;}

    if(Joystick==64 || Joystick==128)
    {return LV_KEY_DOWN;}

    if(Joystick==96 || Joystick==32)
    {return LV_KEY_UP;}

    return LV_KEY_END;
  }

  uint32_t TlacidkoMap(uint8_t Tlacidko)
  {
    if(Tlacidko==2)
    {return LV_KEY_ENTER;} //A
    if(Tlacidko==8)
    {return LV_KEY_BACKSPACE;}  //C
    if(Tlacidko==1)
    {return LV_KEY_PREV;} //B
    if(Tlacidko==16)
    {return LV_KEY_NEXT;} //D
    if(Tlacidko==64)
    {return LV_KEY_HOME;} //Y spodne
    if(Tlacidko==128)
    {return LV_KEY_DEL;} //X vrchne

    return LV_KEY_END;
  }

  //When the BLE Server sends a new Joystick reading with the notify property
  static void JoystickNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
  {

    static BleKey Klavesa;
    static uint32_t Joystick=80; //144,160 vpravo, 16,0 vlavo, 64,128 dole, 96,32 hore, 80 vypnute pohlad na ovladac ak je joystick vlavo
    static uint8_t Tlacidlo=0;  //bin hodnota: c,a,0,b,d,0,y,x (x-> horne, y -> spodne)

    /*Serial.print("key: ");
    Serial.println(pData[0]);
    Serial.print("Joy: ");
    Serial.println(pData[1]);*/

    if(pData[1]!=Joystick) //Ak je zmena na Joysticku
    {

      if(Joystick!=80) // pokiaľ sa predch hodnota !=nestlačene, zruš stlačene
      {
        Klavesa.Key=JoystickMap(Joystick);
        Klavesa.State=false;
        if(xQueueSend(BleKeyboardQueue,(void*)&Klavesa,10)!=pdTRUE)
          {Serial.println("Queue sending problem ");}
      }

      Joystick=pData[1]; // Uloz novu hodnotu

      if(Joystick!=80)  //Ak sa nova hodnota nerovna nestlacene tak ju nastav
      {
        Klavesa.Key=JoystickMap(Joystick);
        Klavesa.State=true;
        if(xQueueSend(BleKeyboardQueue,(void*)&Klavesa,10)!=pdTRUE)
          {Serial.println("Queue sending problem ");}
      }
    }

    if(pData[0]!=Tlacidlo) //Ak je zmena na klavese
    {
        for (uint8_t x=0x80;x!=0;x>>=1) // x-> 128,64,32,16,8,4,2,1
        {
        /* Serial.print("x:");
          Serial.print(x);
          Serial.print(" Tlac&x:");
          Serial.print(Tlacidlo&x);
          Serial.print(" pData[0]&x:");
          Serial.print(pData[0]&x);*/

          if((Tlacidlo&x) != (pData[0]&x)) //pokial sa stara hodnota bitu nerovna novej
          {
            Klavesa.Key=TlacidkoMap(x);
            //Serial.print("posielam Tlacitko: ");
          // Serial.println(Klavesa.Key);
            Klavesa.State=pData[0]&x;
            if(xQueueSend(BleKeyboardQueue,(void*)&Klavesa,10)!=pdTRUE)
              {Serial.println("Queue sending problem ");}
          }
          else
          {
            //Serial.println(" No change");
          }
        }
        Tlacidlo=pData[0];
    }

  }

  //When the BLE Server sends a new Battery reading with the notify property
  static void BatteryNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                      uint8_t* pData, size_t length, bool isNotify) {
    //store Battery value
    //BatteryChar = (char*)pData;

  }



  //Connect to the BLE Server that has the name, Service, and Characteristics
  bool connectToServer(BLEAddress pAddress) {

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
      if(pClient->isConnected()==false && connected==true)
      {
        Serial.println("MyBluetooth -> isCOnnected false");
        pClient->disconnect();
        connected=false;
      }
      vTaskDelay(500);
      if (doConnect == true)
      {
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

#endif

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];
//static lv_color_t* buf2=(lv_color_t*)ps_malloc(screenWidth * screenHeight / 10);
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */


static lv_indev_drv_t indev_drv;
static lv_indev_drv_t indev_drv2;
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

/*Vycitanie klaves pre display*/
void my_Keyboard_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  static BleKey Klavesa;
  if(xQueueReceive(BleKeyboardQueue,(void *)&Klavesa,0)==pdTRUE)
  {
      data->key = Klavesa.Key;//last_key();            /*Get the last pressed or released key*/
      if(Klavesa.State)
      {
        //Serial.println("Joystick bol stlaceny");
        data->state=LV_INDEV_STATE_PR;
      }
      else
      {
        //Serial.println("Joystick bol pusteny");
        data->state = LV_INDEV_STATE_REL;
      }
      // Serial.print("Klavesa:");
      // Serial.print(Klavesa.Key);
      // Serial.print("    Hodnota:");
      // Serial.println(Klavesa.State);

  }

  //if(key_pressed()) data->state = LV_INDEV_STATE_PR;
  //else data->state = LV_INDEV_STATE_REL;

  //return false; /*No buffering now so no more data read*/
}

void myTOuch_read(lv_indev_drv_t *indev_driver, lv_indev_data_t * data  )
{

    uint16_t touchX, touchY;

    bool touched = tft.getTouch( &touchX, &touchY, 600 );
    //touchY=480-(touchY+88); //306,3133,88,3781,7

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        touchX=map(touchX,0,320,0,480);
        touchY=map(touchY,0,480,0,320);
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( data->point.x);

        Serial.print( "Data y " );
        Serial.println( data->point.y );
    }
}

void printMaxScore (lv_obj_t *UI, int MaxScore)
{
  lv_obj_t *MaxScoreObj=ui_comp_get_child(UI,3); // Maxim. skore
   if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
      lv_label_set_text_fmt(MaxScoreObj,"Best:%d",MaxScore);
      xSemaphoreGive(xGuiSemaphore);
    }
}

#if collaps // BrickGame
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
      /*Serial.print("X:");
      Serial.print((lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionX);
      Serial.print("Y:");
      Serial.print((lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionY);*/
      vTaskDelay(30);
    }

  }

#endif



#if collaps // AmiGame
struct StructPoloha
{
  float x;
  float y;
};


struct StructKosticka
{
  //char poloha;
  StructPoloha pozicia;
  int speed;
  float rotace;
};

StructPoloha lt={19,73}; //left top
StructPoloha lb={19,150}; //left bottom
StructPoloha rt={365,73};   // right top
StructPoloha rb={380,140}; //rightbottom

static uint AmiActualScore=0;
static int AmiZivot=100;


void KostickaMove(int DeltaTime, StructKosticka * iKosticka)
{
  
  //log_e("DeltaTIme:%d",DeltaTime);
 /* if(DeltaTime<50)
  {
    DeltaTime=80;
  }*/
  float Koef=(float)((float)DeltaTime/1000)*((float)iKosticka->speed/(float)1000);
  //log_e("koef:%f",Koef);
  if(iKosticka->pozicia.x<200)
  {
    iKosticka->pozicia.x=iKosticka->pozicia.x+Koef;
    iKosticka->rotace=(iKosticka->rotace)+(Koef*100);
    //log_e("Normalna Rotace:%f",(iKosticka->rotace));
  }
  else
  {
    iKosticka->pozicia.x=iKosticka->pozicia.x-Koef;
    iKosticka->rotace=(iKosticka->rotace)-(Koef*100);
    //log_e("Normalna Rotace:%f",(iKosticka->rotace));
  }
  iKosticka->pozicia.y=iKosticka->pozicia.y+Koef*0.5;
  
  //log_e("out X:%f Y:%f",(iKosticka->pozicia.x),(iKosticka->pozicia.y));

}

int8_t FindKostickaInitPosition(StructKosticka * ik)
{
  if(ik->pozicia.x==lt.x && ik->pozicia.y == lt.y)
  {return EnumAmiTopLeft;}

  if(ik->pozicia.x==lb.x && ik->pozicia.y == lb.y)
  {return EnumAmiBottomLeft;}

  if(ik->pozicia.x==rt.x && ik->pozicia.y == rt.y)
  {return EnumAmiTopRight;}

  if(ik->pozicia.x==rb.x && ik->pozicia.y == rb.y)
  {return EnumAmiBottomRight;}

return 5;
}

int createTon(int pozice, int initpozice)
{
  return initpozice*2;
}


void Kosticka (void *param)  // bude bezat viac krat, pre kazdu kosticku zvlast
{
  #if collaps // Kosticka Init
    // Ulozenie vstupnych hodnôt. Vstupne hodnoty byvaju prepisovane (pointre) takze je ichokamzite potreba ulozit 
    StructKosticka * VstupKosticka=(StructKosticka *)param;
    StructKosticka internalKosticka;
    internalKosticka.pozicia.x=VstupKosticka->pozicia.x;
    internalKosticka.pozicia.y=VstupKosticka->pozicia.y;
    int8_t KostickaInitPozition=FindKostickaInitPosition(&internalKosticka);
   //log_e("Kosticka pozice:%d",KostickaInitPozition);
    //log_e("internalPoloha X:%f, Y:%f",internalKosticka.pozicia.x,internalKosticka.pozicia.y);
    //internalKosticka.poloha=VstupKosticka->poloha;
   
    internalKosticka.speed=VstupKosticka->speed;
    //log_e("kosticka speed:%d",internalKosticka.speed);
    internalKosticka.rotace=VstupKosticka->rotace;
    // Na vypocet pozicii kosticky -> jednotlive rozdiely znamenaju nejaka pozicia polohy kosticky. asi 4 alebo 5
    float initPoziciaX=internalKosticka.pozicia.x;
    float initPoziciaY=internalKosticka.pozicia.y;

    int pozice=0;
    int ton=0;
   
    //Na meranie casu pre vypocet polohy podla casu
    TickType_t MoveStartTime=xTaskGetTickCount();


    Serial.println("Bezi task kosticky");
    TaskHandle_t localTask= xTaskGetCurrentTaskHandle();
    
    // TaskStatus_t xTaskDetails;
    
    // xTaskGetInfo(localTask,&xTaskDetails,NULL,eInvalid);
    Serial.println((int)&localTask);

    //Vytvorenie objektu kosticky
    lv_obj_t *uiKosticka;
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
      uiKosticka = lv_img_create(ui_GameArea2);
      lv_img_set_src(uiKosticka, &ui_img_391577990); //kosticka
      lv_obj_set_width( uiKosticka, LV_SIZE_CONTENT);  /// 1
      lv_obj_set_height( uiKosticka, LV_SIZE_CONTENT);   /// 1
      lv_obj_set_x( uiKosticka, (int16_t)internalKosticka.pozicia.x );
      lv_obj_set_y( uiKosticka, (int16_t)internalKosticka.pozicia.y );
      lv_img_set_angle(uiKosticka,(int16_t)internalKosticka.rotace);
      lv_obj_add_flag( uiKosticka, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
      lv_obj_clear_flag( uiKosticka, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
      lv_img_set_zoom(uiKosticka,150);
      xSemaphoreGive(xGuiSemaphore);
    #endif
  }
  for(;;)
  {
        
    if(pdTRUE == xSemaphoreTake(MutexTasky,portMAX_DELAY))
    {
      if (taskAmiGame==NULL || eTaskGetState(taskAmiGame)==eDeleted )
      {
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
          lv_obj_del(uiKosticka);
          vTaskDelay(5);
          xSemaphoreGive(xGuiSemaphore);
        }
        //log_e("Kosticka mimo rozsah, task suspended");
        vTaskDelete(NULL);
      }
    xSemaphoreGive(MutexTasky);
    }
    

    int deltaTime=xTaskGetTickCount()-MoveStartTime;
    TickType_t
    MoveStartTime=xTaskGetTickCount();
    //log_e("init X:%f Y:%f",(internalKosticka.pozicia.x),(internalKosticka.pozicia.y));
    KostickaMove(deltaTime,&internalKosticka);
    //log_e("OUT X:%f Y:%f",(internalKosticka.pozicia.x),(internalKosticka.pozicia.y));


    int deltaX=(int)(abs(internalKosticka.pozicia.x-initPoziciaX));
    
    if(deltaX>=0 && deltaX<10)
    {
      if(pozice!=1)
      {
        pozice=1;
        //Kosticka vstupila do pozice 1
        ton=createTon(pozice,KostickaInitPozition);
         if(xQueueSend(reprakQueue,(void*)&ton,10)!=pdTRUE)
          {Serial.println("Queue sending problem ");}
          //log_e("pozice:%d",pozice);
      }
    }
    if(deltaX>=10 && deltaX<30)
    {
      if(pozice!=2)
      {
        //Kosticka vstupila do pozice 2
        pozice=2;
        ton=createTon(pozice,KostickaInitPozition);
        if(xQueueSend(reprakQueue,(void*)&ton,10)!=pdTRUE)
        {Serial.println("Queue sending problem ");}
        //log_e("pozice:%d",pozice);
      }
    }
    if(deltaX>=30 && deltaX<50)
    {
      if(pozice!=3)
      {
        //Kosticka vstupila do pozice 3
        pozice=3;
        ton=createTon(pozice,KostickaInitPozition);
        if(xQueueSend(reprakQueue,(void*)&ton,10)!=pdTRUE)
        {Serial.println("Queue sending problem ");}
       // log_e("pozice:%d",pozice);
      }
    }
     if(deltaX>=50 && deltaX<75) // Ami musi chytit
    {
      if(pozice!=4)
      {
        //Kosticka vstupila do pozice 4
        pozice=4;
        ton=createTon(pozice,KostickaInitPozition);
        if(xQueueSend(reprakQueue,(void*)&ton,10)!=pdTRUE)
        {Serial.println("Queue sending problem ");}
        //log_e("pozice:%d",pozice);
      }
      if(AmiPosition==KostickaInitPozition)
      {
        while (xSemaphoreTake(MutexScore,portMAX_DELAY)!=pdTRUE)
        {
          vTaskDelay(5);
          log_e("Waiting to MutexScore");
        }

        AmiActualScore++;
        xSemaphoreGive(MutexScore);
        //log_e("deleting current Kosticka task -> Pass");

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
          lv_obj_del(uiKosticka);
          vTaskDelay(5);
          xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelete(NULL);
      }
    }
    if(deltaX>=75) // Ami nechytila, mínus zivot
    {
      if(pozice!=5)
      {
        //Kosticka vstupila do pozice 5
        pozice=5;
        ton=8;
        //log_e("pozice:%d",pozice);
        if(xQueueSend(reprakQueue,(void*)&ton,10)!=pdTRUE)
        {Serial.println("Queue sending problem ");}
        while (xSemaphoreTake(MutexScore,portMAX_DELAY)!=pdTRUE)
        {
          vTaskDelay(5);
          log_e("Waiting to MutexScore");
        }
        AmiZivot=AmiZivot-25;
        xSemaphoreGive(MutexScore);
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
          lv_obj_del(uiKosticka);
          vTaskDelay(5);
          xSemaphoreGive(xGuiSemaphore);
        }
        //log_e("Kosticka mimo rozsah, task suspended");
        vTaskDelete(NULL);
      }
    }


    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
      lv_obj_set_x( uiKosticka, (int16_t)internalKosticka.pozicia.x);
      lv_obj_set_y( uiKosticka, (int16_t)internalKosticka.pozicia.y );
      lv_img_set_angle(uiKosticka,(int16_t)internalKosticka.rotace);
      xSemaphoreGive(xGuiSemaphore);
    }
    vTaskDelay(50);
    
  }
}

void AmiGame (void *param)  // Bude spustat a zastavovat tasky kosticiek a ovladat Ami
{
  int bestScore=EEPROM.readInt(MemAmiMaxScore);
  hlasitost=EEPROM.readInt(MemHlasitost);
  StructPoloha InitPozicie [4]={lt,lb,rt,rb};
  StructPoloha poloha1=InitPozicie[0];
  StructKosticka AmiKosticka = {InitPozicie[0],5000,0};
  AmiActualScore=0;
  AmiZivot=100;
  int kostickaSpeed=15000;
  uint dalsiaKostickaDelay=10000;
  int RychlostHry=1000;
  int LocalAmiZivot,LocalAmiActualScore, PrintAmiZivot,PrintAmiActualScore;
  static TickType_t GenerovanieKStartTime=0;
  static TickType_t deltaTforK;

   lv_obj_t *ZivotBar=ui_comp_get_child(ui_StatusPanelWolf,1); // extern na Status bar
   lv_obj_t *ActScore=ui_comp_get_child(ui_StatusPanelWolf,2); // extern na Status bar
   printMaxScore(ui_StatusPanelWolf,bestScore);
   if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
      lv_label_set_text_fmt(ActScore,"Score:%d",AmiActualScore);
      lv_bar_set_value(ZivotBar,AmiZivot,LV_ANIM_OFF);
      lv_bar_set_value(ZivotBar,PrintAmiZivot,LV_ANIM_OFF);
      
      lv_img_set_src(ui_Ami,&ui_img_ami3_png);
      lv_obj_set_x( ui_Ami, 8 );
      lv_obj_set_y( ui_Ami, 73 );
      lv_img_set_zoom(ui_Ami,256);
      lv_obj_move_background(ui_NahraneScore);
      xSemaphoreGive(xGuiSemaphore);
    }

  
  for(;;)
  { 
    
    for (int x=0;x<maxPocetKosticiek;x++)  //Vynulovanie deleted Tasku
    {
      if(pdTRUE == xSemaphoreTake(MutexTasky,portMAX_DELAY))
      {
        if(taskKosticky[x]!=NULL)
        {
          eTaskState myStatus;
          myStatus=eTaskGetState(taskKosticky[x]);
          if(myStatus ==eDeleted)
          {
           // Serial.println("task pointer vynulovany!");
            taskKosticky[x]=NULL;
          }
        }
        xSemaphoreGive(MutexTasky);
      }
    }  
    
    while (xSemaphoreTake(MutexScore,portMAX_DELAY)!=pdTRUE)
    {
      vTaskDelay(5);
      log_e("Waiting to MutexScore");
    }
    LocalAmiZivot=AmiZivot;
    LocalAmiActualScore=AmiActualScore;
    xSemaphoreGive(MutexScore);
    
    if(PrintAmiActualScore!=LocalAmiActualScore)
   {
    PrintAmiActualScore=LocalAmiActualScore;
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
      lv_label_set_text_fmt(ActScore,"Score:%d",PrintAmiActualScore);
      xSemaphoreGive(xGuiSemaphore);
    }
    //////Vykresli Actual Score
   }

   if(PrintAmiZivot!=LocalAmiZivot)
   {
    PrintAmiZivot=LocalAmiZivot;
    //////Vykresli Ami Zivot
    log_e("Vykreslujem Ami Zivot :%d",PrintAmiZivot);
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {  //Vykreslenie pohybu kosticky
      lv_bar_set_value(ZivotBar,PrintAmiZivot,LV_ANIM_ON);
      xSemaphoreGive(xGuiSemaphore);
    }
   }
    
   if(LocalAmiZivot>0) // Logika hry
   {
    TickType_t currTime=xTaskGetTickCount();
    deltaTforK=currTime-GenerovanieKStartTime;
    //log_e("LocalAmiZivot:%d, deltaTime:%d, actualTime:%d, GenkST:%d",LocalAmiZivot,deltaTforK,currTime,GenerovanieKStartTime);
    
    if(deltaTforK>dalsiaKostickaDelay)
    {
      if(xPortGetFreeHeapSize()>4000)
      {
        GenerovanieKStartTime=currTime;
        for(int8_t x=0;x<maxPocetKosticiek;x++) // vygenerovanie kosticky
        {
          if(pdTRUE==xSemaphoreTake(MutexTasky,portMAX_DELAY))
          {
            //log_e("TaskKosticky x:%d, value:%d",x,taskKosticky[x]);
            if(taskKosticky[x]==NULL)
            {
              int KostickaPositionSeed=random(0,4);
              //log_e("random Cislo:%d",KostickaPositionSeed);
              //log_e("Kosticka [%d] Speed:%d",x,kostickaSpeed);
              AmiKosticka =  {InitPozicie[KostickaPositionSeed],kostickaSpeed,0};
              const char * TaskName="Kosticka"+x;
              xTaskCreate(Kosticka,TaskName,2000,&AmiKosticka,1,&taskKosticky[x]);
              kostickaSpeed=kostickaSpeed+RychlostHry;
              if(dalsiaKostickaDelay>RychlostHry)
              {
                dalsiaKostickaDelay=dalsiaKostickaDelay-RychlostHry;
              }
              xSemaphoreGive(MutexTasky);
              break;
            }
            else
            {
              xSemaphoreGive(MutexTasky);
            }
          }
          
        }
      }
    }
   }
   else
   {
    if(bestScore<AmiActualScore)
    {
      log_e("Zapisujem Najlepsie skore:%d",AmiActualScore);
      EEPROM.writeInt(MemAmiMaxScore,AmiActualScore);
      EEPROM.commit();
    }
    // Zastav kosticky, vynuluj pamat, a vymaz tento task
    
    
    //Zobraz Score Tabulku, Zmen Ami Obrazok
     while (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)!=pdTRUE)
    {
      vTaskDelay(5);
      log_e("Waiting to xGuiSemaphore");
    }
      lv_group_remove_all_objs(MyControlGroup);  //Zruš Ble  ovladanie
      lv_bar_set_value(ZivotBar,PrintAmiZivot,LV_ANIM_OFF);
    
      lv_img_set_src(ui_Ami,&ui_img_amicatch_png);
      lv_obj_set_x( ui_Ami, 8 );
      lv_obj_set_y( ui_Ami, 73 );
      lv_img_set_zoom(ui_Ami,150);
      lv_obj_move_foreground(ui_NahraneScore);
      if(bestScore<AmiActualScore)
      {
        lv_label_set_text_fmt(ui_NahraneScore,"Vitaz/ka!\n najlepsie skore!\n Tvoje Score:%d",AmiActualScore);
      }
      else
      {
        lv_label_set_text_fmt(ui_NahraneScore,"Prehral/a si :-(\nNajlepsie skore je:%d, Tvoje score:%d",bestScore,AmiActualScore);
      }
      
      xSemaphoreGive(xGuiSemaphore);

    vTaskDelay(5000);  
    vTaskDelete(NULL);
   }

  vTaskDelay(100);

  }
}

#endif

// Vykreslovanie displayu
void lv_timer_han(void *param)
{
  if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {

      lv_init();

    #if LV_USE_LOG != 0
        lv_log_register_print_cb( my_print ); /* register print function for debugging */
    #endif

    tft.begin();          /* TFT init */
    tft.setRotation( 1 ); /* Landscape orientation, flipped */
    uint16_t CallData[5]={300,3600,88,3781,7};
    tft.setTouch(CallData);
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
    lv_indev_drv_init( &indev_drv2 );
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = my_Keyboard_read;
    KeyDriver= lv_indev_drv_register( &indev_drv );

    indev_drv2.type=LV_INDEV_TYPE_POINTER;
    indev_drv2.read_cb=myTOuch_read;
    TouchDriver=lv_indev_drv_register(&indev_drv2);
    //lv_indev_delete(TouchDriver);




    ui_init();
    xSemaphoreGive(xGuiSemaphore);
  }
  int cnt=0;
  for(;;)
  {

  if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
    lv_task_handler();
    xSemaphoreGive(xGuiSemaphore);
  }
          // lv_timer_handler_run_in_period(5);

  vTaskDelay(5);

  }

}

///Uvodna Obrazovka, co sa napise a nastavi ked sa Utopia pripoji
void initializeAfterConnect()
{
  if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
    lv_obj_add_state(ui_Spinner1,LV_STATE_DISABLED);
    lv_textarea_set_text(ui_TextArea1,"Utopia Connected");
    lv_textarea_set_text(ui_TextArea2,"Press 'A' key for continue");
    lv_group_add_obj(MyControlGroup,ui_TextArea2);
    lv_indev_set_group(KeyDriver,MyControlGroup);
    xSemaphoreGive(xGuiSemaphore);
  }


}

void SettingsMenu (void * param)
{
  BrickMaxScore=EEPROM.readInt(MemBrickMaxScore);
  AmiMaxScore=EEPROM.readInt(MemAmiMaxScore);
  while( xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)!=pdTRUE)
  {
    log_e("waiting to xGuiSemaphore mutex");
    vTaskDelay(5);
  }
  lv_group_remove_all_objs(MyControlGroup);
    lv_group_add_obj(MyControlGroup,ui_VolumeSlider); 
    lv_group_add_obj(MyControlGroup,ui_BrickScoreResetButton);
    lv_group_add_obj(MyControlGroup,ui_BrickScoreResetButton1);
    lv_group_add_obj(MyControlGroup,ui_BackSettingsButton);
    

  lv_slider_set_value(ui_VolumeSlider,hlasitost,LV_ANIM_OFF);
  lv_label_set_text_fmt(ui_BrickBestScoreSettingsValue,"Brick Break:%d",BrickMaxScore);
  lv_label_set_text_fmt(ui_WolfBestScoreSettingsValue,"Ami:%d",AmiMaxScore);
  xSemaphoreGive(xGuiSemaphore);

  for(;;)
    {
    
    if(Roller!=EnumSetting)
    {
      vTaskDelete(NULL);
    }

    if(Settings==EnumVynulujAmiMaxScore)
    {
      Settings=EnumNic;
      EEPROM.writeInt(MemAmiMaxScore,0);
      EEPROM.commit();
    }

    if(Settings==EnumVynulujBrickMaxScore)
    {
      Settings=EnumNic;
      EEPROM.writeInt(MemBrickMaxScore,0);
      EEPROM.commit();
    }

      if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
      //hlasitost
      int volume=lv_slider_get_value(ui_VolumeSlider);
      //log_e("volume:%d",volume);
      xSemaphoreGive(xGuiSemaphore);
      EEPROM.writeInt(MemHlasitost,volume);
      hlasitost=volume;
    }
  }
  vTaskDelay(200);

}

///Exekucia kodu este neviem na co vsetko ma sluzit
void lv_exec(void *param)
{
  if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
    MyControlGroup=lv_group_create();
    xSemaphoreGive(xGuiSemaphore);
  }

  for(;;)
  {
    static bool wasConnected=false;  // priznak ci bol uz pripojeny bluetooth alebo nie
    vTaskDelay(100);

    if(!connected)
    {
      if(wasConnected) // ak bol pripojeny ale uz nie je -> vratit na prvy screen
      {
        wasConnected=false;
        Roller=EnumBleVyhladavac; // Vraciam na prvy screen
        Serial.println("Bluetooth disconnected -> Vraciam na prvy screen");
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
          lv_group_remove_all_objs(MyControlGroup);
          _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
          lv_obj_clear_state(ui_Spinner1,LV_STATE_DISABLED);
          lv_textarea_set_text(ui_TextArea1,"Waiting for Utopia 360");
          lv_textarea_set_text(ui_TextArea2,"Utopia was disconnected!\n Waiting for new connection");
          xSemaphoreGive(xGuiSemaphore);
        }
      }
    }






    if(connected==true && wasConnected==false)  // Je pripojeny a nebol Pripojeny
      {
        Serial.println("Je pripojeny a nebol Pripojeny");
        wasConnected=true;
        initializeAfterConnect(); //Je pripojeny
        Roller=EnumBleVyhladany;
      }

    if(connected && Roller==EnumBleVyhladavac)
    {
      Serial.println("Moze nastat tento stav?");
      Roller=EnumBleVyhladany;
      initializeAfterConnect(); // Pripojeny prvy krat

      //Serial.println("Utopia connected");
    }

    if(connected && Roller==EnumBleVyhladany)
    {
      ;
    }

    if(connected && Roller==EnumBrickStart)
    {

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
          xTaskCreate(BallMove,"BallMove",5000,NULL,1,NULL);
          lv_obj_set_x(ui_Ball,(lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionX);
          lv_obj_set_y(ui_Ball,(lv_coord_t)MojaPrvaHra.HP.Lopty[0].BallActualPositionY);
          xSemaphoreGive(xGuiSemaphore);
        }
        Roller=EnumBrickGame;
    }

    if(connected && Roller==EnumAmiStart)
    {
      Roller=EnumAmiHra;
      xTaskCreate(AmiGame,"AmiGame",5000,NULL,1,&taskAmiGame);
      if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
          xSemaphoreGive(xGuiSemaphore);
      }
    }

    if(Roller == EnumAmiHra)
    {
      while (xSemaphoreTake(MutexTasky,(TickType_t)50)!=pdTRUE)
      {
        log_e("Waiting to MutexTasky");
        vTaskDelay(5);
      }
        if(taskAmiGame!=NULL)
        {
          static eTaskState AktTask;
          AktTask=eTaskGetState(taskAmiGame);
          if(AktTask==eDeleted )
          {
            taskAmiGame=NULL;
          }   
        }
      xSemaphoreGive(MutexTasky);

      if(taskAmiGame==NULL)
      {
        while (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)!=pdTRUE)
        {
          log_e("Waiting to semapthore");
          vTaskDelay(5);
        }
        log_e("z Ami na Menu");
        lv_group_remove_all_objs(MyControlGroup);
        _ui_screen_change( &ui_Menu, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Menu_screen_init);
        lv_group_add_obj(MyControlGroup,ui_PanelBlrickGame);
        lv_group_add_obj(MyControlGroup,ui_PanelWolfGame);
        lv_group_add_obj(MyControlGroup,ui_SettingsButton);
        xSemaphoreGive(xGuiSemaphore);
        Roller=EnumMenu;
      }
         
    }

    if(connected && Roller==EnumSetting)
    {

      xTaskCreate(SettingsMenu,"SettingsMenu",3000,NULL,1,&taskSettings);

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
    vTaskDelay(5000);

    for(int x=0;x<5;x++)
    { //   static int cnt=0;
      Serial.print("STACK [");
      Serial.print(x);
      Serial.print("] left:");
      Serial.println(uxTaskGetStackHighWaterMark(taskHandles[x]));
    }

    
    for(int x=0;x<maxPocetKosticiek;x++)
    { //   static int cnt=0;
      
      if(pdTRUE==xSemaphoreTake(MutexTasky,portMAX_DELAY))
      {
        eTaskState myStatus;
        uint taskHiwhWaterMark;

        if(taskKosticky[x]!=NULL)
        {
          myStatus=eTaskGetState(taskKosticky[x]);
          if(myStatus !=eDeleted)
            taskHiwhWaterMark=uxTaskGetStackHighWaterMark(taskKosticky[x]);
        }
        xSemaphoreGive(MutexTasky);

        if(myStatus !=eDeleted && taskKosticky[x]!=NULL)
        {
          Serial.print(" STACK Kosticky [");
          Serial.print(x);
          Serial.print("] left:");
          Serial.println(taskHiwhWaterMark);
        }
      }
    }

  Serial.print("Total HEAP: ");
  Serial.println(ESP.getHeapSize());

  Serial.print("Free HEAP: ");
  Serial.println(xPortGetFreeHeapSize());

  Serial.print("TOTAL PSRAM: ");
  Serial.println(ESP.getPsramSize());
  Serial.print("Free PSRAM: ");
  Serial.println(ESP.getFreePsram());

 }
}

void Reprak(void *param)
{
  static int ton=0;
  for(;;)
  {
    static int del=100;
    if(xQueueReceive(reprakQueue,(void *)&ton,0)==pdTRUE)
    {
      //log_e("ton:%d",ton);
      if(ton>8)
      {
        log_e("Error ton je vysoky, nastavujem na 8");
        ton=8;
      }
      if(ton>=8)
      {
        del=300;
      }
      else
      {del=100;}

      ledcWriteNote(0,(note_t)ton,ton);
      ledcWrite(0,hlasitost);
      vTaskDelay(del);
      ledcWrite(0,0);
    } 
    vTaskDelay(50);
  }
}

void setup()
{
  #if wemos_d1_mini32_LVGL_1Pok_SPI_Velky
    pinMode(TFT_BL,OUTPUT);
    digitalWrite(TFT_BL,HIGH);
  #endif
    pinMode(reprakPin,OUTPUT);

    
    //ledcSetup(0, 5000, 8);
    ledcAttachPin(reprakPin, 0);
    ledcWrite(0,0);


    Serial.begin( 115200 ); ///* prepare for possible serial debug ///
    Serial.print("CPU FREQUENCY:");
    Serial.println(getCpuFrequencyMhz());
    Serial.print("init Heap:");
    Serial.println(xPortGetFreeHeapSize());
    Serial.printf("1. Mun of tasks:%d\n",uxTaskGetNumberOfTasks());

    
    xGuiSemaphore=xSemaphoreCreateBinary();
    xSemaphoreGive(xGuiSemaphore);
    MutexTasky=xSemaphoreCreateBinary();
    xSemaphoreGive(MutexTasky);


    BleKeyboardQueue=xQueueCreate(10, sizeof(BleKey));
    reprakQueue=xQueueCreate(10,sizeof(int));
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );


    EEPROM.begin(200);
    hlasitost=EEPROM.readInt(MemHlasitost);
    AmiMaxScore=EEPROM.readInt(MemAmiMaxScore);
    BrickMaxScore=EEPROM.readInt(MemBrickMaxScore);

   // log_e("Hlasitost:%d, AmiMaxScore:%d BrickMaxScore:%d",hlasitost, AmiMaxScore,BrickMaxScore);
    


    //
    //xTaskCreate(MySerialDebug,"MySerialDebug",2024,NULL,1,&taskHandles[0]);
    Serial.print("Free HEAP after MySerialDebug:");
    Serial.println(xPortGetFreeHeapSize());
    MutexScore=xSemaphoreCreateMutex();

    xTaskCreate(Reprak,"Reprak",2024,NULL,1,NULL);  

    xTaskCreatePinnedToCore(lv_timer_han,"lv_timer_handler",7000,NULL,3,&taskHandles[1],0);
    Serial.print("Free HEAP after lv_timer_handler:");
    Serial.println(xPortGetFreeHeapSize());
    xTaskCreatePinnedToCore(lv_exec,"lv_exec",3000,NULL,3,&taskHandles[2],0);
    Serial.print("Free HEAP after lv_exec:");
    Serial.println(xPortGetFreeHeapSize());
    xTaskCreatePinnedToCore(MyBluetooth,"MyBluetooth",3000,NULL,1,&taskHandles[3],1);
    Serial.print("Free HEAP after MyBluetooth:");
    Serial.println(xPortGetFreeHeapSize());

    Serial.println( "Setup done" );
    log_e("setup","kontrola log_e");
/*
for(int x=0;x<=11;x++)
{
    if(xQueueSend(reprakQueue,(void*)&x,10)!=pdTRUE)
          {Serial.println("Queue sending problem ");}
          vTaskDelay(1000);
}   
*/
}

void loop()
{
    //lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(50);
    //Serial.print(uxTaskPriorityGet(NULL));

}

