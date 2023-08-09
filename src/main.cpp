#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <esp_task_wdt.h>


static SemaphoreHandle_t mutex;


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
            lv_timer_handler();
            xSemaphoreGive(mutex);
            vTaskDelay(50);
        }
        else
        {
            Serial.print("Wait for Mutex LV_up");
        }
        //Serial.print("task Executed");
        // vTaskDelay(5);



        // if(cnt==0 && millis()>5000)
        // {   cnt++;
        //     char str[20];
        //     sprintf(str,"xTaskGetTickCount:%d\n",xTaskGetTickCount());
        //     Serial.print(str);
        //     lv_event_t *ev=new lv_event_t();
        //     ev->code=LV_EVENT_PRESSED;
        //     ev->target=ui_Clock;

        //     ui_event_Clock(ev);
        //     Serial.print("Clicked 1");

        //     sprintf(str, "Task is running on the core: %d, \n\0",xPortGetCoreID());
        //     Serial.print(str);

        //     sprintf(str,"xTaskGetTickCount:%i \n\0",xTaskGetTickCount());
        //     Serial.print(str);

        // }

        // if(cnt==1 && millis()>10000)
        // {
        //     lv_event_t *ev=new lv_event_t();
        //     ev->code=LV_EVENT_PRESSED;
        //     ev->target=ui_Call;
        //     Serial.print("Clicked 2");
        //     ui_event_Call(ev);

        //     cnt++;
        // }



    //    Serial.print(xTaskGetTickCount());
    //    Serial.print(", ");
    //    Serial.print(millis());
    //    Serial.print(" \n");
    //Serial.print("-");
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
    Serial.print("I am in the function");
    int cnt=0;
    for(;;)
    {
        
        vTaskDelay(500);
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

void nejakaF(void * param)
{   
    Serial.print("I am in the function");
    int cnt=0;
    for(;;)
    {
        Serial.print("Kurva!!!!");
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );


    //

    mutex=xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(lv_timer_han,"lv_timer_handler",10024,NULL,2,NULL,1);
    xTaskCreatePinnedToCore(lv_exec,"lv_exec",5024,NULL,3,NULL,1);
    //xTaskCreatePinnedToCore(nejakaF,"NejakaskurvenaFCIA",1024,NULL,1,NULL,1);
    //xTaskCreate(lv_exec,"lv_exec",100024,NULL,3,NULL);
    //xTaskCreatePinnedToCore(Wdt_reset,"watchdog",1024,NULL,1,NULL,0);
    Serial.println( "Setup done" );


}

void loop()
{
    //lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(50);
    //Serial.print(uxTaskPriorityGet(NULL));

}

