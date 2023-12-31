// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"


///////////////////// VARIABLES ////////////////////
lv_group_t * MyControlGroup;
// enum Obrazovka
// {
//   BleVyhladavac,BleVyhladany,Menu,Brick,Ami,Setting
// };

int Roller=EnumBleVyhladavac; // hovori ktora obrazovka je spustena
int Settings=EnumNic;
char AmiPosition; // Pozícia Ami


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
lv_obj_t *ui_Screen1;
lv_obj_t *ui_Spinner1;
lv_obj_t *ui_TextArea1;
void ui_event_TextArea2( lv_event_t * e);
lv_obj_t *ui_TextArea2;


// SCREEN: ui_Menu
void ui_Menu_screen_init(void);
void ui_event_Menu( lv_event_t * e);
lv_obj_t *ui_Menu;
void ui_event_PanelBlrickGame( lv_event_t * e);
lv_obj_t *ui_PanelBlrickGame;
void ui_event_PanelWolfGame( lv_event_t * e);
lv_obj_t *ui_PanelWolfGame;
void ui_event_SettingsButton( lv_event_t * e);
lv_obj_t *ui_SettingsButton;
lv_obj_t *ui_Label3;


// SCREEN: ui_BrickBall
void ui_BrickBall_screen_init(void);
void ui_event_BrickBall( lv_event_t * e);
lv_obj_t *ui_BrickBall;
lv_obj_t *ui_GameArea1;
lv_obj_t *ui_Brick;
lv_obj_t *ui_Ball;
lv_obj_t *ui_Nest;
lv_obj_t *ui_StatusPanelBrick;


// SCREEN: ui_Wolf
void ui_Wolf_screen_init(void);
void ui_event_Wolf( lv_event_t * e);
lv_obj_t *ui_Wolf;
lv_obj_t *ui_GameArea2;
lv_obj_t *ui_NahraneScore;
lv_obj_t *ui_Image2;
void ui_event_Ami( lv_event_t * e);
lv_obj_t *ui_Ami;
//lv_obj_t *ui_Image6;
lv_obj_t *ui_StatusPanelWolf;


// SCREEN: ui_Settings
void ui_Settings_screen_init(void);
lv_obj_t *ui_Settings;
lv_obj_t *ui_SettingsLable;
lv_obj_t *ui_VolumePanel;
lv_obj_t *ui_LabelVolMax1;
lv_obj_t *ui_LabelVolMax;
lv_obj_t *ui_VolumeSlider;
lv_obj_t *ui_LabelVolume;
lv_obj_t *ui_PanelGameSettings;
lv_obj_t *ui_LabelGameScoreReset;
void ui_event_BrickScoreResetButton( lv_event_t * e);
lv_obj_t *ui_BrickScoreResetButton;
lv_obj_t *ui_Label1;
lv_obj_t *ui_BrickBestScoreSettingsValue;
lv_obj_t *ui_WolfBestScoreSettingsValue;
void ui_event_BrickScoreResetButton1( lv_event_t * e);
lv_obj_t *ui_BrickScoreResetButton1;
lv_obj_t *ui_Label2;
void ui_event_BackSettingsButton( lv_event_t * e);
lv_obj_t *ui_BackSettingsButton;
lv_obj_t *ui_Label12;
lv_obj_t *ui_PanelDeviceInfo;
lv_obj_t *ui_LabelDeviceInfo;
lv_obj_t *ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_TextArea2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        lv_group_add_obj(MyControlGroup,ui_PanelBlrickGame);
        lv_group_add_obj(MyControlGroup,ui_PanelWolfGame);
        lv_group_add_obj(MyControlGroup,ui_SettingsButton);
        _ui_screen_change( &ui_Menu, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_Menu_screen_init);
        Roller=EnumMenu;
    }
}
void ui_event_Menu( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
        Roller=EnumBleVyhladavac;
    }
}
void ui_event_PanelBlrickGame( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        //lv_group_remove_all_objs(MyControlGroup);
        //_ui_screen_change( &ui_BrickBall, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_BrickBall_screen_init);
        //Roller=EnumBrickStart;
    }
}
void ui_event_PanelWolfGame( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        lv_group_add_obj(MyControlGroup,ui_Ami);
        _ui_screen_change( &ui_Wolf, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Wolf_screen_init);
        Roller=EnumAmiStart;
    }
}
void ui_event_SettingsButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        _ui_screen_change( &ui_Settings, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_Settings_screen_init);
        Roller=EnumSetting;
    }
}
void ui_event_BrickBall( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
        Roller=EnumBleVyhladavac;
    }
}
void ui_event_Wolf( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
        Roller=EnumBleVyhladavac;
    }
}

void ui_event_Ami( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    //log_e(event_code,"EventAmiRight_fc");
    if ( event_code == LV_EVENT_KEY &&  lv_event_get_key(e) == LV_KEY_UP  ) {
        EventAmiUP_fc( e );
    }
    if ( event_code == LV_EVENT_KEY &&  lv_event_get_key(e) == LV_KEY_DOWN  ) {
        EventAmiDown_fc( e );
    }
    if ( event_code == LV_EVENT_KEY &&  lv_event_get_key(e) == LV_KEY_LEFT  ) {
        EventAmiLeft_fc( e );
    }
    if ( event_code == LV_EVENT_KEY &&  lv_event_get_key(e) == LV_KEY_RIGHT  ) {
        EventAmiRight_fc( e );
    }
}

void ui_event_BrickScoreResetButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        Settings=EnumVynulujBrickMaxScore;
        lv_label_set_text_fmt(ui_BrickBestScoreSettingsValue,"Brick Break:0");
      
        
    }
}
void ui_event_BrickScoreResetButton1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        Settings=EnumVynulujAmiMaxScore;
        lv_label_set_text_fmt(ui_WolfBestScoreSettingsValue,"Ami:0");
        //ResetScoreWolfFunction( e );
    }
}
void ui_event_BackSettingsButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    if ( event_code == LV_EVENT_CLICKED) {
        lv_group_remove_all_objs(MyControlGroup);
        lv_group_add_obj(MyControlGroup,ui_PanelBlrickGame);
        lv_group_add_obj(MyControlGroup,ui_PanelWolfGame);
        lv_group_add_obj(MyControlGroup,ui_SettingsButton);
        _ui_screen_change( &ui_Menu, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_Menu_screen_init);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init( void )
{
    LV_EVENT_GET_COMP_CHILD = lv_event_register_id();
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Menu_screen_init();
    ui_BrickBall_screen_init();
    ui_Wolf_screen_init();
    ui_Settings_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr( ui_Screen1);
    //lv_disp_load_scr(ui_Settings);
}
