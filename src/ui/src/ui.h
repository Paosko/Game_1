// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
  #if __has_include("lvgl.h")
    #include "lvgl.h"
  #elif __has_include("lvgl/lvgl.h")
    #include "lvgl/lvgl.h"
  #else
    #include "lvgl.h"
  #endif
#else
  #include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "components/ui_comp.h"
#include "components/ui_comp_hook.h"
#include "ui_events.h"



extern lv_group_t * MyControlGroup;
enum {
  EnumBleVyhladavac,EnumBleVyhladany,EnumMenu,EnumBrickStart,EnumBrickGame,EnumAmiStart,EnumAmiHra,EnumSetting
};
extern int Roller;

enum{
  EnumNic, EnumVynulujBrickMaxScore, EnumVynulujAmiMaxScore
};
extern int Settings;


enum {
  EnumAmiTopLeft,EnumAmiBottomLeft,EnumAmiTopRight,EnumAmiBottomRight
};
extern char AmiPosition;


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t *ui_Screen1;
extern lv_obj_t *ui_Spinner1;
extern lv_obj_t *ui_TextArea1;
void ui_event_TextArea2( lv_event_t * e);
extern lv_obj_t *ui_TextArea2;
// SCREEN: ui_Menu
void ui_Menu_screen_init(void);
void ui_event_Menu( lv_event_t * e);
extern lv_obj_t *ui_Menu;
void ui_event_PanelBlrickGame( lv_event_t * e);
extern lv_obj_t *ui_PanelBlrickGame;
void ui_event_PanelWolfGame( lv_event_t * e);
extern lv_obj_t *ui_PanelWolfGame;
void ui_event_SettingsButton( lv_event_t * e);
extern lv_obj_t *ui_SettingsButton;
extern lv_obj_t *ui_Label3;
// SCREEN: ui_BrickBall
void ui_BrickBall_screen_init(void);
void ui_event_BrickBall( lv_event_t * e);
extern lv_obj_t *ui_BrickBall;
extern lv_obj_t *ui_GameArea1;
extern lv_obj_t *ui_Brick;
extern lv_obj_t *ui_Ball;
extern lv_obj_t *ui_Nest;
extern lv_obj_t *ui_StatusPanelBrick;
// SCREEN: ui_Wolf
void ui_Wolf_screen_init(void);
void ui_event_Wolf( lv_event_t * e);
extern lv_obj_t *ui_Wolf;
extern lv_obj_t *ui_GameArea2;
extern lv_obj_t *ui_NahraneScore;
extern lv_obj_t *ui_Image2;
void ui_event_Ami( lv_event_t * e);
extern lv_obj_t *ui_Ami;
extern lv_obj_t *ui_Image6;
extern lv_obj_t *ui_StatusPanelWolf;
// SCREEN: ui_Settings
void ui_Settings_screen_init(void);
extern lv_obj_t *ui_Settings;
extern lv_obj_t *ui_SettingsLable;
extern lv_obj_t *ui_VolumePanel;
extern lv_obj_t *ui_LabelVolMax1;
extern lv_obj_t *ui_LabelVolMax;
extern lv_obj_t *ui_VolumeSlider;
extern lv_obj_t *ui_LabelVolume;
extern lv_obj_t *ui_PanelGameSettings;
extern lv_obj_t *ui_LabelGameScoreReset;
void ui_event_BrickScoreResetButton( lv_event_t * e);
extern lv_obj_t *ui_BrickScoreResetButton;
extern lv_obj_t *ui_Label1;
extern lv_obj_t *ui_BrickBestScoreSettingsValue;
extern lv_obj_t *ui_WolfBestScoreSettingsValue;
void ui_event_BrickScoreResetButton1( lv_event_t * e);
extern lv_obj_t *ui_BrickScoreResetButton1;
extern lv_obj_t *ui_Label2;
void ui_event_BackSettingsButton( lv_event_t * e);
extern lv_obj_t *ui_BackSettingsButton;
extern lv_obj_t *ui_Label12;
extern lv_obj_t *ui_PanelDeviceInfo;
extern lv_obj_t *ui_LabelDeviceInfo;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_brickball_png);   // assets\brickBall.png
LV_IMG_DECLARE( ui_img_wolfbg_png);   // assets\wolfbg.png
LV_IMG_DECLARE( ui_img_ami3_png);   // assets\Ami3.png
LV_IMG_DECLARE( ui_img_ami3_mirrored_png);   // assets\Ami3_mirrored.png
LV_IMG_DECLARE( ui_img_391577990);   // assets\bone-32.png
LV_IMG_DECLARE( ui_img_amicatch_png);   // assets\amiCatch.png
LV_IMG_DECLARE( ui_img_pot_ver_line_png);   // assets\pot_ver_line.png
LV_IMG_DECLARE( ui_img_pot_ver_knob_png);   // assets\pot_ver_knob.png
//LV_IMG_DECLARE( ui_img_1665173421);   // assets\pokusy\bone-157272_640.png
//LV_IMG_DECLARE( ui_img_847650150);   // assets\pokusy\bone-64.png
//LV_IMG_DECLARE( ui_img_pokusy_wolf2_png);   // assets\pokusy\wolf2.png
//LV_IMG_DECLARE( ui_img_pokusy_wolflayout2_png);   // assets\pokusy\wolfLayout2.png
//LV_IMG_DECLARE( ui_img_pokusy_wolflayout3_png);   // assets\pokusy\wolfLayout3.png
//LV_IMG_DECLARE( ui_img_pokusy_wolflayout7_png);   // assets\pokusy\wolfLayout7.png
LV_IMG_DECLARE( ui_img_wolf_png);   // assets\wolf2.png
//LV_IMG_DECLARE( ui_img_1771879469);   // assets\bone-64.png



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
