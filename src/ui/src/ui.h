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
#include "ui_events.h"
// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1( lv_event_t * e);
extern lv_obj_t *ui_Screen1;
extern lv_obj_t *ui_Spinner1;
extern lv_obj_t *ui_TextArea1;
extern lv_obj_t *ui_TextArea2;
// SCREEN: ui_Menu
void ui_Menu_screen_init(void);
void ui_event_Menu( lv_event_t * e);
extern lv_obj_t *ui_Menu;
void ui_event_PanelBlrickGame( lv_event_t * e);
extern lv_obj_t *ui_PanelBlrickGame;
void ui_event_PanelWolfGame( lv_event_t * e);
extern lv_obj_t *ui_PanelWolfGame;
// SCREEN: ui_BrickBall
void ui_BrickBall_screen_init(void);
void ui_event_BrickBall( lv_event_t * e);
extern lv_obj_t *ui_BrickBall;
extern lv_obj_t *ui_Panel2;
extern lv_obj_t *ui_Brick;
extern lv_obj_t *ui_Ball;
extern lv_obj_t *ui_Nest;
// SCREEN: ui_Wolf
void ui_Wolf_screen_init(void);
void ui_event_Wolf( lv_event_t * e);
extern lv_obj_t *ui_Wolf;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_brickball_png);   // assets\brickBall.png
LV_IMG_DECLARE( ui_img_wolf_png);   // assets\wolf.png



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
