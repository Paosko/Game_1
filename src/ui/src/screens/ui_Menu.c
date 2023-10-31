// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Menu_screen_init(void)
{
Roller=EnumMenu;  // Zda sa ze nefunguje
ui_Menu = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Menu, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Menu, lv_color_hex(0x4C4E5B), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Menu, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Menu, lv_color_hex(0x393B46), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_main_stop(ui_Menu, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_stop(ui_Menu, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_Menu, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PanelBlrickGame = lv_obj_create(ui_Menu);
lv_obj_set_width( ui_PanelBlrickGame, 150);
lv_obj_set_height( ui_PanelBlrickGame, 85);
lv_obj_set_x( ui_PanelBlrickGame, -100 );
lv_obj_set_y( ui_PanelBlrickGame, 0 );
lv_obj_set_align( ui_PanelBlrickGame, LV_ALIGN_CENTER );
//lv_obj_add_state( ui_PanelBlrickGame, LV_STATE_FOCUSED );     /// States
lv_obj_clear_flag( ui_PanelBlrickGame, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_img_src( ui_PanelBlrickGame, &ui_img_brickball_png, LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_img_src( ui_PanelBlrickGame, &ui_img_brickball_png, LV_PART_MAIN | LV_STATE_FOCUSED );
lv_obj_set_style_shadow_color(ui_PanelBlrickGame, lv_color_hex(0x666666), LV_PART_MAIN | LV_STATE_FOCUSED );
lv_obj_set_style_shadow_opa(ui_PanelBlrickGame, 255, LV_PART_MAIN| LV_STATE_FOCUSED);
lv_obj_set_style_shadow_width(ui_PanelBlrickGame, 25, LV_PART_MAIN| LV_STATE_FOCUSED);
lv_obj_set_style_shadow_spread(ui_PanelBlrickGame, 15, LV_PART_MAIN| LV_STATE_FOCUSED);

ui_PanelWolfGame = lv_obj_create(ui_Menu);
lv_obj_set_width( ui_PanelWolfGame, 150);
lv_obj_set_height( ui_PanelWolfGame, 85);
lv_obj_set_x( ui_PanelWolfGame, 100 );
lv_obj_set_y( ui_PanelWolfGame, 0 );
lv_obj_set_align( ui_PanelWolfGame, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_PanelWolfGame, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_img_src( ui_PanelWolfGame, &ui_img_wolf_png, LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_img_src( ui_PanelWolfGame, &ui_img_wolf_png, LV_PART_MAIN | LV_STATE_FOCUSED );
lv_obj_set_style_shadow_color(ui_PanelWolfGame, lv_color_hex(0x00FF08), LV_PART_MAIN | LV_STATE_FOCUSED );
lv_obj_set_style_shadow_opa(ui_PanelWolfGame, 255, LV_PART_MAIN| LV_STATE_FOCUSED);
lv_obj_set_style_shadow_width(ui_PanelWolfGame, 25, LV_PART_MAIN| LV_STATE_FOCUSED);
lv_obj_set_style_shadow_spread(ui_PanelWolfGame, 15, LV_PART_MAIN| LV_STATE_FOCUSED);

ui_SettingsButton = lv_obj_create(ui_Menu);
lv_obj_set_width( ui_SettingsButton, 100);
lv_obj_set_height( ui_SettingsButton, 50);
lv_obj_set_x( ui_SettingsButton, 155 );
lv_obj_set_y( ui_SettingsButton, 110 );
lv_obj_set_align( ui_SettingsButton, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_SettingsButton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_SettingsButton, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_SettingsButton, lv_color_hex(0x9399A1), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SettingsButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_SettingsButton, lv_color_hex(0x484B4F), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_main_stop(ui_SettingsButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_stop(ui_SettingsButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_dir(ui_SettingsButton, LV_GRAD_DIR_VER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_SettingsButton, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_SettingsButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_color(ui_SettingsButton, lv_color_hex(0x00FF08), LV_PART_MAIN | LV_STATE_FOCUSED );
lv_obj_set_style_shadow_opa(ui_SettingsButton, 255, LV_PART_MAIN| LV_STATE_FOCUSED);
lv_obj_set_style_shadow_width(ui_SettingsButton, 25, LV_PART_MAIN| LV_STATE_FOCUSED);
lv_obj_set_style_shadow_spread(ui_SettingsButton, 15, LV_PART_MAIN| LV_STATE_FOCUSED);

ui_Label3 = lv_label_create(ui_SettingsButton);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"Settings");

lv_obj_add_event_cb(ui_PanelBlrickGame, ui_event_PanelBlrickGame, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_PanelWolfGame, ui_event_PanelWolfGame, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_SettingsButton, ui_event_SettingsButton, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Menu, ui_event_Menu, LV_EVENT_ALL, NULL);

}
