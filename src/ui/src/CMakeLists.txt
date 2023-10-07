SET(SOURCES components/ui_comp_statuspanel.c
    components/ui_comp.c
    screens/ui_Screen1.c
    screens/ui_Menu.c
    screens/ui_BrickBall.c
    screens/ui_Wolf.c
    screens/ui_Settings.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_brickball_png.c
    images/ui_img_wolf_png.c
    images/ui_img_wolfbg_png.c
    images/ui_img_ami3_png.c
    images/ui_img_391577990.c
    images/ui_img_pot_ver_line_png.c
    images/ui_img_pot_ver_knob_png.c
    images/ui_img_wolf2_png.c
    images/ui_img_wolflayout2_png.c
    images/ui_img_wolflayout3_png.c
    images/ui_img_wolflayout7_png.c
    images/ui_img_ami1_png.c
    images/ui_img_ami2_png.c
    images/ui_img_210898296.c
    images/ui_img_1771879469.c)

add_library(ui ${SOURCES})
