SET(SOURCES screens/ui_Screen1.c
    screens/ui_Menu.c
    screens/ui_BrickBall.c
    screens/ui_Wolf.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_brickball_png.c
    images/ui_img_wolf_png.c)

add_library(ui ${SOURCES})
