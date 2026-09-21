//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[11];
extern ui_interface_string_t ui_g_now_strings[4];
extern uint8_t ui_g_dirty_figure[11];
extern uint8_t ui_g_dirty_string[4];

#define ui_g_uplift_uplift_top ((ui_interface_line_t*)&(ui_g_now_figures[0]))
#define ui_g_uplift_mid_uplift_bar ((ui_interface_line_t*)&(ui_g_now_figures[1]))
#define ui_g_uplift_uplift_bottom ((ui_interface_line_t*)&(ui_g_now_figures[2]))
#define ui_g_uplift_moving_range ((ui_interface_rect_t*)&(ui_g_now_figures[3]))
#define ui_g_uplift_uplift_cur_pos ((ui_interface_line_t*)&(ui_g_now_figures[4]))
#define ui_g_ARM_L1 ((ui_interface_line_t*)&(ui_g_now_figures[5]))
#define ui_g_ARM_L2 ((ui_interface_line_t*)&(ui_g_now_figures[6]))
#define ui_g_ARM_L3 ((ui_interface_line_t*)&(ui_g_now_figures[7]))
#define ui_g_ground_catch_ground_catch_mark ((ui_interface_ellipse_t*)&(ui_g_now_figures[8]))
#define ui_g_camera_PITCH_V ((ui_interface_number_t*)&(ui_g_now_figures[9]))
#define ui_g_camera_YAW_V ((ui_interface_number_t*)&(ui_g_now_figures[10]))

#define ui_g_camera_PITCH (&(ui_g_now_strings[0]))
#define ui_g_camera_YAW (&(ui_g_now_strings[1]))
#define ui_g_camera_PUMP (&(ui_g_now_strings[2]))
#define ui_g_camera_PUMP_V (&(ui_g_now_strings[3]))

#ifdef MANUAL_DIRTY
#define ui_g_uplift_uplift_top_dirty (ui_g_dirty_figure[0])
#define ui_g_uplift_mid_uplift_bar_dirty (ui_g_dirty_figure[1])
#define ui_g_uplift_uplift_bottom_dirty (ui_g_dirty_figure[2])
#define ui_g_uplift_moving_range_dirty (ui_g_dirty_figure[3])
#define ui_g_uplift_uplift_cur_pos_dirty (ui_g_dirty_figure[4])
#define ui_g_ARM_L1_dirty (ui_g_dirty_figure[5])
#define ui_g_ARM_L2_dirty (ui_g_dirty_figure[6])
#define ui_g_ARM_L3_dirty (ui_g_dirty_figure[7])
#define ui_g_ground_catch_ground_catch_mark_dirty (ui_g_dirty_figure[8])
#define ui_g_camera_PITCH_V_dirty (ui_g_dirty_figure[9])
#define ui_g_camera_YAW_V_dirty (ui_g_dirty_figure[10])

#define ui_g_camera_PITCH_dirty (ui_g_dirty_string[0])
#define ui_g_camera_YAW_dirty (ui_g_dirty_string[1])
#define ui_g_camera_PUMP_dirty (ui_g_dirty_string[2])
#define ui_g_camera_PUMP_V_dirty (ui_g_dirty_string[3])
#endif

void ui_init_g(void);
void ui_update_g(void);

#endif //UI_g_H
