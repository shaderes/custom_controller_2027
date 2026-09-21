//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 11
#define TOTAL_STRING 4

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
uint8_t ui_g_dirty_string[TOTAL_STRING];
#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

void ui_init_g() {
    ui_g_uplift_uplift_top->figure_tpye = 0;
    ui_g_uplift_uplift_top->layer = 1;
    ui_g_uplift_uplift_top->start_x = 1530;
    ui_g_uplift_uplift_top->start_y = 824;
    ui_g_uplift_uplift_top->end_x = 1630;
    ui_g_uplift_uplift_top->end_y = 824;
    ui_g_uplift_uplift_top->color = 2;
    ui_g_uplift_uplift_top->width = 3;

    ui_g_uplift_mid_uplift_bar->figure_tpye = 0;
    ui_g_uplift_mid_uplift_bar->layer = 1;
    ui_g_uplift_mid_uplift_bar->start_x = 1580;
    ui_g_uplift_mid_uplift_bar->start_y = 344;
    ui_g_uplift_mid_uplift_bar->end_x = 1580;
    ui_g_uplift_mid_uplift_bar->end_y = 824;
    ui_g_uplift_mid_uplift_bar->color = 2;
    ui_g_uplift_mid_uplift_bar->width = 3;

    ui_g_uplift_uplift_bottom->figure_tpye = 0;
    ui_g_uplift_uplift_bottom->layer = 1;
    ui_g_uplift_uplift_bottom->start_x = 1530;
    ui_g_uplift_uplift_bottom->start_y = 344;
    ui_g_uplift_uplift_bottom->end_x = 1630;
    ui_g_uplift_uplift_bottom->end_y = 344;
    ui_g_uplift_uplift_bottom->color = 2;
    ui_g_uplift_uplift_bottom->width = 3;

    ui_g_uplift_moving_range->figure_tpye = 1;
    ui_g_uplift_moving_range->layer = 1;
    ui_g_uplift_moving_range->start_x = 1530;
    ui_g_uplift_moving_range->start_y = 344;
    ui_g_uplift_moving_range->color = 6;
    ui_g_uplift_moving_range->width = 3;
    ui_g_uplift_moving_range->end_x = 1630;
    ui_g_uplift_moving_range->end_y = 584;

    ui_g_uplift_uplift_cur_pos->figure_tpye = 0;
    ui_g_uplift_uplift_cur_pos->layer = 1;
    ui_g_uplift_uplift_cur_pos->start_x = 1530;
    ui_g_uplift_uplift_cur_pos->start_y = 344;
    ui_g_uplift_uplift_cur_pos->end_x = 1630;
    ui_g_uplift_uplift_cur_pos->end_y = 344;
    ui_g_uplift_uplift_cur_pos->color = 3;
    ui_g_uplift_uplift_cur_pos->width = 8;

    ui_g_ARM_L1->figure_tpye = 0;
    ui_g_ARM_L1->layer = 0;
    ui_g_ARM_L1->start_x = 363;
    ui_g_ARM_L1->start_y = 252;
    ui_g_ARM_L1->end_x = 193;
    ui_g_ARM_L1->end_y = 402;
    ui_g_ARM_L1->color = 2;
    ui_g_ARM_L1->width = 3;

    ui_g_ARM_L2->figure_tpye = 0;
    ui_g_ARM_L2->layer = 0;
    ui_g_ARM_L2->start_x = 194;
    ui_g_ARM_L2->start_y = 402;
    ui_g_ARM_L2->end_x = 335;
    ui_g_ARM_L2->end_y = 603;
    ui_g_ARM_L2->color = 2;
    ui_g_ARM_L2->width = 3;

    ui_g_ARM_L3->figure_tpye = 0;
    ui_g_ARM_L3->layer = 0;
    ui_g_ARM_L3->start_x = 333;
    ui_g_ARM_L3->start_y = 602;
    ui_g_ARM_L3->end_x = 353;
    ui_g_ARM_L3->end_y = 622;
    ui_g_ARM_L3->color = 4;
    ui_g_ARM_L3->width = 5;

    ui_g_ground_catch_ground_catch_mark->figure_tpye = 3;
    ui_g_ground_catch_ground_catch_mark->layer = 0;
    ui_g_ground_catch_ground_catch_mark->rx = 50;
    ui_g_ground_catch_ground_catch_mark->ry = 21;
    ui_g_ground_catch_ground_catch_mark->start_x = 965;
    ui_g_ground_catch_ground_catch_mark->start_y = 220;
    ui_g_ground_catch_ground_catch_mark->color = 3;
    ui_g_ground_catch_ground_catch_mark->width = 8;

    ui_g_camera_PITCH_V->figure_tpye = 6;
    ui_g_camera_PITCH_V->layer = 0;
    ui_g_camera_PITCH_V->font_size = 30;
    ui_g_camera_PITCH_V->start_x = 280;
    ui_g_camera_PITCH_V->start_y = 858;
    ui_g_camera_PITCH_V->color = 1;
    ui_g_camera_PITCH_V->number = 0;
    ui_g_camera_PITCH_V->width = 3;

    ui_g_camera_YAW_V->figure_tpye = 6;
    ui_g_camera_YAW_V->layer = 0;
    ui_g_camera_YAW_V->font_size = 30;
    ui_g_camera_YAW_V->start_x = 280;
    ui_g_camera_YAW_V->start_y = 798;
    ui_g_camera_YAW_V->color = 1;
    ui_g_camera_YAW_V->number = 0;
    ui_g_camera_YAW_V->width = 3;

    ui_g_camera_PITCH->figure_tpye = 7;
    ui_g_camera_PITCH->layer = 0;
    ui_g_camera_PITCH->font_size = 30;
    ui_g_camera_PITCH->start_x = 95;
    ui_g_camera_PITCH->start_y = 854;
    ui_g_camera_PITCH->color = 3;
    ui_g_camera_PITCH->str_length = 5;
    ui_g_camera_PITCH->width = 3;
    strcpy(ui_g_camera_PITCH->string, "PITCH");

    ui_g_camera_YAW->figure_tpye = 7;
    ui_g_camera_YAW->layer = 0;
    ui_g_camera_YAW->font_size = 30;
    ui_g_camera_YAW->start_x = 121;
    ui_g_camera_YAW->start_y = 798;
    ui_g_camera_YAW->color = 3;
    ui_g_camera_YAW->str_length = 3;
    ui_g_camera_YAW->width = 3;
    strcpy(ui_g_camera_YAW->string, "YAW");

    ui_g_camera_PUMP->figure_tpye = 7;
    ui_g_camera_PUMP->layer = 0;
    ui_g_camera_PUMP->font_size = 30;
    ui_g_camera_PUMP->start_x = 110;
    ui_g_camera_PUMP->start_y = 731;
    ui_g_camera_PUMP->color = 3;
    ui_g_camera_PUMP->str_length = 4;
    ui_g_camera_PUMP->width = 3;
    strcpy(ui_g_camera_PUMP->string, "PUMP");

    ui_g_camera_PUMP_V->figure_tpye = 7;
    ui_g_camera_PUMP_V->layer = 0;
    ui_g_camera_PUMP_V->font_size = 30;
    ui_g_camera_PUMP_V->start_x = 262;
    ui_g_camera_PUMP_V->start_y = 731;
    ui_g_camera_PUMP_V->color = 1;
    ui_g_camera_PUMP_V->str_length = 2;
    ui_g_camera_PUMP_V->width = 3;
    strcpy(ui_g_camera_PUMP_V->string, "ON");


    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_tpyel = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_tpyel = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_tpyel = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_tpyel = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);
}
