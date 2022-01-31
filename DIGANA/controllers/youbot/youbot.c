#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TIME_STEP 32
#define LIQUID_LEVEL 1000

int move_to_charging_base(WbDeviceTag position_sensors[6]);
int recharge();
int wait_for_sanitizing();
int goFrom1To2(WbDeviceTag position_sensors[6]);
int goFrom2To3(WbDeviceTag position_sensors[6]);
int goFrom3To4(WbDeviceTag position_sensors[6]);
double base1[3] = {-2.57506, -2.08632, -3.14159};
double base2[3] = {-2.56506, 2.24368, -3.14159};
double base3[3] = {0.33494, 2.24368, -3.14159};
double base4[3] = {0.434938, -2.18632, -3.14159};
void avoid_obstacles(WbDeviceTag position_sensor[6]);
void disable_position_sensors(WbDeviceTag position_sensor[6]);
void release_liquid();

//---------------//
// PASSIVE WAIT //
//-------------//
static void step()
{
    if (wb_robot_step(TIME_STEP) == -1)
    {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void passive_wait(double sec)
{
    double start_time = wb_robot_get_time();
    do
    {
        step();
    } while (start_time + sec > wb_robot_get_time());
}

//------------//
// MAIN LOOP //
//-----------//
int main(int argc, char **argv)
{
    wb_robot_init();
    WbDeviceTag camera = wb_robot_get_device("camera");
    WbDeviceTag position_sensors[6];
    wb_camera_enable(camera, TIME_STEP);

    int state = 0;
    // struct object_detection OB;
    // static const struct object_detection OB_empty = {-1, -1, -1, -1};
    while (wb_robot_step(TIME_STEP) != -1)
    {
        switch (state)
        {
        case 0:
            //printf("%s\n", "STATO 0 - VADO ALLA BASE DI RICARICA");
            state = move_to_charging_base(position_sensors);
            break;
        case 1:
            //printf("%s\n", "STATO 1 - EFFETTUO LA RICARICA");
            state = recharge();
            break;
        case 2:
            //printf("%s\n", "STATO 2 - ATTENDO CHE PASSINO 4 PERSONE");
            state = wait_for_sanitizing();
            break;
        case 3:
            //printf("%s\n", "STATO 3 - SANIFICO");
            state = goFrom1To2(position_sensors);
            break;
        case 4:
            //printf("%s\n", "STATO 3 - SANIFICO");
            state = goFrom2To3(position_sensors);
            break;
        case 5:
            //printf("%s\n", "STATO 3 - SANIFICO");
            state = goFrom3To4(position_sensors);
            break;
        }
    }

    wb_robot_cleanup();
    return 0;
}
//------------//
// STATO - 0 //
//-----------//
int move_to_charging_base(WbDeviceTag position_sensors[6]) {
    WbNodeRef check_field = wb_supervisor_node_get_from_def("CHECK");
    WbFieldRef description = wb_supervisor_node_get_field(check_field, "description");
    wb_supervisor_field_set_sf_string(description, "false");

    avoid_obstacles(position_sensors);
    base_init();
    base_goto_init(TIME_STEP);

    base_goto_set_target(base1[0], base1[1], base1[2]);

    base_goto_run();
    if (base_goto_reached())
    {
        disable_position_sensors(position_sensors);
        release_liquid();
        return 1;
    }
    return 0;
}

//---------------------//
// OBSTACLE-AVOIDANCE //
//-------------------//
void avoid_obstacles(WbDeviceTag position_sensors[6])
{
    base_init();
    int i;
    char ps_names[6][30] = {"D_S_left", "D_S_right", "D_S_left_side", "D_S_right_side", "D_S_left_side2", "D_S_right_side2"};
    for (i = 0; i < 6; i++)
    {
        position_sensors[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(position_sensors[i], TIME_STEP);
    }
    base_forwards();
    double ps_values[6];
    for (i = 0; i < 6; i++)
    {
        ps_values[i] = wb_distance_sensor_get_value(position_sensors[i]);
    }
    bool left_obstacles = ps_values[0] < 950;
    bool right_obstacles = ps_values[1] < 950;
    bool left_side_obstacles = ps_values[2] < 950;
    bool right_side_obstacles = ps_values[3] < 950;
    bool left_side2_obstacles = ps_values[4] < 950;
    bool right_side2_obstacles = ps_values[5] < 950;
    if (left_obstacles)
    {
        base_backwards();
        passive_wait(3.0);
        base_turn_right();
        passive_wait(3.0);
        base_forwards();
    }
    else if (right_obstacles)
    {
        base_backwards();
        passive_wait(3.0);
        base_turn_left();
        passive_wait(3.0);
        base_forwards();
    }
    else if (left_side_obstacles)
    {
        base_backwards();
        passive_wait(3.0);
        base_turn_right();
        passive_wait(3.0);
        base_forwards();
    }
    else if (right_side_obstacles)
    {
        base_backwards();
        passive_wait(3.0);
        base_turn_left();
        passive_wait(3.0);
        base_forwards();
    }
    else if (left_side2_obstacles)
    {
        base_forwards();
        passive_wait(3.0);
        base_turn_right();
    }
    else if (right_side2_obstacles)
    {
        base_forwards();
        passive_wait(3.0);
        base_turn_left();
    }
}

void disable_position_sensors(WbDeviceTag position_sensor[6])
{
    int i;
    for (i = 0; i < 6; i++)
    {
        wb_distance_sensor_disable(position_sensor[i]);
    }
}

//------------//
// STATO - 1 //
//-----------//
int recharge()
{
    // Supponiamo per adesso che sia necessaria la sola ricarica del liquido
    WbNodeRef container = wb_supervisor_node_get_from_def("CONTAINER");
    char string_level[10];
    itoa(LIQUID_LEVEL, string_level, 10);
    WbFieldRef description = wb_supervisor_node_get_field(container, "description");
    // Simuliamo il tempo di ricarica a 3 secondi
    passive_wait(3.0);
    wb_supervisor_field_set_sf_string(description, string_level);

    return 2;
}

//------------//
// STATO - 2 //
//-----------//
int wait_for_sanitizing()
{
    WbNodeRef check_field = wb_supervisor_node_get_from_def("CHECK");
    WbFieldRef description = wb_supervisor_node_get_field(check_field, "description");
    // char check_string[5];
    // check_string = wb_supervisor_field_get_sf_string(description);
    const char *false_string = "false";

    // printf("Content: %s\n", wb_supervisor_field_get_sf_string(description));
    // printf("Result: %d\n", strcmp(wb_supervisor_field_get_sf_string(description), false_string));

    if (strcmp(wb_supervisor_field_get_sf_string(description), false_string) == 0)
    {
        // printf("%s\n", "Entro");
        return 2;
    }

    return 3;
}

//------------//
// STATO - 3 //
//-----------//
int goFrom1To2(WbDeviceTag position_sensors[6]) {
    avoid_obstacles(position_sensors);
    
    base_init();
    base_goto_init(TIME_STEP);
    base_goto_set_target(base2[0], base2[1], base2[2]);
    base_goto_run();

    if (base_goto_reached()) {
        //disable_position_sensors(position_sensors);
        release_liquid();
        return 4;
    }
    return 3;
}

//------------//
// STATO - 4 //
//-----------//
int goFrom2To3(WbDeviceTag position_sensors[6]) {
    avoid_obstacles(position_sensors);
    
    base_init();
    base_goto_init(TIME_STEP);
    base_goto_set_target(base3[0], base3[1], base3[2]);
    base_goto_run();

    if (base_goto_reached()) {
        //disable_position_sensors(position_sensors);
        release_liquid();
        return 5;
    }
    return 4;
}

//------------//
// STATO - 5 //
//-----------//
int goFrom3To4(WbDeviceTag position_sensors[6]) {
    avoid_obstacles(position_sensors);
    
    base_init();
    base_goto_init(TIME_STEP);
    base_goto_set_target(base4[0], base4[1], base4[2]);
    base_goto_run();

    if (base_goto_reached()) {
        //disable_position_sensors(position_sensors);
        release_liquid();
        return 0;
    }
    return 5;
}

void release_liquid() {
    // Decremento liquido
    WbNodeRef container = wb_supervisor_node_get_from_def("CONTAINER");
    WbFieldRef description = wb_supervisor_node_get_field(container, "description");
    // char string_level[10] = wb_supervisor_field_get_sf_string(description);
    char string_level[10];
    // int int_level = atoi(string_level);
    int int_level = atoi(wb_supervisor_field_get_sf_string(description));
    int_level -= 250;
    itoa(int_level, string_level, 10);
    wb_supervisor_field_set_sf_string(description, string_level);
}