//
// Created by lml on 2026/3/13.
//

#ifndef PATROL_ROBOT_CAR_SPORT_H
#define PATROL_ROBOT_CAR_SPORT_H

#include "main.h"
#include "brushless_motor.h"

#define width   12      // cm

void car_sport_init(void);
void car_turn_angle(char direction,float angle,float radius,float speed);

#endif //PATROL_ROBOT_CAR_SPORT_H