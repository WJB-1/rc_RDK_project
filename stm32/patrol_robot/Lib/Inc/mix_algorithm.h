//
// Created by lml on 2026/3/19.
//

#ifndef PATROL_ROBOT_MIX_ALGORITHM_H
#define PATROL_ROBOT_MIX_ALGORITHM_H

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "JY901S.h"
#include "angle_unwrap.h"

float safe_extract_float(const char *receiveData);
void Read_IMU_Data();

#endif //PATROL_ROBOT_MIX_ALGORITHM_H