/**
 * Authors: Oleg Maksimov, Guy Levy, Aviad Fuchs and Mor Sinay, AIM Lab, Bar-Ilan University
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 AIM Lab, Bar-Ilan University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef DEFINES_H
#define DEFINES_H
#define GRID_ROWS 16
#define GRID_COLS 30
#define GRID_CELL_SIZE_METERS 500
#define NUMBER_OF_DRONES 1
#define FIRST_CELL_IN_BORDER 59

// #define MINOR_PROBABILITY 0.0001 // 
// #define MINOR_PROBABILITY 0.000001 // 
// #define MINOR_PROBABILITY 0.0000001 // 


#define DETECTION_DURATION 0
#define TICK 1
//#define INT_MAX __INT_MAX__
#define UAV_SPEED_KMpH 90
#define ENEMY_SPEED_KMpH 30

//#define UAV_METTER_p_TICK
#define UAV_SPEED (int)(UAV_SPEED_KMpH * TICK * (double)(1000.0 / 3600.0))
//#define ENEMY_METTER_P_TICK
#define ENEMY_SPEED (int)(ENEMY_SPEED_KMpH * TICK * (double)(1000.0 / 3600.0))//meters per tick - the amount of meters that the enemy passes in one tick
//#define ENEMY_SPEED (int)(ENEMY_SPEED_KMpH * (double)(1000.0 / 3600.0))
static int DRONES_INIT_CELL = 235;
#define INIT_ENEMIES_COUNT 1;
#define INIT_UAVS_COUNT 1;

#define VERBOSE_LOG false

static double UAV_DELAY = 1000.0 * TICK;//470.0
#define ENEMY_WAY_INTERVAL_RATIO 0.0

//Hipper parameter for calculate the aggregation value between next sections probs and uniform
// static double ALPHA_AGGREGATE = 1.0;

//Hipper parameter for determining the threshold of uniform probability for the next section in recursive function in next section algorithm
// static double ALPHA_THR = 0.8;

//next section algorithm kind:
// a - Aviad
// m - Mor
// h - hybrid
// #define NEXT_SECTIONS_ALGORITHM_KIND 'a'

// #define IS_LOAD_PATHS true

// static vector<int> NUM_PATH = {0,3};

//Define SETUP_OWN for compatible path 
//Oleg - 0
//Aviad - 1
#define SETUP_OWN 0

//flags
//#define DRAW_GRID true
// #define IS_UAV_TRACKING false
//#define IS_UAV_TRACKING true

//#define DRAW_GRID true


enum Algo { SCOUT, OMEGA_MAX, OMEGA_MIN, MaxProb, Rand, Entropy };
#endif /* DEFINES_H */
