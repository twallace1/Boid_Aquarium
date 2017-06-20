/******************************************************************************
 * File:    parser.h
 * Project: CMSC 435 Boid-fish Simulation, Spring 2017
 * Author:  Thomas Wallace
 * Date:    17 April 2017
 * Review:  20 June 2017
 * E-mail:  thwalla1@umbc.edu
 *
 * Description:
 *          Header file for parser.cpp. For full description see parser.cpp.
 *
 *****************************************************************************/
#ifndef PARSER_H
#define PARSER_H

#include <fstream>
#include <sstream>
#include <string>
#include "boidsim.h"

int ReadFile(std::string infile, Sim &sim);
int ParseSimInit(std::ifstream &entity, Sim &sim);

#endif //PARSER_H
