/******************************************************************************
 * File:    parser.cpp
 * Project: CMSC 435 Boid-fish Simulation, Spring 2017
 * Author:  Thomas Wallace
 * Date:    17 April 2017
 * Review:  20 June 2017
 * E-mail:  thwalla1@umbc.edu
 *
 * Description:
 *          This file contains the input file parser for the boid-fish sim.
 *
 *****************************************************************************/
#include "parser.h"

using namespace std;

//ReadFile - Open file and parse for objects

int ReadFile(string filename, Sim &sim) {

  //Local Variable Declaration
  ifstream inStream(filename.c_str());
  int status;

  //Check Writable File
  if(inStream.fail()) {
    inStream.close();
    return -1;
  }

  //Parse Boid Initialization File
  status = ParseSimInit(inStream, sim);

  inStream.close();

  return status;
}

//ParseSimInit - Parse boid simulation initialization file

int ParseSimInit(ifstream &entity, Sim &sim) {

  //Local Variable Declaration
  string next, token;
  stringstream line;
  int lnum = 1;
  double size, range, nmax, mass, ka, kc, kv, kh, kd;

  //Parse Initial Boid Variable Line
  try {
    getline(entity, next, '\n');
    line.str("");
    line.clear();
    line << next;

    getline(line, token, ' ');
    size = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    range = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    nmax = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    mass = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    ka = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    kc = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    kv = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    kh = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    kd = strtod(token.c_str(), NULL);
    getline(line, token, ' ');
    sim.dt = strtod(token.c_str(), NULL);
    getline(line, token, '\n');
    sim.duration = strtod(token.c_str(), NULL);
  }
  catch(...) {
    return lnum;
  }
  lnum++;

  while(getline(entity, next, '\n')) {
    try {
      line.str("");
      line.clear();
      line << next;

      //Get Number of Boids
      if(isdigit(line.peek())) {

        int bnum = atoi(next.c_str());
        lnum++;

        //Create New Boid Class
        sim.AddBoidClass(size, range, nmax, mass, ka, kc, kv, kh, kd, bnum);

        //Parse Boid Initial Positions and Velocities
        for(int i = 0; i < bnum; i++) {

          //Local Temp Variables
          SlVector3 pos;
          SlVector3 vel;

          getline(entity, next, '\n');
          line.str("");
          line.clear();
          line << next;

          //Parse for Boid Position, Velocity, Spawn
          if(line.peek() == '[') {

            getline(line, token, '[');
            getline(line, token, ',');
            pos[0] = strtod(token.c_str(), NULL);
            getline(line, token, ',');
            pos[1] = strtod(token.c_str(), NULL);
            getline(line, token, ']');
            pos[2] = strtod(token.c_str(), NULL);
            getline(line, token, '[');
            getline(line, token, ',');
            vel[0] = strtod(token.c_str(), NULL);
            getline(line, token, ',');
            vel[1] = strtod(token.c_str(), NULL);
            getline(line, token, ']');
            vel[2] = strtod(token.c_str(), NULL);
            sim.boids.back()->position[i] = pos;
            sim.boids.back()->velocity[i] = vel;

            //Check for Spawn Time
            getline(line, token, '\n');
            if(isdigit(token[1])) {
              sim.boids.back()->spawn[i] = strtod(token.c_str(), NULL);
            }
            else {
              sim.boids.back()->spawn[i] = 0.0;
            }
          }
            //Discard Spawn Gap
          else if(line.peek() == '\n' || next.empty()) {
            i--;
          }
            //Throw Unrecognized Parse Error
          else {
            throw lnum;
          }
          lnum++;
        }
      }
        //Discard Newline
      else if(line.peek() == '\n' || next.empty()) {
      }
        //Throw Unrecognized Token Error
      else {
        throw lnum;
      }
      lnum++;
    }
    catch(...) {
      return lnum;
    }
  }
  return 0;
}
