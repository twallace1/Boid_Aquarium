/******************************************************************************
 * File:    aquarium.cpp
 * Project: CMSC 435 Animation Project, Spring 2017
 * Author:  Thomas Wallace
 * Date:    17 April 2017
 * Review:  20 June 2017 
 * E-mail:  thwalla1@umbc.edu
 *
 * Description:
 *          This file contains the project main responsible for setting up
 * boid structures and conducting stepwise simulation of boid interactions.
 *
 *****************************************************************************/
#include "aquarium.h"

#define BOUNDX 0.5
#define BOUNDY 0.25
#define BOUNDZ 0.125

using namespace std;

int main(int argc, char *argv[]) {

  //Command Variable Definition
  string infile;
  string outfile;
  bool bubbles;

  //Check for Command Line Arguments
  if((argc < 3) || (argc > 4)) {
    cerr << "Error: Invalid command arguments" << endl;
    exit(0);
  }
  //Set Command Variables
  try {
    infile = argv[1];
    outfile = argv[2];
    if(argc == 4) {
      string cmd(argv[3]);
      if(cmd == "-b") bubbles = true;
    }
  }
  catch(...) {
    cerr << "Error: Invalid command line format" << endl;
    exit(0);
  }

  //Simulation Variable Declaration
  int io_status = 0;
  int sim_status = 0;
  Sim sim(BOUNDX, BOUNDY, BOUNDZ);

  //Load Scene Input
  cout << "Parsing " << infile << endl;
  io_status = ReadFile(infile, sim);
  if(io_status == -1) {
    cerr << "Error: Unable to open " << infile << endl;
    exit(1);
  }
  if(io_status > 0) {
    cerr << "Error: Unrecognized pattern on line " << io_status << endl;
    exit(1);
  }
  cout << "Parsing Complete" << endl;

  //Boid Behavior Init 
  sim.boids[0]->rank = 1;

  //Boundary Check
  sim.ClampBoids();

  //Run Aquarium Simulation
  cout << "Computing Animation" << endl;
  if(bubbles) cout << "Computing Animation with Bubbles" << endl;
  while((sim.time < sim.duration) && (io_status == 0) && (sim_status == 0)) {
    sim_status = sim.StepSimulation();
    io_status = sim.ExportCurrent(outfile);
  }
  //Simulation Error Reporting
  if(io_status == -1) {
    cerr << "Error: Unable to open " << outfile << endl;
  }
  if(io_status > 0) {
    cerr << "Error: Failed to write frame " << io_status << endl;
  }
  if(sim_status > 0) {
    cerr << "Error: Step " << sim_status << " returned on frame "
            << sim.nframes << endl;
  }
  if((io_status != 0) || (sim_status != 0)) {
    exit(1);
  }
  cout << "Animation Complete" << endl;

  return 0;
}
