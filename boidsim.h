/******************************************************************************
 * File:    boidsim.h
 * Project: CMSC 435 Animation Project, Spring 2017
 * Author:  Thomas Wallace
 * Date:    17 April 2017
 * Review:  20 June 2017 
 * E-mail:  thwalla1@umbc.edu
 *
 * Description:
 *          This file contains function declarations for the boid sim. See
 * boidsim.cpp for detailed description.
 *
 *****************************************************************************/
#ifndef BOIDSIM_H
#define BOIDSIM_H

#include <cfloat>
#include <vector>
#include <fstream>
#include <string>
#include "lib/slVector.H"
#include "lib/kdTree.H"

#define EPS 0.00001
#define OFFSET 0.0001

class Boid {
public:
  Boid();
  Boid(double _size, double _range, double _nmax, double _mass, double _ka,
          double _kc, double _kv, double _kh, double _kd);
  ~Boid();

  double size;
  double range;
  double nmax;
  double mass;
  double ka;
  double kc;
  double kv;
  double kh;
  double kd;
  int rank;
  int nactive;
  std::vector<SlVector3> position;
  std::vector<SlVector3> velocity;
  std::vector<SlVector3> force;
  std::vector<double> spawn;

private:

};

class Sim {
public:
  Sim(double _x, double _y, double _z);
  void AddBoidClass(double size, double range, double nmax, double mass,
          double ka, double kc, double kv, double kh, double kd,
          int bnum);
  void DeleteBoid(int i, int j);
  void ClampBoids();
  bool RedirectBoid(SlVector3 &pos, SlVector3 &vel, bool reflect);
  int CalcBoidForces();
  int UpdateBoidPositions();
  int CheckOutBounds();
  int StepSimulation();
  int ExportCurrent(std::string outfile);
  ~Sim();

  SlVector3 boundary;
  double dt;
  double time;
  double duration;
  int nframes;
  std::vector<Boid *> boids;
  std::vector<Boid *>::iterator boidItr;
  std::vector<KDTree *> kdtrees;
  std::vector<KDTree *>::iterator treeItr;

private:

};

#endif //BOIDSIM_H
