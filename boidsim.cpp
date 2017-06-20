/******************************************************************************
 * File:    boidsim.cpp
 * Project: CMSC 435 Animation Project, Spring 2017
 * Author:  Thomas Wallace
 * Date:    17 April 2017
 * Review:  20 June 2017
 * E-mail:  thwalla1@umbc.edu
 *
 * Description:
 *          This file contains function and struct definitions for a boid sim.
 * boidsim contains a boid storage class and sim class with functions used in 
 * calculating boid interactions. Individual boids are identified as indicies
 * into the boid class position, velocity, and force vectors.
 *
 *****************************************************************************/
#include "boidsim.h"

using namespace std;

Boid::Boid() : size(1.0), range(1.0), nmax(1.0), mass(1.0), ka(1.0), kc(1.0),
kv(1.0), kh(1.0), kd(1.0), rank(0), nactive(0) {
};

Boid::~Boid() {
};

Boid::Boid(double _size, double _range, double _nmax, double _mass, double _ka,
        double _kc, double _kv, double _kh, double _kd)
: size(_size), range(_range), nmax(_nmax), mass(_mass), ka(_ka), kc(_kc),
kv(_kv), kh(_kh), kd(_kd), rank(0), nactive(0) {
};

Sim::Sim(double _x, double _y, double _z)
: dt(0.1), time(0.0), duration(1.0), nframes(0) {
  boundary[0] = _x;
  boundary[1] = _y;
  boundary[2] = _z;
};

Sim::~Sim() {

  //Free Allocated Boids
  for(boidItr = boids.begin(); boidItr < boids.end(); boidItr++) {
    delete *boidItr;
    *boidItr = NULL;
  }
  boids.clear();
}

//AddBoidClass - Adds a new boid class

void Sim::AddBoidClass(double size, double range, double nmax, double mass,
        double ka, double kc, double kv, double kh, double kd,
        int bnum) {
  boids.push_back(new Boid(size, range, nmax, mass, ka, kc, kv, kh, kd));

  //Resize Boid Vectors
  boids.back()->position.resize(bnum);
  boids.back()->velocity.resize(bnum);
  boids.back()->force.resize(bnum);
  boids.back()->spawn.resize(bnum);
}

//DeleteBoid - Erases a boid from its storage class

void Sim::DeleteBoid(int i, int j) {

  boids[i]->position.erase(boids[i]->position.begin() + j);
  boids[i]->velocity.erase(boids[i]->velocity.begin() + j);
  boids[i]->force.erase(boids[i]->force.begin() + j);
  boids[i]->spawn.erase(boids[i]->spawn.begin() + j);
}

//ClampBoids - Initial position clamp to boundary box

void Sim::ClampBoids() {

  //Boid List List
  for(unsigned int i = 0; i < boids.size(); i++) {
    for(unsigned int j = 0; j < boids[i]->spawn.size(); j++) {

      //Bounding Box Dimension Loop
      for(int d = 0; d < 3; d++) {

        //Check Positive Bound
        if(boids[i]->position[j][d] > boundary[d]) {
          boids[i]->position[j][d] = (boundary[d] - OFFSET);
        }
        //Check Negative Bound
        if(boids[i]->position[j][d] < -boundary[d]) {
          boids[i]->position[j][d] = (-boundary[d] + OFFSET);
        }
      }
    }
  }
}

//RedirectBoid - Checks boid outside bounding box and reflects velocity

bool Sim::RedirectBoid(SlVector3 &pos, SlVector3 &vel, bool reflect) {

  //Local Variable Declaration
  bool isextern = false;

  //Bounding Box Dimension Loop
  for(int i = 0; i < 3; i++) {

    //Check Positive Bound
    if(pos[i] > boundary[i]) {
      isextern = true;
      if(reflect) {
        pos[i] = (boundary[i] - OFFSET);

        SlVector3 normal;
        normal[i] = -1.0;
        vel = -((2.0 * normal * dot(vel, normal)) - vel);
      }
      else {
        break;
      }
    }
    //Check Negative Bound
    if(pos[i] < -boundary[i]) {
      isextern = true;
      if(reflect) {
        pos[i] = (-boundary[i] + OFFSET);

        SlVector3 normal;
        normal[i] = 1.0;
        vel = -((2.0 * normal * dot(vel, normal)) - vel);
      }
      else {
        break;
      }
    }
  }
  return isextern;
}

//CalcBoidForces - Calculates forces exerted upon each boid

int Sim::CalcBoidForces() {

  try {
    //Boid Interactions Loop
    for(unsigned int i = 0; i < boids.size(); i++) {
      boids[i]->nactive = 0;
      for(unsigned int j = 0; j < boids[i]->spawn.size(); j++) {

        //Check Boid is Spawned
        if(boids[i]->spawn[j] <= time) {
          boids[i]->nactive++;

          //Forager Boid Interactions
          if(boids[i]->rank > 0) {

            //Local Variable Declaration
            vector<int> friends;
            int foodnum = -1;
            double fooddist = DBL_MAX;
            SlVector3 center;
            SlVector3 speed;
            SlVector3 favoid;
            SlVector3 fcenter;
            SlVector3 fspeed;
            SlVector3 ffood;
            SlVector3 ftotal;

            //Get Neighbor Indicies
            kdtrees[i]->neighbors(boids[i]->position, boids[i]->position[j], 0,
                    boids[i]->range, friends);

            //Find Nearest Food Boid
            unsigned int f = 0;
            for(; (f < boids.size()); f++) {
              if(boids[f]->rank < boids[i]->rank) {
                for(unsigned int t = 0;
                        t < boids[f]->position.size(); t++) {
                  if(boids[f]->spawn[t] <= time) {
                    SlVector3 dist = (boids[f]->position[t] -
                            boids[i]->position[j]);
                    double magnitude = mag(dist);
                    if(magnitude < fooddist) {
                      fooddist = magnitude;
                      foodnum = t;
                    }
                  }
                }
                break;
              }
            }
            if(friends.size() > 0) {
              int nfriends = 0;
              //Calc Avoidance Forces
              for(unsigned int k = 0;
                      (k < friends.size() && k < boids[i]->nmax); k++) {
                double divisor = pow(mag((boids[i]->position[j] -
                        boids[i]->position[friends[k]])), 2.0);
                nfriends++;
                if(divisor < EPS) {
                  divisor = EPS;
                }
                favoid += ((boids[i]->position[j] -
                        boids[i]->position[friends[k]]) / divisor);
              }
              favoid = (favoid * boids[i]->ka);

              //Calc Centering Force
              for(unsigned int k = 0;
                      (k < friends.size() && k < boids[i]->nmax); k++) {
                center += boids[i]->position[friends[k]];
              }
              fcenter = (((center / nfriends) - boids[i]->position[j]) *
                      boids[i]->kc);

              //Calc Velocity Matching Force
              for(unsigned int k = 0;
                      (k < friends.size() && k < boids[i]->nmax); k++) {
                speed += boids[i]->velocity[friends[k]];
              }
              fspeed = (((speed / nfriends) - boids[i]->position[j]) *
                      boids[i]->kv);
            }
            if((foodnum >= 0) && (fooddist <= boids[i]->range)) {
              //Check Food Consumed
              if(fooddist <= (boids[i]->size + boids[f]->size + OFFSET)) {
                boids[f]->spawn[foodnum] = DBL_MAX;
                boids[f]->nactive--;
              }
              else {
                //Calc Closest Food Hunger Force
                ffood = ((boids[f]->position[foodnum] -
                        boids[i]->position[j]) * boids[i]->kh);
              }
            }
            //Sum Boid Forces
            ftotal = (favoid + normalize(fcenter) + fspeed + ffood);
            boids[i]->force[j].set(ftotal);
          }
        }
      }
    }
  }
  catch(...) {
    return 1;
  }
  return 0;
}

//UpdateBoidPositions - Calculates new boid position

int Sim::UpdateBoidPositions() {

  try {
    //Boid Update Loop
    for(unsigned int i = 0; i < boids.size(); i++) {
      for(unsigned int j = 0; j < boids[i]->spawn.size(); j++) {

        //Check Boid is Spawned
        if(boids[i]->spawn[j] <= time) {

          //Biome Boid Movement
          if(boids[i]->rank == 0) {
            boids[i]->velocity[j] = ((boids[i]->velocity[j] +
                    ((boids[i]->force[j] / boids[i]->mass) *
                    dt)) * boids[i]->kd);
            boids[i]->velocity[j][0] =
                    (0.001 * (double) (rand() % (int) (((2 * boundary[0]) + 1) -
                    boundary[0])));
            boids[i]->velocity[j][2] =
                    (0.001 * (double) (rand() % (int) (((2 * boundary[0]) + 1) -
                    boundary[0])));
            boids[i]->position[j] = (boids[i]->position[j] +
                    (boids[i]->velocity[j] * dt));
          }
          //Forager Boid Movement
          if(boids[i]->rank > 0) {
            boids[i]->velocity[j] = ((boids[i]->velocity[j] +
                    ((boids[i]->force[j] / boids[i]->mass) *
                    dt)) * boids[i]->kd);
            boids[i]->position[j] = (boids[i]->position[j] +
                    (boids[i]->velocity[j] * dt));
          }
        }
      }
    }
  }
  catch(...) {
    return 1;
  }
  return 0;
}

//CheckOutBounds - Checks boid out of bounds conditions

int Sim::CheckOutBounds() {

  try {
    //Boid Update Loop
    for(unsigned int i = 0; i < boids.size(); i++) {
      for(unsigned int j = 0; j < boids[i]->spawn.size(); j++) {

        //Check Boid is Spawned
        if(boids[i]->spawn[j] <= time) {

          //Food Boid Boundary Check
          if(boids[i]->rank == 0) {
            if(RedirectBoid(boids[i]->position[j],
                    boids[i]->velocity[j], false)) {
              //Despawn External Boid
              boids[i]->spawn[j] = DBL_MAX;
              boids[i]->nactive--;
            }
          }

          //Forager Boid Boundary Check
          if(boids[i]->rank > 0) {
            if(RedirectBoid(boids[i]->position[j],
                    boids[i]->velocity[j], true)) {
            }
          }
        }
      }
    }
  }
  catch(...) {
    return 1;
  }
  return 0;
}

//StepSimulation - Performs one time step state update of simulation

int Sim::StepSimulation() {

  try {
    //Build KDTrees
    for(unsigned int i = 0; i < boids.size(); i++) {
      kdtrees.push_back(new KDTree(boids[i]->position));
    }
  }
  catch(...) {
    //Free Allocated KDTrees
    for(treeItr = kdtrees.begin(); treeItr < kdtrees.end(); treeItr++) {
      delete *treeItr;
      *treeItr = NULL;
    }
    kdtrees.clear();
    return 1;
  }

  //Calculate Boid Forces
  if(CalcBoidForces() > 0) {
    return 2;
  }

  //Update Boid Positions
  if(UpdateBoidPositions() > 0) {
    return 3;
  }

  //Check Boid Out of Bounds
  if(CheckOutBounds() > 0) {
    return 4;
  }

  try {
    //Clean Up Despawned Boids
    for(unsigned int i = 0; i < boids.size(); i++) {
      for(unsigned int j = 0; j < boids[i]->spawn.size(); j++) {
        if(boids[i]->spawn[j] == DBL_MAX) {
          DeleteBoid(i, j);
        }
      }
    }
  }
  catch(...) {
    return 5;
  }

  //Free Allocated KDTrees
  for(treeItr = kdtrees.begin(); treeItr < kdtrees.end(); treeItr++) {
    delete *treeItr;
    *treeItr = NULL;
  }
  kdtrees.clear();

  time += dt;
  nframes++;
  return 0;
}

//ExportCurrent - Outputs current simulation state to file

int Sim::ExportCurrent(string filename) {

  ofstream outfile;

  //Open given filename
  outfile.open(filename.c_str(), fstream::app);

  //Export Current Simulation State
  if(outfile.is_open()) {
    try {
      //Output Total Frames
      if(nframes == 1) {
        int tframes = ceil(duration / dt);
        outfile << tframes << endl;
      }
      //Output Current Boid States
      for(unsigned int i = 0; i < boids.size(); i++) {
        if(boids[i]->nactive > 0) {
          outfile << boids[i]->nactive << endl;
          for(unsigned int j = 0; j < boids[i]->position.size(); j++) {
            if(boids[i]->spawn[j] <= time) {
              outfile << "[" << boids[i]->position[j][0]
                      << "," << boids[i]->position[j][1]
                      << "," << boids[i]->position[j][2] << "]";
              if(boids[i]->rank > 0) {
                outfile << "[" << boids[i]->velocity[j][0]
                        << "," << boids[i]->velocity[j][1]
                        << "," << boids[i]->velocity[j][2] << "]";
              }
              outfile << endl;
            }
          }
        }
        else {
          outfile << "0" << endl;
        }
      }
      if(boids.size() == 1) {
        outfile << "0" << endl;
      }
    }
    catch(...) {
      outfile.close();
      return nframes;
    }
  }
  else {
    return -1;
  }
  outfile.close();
  return 0;
}
