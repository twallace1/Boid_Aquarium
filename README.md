Boid Aquarium Animation
=======================
Author: Thomas Wallace

Description
-----------
The Boid Aquarium Animation project implements Craig Reynold's distributed 
behavioral model [1] to generate a pre-rendered animation of fish-like boids 
schooling within a constrained three dimensional volume. The project contains 
multiple components:

* aquarium - main object setup and error handling
* parser - boidsim init file parser
* boidsim - boid simulation and animation file output
* viewer - displays pre-rendered aquarium animation (separate program) [2]

The aquarium uses C++ with Berkeley Fluid Animation & Simulation vector library 
(provided). The viewer utilizes OpenGL Utility Toolkit which requires 
installation to compile and run. Sample init files of 50 boids with and without 
food are included. C++03 and later compiler standards supported.

Directory Descriptions
----------------------
* init - Contains boid input files
* lib - Contains vector libraries
* viewer - Contains separate viewer

Command Line Execution Format
-----------------------------
> aquarium boids.in boids.out

> viewer boids.out

Implementation Notes
--------------------
viewer compiles separately of aquarium. Modification of viewer includes and 
Makefile may be necessary for compilation (platform dependent).

Command option -b for bubbles is not utilized.

References
----------
[1] C. Reynolds, “Flocks, Herds, and Schools: A Distributed Behavioral Model,” 
Computer Graphics, vol. 21, no. 4, July 1987. 
Available: http://www.eng.utah.edu/~cs6665/Reynolds-1987-FHS.pdf

[2] viewer, A. Bargteil, Catonsville, MD, 2017.
