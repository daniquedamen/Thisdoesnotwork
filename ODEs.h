#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#ifndef ODES_H
#define ODES_H

using namespace std;

class ODEs
{
    /*This is the interface class of the two drone types, which are heriditairy to this class.
    We can construct objects of this class (as pointers -> explained in NIM.h) which later in NIM.h
    get switched to either drone types. The reason such an interface class is used for building objects
    is to have different object sizes and properties while still existing in the same parent class AND
    with the same/similar member functions (methods)*/

public:
    ODEs() {}; // constructor

    /* In this file we will construct one main class ODEs. This will contain all general information
    and computuation required by both ode's methods. Important to note is that the states will be build
    as a vector of differet sizes.

    Then we will make two heriditary classes (here) each containing its own ODE version*/


    // Pure virtual functions:
    virtual vector<double> getInitialState() = 0;
    virtual vector<double> calcDiff(vector<double> x, vector<double> u) = 0;



protected:
    double m = 3.00;                                    // mass in kg
    double Cdrag = 0.1;                                 // Ns^2m^2 drag constant
    double g = 9.81;                                    // Gravity
    double h = 0.01;                                    // time step [s] 

    double x3_min = 1;                                     // for now
    double x3_max = 2;                                     // for now 

    // Initialize solution to differential equation                         
    vector<double> xdot;


};

// Class for drone WITHOUT cargo
class DroneWithoutCargo : public ODEs
{
public:
    DroneWithoutCargo() {};

    vector<double> getInitialState() override {
        xdot = { 0 ,0, 0, 0, 0 };
        return { 0 ,0, 0, 0, 0 };
    }

    vector<double> calcDiff(vector<double> x, vector<double> u) override {
        //inputs of this needs to be the state (from NIM) and input (from topic). Same with other dynamics
        for (int i = 0; i < xdot.size(); i++)
        {
            if (i == 0) {                                                                                      // state xdot1 
                xdot[i] = x[3];
            }
            else if (i == 1) {                                                                                   // state xdot2 
                xdot[i] = x[4];
            }
            else if (i == 2) {                                                                                     // state xdot3
                // rotational stop
                if (x[2] <= x3_min && u[1] < 0) {              // don't rotate for:
                    xdot[i] = 0;
                }
                else if (x[2] >= x3_max && u[1] > 0) {          // don't rotate for:
                    xdot[i] = 0;
                }
                else {                                       // rotate for:
                    xdot[i] = u[1];
                }
            }
            else if (i == 3) {                                                                                     // state xdot4           
                xdot[i] = (1.00 / m) * (-1.00 * u[0] * sin(x[2]) - Cdrag * sqrt((x[3] * x[3]) + (x[4] * x[4])) * x[3]);
            }
            else if (i == 4) {                                                                                     // state xdot5 
                xdot[i] = (1 / m) * (-1.00 * u[0] * cos(x[2]) - Cdrag * sqrt((x[3] * x[3]) + (x[4] * x[4])) * x[3]) - g;
            }
        }
        //std::cout << "\nxdot is returned\n";
        return xdot;
    };



protected:

};

// Class for drone WITH cargo
class DroneWithCargo : public ODEs
{
public:
    DroneWithCargo() {};       // constructor

    vector<double> getInitialState() override {
        xdot = { 0, 0, 0, 0, 0, 0, 0, 0, -l_rope0 };
        //std::cout << "\n state is initialised as:" << xdot.size();
        return { 0, 0, 0, 0, 0, 0, 0, 0, -l_rope0 };
    }          //x, y, o,xd, yd,x, y,xd, yd  

    vector<double> ropeLength(vector<double> x) { // Fucntion for determining the rope length
        double y1 = sqrt(((x[0] - x[5]) * (x[0] - x[5])) + ((x[1] - x[6]) * (x[1] - x[6])));
        double y2 = ((x[0] - x[5]) * (x[3] - x[7]) + (x[1] - x[6]) * (x[4] - x[8])) / y1;
        return { y1, y2 };
    }

    vector<double> calcRopeForce(vector<double> x)
    {
        double F = k_rope * (ropeLength(x)[0] - l_rope0) + (d_rope * ropeLength(x)[1]);
        double F_x = F * ((x[0] - x[5]) / ropeLength(x)[0]);
        double F_y = F * ((x[1] - x[6]) / ropeLength(x)[0]);
        return { F_x, F_y };
    }

    vector<double> calcDiff(vector<double> x, vector<double> u) override {
        /* This function contains the bulk of the calculations done. It takes the states and the derivatives
        of the states as theinput, and from this computes the new derivatives. The function returns this
        derivative.*/
        //std::cout << "\n state is initialised as:" << x.size();
        for (int i = 0; i < xdot.size(); i++)
        {
            std::cout << "calcdiff is reached\n";
            if (i == 0) {                                                                                      // state xdot1 
                xdot[i] = x[3];
            }
            else if (i == 1) {                                                                                   // state xdot2 
                xdot[i] = x[4];
            }
            else if (i == 2) {                                                                                     // state xdot3
                // rotational stop
                if (x[2] <= x3_min && u[1] < 0) {              // don't rotate for:
                    xdot[i] = 0;
                }
                else if (x[2] >= x3_max && u[1] > 0) {          // don't rotate for:
                    xdot[i] = 0;
                }
                else {                                       // rotate for:
                    xdot[i] = u[1];
                }
            }
            else if (i == 3) {                                                                                     // state xdot4           
                xdot[i] = (1 / m) * (-1.00 * u[0] * sin(x[2]) - Cdrag * sqrt((x[3] * x[3]) + (x[4] * x[4])) * x[3]);
            }
            else if (i == 4) {                                                                                     // state xdot5 
                xdot[i] = (1 / m) * (-1.00 * u[0] * cos(x[2]) - Cdrag * sqrt((x[3] * x[3]) + (x[4] * x[4])) * x[3]) - g;
            }
            else if (i == 5) {                                                                                     // state xdot6  
                xdot[i] = x[7];
            }
            else if (i == 6) {                                                                                     // state xdot7
                xdot[i] = x[8];
                
            }
            else if (i == 7) {                                                                                     // state xdot8 
                xdot[i] = (1 / m_c) * (-1 * Cdrag_c * sqrt((x[7] * x[7]) + (x[8] * x[8])) * x[7] + calcRopeForce(x)[0]);
            }
            else if (i == 8) {                                                                                     // state xdot9 
                xdot[i] = (1 / m_c) * (-1 * Cdrag_c * sqrt((x[7] * x[7]) + (x[8] * x[8])) * x[7] + calcRopeForce(x)[1]) - g;
            }
        }
        //std::cout << "\nxdot is returned!\n";
        return xdot;
    }



protected:
    double m_c = 3.00;                                    // mass in kg
    double Cdrag_c = 0.1;                                 // Ns^2m^2 drag constant
    double h = 0.01;

    double k_rope = 400000;
    double d_rope = 50;
    double l_rope0 = 1.5;

};


#endif
