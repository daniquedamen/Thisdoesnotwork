#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "ODEs.h"

#ifndef NIM_H
#define NIM_H

using namespace std;


// NIM (SUPER) CLASS;
class NIM
{
protected:
	double h = 0.001; // timestep

	vector<double> x;
	bool cargo; // dronewithoutcargo = 0 / dronewithcargo = 1;

	//friend class ODEs; 
	ODEs* ode;			 			// Change this line for different situation: ( DroneWithoutCargo / DroneWithCargo )


public:
	NIM() {
		std::cout << "\nPlease enter 0 for a Drone without cargo, or 1 for a Drone with cargo :)\n";
		std::cin >> cargo;
		if (cargo)
		{
			ode = new DroneWithCargo();
		}
		else
		{
			ode = new DroneWithoutCargo();
		}
	};

	virtual vector<double> getInitialState() = 0;
	virtual vector<double> update(vector<double> x, vector<double> u) = 0;

	vector<double> vec_scal(vector<double> x, double s) {
		for (int i = 0; i < x.size(); i++) {
			x[i] = s * x[i];
		}
		return x;

	}

	vector<double> vec_vec(vector<double> x, vector<double> y, string type) {
		if (x.size() == y.size()) {							// This function only works if x and y have the same length
			for (int i = 0; i < x.size(); i++) {			// Loop over the length of (one of) the vectors
				if (type == "mult") {						// if multiplication is selected -> multiply the vector element-wise
					x[i] = x[i] * y[i];
				} // WHOOPS there is no vector multiplication required anywhere but at least the function allows it :)
				else if (type == "sum") {					// if addition is selected -> sum the vector element-wise
					x[i] = x[i] + y[i];
				}
			}
			return x;											// After one of the operations, return vector x
		}
		else {												// if the vectors are of unequal size, the function will give the error
			cout << "ERROR: vector sizes are unequal" << endl << x.size() << y.size();
		}
	}



};


// FORWARD EULER : HERIDITAIRY TO NIM;
class f_euler : public NIM
{


public:
	f_euler()
	{
		x = ode->getInitialState();							// We call the initilizer of the state
		// Constructor
	}

	vector<double> getInitialState() override {
		return ode->getInitialState();
	}

	vector<double> update(vector<double> x, vector<double> u) override {
		return forward_euler(x, u);
	}

	vector<double> forward_euler(vector<double> x, vector<double> u) {

		x = vec_vec(x, vec_scal(ode->calcDiff(x, u), h), "sum");						// attempt to consturct simple dynamics  
		
		return x;
	};

protected:


};

// RUNGE KUTTA 4 : HERIDITAIRY TO NIM;
class runge_kutta : public NIM
{

public:
	runge_kutta()
	{
		x = ode->getInitialState();
		// Constructor
	}

	vector<double> getInitialState() {
		return ode->getInitialState();
	}

	vector<double> update(vector<double> x, vector<double> u) override {
		return R_K(x, u);
	}

	vector<double> R_K(vector<double> x, vector<double> u) {
		K1 = ode->calcDiff(x, u);
		K2 = ode->calcDiff(vec_vec(x, vec_scal(K1, (h / 2.00)), "sum"), u);
		K3 = ode->calcDiff(vec_vec(x, vec_scal(K2, (h / 2.00)), "sum"), u);
		K4 = ode->calcDiff(vec_vec(x, vec_scal(K3, h), "sum"), u);
		x = vec_vec(x, vec_scal(vec_vec(vec_vec(vec_vec(K1, vec_scal(K2, 2.00), "sum"), vec_scal(K3, 2.00), "sum"), K4, "sum"), (1.00 / 6.00) * h), "sum");

		return x;

	}

protected:
	vector<double> K1;
	vector<double> K2;
	vector<double> K3;
	vector<double> K4;
};


#endif
