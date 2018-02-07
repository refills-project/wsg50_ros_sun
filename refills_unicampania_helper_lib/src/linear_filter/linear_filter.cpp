/*
    Linear Filter Class

    Copyright 2015-2018 Università della Campania Luigi Vanvitelli

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "linear_filter.h"

using namespace std;

LinearFilter::LinearFilter (int order,int num_inputs,int num_outputs) :
	_order(order),
    _num_inputs(num_inputs),
    _num_outputs(num_outputs),
	A(Zeros(order)),
	B(Zeros(order,num_inputs)),
	C(Zeros(num_outputs,order)),
	D(Zeros(num_outputs,num_inputs)),
	x(Zeros(order))
  {
  }   

Vector<> LinearFilter::apply_filter(Vector<> u)
 { 
	Vector <> y = Zeros(_num_outputs); 
	 
	x = A * x + B * u;
	y = C * x + D * u;
	
	return y;
 }

Vector<> LinearFilter::Get_x() { return this->x; }

void LinearFilter::Set_x(Vector<> x0) { this->x = x0; }

Matrix<> LinearFilter::Get_A() { return this->A; }

void LinearFilter::Set_A(Matrix<> aA) { this->A = aA; }

Matrix<> LinearFilter::Get_B() { return this->B; }

void LinearFilter::Set_B(Matrix<> aB) { this->B = aB; }

Matrix<> LinearFilter::Get_C() { return this->C; }

void LinearFilter::Set_C(Matrix<> aC) { this->C = aC; }

Matrix<> LinearFilter::Get_D() { return this->D; }

void LinearFilter::Set_D(Matrix<> aD) { this->D = aD; }

void LinearFilter::Set_Filter(Vector<> x0,Matrix<> aA,Matrix<> aB,Matrix<> aC,Matrix<> aD)
{
	this->x = x0;
	//this->A = aA;
	this->Set_A(aA); //alternative form just to learn!
	this->B = aB;
	this->C = aC;
	this->D = aD;
	
	}

void LinearFilter::display()
{
	cout << "\n\033[1;34mLinear filter of order " << _order << ", with " << _num_inputs << " inputs and " << _num_outputs << " outputs\n" << endl;
	
	cout << "A:\033[1;37m\n" << A << endl;
	 
	cout << "\033[1;34mB:\033[1;37m\n" << B << endl;
	
	cout << "\033[1;34mC:\033[1;37m\n" << C << endl;

	cout << "\033[1;34mD:\033[1;37m\n" << D << endl;
	
	cout << "\033[1;34mx0:\033[1;37m\n" << x << endl; 
	}

