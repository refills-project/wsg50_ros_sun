
/*
    ANN_Layer Class

    Copyright 2017-2018 Università della Campania Luigi Vanvitelli

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

#ifndef ANN_Layer_1_0_LIB
#define ANN_Layer_1_0_LIB


#include <TooN/TooN.h>
#include <Helper1.0.h>

using namespace TooN;

typedef Vector<> (*FF_OUT)(Vector<>); //output function

class ANN_Layer{
private:

	ANN_Layer(); //NO DEFAULT CONSTRUCTOR

	unsigned int n_neurons, dimInput;
	Matrix<> * W;
	Vector<> * b;
	FF_OUT myFF;

public:
/*===============CONSTRUCTORS===================*/
	ANN_Layer(unsigned int n_neurons,unsigned  int dimInput, Matrix<> W, Vector<> b, FF_OUT fun); //COMPLETE CONSTRUCTOR

	ANN_Layer(Matrix<> W, Vector<> b, FF_OUT fun); //dimensioni dalle matrici

	ANN_Layer(unsigned int n_neurons,unsigned  int dimInput, const char* pathW, const char* pathB, FF_OUT fun); //COMPLETE CONSTRUCTOR FROM FILE

	ANN_Layer(unsigned int n_neurons,unsigned  int dimInput, Matrix<> W, Vector<> b); //fun = linear;

	ANN_Layer(Matrix<> W, Vector<> b); //fun = linear;

	ANN_Layer(unsigned int n_neurons,unsigned  int dimInput, const char* pathW, const char* pathB); //FROM FILE fun = linear

	ANN_Layer(unsigned int n_neurons,unsigned  int dimInput); //fun = linear

	ANN_Layer(const ANN_Layer& myLayer);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
	~ANN_Layer(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
	unsigned int getNNeurons();
	unsigned int getDimInput();
	unsigned int getDimOutput();
	Matrix<> getW();
	Vector<> getB();
	FF_OUT getFun();
/*==============================================*/

/*=============SETTER===========================*/
	void setW( Matrix<> W );
	void setB( Vector<> b );
	void setFun( FF_OUT ff );
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
	void setW(const char* path);
	void setB(const char* path);
/*========================================================*/

/*=============RUNNER===========================*/
	Vector<> compute( Vector<> input );
/*==============================================*/

/*=============VARIE===========================*/
	void display();
/*==============================================*/

};


/*=============STATIC FUNS===========================*/
	Vector<> ANN_Layer_SIGMA(Vector<> x);
	Vector<> ANN_Layer_LINEAR(Vector<> x);
/*==============================================*/

#endif
