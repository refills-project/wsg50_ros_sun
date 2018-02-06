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
