#ifndef ANN_1_0_LIB
#define ANN_1_0_LIB

//#define HYBRID_NORM

#include "ANN_Layer1.0.h"
#include <vector>
#include <string.h>

using namespace std;

#define NORM_TYPE_NULL 0
#define NORM_TYPE_MAPMINMAX 1
#define NORM_TYPE_MAX 2
#ifdef HYBRID_NORM 
	#define NORM_TYPE_HYBRID 3
#endif

class ANN{

	typedef Vector<> (ANN::*FF_NORM)(Vector<>);
	//int (TMyClass::*pt2ConstMember)(float, char, char) const = NULL;

private:
	unsigned int dimInput;
	unsigned int dimOutput;

	vector<ANN_Layer> layers;

/*======FOR MAPMINMAX========*/
	Vector<> * minInput;
	Vector<> * maxInput;
	Vector<> * minOutput;
	Vector<> * maxOutput;
/*============================*/
/*======FOR MAPMINMAX========*/
#ifdef HYBRID_NORM 
	Vector<> * inputNormMax;
	Vector<> * outputNormMax;
#endif
/*============================*/

	int normType;
	FF_NORM normFun;
	FF_NORM invNormFun;

/*==========NORMS======================*/
	Vector<> nullNorm( Vector<> in );

	Vector<> mapMinMax(Vector<> in);
	Vector<> InvMapMinMax(Vector<> in);

	Vector<> normMax(Vector<> in);
	Vector<> InvNormMax(Vector<> in);

#ifdef HYBRID_NORM 
	Vector<> normHybrid(Vector<> in);
	Vector<> invNormHybrid(Vector<> in);
#endif
/*==============================================*/

public:

/*===============CONSTRUCTORS===================*/
	ANN( vector<ANN_Layer> layers, Vector<> minInput, Vector<> maxInput, Vector<> minOutput, Vector<> maxOutput );

	ANN( unsigned int dimInput , unsigned int dimOutput );

	ANN(unsigned int dimInput,unsigned  int dimOutput, int normType);

	ANN( const ANN & obj ); 

	ANN( string config_folder);

	~ANN();
/*==============================================*/

/*=============GETTER===========================*/
	unsigned int getDimInput();
	unsigned int getDimOutput();
	unsigned int getNumLayers();

	int getNormType();

	vector<ANN_Layer> getLayers();

	ANN_Layer getLayer( unsigned int index );

	Vector<> getMinInput();
	Vector<> getMaxInput();
	Vector<> getMinOutput();
	Vector<> getMaxOutput();

	void display();

/*==============================================*/

#ifdef HYBRID_NORM
	Vector<> getInputNormMax();
	Vector<> getOutputNormMax();

	void setInputNormMax( Vector<> inputNormMax);
	void setOutputNormMax( Vector<> outputNormMax);

	void setInputNormMax(const char* path);
	void setOutputNormMax(const char* path);
#endif

/*=============SETTER===========================*/
	void setLayers(vector<ANN_Layer> layers);

	void setNormType( int nt );

	void push_back_Layer( ANN_Layer layer );
	void pop_back_Layer();
	void changeLayer(unsigned int index , ANN_Layer layer );

	void setMinInput(Vector<> minInput);
	void setMaxInput(Vector<> maxInput);
	void setMinOutput(Vector<> minOutput);
	void setMaxOutput(Vector<> maxOutput);
/*==============================================*/

/*=============RUNNER===========================*/
	Vector<> compute( Vector<> input );
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
	void push_back_Layer(unsigned int n_neurons, unsigned int dimInput, const char * path, FF_OUT fun);
	void changeLayer(unsigned int index , unsigned int n_neurons, unsigned int dimInput, const char * path, FF_OUT fun);

	void setMinInput(const char* path);
	void setMaxInput(const char* path);
	void setMinMaxInput(const char* path);

	void setMinOutput(const char* path);
	void setMaxOutput(const char* path);
	void setMinMaxOutput(const char* path);
/*========================================================*/

};

ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, Matrix<> WH, Vector<> bh, Matrix<> WO, Vector<> bo, Vector<> minInput, Vector<> maxInput,  Vector<> minOutput, Vector<> maxOutput);

ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* minInput, const char* maxInput, const char* minOutput, const char* maxOutput);

ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* minMaxInput, const char* minMaxOutput);

ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, int normType);

ANN ANN2MAX( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* MaxInput, const char* MaxOutput);

ANN ANN2NULLNORM( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo);


#endif 
