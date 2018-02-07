/*
    ANN Class

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

#include "ANN1.0.h"

/*===============CONSTRUCTORS===================*/
	ANN::ANN( vector<ANN_Layer> layers, Vector<> minInput, Vector<> maxInput, Vector<> minOutput, Vector<> maxOutput ){

		this->minInput = NULL;
		this->maxInput = NULL;
		this->minOutput = NULL;
		this->maxOutput = NULL;

	#ifdef HYBRID_NORM 
		this->inputNormMax = NULL;
		this->outputNormMax = NULL;
	#endif

		this->dimInput = layers.front().getDimInput();

		this->dimOutput = layers.back().getDimOutput();

		ANN::setLayers(layers);

		ANN::setMinInput( minInput );

		ANN::setMaxInput( maxInput );

		ANN::setMinOutput( minOutput );

		ANN::setMaxOutput( maxOutput );

		setNormType(NORM_TYPE_MAPMINMAX);

	}

	ANN::ANN(unsigned int dimInput,unsigned  int dimOutput, int normType){
		this->dimInput = dimInput;

		this->dimOutput = dimOutput;

		setNormType(normType);

		minInput = NULL;
		maxInput = NULL;
		minOutput = NULL;
		maxOutput = NULL;

	#ifdef HYBRID_NORM 
		this->inputNormMax = NULL;
		this->outputNormMax = NULL;
	#endif

	}

	ANN::ANN( unsigned int dimInput , unsigned int dimOutput ): ANN(dimInput, dimOutput, NORM_TYPE_MAPMINMAX){
	}

	ANN::~ANN(){
		if(minInput!=NULL)
			delete minInput;  
		if(maxInput!=NULL)
			delete maxInput;
		if(minOutput!=NULL)
			delete minOutput;
		if(maxOutput!=NULL)
			delete maxOutput;
	}

ANN::ANN( string config_folder){

		string meta_file = config_folder + "/meta.txt";
		Vector<2> metaV = readFileV(meta_file.c_str(),2);

		int numLayers = metaV[0];
		int dimInput = metaV[1];

		string neurons_file = config_folder + "/sizeLayers.txt";	
		Vector<> neurons = readFileV(neurons_file.c_str(),numLayers);

		this->dimInput = dimInput;

		this->dimOutput = neurons[numLayers-1];

		setNormType(NORM_TYPE_MAPMINMAX);
		
		
		string minMax_Input_file = config_folder + "/mM_i.txt";

		setMinMaxInput( minMax_Input_file.c_str() );

		string minMax_Output_file = config_folder + "/mM_o.txt";

		setMinMaxOutput( minMax_Output_file.c_str() );

		for( int jj = 0; jj < numLayers; jj++ ){

			int dimInput_L;
			if(jj == 0)
				dimInput_L = dimInput;
			else
				dimInput_L = neurons[jj-1];

			string W_file = config_folder + "/W" + to_string(jj) + ".txt";
			string b_file = config_folder + "/b" + to_string(jj) + ".txt";	
			
			if(jj == (numLayers-1))
				push_back_Layer( ANN_Layer(neurons[jj] , dimInput_L, W_file.c_str(), b_file.c_str(), ANN_Layer_LINEAR) );
			else
				push_back_Layer( ANN_Layer(neurons[jj] , dimInput_L, W_file.c_str(), b_file.c_str(), ANN_Layer_SIGMA) );
		}

	}



	ANN::ANN( const ANN & obj ){

		dimInput = obj.dimInput;
		dimOutput = obj.dimOutput;

		minInput = new Vector<>(*(obj.minInput));
		maxInput = new Vector<>(*(obj.maxInput));
		minOutput = new Vector<>(*(obj.minOutput));
		maxOutput = new Vector<>(*(obj.maxOutput));

		setNormType( obj.normType );

		layers = vector<ANN_Layer>(obj.layers);

		
	}
/*==============================================*/

/*=============GETTER===========================*/
	unsigned int ANN::getDimInput(){ return dimInput;}
	unsigned int ANN::getDimOutput(){ return dimOutput;}
	unsigned int ANN::getNumLayers(){ return layers.size(); }

	int ANN::getNormType(){ return normType;}

	vector<ANN_Layer> ANN::getLayers(){ return layers;}

	ANN_Layer ANN::getLayer( unsigned int index ){
		if(index >= getNumLayers() ){
			printf( BOLDRED "ERROR! ANN::getLayer()" CRESET RED 
				" - Invalid LayerIndex " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "<[%d]" CRESET "\n", index, ANN::getNumLayers());
			exit(1);
		}
		return layers[index];
	}

	Vector<> ANN::getMinInput(){ 
		if(minInput == NULL){
			printf(BOLDRED "ERROR! getMinInput() - NULL Vector - \n" CRESET);
			exit(1);
		}
		return Vector<>(*minInput);
	}

	Vector<> ANN::getMaxInput(){ 
		if(maxInput == NULL){
			printf(BOLDRED "ERROR! getMaxInput() - NULL Vector - \n" CRESET);
			exit(1);
		}
		return Vector<>(*maxInput);
	}

	Vector<> ANN::getMinOutput(){
		if(minOutput == NULL){
			printf(BOLDRED "ERROR! getMinOutput() - NULL Vector - \n" CRESET);
			exit(1);
		}
		return Vector<>(*minOutput);
	}
	Vector<> ANN::getMaxOutput(){
		if(maxOutput == NULL){
			printf(BOLDRED "ERROR! getMaxOutput() - NULL Vector - \n" CRESET);
			exit(1);
		}
		return Vector<>(*maxOutput);
	}

	void ANN::display(){
		cout << endl << BOLDMAGENTA << "==============================================="  << endl;
		cout << "ANN DISPLAY" << endl;
		cout << "dimInputs" << dimInput << " | dimOut " << dimOutput << CRESET << endl;
		for( int i=0; i<layers.size(); i++ ){
			cout << "Layer " << i << endl;
			layers[i].display();
		}
		cout << endl << BOLDMAGENTA "===============================================" << CRESET << endl;
		cout << "minOutput = " << endl;
		cout << *minOutput << endl;
		cout << "maxOutput = " << endl;
		cout << *maxOutput << endl;
		cout << "maxInput = " << endl;
		cout << *maxInput << endl;
		cout << "minInput = " << endl;
		cout << *minInput << endl;
		
		
		cout << endl << BOLDMAGENTA "===============================================" << CRESET << endl;
	}
		
/*==============================================*/

#ifdef HYBRID_NORM
	Vector<> ANN::getInputNormMax(){
		if(inputNormMax == NULL){
			printf(BOLDRED "ERROR! getInputNormMax() - NULL Vector - \n" CRESET);
			exit(1);
		}
		return Vector<>(*inputNormMax);
	}

	Vector<> ANN::getOutputNormMax(){
		if(outputNormMax == NULL){
			printf(BOLDRED "ERROR! getOutputNormMax() - NULL Vector - \n" CRESET);
			exit(1);
		}
		return Vector<>(*outputNormMax);
	}

	void ANN::setInputNormMax( Vector<> inputNormMax){
		if( inputNormMax.size() != dimInput ){
			printf( BOLDYELLOW "WARNING! ANN::setInputNormMax()" CRESET BOLDYELLOW 
				" - input dim " CRESET BOLDWHITE "[%d]" CRESET BOLDYELLOW" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", inputNormMax.size(), dimInput);
		}

		if( this->inputNormMax != NULL )
			delete this->inputNormMax;

		this->inputNormMax = new Vector<>(inputNormMax);
	}

	void ANN::setOutputNormMax( Vector<> outputNormMax){
				if( outputNormMax.size() != dimOutput ){
			printf( BOLDYELLOW "WARNING! ANN::setOutputNormMax()" CRESET BOLDYELLOW 
				" - output dim " CRESET BOLDWHITE "[%d]" CRESET BOLDYELLOW" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", outputNormMax.size(), dimOutput);
		}

		if( this->outputNormMax != NULL )
			delete this->outputNormMax;

		this->outputNormMax = new Vector<>(outputNormMax);
	}

	void ANN::setInputNormMax(const char* path){
		setInputNormMax( readFileV(path , getDimInput() ) );
	}

	void ANN::setOutputNormMax(const char* path){
		setOutputNormMax( readFileV(path , getDimOutput() ) );
	}

#endif

/*=============SETTER===========================*/

	void ANN::setNormType( int nt ){

		switch(nt){
			case NORM_TYPE_NULL : {
				normFun=&ANN::nullNorm;
				invNormFun=&ANN::nullNorm;
				break;
			}
			case NORM_TYPE_MAPMINMAX : {
				normFun=&ANN::mapMinMax;
				invNormFun=&ANN::InvMapMinMax;
				break;
			}
			case NORM_TYPE_MAX : {
				normFun=&ANN::normMax;
				invNormFun=&ANN::InvNormMax;
				break;
			}
		#ifdef HYBRID_NORM
			case NORM_TYPE_HYBRID : {
				normFun=&ANN::normHybrid;
				invNormFun=&ANN::invNormHybrid;
				break;
			}
		#endif
			default : {
				printf( BOLDRED "ERROR! ANN::setNormType()" CRESET RED 
				" - Invalid normType " CRESET);
				exit(1);
			}
		}
		this->normType = nt; 
	}

	void ANN::setLayers(vector<ANN_Layer> layers){
		/*if( layers.front().getDimInput() != dimInput ){
			printf( BOLDRED "ERROR! ANN::setLayers()" CRESET RED 
				" - Invalid FirstLayer input dim " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", layers.front().getDimInput(), dimInput);
			exit(1); 
		}
		if( layers.back().getDimOutput() != dimOutput ){
			printf( BOLDRED "ERROR! ANN::setLayers()" CRESET RED 
				" - Invalid LastLayer output dim " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", layers.back().getDimInput(), dimOutput);
			exit(1); 
		}*/

		this->dimInput = layers.front().getDimInput();

		this->dimOutput = layers.back().getDimOutput();

		this->layers = layers;

	}

	void ANN::push_back_Layer( ANN_Layer layer ){

		if(layers.empty()){
			this->dimInput = layer.getDimInput();
		}
		;
		this->dimOutput = layer.getDimOutput();
		
		layers.push_back(layer);
	}

	void ANN::pop_back_Layer(){
		if(layers.empty()){
			printf( BOLDYELLOW "WARNING! ANN::pop_back_Layer() - NO LAYERS\n" CRESET);
			return;
		}
		layers.pop_back();
		if(layers.empty()){
			return;
		}
		this->dimOutput = layers.back().getDimOutput(); 
	}

	void ANN::changeLayer(unsigned int index , ANN_Layer layer ){

		if(index >= ANN::getNumLayers()){
			printf( BOLDRED "ERROR! ANN::changeLayer()" CRESET RED 
				" - Invalid LayerIndex " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "<[%d]" CRESET "\n", index, ANN::getNumLayers());
			exit(1);
		}
		if(index == (ANN::getNumLayers()-1) ){
			ANN::pop_back_Layer();
			ANN::push_back_Layer( layer );
			return;
		}

		if(index != 0){
			if( getLayer(index-1).getDimOutput() != layer.getDimInput() ){
				printf( BOLDRED "ERROR! ANN::changeLayer()" CRESET RED 
				" - Invalid LayerInputDim " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "<[%d]" CRESET "\n", layer.getDimInput(), getLayer(index-1).getDimOutput());
				exit(1);
			}
		}
		if( getLayer(index+1).getDimInput() != layer.getDimOutput() ){
				printf( BOLDRED "ERROR! ANN::changeLayer()" CRESET RED 
				" - Invalid LayerOutputDim " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "<[%d]" CRESET "\n", layer.getDimOutput(), getLayer(index-1).getDimInput());
				exit(1);
			}

		layers.insert( layers.erase( layers.begin() +index ) , layer );

	}


	void ANN::setMinInput(Vector<> minInput_){

		if( minInput_.size() != dimInput ){
			printf( BOLDYELLOW "WARNING! ANN::setMinInput()" CRESET BOLDYELLOW 
				" - input dim " CRESET BOLDWHITE "[%d]" CRESET BOLDYELLOW" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", minInput_.size(), dimInput);
			exit(1);
		}

		//if( this->minInput != NULL )
		//	delete this->minInput;

		this->minInput = new Vector<>(minInput_);
	
	}

	void ANN::setMaxInput(Vector<> maxInput){
		if( maxInput.size() != dimInput ){
			printf( BOLDYELLOW "WARNING! ANN::setMaxInput()" CRESET BOLDYELLOW 
				" - input dim " CRESET BOLDWHITE "[%d]" CRESET BOLDYELLOW" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", maxInput.size(), dimInput);
		}

		//if( this->maxInput != NULL )
		//	delete this->maxInput;

		this->maxInput = new Vector<>(maxInput);

	}
	void ANN::setMinOutput(Vector<> minOutput){
		if( minOutput.size() != dimOutput ){
			printf( BOLDYELLOW "WARNING! ANN::setMinOutput()" CRESET BOLDYELLOW 
				" - output dim " CRESET BOLDWHITE "[%d]" CRESET BOLDYELLOW" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", minOutput.size(), dimOutput);
		}

		//if( this->minOutput != NULL )
		//	delete this->minOutput;

		this->minOutput = new Vector<>(minOutput);
	}

	void ANN::setMaxOutput(Vector<> maxOutput){
		if( maxOutput.size() != dimOutput ){
			printf( BOLDYELLOW "WARNING! ANN::setMaxOutput()" CRESET BOLDYELLOW 
				" - output dim " CRESET BOLDWHITE "[%d]" CRESET BOLDYELLOW" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", maxOutput.size(), dimOutput);
		}

		//if( this->maxOutput != NULL )
		//	delete this->maxOutput;

		this->maxOutput = new Vector<>(maxOutput);
	}
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
	void ANN::push_back_Layer(unsigned int n_neurons, unsigned int dimInput, const char * path, FF_OUT fun){
		char strW[100], strB[100];
  		strcpy (strW,path);
  		strcat (strW,"/W.txt");
  		strcpy (strB,path);
  		strcat (strB,"/B.txt");
		ANN_Layer myLayer = ANN_Layer(n_neurons, dimInput, strW , strB, fun);
		push_back_Layer(myLayer);
	}

	void ANN::changeLayer(unsigned int index , unsigned int n_neurons, unsigned int dimInput, const char * path, FF_OUT fun){
		char strW[100], strB[100];
  		strcpy (strW,path);
  		strcat (strW,"/W.txt");
  		strcpy (strB,path);
  		strcat (strB,"/B.txt");
		ANN_Layer myLayer = ANN_Layer(n_neurons, dimInput, strW, strB, fun);
		changeLayer(index , myLayer );
	}

	void ANN::setMinInput(const char* path){
   
		setMinInput(  readFileV(path, getDimInput() ) );
	}

	void ANN::setMaxInput(const char* path){
	
		setMaxInput( readFileV(path , getDimInput() ) );
	}

	void ANN::setMinMaxInput(const char* path){
		
		Matrix<> TMP = readFileM(path, getDimInput() , 2);
		
		setMinInput( TMP.T()[0] );
		
		setMaxInput( TMP.T()[1] );
		
	}

	void ANN::setMinOutput(const char* path){
	
		setMinOutput(  readFileV(path, getDimOutput() ) );
	}

	void ANN::setMaxOutput(const char* path){
	
		setMaxOutput( readFileV(path , getDimOutput() ) );
	}

	void ANN::setMinMaxOutput(const char* path){
		Matrix<> TMP = readFileM(path, getDimOutput() , 2);
		setMinOutput( TMP.T()[0] );
		setMaxOutput( TMP.T()[1] );
	}
/*========================================================*/

/*=============RUNNER===========================*/
	Vector<> ANN::compute( Vector<> input ){
		
		//input = (this->*normFun)(input);
		Vector<> * TMP1 = new Vector<>( (this->*normFun)(input) );
		Vector<> * TMP2;
		
		for( int i=0; i<layers.size(); i++ ){
			
			//input = layers[i].compute(input);
			TMP2 = new Vector<>( layers[i].compute(*TMP1) );
			delete TMP1;
			TMP1 = TMP2;
		}
		
		Vector<> OO = (this->*invNormFun)(*TMP1);
		
		delete TMP1;
		
		return OO;

	}
/*==============================================*/


/*=========PRIVATE METH==========================*/

	Vector<> ANN::nullNorm( Vector<> in ){ return in; }

	Vector<> ANN::mapMinMax(Vector<> in){
		for (int s = 0; s < getDimInput(); s++) {
			in[s] = ((2.0 / ((*maxInput)[s] - (*minInput)[s])) * (in[s] - (*minInput)[s])) - 1.0; 
		}
		return in;
	}

	Vector<> ANN::InvMapMinMax(Vector<> in){
		
		for (int h = 0; h < getDimOutput(); h++) {  
			in[h] = ( ((*maxOutput)[h] - (*minOutput)[h]) / 2.0 * (in[h] + 1.0) + (*minOutput)[h] );
		}
		
		return in;
	}

	Vector<> ANN::normMax(Vector<> in){
		for (int s = 0; s < getDimInput(); s++) {
			in[s] = in[s] / (*maxInput)[s]; 
		}
		return  in;
	}

	Vector<> ANN::InvNormMax(Vector<> in){
		for (int s = 0; s < getDimOutput(); s++) {
			in[s] = in[s] * (*maxOutput)[s]; 
		}
		return in;
	}

#ifdef HYBRID_NORM
	Vector<> ANN::normHybrid(Vector<> in){
		return mapMinMax( normMax( in ) );
	}
	Vector<> ANN::invNormHybrid(Vector<> in){
		return InvNormMax( InvMapMinMax( in ) );
	}
#endif

/*==============================================*/

/*==============STATIC FUN========================*/

	ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, Matrix<> WH, Vector<> bh, Matrix<> WO, Vector<> bo, Vector<> minInput, Vector<> maxInput,  Vector<> minOutput, Vector<> maxOutput){
		
		ANN myANN = ANN( dimInput , OL_NumNeurons );

		myANN.push_back_Layer( ANN_Layer(HL_NumNeurons , dimInput, WH, bh, ANN_Layer_SIGMA) );

		myANN.push_back_Layer( ANN_Layer(OL_NumNeurons , HL_NumNeurons, WO, bo, ANN_Layer_LINEAR) );

		myANN.setMinInput( minInput );
		myANN.setMaxInput( maxInput );

		myANN.setMinOutput( minOutput );
		myANN.setMaxOutput( maxOutput );

		return myANN;

	}

	ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* minInput, const char* maxInput, const char* minOutput, const char* maxOutput){
		
		ANN myANN = ANN( dimInput , OL_NumNeurons );

		myANN.push_back_Layer( ANN_Layer(HL_NumNeurons , dimInput, WH, bh, ANN_Layer_SIGMA) );

		myANN.push_back_Layer( ANN_Layer(OL_NumNeurons , HL_NumNeurons, WO, bo, ANN_Layer_LINEAR) );

		myANN.setMinInput( minInput );
		myANN.setMaxInput( maxInput );

		myANN.setMinOutput( minOutput );
		myANN.setMaxOutput( maxOutput );

		return myANN;
	}

	ANN ANN2( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* minMaxInput, const char* minMaxOutput){
		
		ANN myANN = ANN( dimInput , OL_NumNeurons );
		
		myANN.push_back_Layer( ANN_Layer(HL_NumNeurons , dimInput, WH, bh, ANN_Layer_SIGMA) );
		
		myANN.push_back_Layer( ANN_Layer(OL_NumNeurons , HL_NumNeurons, WO, bo, ANN_Layer_LINEAR) );
		
		myANN.setMinMaxInput( minMaxInput );
		
		myANN.setMinMaxOutput( minMaxOutput );
		
		return myANN;
	}

	ANN ANN2MAX( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* MaxInput, const char* MaxOutput){

		ANN myANN = ANN( dimInput , OL_NumNeurons, NORM_TYPE_MAX );
		
		myANN.push_back_Layer( ANN_Layer(HL_NumNeurons , dimInput, WH, bh, ANN_Layer_SIGMA) );
		
		myANN.push_back_Layer( ANN_Layer(OL_NumNeurons , HL_NumNeurons, WO, bo, ANN_Layer_LINEAR) );
		
		myANN.setMaxInput( MaxInput );
		
		myANN.setMaxOutput( MaxOutput );
		
		return myANN;
	}

	ANN ANN2NULLNORM( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo){

		ANN myANN = ANN( dimInput , OL_NumNeurons, NORM_TYPE_NULL );
		
		myANN.push_back_Layer( ANN_Layer(HL_NumNeurons , dimInput, WH, bh, ANN_Layer_SIGMA) );
		
		myANN.push_back_Layer( ANN_Layer(OL_NumNeurons , HL_NumNeurons, WO, bo, ANN_Layer_LINEAR) );

		return myANN;
	}

/*#ifdef HYBRID_NORM NON FATTA
		ANN ANN2HYBRID( unsigned int dimInput, unsigned int HL_NumNeurons, unsigned int OL_NumNeurons, const char* WH, const char* bh, const char* WO, const char* bo, const char* MaxInput, const char* MaxOutput){
		
		cout << "NORM_TYPE_MAX" << NORM_TYPE_MAX << endl;

		ANN myANN = ANN( dimInput , OL_NumNeurons, NORM_TYPE_MAX );
		
		myANN.push_back_Layer( ANN_Layer(HL_NumNeurons , dimInput, WH, bh, ANN_Layer_SIGMA) );
		
		myANN.push_back_Layer( ANN_Layer(OL_NumNeurons , HL_NumNeurons, WO, bo, ANN_Layer_LINEAR) );
		
		myANN.setMinMaxInput( MaxInput );
		
		myANN.setMinMaxOutput( MaxOutput );
		
		return myANN;
	}
#endif*/
/*==============================================*/
