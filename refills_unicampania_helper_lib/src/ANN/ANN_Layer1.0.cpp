#include "ANN_Layer1.0.h"

/*===============CONSTRUCTORS===================*/
ANN_Layer::ANN_Layer(unsigned int n_neurons, unsigned int dimInput, Matrix<> W, Vector<> b, FF_OUT fun){ //COMPLETE CONSTRUCTOR

	this->W=NULL;
	this->b=NULL;

	this->n_neurons = n_neurons;

	this->dimInput = dimInput;

	ANN_Layer::setW(W);

	ANN_Layer::setB(b);

	ANN_Layer::setFun(fun);

}

ANN_Layer::ANN_Layer(Matrix<> W, Vector<> b, FF_OUT fun) : ANN_Layer::ANN_Layer(W.num_rows(), W.num_cols(), W, b, fun){
}

ANN_Layer::ANN_Layer(unsigned int n_neurons, unsigned int dimInput, const char* pathW, const char* pathB, FF_OUT fun){

	W=NULL;
	b=NULL;


	this->n_neurons = n_neurons;
	this->dimInput = dimInput;

	ANN_Layer::setW( pathW );

	ANN_Layer::setB( pathB );

	ANN_Layer::setFun(fun);

}

ANN_Layer::ANN_Layer(unsigned int n_neurons, unsigned int dimInput, Matrix<> W, Vector<> b) : ANN_Layer::ANN_Layer(n_neurons, dimInput, W, b, ANN_Layer_LINEAR){ //fun = linear;
}

ANN_Layer::ANN_Layer(Matrix<> W, Vector<> b) : ANN_Layer::ANN_Layer(W.num_rows(), W.num_cols(), W, b){
}

ANN_Layer::ANN_Layer(unsigned int n_neurons, unsigned int dimInput, const char* pathW, const char* pathB) : ANN_Layer::ANN_Layer(n_neurons, dimInput, pathW, pathB, ANN_Layer_LINEAR){//FROM FILE fun = linear
}

ANN_Layer::ANN_Layer(unsigned int n_neurons, unsigned int dimInput){ //fun = linear
	
	W=NULL;
	b=NULL;

	this->n_neurons = n_neurons;

	this->dimInput = dimInput;

	ANN_Layer::setFun(ANN_Layer_LINEAR);

}

ANN_Layer::ANN_Layer(const ANN_Layer& myLayer){
	n_neurons = myLayer.n_neurons;
	dimInput = myLayer.dimInput;
	W =new Matrix<>( *(myLayer.W) );
	b = new Vector<>( *(myLayer.b) );
	myFF = myLayer.myFF;
}
/*==============================================*/

/*===============DESTRUCTOR===================*/
ANN_Layer::~ANN_Layer(){ //(destructor)
	if(W!=NULL)
		delete W;
	if(b!=NULL)
		delete b;
}
/*==============================================*/

/*=============GETTER===========================*/
	unsigned int ANN_Layer::getNNeurons(){ return n_neurons; }

	unsigned int ANN_Layer::getDimInput(){ return dimInput; }

	unsigned int ANN_Layer::getDimOutput(){ return getNNeurons(); }

	Matrix<> ANN_Layer::getW(){ return Matrix<>(*W); }

	Vector<> ANN_Layer::getB(){ return Vector<>(*b); }

	FF_OUT ANN_Layer::getFun(){ return myFF; }
/*==============================================*/

/*=============SETTER===========================*/
	void ANN_Layer::setW( Matrix<> W ){

		if( (W.num_rows()!=n_neurons) || (W.num_cols()!=dimInput) ){
			printf( BOLDRED "ERROR! ANN_Layer::setW()" CRESET RED 
				" - Invalid Matrix dim " CRESET BOLDWHITE "[%d,%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "[%d,%d]" CRESET "\n", W.num_rows(), W.num_cols() ,n_neurons,dimInput);
			exit(1); 
		}

		if( this->W != NULL)
			delete this->W;

		this->W = new Matrix<>(W);
	}

	void ANN_Layer::setB( Vector<> b ){

		if(  b.size() != n_neurons ){
			printf( BOLDRED "ERROR! ANN_Layer::setB()" CRESET RED 
				" - Invalid Vector dim " CRESET BOLDWHITE "[%d]" CRESET RED" - expected:"
				 CRESET BOLDWHITE "[%d]" CRESET "\n", b.size(), n_neurons);
			exit(1);
		}

		if( this->b != NULL){
			delete this->b;
		}

		fflush(stdout);
		this->b = new Vector<>(b);
	}

	void ANN_Layer::setFun( FF_OUT ff ){
		myFF = ff;
	}
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
	void ANN_Layer::setW(const char* path){
		ANN_Layer::setW( readFileM(path, n_neurons, dimInput) );
	}


	void ANN_Layer::setB( const char* path){
		ANN_Layer::setB( readFileV(path, n_neurons) );
	}
/*========================================================*/

/*=============RUNNER===========================*/
	Vector<> ANN_Layer::compute( Vector<> input ){
		//Il controllo dovrebbe farlo TooN
		/*if( input.size() != dimInput ){
			print("ERROR! ANN_Layer::compute() - Invalid Vector dim [%d] - [%d]\n",  input.size(), dimInput);
			exit(1);
		}*/
		//std::cout << (*b).size() << " | " << (*W).num_rows() << " | " << (*W).num_cols() << " | " << input.size() << std::endl;
		Vector <> V = Vector<>((*b) + ( (*W)  * (input) ));
		//std::cout << "V=" << V.size() << std::endl;
		return myFF( V );
	}
/*==============================================*/

/*=============VARIE===========================*/
	void ANN_Layer::display(){
	cout << endl << "===========================" << endl;
	cout << "ANN LAYER DISP:" << endl;
	cout << "W = " << endl;
	cout << *W << endl;
	cout << "b = " << endl;
	cout << *b << endl;
	cout << "===========================" << endl;
}
/*==============================================*/

/*=============STATIC FUNS===========================*/	
	Vector<> ANN_Layer_SIGMA(Vector<> x) {
		for(int i = 0; i < x.size(); i++ )
			x[i] = (2.0 / (1.0 + exp(-2.0 * x[i])) - 1.0);
		return x;
	}

	Vector<> ANN_Layer_LINEAR(Vector<> x){ return x; }

/*===================================================*/
