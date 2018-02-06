#include <Helper1.0.h>


Vector<> readFileV(const char* path, unsigned int dim){
	FILE *f;
	f = fopen(path, "r");
	if (f == NULL){
		printf(BOLDRED "Error opening file..." CRESET);
		printf(BOLDBLUE " %s\n" CRESET,path);
		exit(1);
	}

	Vector<> out = Zeros(dim);
	double tmp;
	for (int i = 0; i < dim; i++) {
		fscanf(f, "%lf,", &tmp);
		out[i] = tmp;
	}
	fclose(f);

	return out;
}



Matrix<> readFileM(const char* path, unsigned int n_r, unsigned int n_c){
	FILE *f;
	f = fopen(path, "r");
	if (f == NULL){
		printf(BOLDRED "Error opening file..." CRESET);
		printf(BOLDBLUE " %s\n" CRESET,path);
		exit(1);
	}

	Matrix<> out = Zeros(n_r,n_c);
	double tmp;
	for (int i = 0; i < n_r; i++) {
		for (int j = 0; j < n_c; j++) {
			fscanf(f, "%lf", &tmp);
			out[i][j] = tmp;
		}
	}
	fclose(f);

	return out;
}

Matrix<3,3> skew( Vector<3> v ){
	return Data( 0.0, -v[2] , v[1],
				v[2],  0.0  , -v[0],
				-v[1], v[0] , 0.0);
}




char askForChar( const char* str){
	/* USAGE
	char ans = askForChar( "Continue? [y = YES / n = nextIter / e = exit ]: " );
		switch( ans ){
			case 'n' :
			case 'N' :
				continue;
			case 'y' :
			case 'Y' :
			case 's' :
			case 'S' :
				break;
			default:
				exit(1);			
		}
		ans = 0;
	*/

	char ans;
	cout << str;
	cin >> ans;
	cout << endl;

	return ans;
}
