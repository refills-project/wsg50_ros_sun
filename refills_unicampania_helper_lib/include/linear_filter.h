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

#include <TooN/SVD.h>
#include <TooN/TooN.h>

using namespace TooN;

/* Class definition */
class LinearFilter {
	
	private:
	  	Vector<> x; 
		Matrix<> A;
		Matrix<> B;
		Matrix<> C;
		Matrix<> D;
	
    public:
    
	    int _order;
	    int _num_inputs;
	    int _num_outputs;
        
        /* Constructor */
        LinearFilter(int ord, int num_in, int num_out);
        
        /* Getting the parameters of the LinearFilter */
        Matrix<> Get_A();
        Matrix<> Get_B();
        Matrix<> Get_C();
        Matrix<> Get_D();
        Vector<> Get_x();
               
        /* Setting the parameters of the LinearFilter */
        void Set_A(Matrix<> A);
        void Set_B(Matrix<> B);
        void Set_C(Matrix<> C);
        void Set_D(Matrix<> D);
        void Set_x(Vector<> x);
        
        void Set_Filter(Vector<> x,Matrix<> A,Matrix<> B,Matrix<> C,Matrix<> D);
        
        /* Apply the filter */
        Vector<> apply_filter(Vector<> u);
        
        /* Display the filter matrices */
        void display();
 
    protected:
      
};
