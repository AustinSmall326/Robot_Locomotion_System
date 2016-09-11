#include "Transform.h"
#include <iostream>
#include <vector>

int main(){

    //Test initialization
     Transform test;
     Transform ptest;
     std::vector<double> p;
     
     double PI = 3.1415926535;
     
     //Test long sequence
     double rz = 30*PI/180;
     double ry = 20*PI/180;
     test = test.rotateZ(rz).rotateY(ry).translate(3,2,1);
     //test.print();
     
     ptest = inv(test);
     //ptest.print();
     
     p = position6D(ptest);
     for (int i=0;i<6;i++)
     {
        std::cout<<p[i]<<" ";
     }
     

}