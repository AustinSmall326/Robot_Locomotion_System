/** @file    Transform.h
 *  @brief   Header file to transform class for representation of a 4x4 transformation matrix.
 *  @author  Austin Small.
 */

#ifndef Transform_h_DEFINED
#define Transform_h_DEFINED

#include <vector>

/** @brief		Transform class represents a 4x4 transformation matrix.
 */
class Transform
{
    public:
  
        // Constructors.
        Transform(void);
    
        void clear();
    
        Transform& translate(double x, double y, double z);
        Transform& translateX(double x = 0);
        Transform& translateY(double y = 0);
        Transform& translateZ(double z = 0);
        Transform& rotateX(double a = 0);
        Transform& rotateY(double a = 0);
        Transform& rotateZ(double a = 0);
    
        void operator() (int i, int j, double val);
        double& operator() (int i, int j);
        void print();
    
    private:
    
        double t[4][4];        
};

Transform operator* (Transform t1, Transform t2);
Transform inv (Transform t1);
Transform transform6D(std::vector<double>);
std::vector<double> position6D(Transform t1);

#endif