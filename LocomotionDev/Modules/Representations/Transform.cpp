/** @file    Transform.cpp
 *  @brief   Contains transform class to represent a 4x4 transformation matrix.
 *  @author  Austin Small.
 */

#include "Transform.h"
#include <math.h>
#include <iostream>

#define PI 3.14159265

/** @brief   Transform class constructor.
 */
Transform::Transform(void)
{
  clear();
}

/** @brief   Reset 4x4 matrix to identity matrix.
 */
void Transform::clear()
{
    // Initialize to identity matrix.
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            t[i][j] = 0;
        }
    }

    t[0][0] = 1;
    t[1][1] = 1;
    t[2][2] = 1;
    t[3][3] = 1;
}

/** @brief   Performs translation operation on current transformation matrix.
 *
 *  @param   x		Translation in x direction.
 *  @param   y      Translation in y direction.
 *  @param   z      Translation in z direction.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::translate(double x, double y, double z)
{
    // X component.
    t[0][3] += x;
    
    // Y component.
    t[1][3] += y;
    
    // Z component.
    t[2][3] += z;

    return *this;
}

/** @brief   Performs translation operation on current translation matrix.
 *
 *  @param   x		Translation in x direction.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::translateX(double x)
{
    t[0][3] += x;

    return *this;
}

/** @brief   Performs translation operation on current translation matrix.
 *
 *  @param   y		Translation in y direction.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::translateY(double y)
{
    t[1][3] += y;
    
    return *this;
}

/** @brief   Performs translation operation on current translation matrix.
 *
 *  @param   z		Translation in z direction.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::translateZ(double z)
{
    t[2][3] += z;
    
    return *this;
}

/** @brief   Performs clockwise rotation around the x-axis.
 *
 *  @param   a		The clockwise angle of rotation in radians.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::rotateX(double aRad)
{
    double ca = cos(aRad);
    double sa = sin(aRad);
    
    for (int i = 0; i <= 3; i++)
    {
        double t1 = t[1][i];
        double t2 = t[2][i];
        
        t[1][i] = ca * t1 - sa * t2;
        t[2][i] = sa * t1 + ca * t2;
    }
  
    return *this;
}

/** @brief   Performs clockwise rotation around the y-axis.
 *
 *  @param   a		The clockwise angle of rotation in radians.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::rotateY(double aRad)
{
    double ca = cos(aRad);
    double sa = sin(aRad);
    
    for (int i = 0; i <= 3; i++)
    {
        double t0 = t[0][i];
        double t2 = t[2][i];
        
        t[0][i] = ca * t0 + sa * t2;
        t[2][i] = -sa * t0 + ca * t2;
    }
    
    return *this;
}

/** @brief   Performs clockwise rotation around the z-axis.
 *
 *  @param   a		The clockwise angle of rotation in radians.
 *
 *  @return  A reference to the transform object.
 */
Transform& Transform::rotateZ(double aRad)
{
    double ca = cos(aRad);
    double sa = sin(aRad);
    
    for (int i = 0; i <= 3; i++)
    {
        double t0 = t[0][i];
        double t1 = t[1][i];
        
        t[0][i] = ca * t0 - sa * t1;
        t[1][i] = sa * t0 + ca * t1;
    }
    
    return *this;
}

/** @brief   Sets an index of the transformation matrix to a particular value (useful for testing).
 *
 *  @param   i		The row index.
 *  @param   j		The column index.
 *  @param   val	The value to place at the specified index.
 */

void Transform::operator() (int i, int j, double val)
{
    t[i][j] = val;
}


/** @brief   Returns an element's reference from the 4x4 transformation matrix.
*
*  @param   i		The row index.
*  @param   j		The column index.
*
*  @return  A reference value from the specified index.
*/

double& Transform::operator() (int i, int j)
{
    return t[i][j];
}


/** @brief   Multiplies two 4x4 transformation matrices together.
 *
 *  @param   i		The row index.
 *  @param   j		The column index.
 *
 *  @return  A double value from the specified index.
 */

Transform operator* (Transform t1, Transform t2)
{
    Transform t;
    
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            t(i,j) = t1(i,0) * t2(0,j) + t1(i,1) * t2(1,j) + t1(i,2) * t2(2,j) + t1(i,3) * t2(3,j);
        }
    }
    
    return t;
}

/** @brief   Returns the inverse of a transformation matrix.
 *
 *  @param   t1		The transformation matrix to determine the inverse of.
 *
 *  @return  A transformation matrix representing the inverse of the matrix passed into the operation.
 */

Transform inv (Transform t1)
{
    Transform t;
    
    // Transpose rotation.
    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 3; x++)
        {
            t(x,y) = t1(y,x);
        }
    }
    
    for (int i = 0; i < 3; i++)
    {
        t(i,3) -= t(i,0) * t1(0,3) + t(i,1) * t1(1,3) + t(i,2) * t1(2,3);
    }
    
    return t;
}

/** @brief   Performs the transforms specified by the 6D vector.
 *
 *  @param   p		The 6D vector of transformations to perform.
 *
 *  @return  A transformation matrix resulting from performing the transformations.
 */

Transform transform6D(std::vector<double> p)
{
    Transform t;
    t = t.rotateX(p[3]).rotateY(p[4]).rotateZ(p[5]).translate(p[0],p[1],p[2]);
    return t;
}

/** @brief   Returns the a vector containing x, y, z, (xRotation, yRotation, zRotation).
 *           Note that in order for this operation to work, the initial rotation must have been
 *           performed in the order XYZ, and translation must be performed after rotation.
 *           Futhermore, there are several limitations.  Theta y must be between -pi/2 and pi/2
 *           degrees.
 *
 *  @param   t1		The transformation matrix to extract the 6D vector from.
 *
 *  @return  A 6D vector representing position and orientation.
 */

std::vector<double> position6D(Transform t1)
{
    std::vector<double> p(6);
    
    p[0] = t1(0,3);
    p[1] = t1(1,3);
    p[2] = t1(2,3);
    
    p[3] = atan2(t1(2,1), t1(2,2));
    p[4] = asin(-t1(2,0));
    p[5] = atan2(t1(1,0), t1(0,0));
    
    return p;
}

void Transform::print(){
    for(int i=0; i<=3; i++)
    {
        std::cout<<"| ";
        for(int j=0; j<=3; j++)
        {
            std::cout<<t[i][j]<<" ";
        }
        std::cout<<"|\n";
    }
}
