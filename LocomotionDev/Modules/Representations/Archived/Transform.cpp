/**
 * @file Transform.cpp
 * @brief Contains methods for transform class
 *
 * @author Adapted (and commented) by Alex Baucom for the UPennalizers
 */

#include "Transform.h"
#include <math.h>
#include <iostream>

/** @brief Constructor
 *  @details Initializes to identity matrix
 */

Transform::Transform() {
  clear();
}

/** @brief   Clear matrix
 *  @details Resets to identity matrix
 */

void Transform::clear() {
  // Initialize to identity matrix:
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      t[i][j] = 0;

  t[0][0] = 1;
  t[1][1] = 1;
  t[2][2] = 1;
  t[3][3] = 1;
}

/** @brief   Translate in 3D
 *  @details Translates matrix based on inputs in x, y, and z directions
 *  @param x Distance to translate in X direction
 *  @param y Distance to translate in Y direction
 *  @param z Distance to translate in Z direction
 *  @return Address of transform object that has been translated accordinly
 */

Transform& Transform::translate(double x, double y, double z) {
  t[0][3] += t[0][0]*x + t[0][1]*y + t[0][2]*z;
  t[1][3] += t[1][0]*x + t[1][1]*y + t[1][2]*z;
  t[2][3] += t[2][0]*x + t[2][1]*y + t[2][2]*z;
  return *this;
}

/** @brief   Translate in X direction
 *  @details Translates matrix based on inputs in x direction
 *  @param x Distance to translate in X direction
 *  @return Address of transform object that has been translated accordinly
 */

Transform& Transform::translateX(double x) {
  t[0][3] += t[0][0]*x;
  t[1][3] += t[1][0]*x;
  t[2][3] += t[2][0]*x;
  return *this;
}

/** @brief   Translate in Y direction
 *  @details Translates matrix based on inputs in y direction
 *  @param y Distance to translate in Y direction
 *  @return Address of transform object that has been translated accordinly
 */

Transform& Transform::translateY(double y) {
  t[0][3] += t[0][1]*y;
  t[1][3] += t[1][1]*y;
  t[2][3] += t[2][1]*y;
  return *this;
}

/** @brief   Translate in Z direction
 *  @details Translates matrix based on inputs in z direction
 *  @param z Distance to translate in Z direction
 *  @return Address of transform object that has been translated accordinly
 */

Transform& Transform::translateZ(double z) {
  t[0][3] += t[0][2]*z;
  t[1][3] += t[1][2]*z;
  t[2][3] += t[2][2]*z;
  return *this;
}

/** @brief   Rotate about X axis
 *  @details Rotates matrix about the X axis with an angle a
 *  @param a Relatie angle to rotate matrix (radians)
 *  @return Address of transform object that has been rotated accordinly
 */

Transform& Transform::rotateX(double a) {
  double ca = cos(a);
  double sa = sin(a);
  for (int i = 0; i < 3; i++) {
    double ty = t[i][1];
    double tz = t[i][2];
    t[i][1] = ca*ty + sa*tz;
    t[i][2] = -sa*ty + ca*tz;
  }
  return *this;
}

/** @brief   Rotate about Y axis
 *  @details Rotates matrix about the Y axis with an angle a
 *  @param a Relatie angle to rotate matrix (radians)
 *  @return Address of transform object that has been rotated accordinly
 */

Transform& Transform::rotateY(double a) {
  double ca = cos(a);
  double sa = sin(a);
  for (int i = 0; i < 3; i++) {
    double tx = t[i][0];
    double tz = t[i][2];
    t[i][0] = ca*tx - sa*tz;
    t[i][2] = sa*tx + ca*tz;
  }
  return *this;
}

/** @brief   Rotate about Z axis
 *  @details Rotates matrix about the Z axis with an angle a
 *  @param a Relatie angle to rotate matrix (radians)
 *  @return Address of transform object that has been rotated accordinly
 */

Transform& Transform::rotateZ(double a) {
  double ca = cos(a);
  double sa = sin(a);
  for (int i = 0; i < 3; i++) {
    double tx = t[i][0];
    double ty = t[i][1];
    t[i][0] = ca*tx + sa*ty;
    t[i][1] = -sa*tx + ca*ty;
  }
  return *this;
}

/** @brief   Modified Denavit Hartenberg transform
 *  @details Transforms the matrix accoring to the denavit hartenberg method. This method uses
 *           a sequence of rotations and translations to transform between specially constructed frames.
 *           It is particularly useful for constructing frames of robotic joints in an chain.
 *  @param alpha The angle to rotate about the new X axis
 *  @param a The distance to translate along the new X axis
 *  @param theta The angle to rotate about the original Z axis
 *  @param d The distance to translate along the new Z axis
 *  @return Address of transform object that has been transformed accordinly
 */

Transform& Transform::mDH(double alpha, double a, double theta, double d) {
  this->translateX(a).rotateX(alpha).translateZ(d).rotateZ(theta);
  return *this;
}

/** @brief   Apply to a 3D vector
 *  @details Applies the transform to a vector to tranform it from one frame to another
 *  @param x An array of three values representing a 3-dimensional vector
 */

void Transform::apply(double x[3]) {
  double x0[3];
  for (int i = 0; i < 3; i++) {
    x0[i] = x[i];
  }
  for (int i = 0; i < 3; i++) {
    x[i] = t[i][3];
    for (int j = 0; j < 3; j++) {
      x[i] += t[i][j]*x0[j];
    }
  }
}

/** @brief   () operator
 *  @details Gets a value from the transform matrix
 *  @param i The row of the desried value
 *  @param j The column of the desired value
 *  @return The value at location i,j in the matrix
 */

double const Transform::operator() (int i, int j) const {
  return t[i][j];
}

/** @brief   () operator
 *  @details Gets a value from the transform matrix
 *  @param i The row of the desried value
 *  @param j The column of the desired value
 *  @return A pointer to the value at location i,j in the matrix
 */

double& Transform::operator() (int i, int j) {
  return t[i][j];
}

/** @brief   Directly set the values of the transform matrix
 *  @param mtx[][4] A 4x4 matrix to set the transform 
 */
void Transform::set(double mtx[][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          t[i][j] = mtx[i][j];
}


/** @brief   * operator
 *  @details Multiply two transform matrices together
 *  @param t1 Transform 1 object
 *  @param t2 Transform 2 object
 *  @return The product of the multiplication
 */

Transform operator* (const Transform &t1, const Transform &t2) {
  Transform t;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      t(i,j) = t1(i,0)*t2(0,j) + t1(i,1)*t2(1,j) +
	t1(i,2)*t2(2,j) + t1(i,3)*t2(3,j);
    }
  }
  return t;
}


/** @brief   Transform inversion
 *  @details Inverts the matrix based on rules when dealing with frames
 *           and transforms in order to speed up the process. Only valid for
 *           matrices which are transforms.
 *  @param t1 The transform to invert
 *  @return The inverted transform
 */

Transform inv (const Transform &t1) {
  Transform t;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      // Transpose rotation:
      t(i,j) = t1(j,i);
      // Compute inv translation:
      t(i,3) -= t1(j,i)*t1(j,3);
    }
  }
  return t;
}


/** @brief   Creates transform from 6D vector
 *  @details A transform matrix only has 6 DOF so this method can take 
 *           an array with 6 values and turn it into a transform matrix
 *  @param p The array with 6 values
 *  @return A transform created from the 6 DOF
 */

Transform transform6D(const double p[6]) {
  Transform t;
  //  t = t.translate(p[0],p[1],p[2]).rotateZ(p[5]).rotateY(p[4]).rotateX(p[3]);

  double cwx = cos(p[3]);
  double swx = sin(p[3]);
  double cwy = cos(p[4]);
  double swy = sin(p[4]);
  double cwz = cos(p[5]);
  double swz = sin(p[5]);
  t(0,0) = cwy*cwz;
  t(0,1) = swx*swy*cwz-cwx*swz;
  t(0,2) = cwx*swy*cwz+swx*swz;
  t(0,3) = p[0];
  t(1,0) = cwy*swz;
  t(1,1) = swx*swy*swz+cwx*cwz;
  t(1,2) = cwx*swy*swz-swx*cwz;
  t(1,3) = p[1];
  t(2,0) = -swy;
  t(2,1) = swx*cwy;
  t(2,2) = cwx*cwy;
  t(2,3) = p[2];
  return t;
}

/** @brief   Creates 6D vector from transform
 *  @details A transform matrix only has 6 DOF so this method can take 
 *           a transform and turn it into an array with 6 DOF
 *  @param t1 A transform
 *  @return A vector with 6 values that contain all 6 DOF from the transform
 */

std::vector<double> position6D(const Transform &t1) {
  std::vector<double> p(6);
  p[0] = t1(0,3);
  p[1] = t1(1,3);
  p[2] = t1(2,3);
  p[3] = atan2(t1(2,1), t1(2,2));
  p[4] = -asin(t1(2,0));
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
