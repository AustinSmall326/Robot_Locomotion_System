/**
 * @file Frame.cpp
 * @brief Contains methods for frame class
 * @author Alex Baucom
 */

#include "Frame.h"
#include "Transform.h"
#include <math.h>

/** @brief Constructor
 *  @details Initializes to reference matrix
 */

Frame::Frame(Frame* reference) {
    ref = reference;
    reset();
}

/** @brief   Reset matrix
 *  @details Resets to reference matrix, which is the identity for the global frame
 */

void Frame::reset() {
    
    if (ref == NULL) {
      // Initialize to identity matrix:
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          f[i][j] = 0;

      f[0][0] = 1;
      f[1][1] = 1;
      f[2][2] = 1;
      f[3][3] = 1;
    }
    else
    {
       //Copy reference matrix
       for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          f[i][j] = *ref(i,j); 
    }
}

/** @brief   () operator
 *  @details Gets a value from the transform matrix
 *  @param i The row of the desried value
 *  @param j The column of the desired value
 *  @return The value at location i,j in the matrix
 */

double const Frame::operator() (int i, int j) const {
  return t[i][j];
}

/** @brief Gets origin of frame with respect to reference
 *  @return A vector3 object containing the xyz coordinates of the frame origin.
 * */
 
Vector3<double> Frame::getOrigin(){
    Vector3<double> V(f[0][3], f[1][3], f[2][3]);
    return V;
}

/** @brief Gets orientation of X axis of frame with respect to reference
 *  @return A vector3 object containing the xyz coordinates of the X axis orientation.
 * */
 
Vector3<double> Frame::getXorientation(){
    Vector3<double> V(f[0][0], f[1][0], f[2][0]);
    return V;
}

/** @brief Gets orientation of Y axis of frame with respect to reference
 *  @return A vector3 object containing the xyz coordinates of the Y axis orientation.
 * */
 
Vector3<double> Frame::getYorientation(){
    Vector3<double> V(f[0][1], f[1][1], f[2][1]);
    return V;
}

/** @brief Gets orientation of Z axis of frame with respect to reference
 *  @return A vector3 object containing the xyz coordinates of the Z axis orientation.
 * */
 
Vector3<double> Frame::getZorientation(){
    Vector3<double> V(f[0][2], f[1][2], f[2][2]);
    return V;
}

/** @brief Transforms frame based on transform object
 *  @param tf A transform object which must be relative to the reference frame
 *              referenced by the current frame
 * */
void Frame::tform(const Transform &tf) {    
    //Multiply matrices
    double temp[4][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
          temp[i][j] = tf(i,0)*f[0][j] + tf(i,1)*f[1][j] +
        tf(i,2)*f[2][j] + tf(i,3)*f[3][j];
        }
    }
    //Copy temp into f
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
          f[i][j] = temp[i][j]    
}

/** @brief Get the current frame as a transform
 *  @return The address of a transform object which is relative to the reference frame
 *              referenced by the current frame
 * */

Transform& Frame::getTform() {
    Transform* t;
    t->set(f);   
}



