/**
 * @file Transform.h
 * @brief Header file for transform class
 *
 * @author Adapted (and commented) by Alex Baucom for the UPennalizers
 */

#ifndef Transform_h_DEFINED
#define Transform_h_DEFINED

#include <vector>

/** @brief A transformation matrix class
 *  @details This class represents a transformation
 *  matrix that can be used to switch between two frames
 * 
 *  A standard transform matrix is basically a 3x3 rotation matrix
 *  combined with a 3D translation vector. It looks like:
 *  <br>
 *  /  x1  x2  x3  a \ <br>
 *  |  y1  y2  y3  b | <br>
 *  |  z1  z2  z3  c | <br>
 *  \  0   0   0   1 / <br>
 *  where each of the first 3 columns is a unit vector representing
 *  one of the rotation axes of the transform. The fourth column is
 *  the translation vector from one frame to another. The fourth row
 *  of the transform is essentially a placeholder to make the matrix
 *  square and invertble.
 * 
 */
 
class Transform {
public:

  //Constructor
  Transform();
  
  /** Destructor */
  virtual ~Transform() {}
  
  // Clear matrix (identity)
  void clear();
  
  // Translate in 3 dimensions
  Transform &translate(double x, double y, double z);
  
  // Translate in x direction 
  Transform &translateX(double x = 0);
  
  // Translate in y direction
  Transform &translateY(double y = 0);
  
  // Translate in z direction
  Transform &translateZ(double z = 0);
  
  // Rotate about x axis
  Transform &rotateX(double a = 0);
  
  // Rotate about y axis 
  Transform &rotateY(double a = 0);
  
  // Rotate about z axis 
  Transform &rotateZ(double a = 0);
  
  // Transform using modified denavit hartenberg method
  Transform &mDH(double alpha, double a, double theta, double d);
  
  // Apply transform to a 3D vector 
  void apply(double x[3]);
  
  //Get an entry from the transform matrix
  double& operator() (int i, int j);
  
  //Get a constant entry from the matrix
  const double operator() (int i, int j) const;

  void print();

private:
 
  /** Four dimensional matrix to hold transform values */
  double t[4][4];
};

// Multiply two transforms together 
Transform operator* (const Transform &t1, const Transform &t2);

// Invert transform matrix 
Transform inv (const Transform &t1);

// Create transform matrix from 6 dimesnsoinal vector 
Transform transform6D(const double p[6]);

// Create 6 dimensional vector from transform matrix
std::vector<double> position6D(const Transform &t1);

#endif
