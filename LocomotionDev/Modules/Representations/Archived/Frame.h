/** @file    Frame.h
 *  @brief   A header file containing a frame class
 *  @author  Alex Baucom
 */
 
 #pragma once
 #include "Vector3.h"
 #include "Transform.h"
 
 /** @brief		A frame object which represents a coordinate system on the robot
 *  @details  	Each frame instance references another frame which it is relative to
 *              The defualt reference is NULL which represents the global frame. It
 *              is suggested to instantiate only one global frame and reference all other 
 *              frame to this global reference or another frame which references back to
 *              the global frame in some way (possibly through multiple transforms). 
 *              This will help keep consistancy in the frames
 *              and transforms and will maintain physical representation.
 */
 
 class Frame 
 {
private: 

    /** The frame matrix*/
    double f[4][4] 
    
    /** The reference frame */
    Frame* ref;
    
public:

    // Constructor
    Frame(Frame* reference = NULL);
    
    //Destructor, defualt is fine since we don't want to actually delete what ref points to.
    ~Frame();
    
    // Reset frame to reference frame
    void reset();
    
    //Get a constant entry from the matrix
    const double operator() (int i, int j) const; 
    
    // Get origin of frame with respect to the reference
    Vector3<double> getOrigin();
    
    // Get orientation of X axis with respect to reference
    Vector3<double> getXorientation();
    
    // Get orientation of y axis with respect to reference
    Vector3<double> getYorientation();
    
    // Get orientation of Z axis with respect to reference
    Vector3<double> getZorientation();
    
    // Transform frame according to transform object
    void tform(const Transform &tf); 
    
    // Get transform between current frame and reference
    Transform& getTform();
     
 }