/** @file    Trajectory.h
 *  @brief   A header file containing a trajectory class
 *  @author  Alex Baucom
 */

#pragma once
#include "Vector3.h"
#include "Frame.h"


/** @brief		A 3d trajectory consisting of time-stamped points
 *  @details  	This object contains a set of n points that
 *  			define a trajectory between two points and
 *  			have time stamps for each point. It also has a
 *              reference frame embedded that all points are relative to.
 */

class Trajectory
{

private:

	int n;                  /**< Number of points in trajectory*/
	Vector3<double>* p;     /**< Pointer to array of points in trajectory*/
	double* t;              /**< Pointer to array of timestamps for trajectory*/
    int path;               /**< type of path used for auto generation */
    Frame ref;              /**< reference frame used for trajectory */
     
public:
    
     //Constructor 1 - auto generated
     Trajectory(int numPts, Vector3<double> startPt, Vector3<double> endPt, double startTime, double endTime, int pathType, const Frame &reference);
     
     //Constructor 2 - manual specification
     Trajectory(int numPts, Vector3<double>* pts, double* timestamps, const Frame &reference);
     
     //Destructor
     ~Trajectory();
     
     //Get length of array (number of points)
     int getLength();
     
     //Get path type
     int getPath();
     
     //Get point n
     Vector3<double> getPoint(int n);
     
     //Get timestamp
     double getTime(int n);
     
     //Other methods to add? Transform trajectory, measure in different frame, add angles for motor trajectory? (That should probably be seperate thing)

};
