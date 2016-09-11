/** @file    Trajectory.cpp
 *  @brief   A file containing methods for a trajectory class
 *  @author  Alex Baucom
 */
 
 #include "Trajectory.h"
 
 /** @brief Manual constructor
 *  @details Initializes trajectory to given input vectors using arrays.
 *  @param numPts number of points in arrays p and t.
 *  @param pts Pointer to array of vector3 objects contatining the trajectory data
 *  @param timestamps pointer to array of timestamps coresponding to the vector data
 *  @param reference Frame obejct which all vector data is relative to
 */
 
 Trajectory::Trajectory(int numPts, Vector3<double>* pts, double* timestamps, const Frame &reference){
     
     //Just assign everything to the right places
     n=numPts;
     ref = reference;
     p = pts;
     t = timestamps;
     //0 is manually specified path
     path = 0;
 }
 
  /** @brief Auto constructor
 *  @details Initializes trajectory to given starting and ending conditions based on the given path type. The trajectory will
 *            be generated with constant time steps based on the number of points given
 *  @param numPts number of points to generate
 *  @param startPt Vector object of the starting position
 *  @param endPt Vector object of the ending position
 *  @param startTime Starting time of the trajectory
 *  @param endTime Ending time of the trajectory
 *  @param pathType 1=Linear path, more options to come if needed
 *  @param reference Frame obejct which all vector data is relative to
 */
 
 Trajectory::Trajectory(int numPts, Vector3<double> startPt, Vector3<double> endPt, double startTime, double endTime, int pathType, const Frame &reference){
     
     //Assign member data
     n = numPts;
     ref = reference;
     path = pathType;
     
     //Linear path generation
     if (path == 1) {
         
         // Allocate memory
         p = new Vector3<double> [n];
         t = new double [n];
         
         //Calculate slopes
         double t_slope;
         Vector3<double> p_slope;         
         t_slope = (endTime-startTime)/numPts;
         p_slope = (endPt - startPt)/numPts;
         
         //Loop to create each point
         p[0] = startPt;
         t[0] = startTime;
         for(int i=1; i++, i<numPts) {
             p[i] = p[i-1] + p_slope * i;
             t[i] = t[i-1] + t_slope * i;
         }         
     }
     else {
         //Need some sort of error thrown?
     }     
 }
 
 
 /** @brief Destructor
 *  @details Cleans up array pointers
 */
 Trajectory::~Trajectory() {
     delete[] p;
     delete[] t;
 }
 
 
/** @brief Get length of trajectory
 *  @return Number of points in the trajectory
 */
 int Trajectory::getLength() {
     return n;    
 }
 
/** @brief Get type of trajectory
 *  @return  0=Manually specified, 1=linear, possibly more to come
 */
 int Trajectory::getPath() {
     return path;    
 }
 
/** @brief Get a specific point from the trajectory
 *  @param n The index of the point
 *  @return The requested point
 */
Vector3<double> Trajectory::getPoint(int n) {
     return p[n];    
 }
 
/** @brief Get a specific timestamp from the trajectory
 *  @param n The index of the timestamp
 *  @return The requested timestamp
 */
double Trajectory::getTime(int n) {
     return t[n];    
 }
 