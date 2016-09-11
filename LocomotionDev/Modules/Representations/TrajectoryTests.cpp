/**
 * @file    Trajectory
 * @brief   Contains tests for Trajectory class.
 * @author  Austin Small.
 */

#include "TrajectoryTests.h"

std::string TrajectoryTests::runTests(void)
{
    std::string output("");
    
    /** Empty constructor. **/
    Trajectory testTrajectory; // This object will be reused for the remaining unit tests.
    
    /** Copy constructor **/
    Transform testTransformCOM;
    testTransformCOM.translate(3.0, 4.0, 5.0);
    testTrajectory.setCOMTransform(testTransformCOM);
    
    Transform testTransformSwingFoot;
    testTransformSwingFoot.translate(4.0, 5.0, 6.0);
    testTrajectory.setSwingFootTransform(testTransformSwingFoot);
    
    testTrajectory.setTime(1.2);
    
    Trajectory returnTrajectory(testTrajectory);
    
    Transform returnCOMTransform(returnTrajectory.getCOMTransform());
    
    Transform returnSwingTransform(returnTrajectory.getSwingFootTransform());
    
    if (returnCOMTransform(0,0) == 1 && returnCOMTransform(0,1) == 0 && returnCOMTransform(0,2) == 0 && returnCOMTransform(0,3) == 3 &&
        returnCOMTransform(1,0) == 0 && returnCOMTransform(1,1) == 1 && returnCOMTransform(1,2) == 0 && returnCOMTransform(1,3) == 4 &&
        returnCOMTransform(2,0) == 0 && returnCOMTransform(2,1) == 0 && returnCOMTransform(2,2) == 1 && returnCOMTransform(2,3) == 5 &&
        returnCOMTransform(3,0) == 0 && returnCOMTransform(3,1) == 0 && returnCOMTransform(3,2) == 0 && returnCOMTransform(3,3) == 1 &&
        
        returnSwingTransform(0,0) == 1 && returnSwingTransform(0,1) == 0 && returnSwingTransform(0,2) == 0 && returnSwingTransform(0,3) == 4 &&
        returnSwingTransform(1,0) == 0 && returnSwingTransform(1,1) == 1 && returnSwingTransform(1,2) == 0 && returnSwingTransform(1,3) == 5 &&
        returnSwingTransform(2,0) == 0 && returnSwingTransform(2,1) == 0 && returnSwingTransform(2,2) == 1 && returnSwingTransform(2,3) == 6 &&
        returnSwingTransform(3,0) == 0 && returnSwingTransform(3,1) == 0 && returnSwingTransform(3,2) == 0 && returnSwingTransform(3,3) == 1 &&
        
        fabs(returnTrajectory.getTime() - 1.2) < 0.001)
    {
        output.append("Copy constructor test passed!\n");
    }
    else
    {
        output.append("Copy constructor test FAILED!\n");
    }
    
    /** Assignment operator. **/
    testTransformCOM.clear();
    testTransformCOM.translate(3.0, 4.0, 5.0);
    testTrajectory.setCOMTransform(testTransformCOM);
    
    testTransformSwingFoot.clear();
    testTransformSwingFoot.translate(4.0, 5.0, 6.0);
    testTrajectory.setSwingFootTransform(testTransformSwingFoot);
    
    testTrajectory.setTime(1.2);
    
    returnTrajectory = testTrajectory;
    returnCOMTransform = returnTrajectory.getCOMTransform();
    returnSwingTransform = returnTrajectory.getSwingFootTransform();
    
    if (returnCOMTransform(0,0) == 1 && returnCOMTransform(0,1) == 0 && returnCOMTransform(0,2) == 0 && returnCOMTransform(0,3) == 3 &&
        returnCOMTransform(1,0) == 0 && returnCOMTransform(1,1) == 1 && returnCOMTransform(1,2) == 0 && returnCOMTransform(1,3) == 4 &&
        returnCOMTransform(2,0) == 0 && returnCOMTransform(2,1) == 0 && returnCOMTransform(2,2) == 1 && returnCOMTransform(2,3) == 5 &&
        returnCOMTransform(3,0) == 0 && returnCOMTransform(3,1) == 0 && returnCOMTransform(3,2) == 0 && returnCOMTransform(3,3) == 1 &&
        
        returnSwingTransform(0,0) == 1 && returnSwingTransform(0,1) == 0 && returnSwingTransform(0,2) == 0 && returnSwingTransform(0,3) == 4 &&
        returnSwingTransform(1,0) == 0 && returnSwingTransform(1,1) == 1 && returnSwingTransform(1,2) == 0 && returnSwingTransform(1,3) == 5 &&
        returnSwingTransform(2,0) == 0 && returnSwingTransform(2,1) == 0 && returnSwingTransform(2,2) == 1 && returnSwingTransform(2,3) == 6 &&
        returnSwingTransform(3,0) == 0 && returnSwingTransform(3,1) == 0 && returnSwingTransform(3,2) == 0 && returnSwingTransform(3,3) == 1 &&
        
        fabs(returnTrajectory.getTime() - 1.2) < 0.001)
    {
        output.append("Assignment operator test passed!\n");
    }
    else
    {
        output.append("Assignment operator test FAILED!\n");
    }

    /** clear **/
    testTransformCOM.clear();
    testTransformCOM.translate(3.0, 4.0, 5.0);
    testTrajectory.setCOMTransform(testTransformCOM);
    
    testTransformSwingFoot.clear();
    testTransformSwingFoot.translate(4.0, 5.0, 6.0);
    testTrajectory.setSwingFootTransform(testTransformSwingFoot);
    
    testTrajectory.setTime(1.2);
    
    testTrajectory.clear();
    
    returnCOMTransform = testTrajectory.getCOMTransform();
    returnSwingTransform = testTrajectory.getSwingFootTransform();
    
    if (returnCOMTransform(0,0) == 1 && returnCOMTransform(0,1) == 0 && returnCOMTransform(0,2) == 0 && returnCOMTransform(0,3) == 0 &&
        returnCOMTransform(1,0) == 0 && returnCOMTransform(1,1) == 1 && returnCOMTransform(1,2) == 0 && returnCOMTransform(1,3) == 0 &&
        returnCOMTransform(2,0) == 0 && returnCOMTransform(2,1) == 0 && returnCOMTransform(2,2) == 1 && returnCOMTransform(2,3) == 0 &&
        returnCOMTransform(3,0) == 0 && returnCOMTransform(3,1) == 0 && returnCOMTransform(3,2) == 0 && returnCOMTransform(3,3) == 1 &&
        
        returnSwingTransform(0,0) == 1 && returnSwingTransform(0,1) == 0 && returnSwingTransform(0,2) == 0 && returnSwingTransform(0,3) == 0 &&
        returnSwingTransform(1,0) == 0 && returnSwingTransform(1,1) == 1 && returnSwingTransform(1,2) == 0 && returnSwingTransform(1,3) == 0 &&
        returnSwingTransform(2,0) == 0 && returnSwingTransform(2,1) == 0 && returnSwingTransform(2,2) == 1 && returnSwingTransform(2,3) == 0 &&
        returnSwingTransform(3,0) == 0 && returnSwingTransform(3,1) == 0 && returnSwingTransform(3,2) == 0 && returnSwingTransform(3,3) == 1 &&
        
        fabs(testTrajectory.getTime() - 0.0) < 0.001)
    {
        output.append("Clear test passed!\n");
    }
    else
    {
        output.append("Clear test FAILED!\n");
    }
    
    /** setTime and getTime **/
    testTrajectory.setTime(145.0);
    
    if ((testTrajectory.getTime() - 145.0) < 0.001)
    {
        output.append("getTime and setTime test passed!\n");
    }
    else
    {
        output.append("getTime and setTime test FAILED!\n");
    }
    
    /* setCOMTransform and getCOMTransform **/
    testTrajectory.clear();
    
    Transform testTransform;
    testTransform.translate(3.0, 4.0, 5.0);
    testTrajectory.setCOMTransform(testTransform);
    
    Transform returnTransform(testTrajectory.getCOMTransform());
    
    if (returnTransform(0,0) == 1 && returnTransform(0,1) == 0 && returnTransform(0,2) == 0 && returnTransform(0,3) == 3 &&
        returnTransform(1,0) == 0 && returnTransform(1,1) == 1 && returnTransform(1,2) == 0 && returnTransform(1,3) == 4 &&
        returnTransform(2,0) == 0 && returnTransform(2,1) == 0 && returnTransform(2,2) == 1 && returnTransform(2,3) == 5 &&
        returnTransform(3,0) == 0 && returnTransform(3,1) == 0 && returnTransform(3,2) == 0 && returnTransform(3,3) == 1)
    {
        output.append("setCOMTransform and getCOMTransform test passed!\n");
    }
    else
    {
        output.append("setCOMTransform and getCOMTransform test FAILED!\n");
    }
    
    /** setSwingFootTransform and getSwingFootTransform **/
    testTrajectory.clear();
    
    testTransform.clear();
    testTransform.translate(5.0, 1.0, 11.0);
    testTrajectory.setSwingFootTransform(testTransform);
    
    returnTransform = testTrajectory.getSwingFootTransform();
    
    if (returnTransform(0,0) == 1 && returnTransform(0,1) == 0 && returnTransform(0,2) == 0 && returnTransform(0,3) == 5 &&
        returnTransform(1,0) == 0 && returnTransform(1,1) == 1 && returnTransform(1,2) == 0 && returnTransform(1,3) == 1 &&
        returnTransform(2,0) == 0 && returnTransform(2,1) == 0 && returnTransform(2,2) == 1 && returnTransform(2,3) == 11 &&
        returnTransform(3,0) == 0 && returnTransform(3,1) == 0 && returnTransform(3,2) == 0 && returnTransform(3,3) == 1)
    {
        output.append("setSwingFootTransform and getSwingFootTransform test passed!\n");
    }
    else
    {
        output.append("setSwingFootTransform and getSwingFootTransform test FAILED!\n");
    }
    
    return output;
}