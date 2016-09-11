/**
 * @file    PointTests
 * @brief   Contains tests for Point class.
 * @author  Austin Small.
 */

#include "PointTests.h"

std::string PointTests::runTests(void)
{
    std::string output("");
    
    /** Empty constructor and getFoot and getStepType. **/
    Point::Foot foot = Point::left;
    Point::StepType stepType = Point::begin;
    
    Point testPoint(foot, stepType); // This object will be reused for the remaining unit tests.
    
    if (testPoint.getFoot() == Point::left && testPoint.getStepType() == Point::begin)
    {
        output.append("Empty constructor and getFoot and getStepType test passed!\n");
    }
    else {
        output.append("Empty constructor and getFoot and getStepType test FAILED!\n");
    }
    
    /** setFootPos and getFootPos **/
    Transform footPos;
    footPos.translate(6.0, 3.0, 4.0);
    testPoint.setFootPos(footPos);
    
    Transform returnFootPos = testPoint.getFootPos();
    
    if (returnFootPos(0,0) == 1 && returnFootPos(0,1) == 0 && returnFootPos(0,2) == 0 && returnFootPos(0,3) == 6 &&
        returnFootPos(1,0) == 0 && returnFootPos(1,1) == 1 && returnFootPos(1,2) == 0 && returnFootPos(1,3) == 3 &&
        returnFootPos(2,0) == 0 && returnFootPos(2,1) == 0 && returnFootPos(2,2) == 1 && returnFootPos(2,3) == 4 &&
        returnFootPos(3,0) == 0 && returnFootPos(3,1) == 0 && returnFootPos(3,2) == 0 && returnFootPos(3,3) == 1)
    {
        output.append("setFootPos and getFootPos test passed!\n");
    }
    else {
        output.append("setFootPos and getFootPos test FAILED!\n");
    }
    
    /** addTrajectory and getTrajectory and getNumTrajectories **/
    // Add first test trajectory.
    Trajectory trajectory;
    trajectory.setTime(0.213);
    
    Transform comPos;
    comPos.translate(3.0, 2.0, 1.0);
    trajectory.setCOMTransform(comPos);
    
    Transform swingFootPos;
    swingFootPos.translate(1.0, 2.0, 3.0);
    trajectory.setSwingFootTransform(swingFootPos);
    
    testPoint.addTrajectory(trajectory);
    
    // Add second test trajectory.
    trajectory.setTime(0.992);
    
    comPos.clear();
    comPos.translate(1.0, 3.0, 3.0);
    trajectory.setCOMTransform(comPos);
    
    swingFootPos.clear();
    swingFootPos.translate(10.0, 20.0, 30.0);
    trajectory.setSwingFootTransform(swingFootPos);
    
    testPoint.addTrajectory(trajectory);
    
    // Add third test trajectory.
    trajectory.setTime(0.000);
    
    comPos.clear();
    comPos.translate(1.0, 1.0, 0.0);
    trajectory.setCOMTransform(comPos);
    
    swingFootPos.clear();
    swingFootPos.translate(5.0, 5.0, 4.0);
    trajectory.setSwingFootTransform(swingFootPos);
    
    testPoint.addTrajectory(trajectory);
    
    Trajectory returnTrajectory = testPoint.getTrajectory(0);
    Transform returnCOMTransform = returnTrajectory.getCOMTransform();
    Transform returnSwingFootTransform = returnTrajectory.getSwingFootTransform();
    
    if (returnCOMTransform(0,0) == 1 && returnCOMTransform(0,1) == 0 && returnCOMTransform(0,2) == 0 && returnCOMTransform(0,3) == 3 &&
        returnCOMTransform(1,0) == 0 && returnCOMTransform(1,1) == 1 && returnCOMTransform(1,2) == 0 && returnCOMTransform(1,3) == 2 &&
        returnCOMTransform(2,0) == 0 && returnCOMTransform(2,1) == 0 && returnCOMTransform(2,2) == 1 && returnCOMTransform(2,3) == 1 &&
        returnCOMTransform(3,0) == 0 && returnCOMTransform(3,1) == 0 && returnCOMTransform(3,2) == 0 && returnCOMTransform(3,3) == 1 &&
        
        returnSwingFootTransform(0,0) == 1 && returnSwingFootTransform(0,1) == 0 && returnSwingFootTransform(0,2) == 0 && returnSwingFootTransform(0,3) == 1 &&
        returnSwingFootTransform(1,0) == 0 && returnSwingFootTransform(1,1) == 1 && returnSwingFootTransform(1,2) == 0 && returnSwingFootTransform(1,3) == 2 &&
        returnSwingFootTransform(2,0) == 0 && returnSwingFootTransform(2,1) == 0 && returnSwingFootTransform(2,2) == 1 && returnSwingFootTransform(2,3) == 3 &&
        returnSwingFootTransform(3,0) == 0 && returnSwingFootTransform(3,1) == 0 && returnSwingFootTransform(3,2) == 0 && returnSwingFootTransform(3,3) == 1 &&
        
        testPoint.getNumTrajectories() == 3)
    {
        output.append("addTrajectory and getTrajectory and getNumTrajectories test one passed!\n");
    }
    else {
        output.append("addTrajectory and getTrajectory and getNumTrajectories test one FAILED!\n");
    }

    returnTrajectory = testPoint.getTrajectory(1);
    returnCOMTransform = returnTrajectory.getCOMTransform();
    returnSwingFootTransform = returnTrajectory.getSwingFootTransform();
    
    if (returnCOMTransform(0,0) == 1 && returnCOMTransform(0,1) == 0 && returnCOMTransform(0,2) == 0 && returnCOMTransform(0,3) == 1 &&
        returnCOMTransform(1,0) == 0 && returnCOMTransform(1,1) == 1 && returnCOMTransform(1,2) == 0 && returnCOMTransform(1,3) == 3 &&
        returnCOMTransform(2,0) == 0 && returnCOMTransform(2,1) == 0 && returnCOMTransform(2,2) == 1 && returnCOMTransform(2,3) == 3 &&
        returnCOMTransform(3,0) == 0 && returnCOMTransform(3,1) == 0 && returnCOMTransform(3,2) == 0 && returnCOMTransform(3,3) == 1 &&
        
        returnSwingFootTransform(0,0) == 1 && returnSwingFootTransform(0,1) == 0 && returnSwingFootTransform(0,2) == 0 && returnSwingFootTransform(0,3) == 10 &&
        returnSwingFootTransform(1,0) == 0 && returnSwingFootTransform(1,1) == 1 && returnSwingFootTransform(1,2) == 0 && returnSwingFootTransform(1,3) == 20 &&
        returnSwingFootTransform(2,0) == 0 && returnSwingFootTransform(2,1) == 0 && returnSwingFootTransform(2,2) == 1 && returnSwingFootTransform(2,3) == 30 &&
        returnSwingFootTransform(3,0) == 0 && returnSwingFootTransform(3,1) == 0 && returnSwingFootTransform(3,2) == 0 && returnSwingFootTransform(3,3) == 1)
    {
        output.append("addTrajectory and getTrajectory test two passed!\n");
    }
    else {
        output.append("addTrajectory and getTrajectory test two FAILED!\n");
    }
    
    returnTrajectory = testPoint.getTrajectory(2);
    returnCOMTransform = returnTrajectory.getCOMTransform();
    returnSwingFootTransform = returnTrajectory.getSwingFootTransform();
    
    if (returnCOMTransform(0,0) == 1 && returnCOMTransform(0,1) == 0 && returnCOMTransform(0,2) == 0 && returnCOMTransform(0,3) == 1 &&
        returnCOMTransform(1,0) == 0 && returnCOMTransform(1,1) == 1 && returnCOMTransform(1,2) == 0 && returnCOMTransform(1,3) == 1 &&
        returnCOMTransform(2,0) == 0 && returnCOMTransform(2,1) == 0 && returnCOMTransform(2,2) == 1 && returnCOMTransform(2,3) == 0 &&
        returnCOMTransform(3,0) == 0 && returnCOMTransform(3,1) == 0 && returnCOMTransform(3,2) == 0 && returnCOMTransform(3,3) == 1 &&
        
        returnSwingFootTransform(0,0) == 1 && returnSwingFootTransform(0,1) == 0 && returnSwingFootTransform(0,2) == 0 && returnSwingFootTransform(0,3) == 5 &&
        returnSwingFootTransform(1,0) == 0 && returnSwingFootTransform(1,1) == 1 && returnSwingFootTransform(1,2) == 0 && returnSwingFootTransform(1,3) == 5 &&
        returnSwingFootTransform(2,0) == 0 && returnSwingFootTransform(2,1) == 0 && returnSwingFootTransform(2,2) == 1 && returnSwingFootTransform(2,3) == 4 &&
        returnSwingFootTransform(3,0) == 0 && returnSwingFootTransform(3,1) == 0 && returnSwingFootTransform(3,2) == 0 && returnSwingFootTransform(3,3) == 1)
    {
        output.append("addTrajectory and getTrajectory test three passed!\n");
    }
    else {
        output.append("addTrajectory and getTrajectory test three FAILED!\n");
    }
    
    /** setTrajectoryParameters and getTrajectoryParemeters **/
    COMContainer container;
    
    container.setX0Pos(1.2);
    container.setX0Vel(2.3);
    container.setY0Pos(3.4);
    container.setY0Vel(4.5);
    container.setR(1.1);
    container.setRWeight(0.3);
    container.setTStart(0.0);
    container.setTEnd(1.0);
    
    testPoint.setTrajectoryParameters(container);
    
    COMContainer returnContainer(testPoint.getTrajectoryParameters());
    
    if (fabs(returnContainer.getX0Pos() - 1.2)  < 0.001 && fabs(returnContainer.getX0Vel() - 2.3)   < 0.001 &&
        fabs(returnContainer.getY0Pos() - 3.4)  < 0.001 && fabs(returnContainer.getY0Vel() - 4.5)   < 0.001 &&
        fabs(returnContainer.getR() - 1.1)      < 0.001 && fabs(returnContainer.getRWeight() - 0.3) < 0.001 &&
        fabs(returnContainer.getTStart() - 0.0) < 0.001 && fabs(returnContainer.getTEnd() - 1.0)    < 0.001)
    {
        output.append("setTrajectoryParameters and getTrajectoryParemeters test passed!\n");
    }
    else
    {
        output.append("setTrajectoryParameters and getTrajectoryParemeters test FAILED!\n");
    }
    
    return output;
}