/**
 * @file    COMContainerTests.cpp
 * @brief   cpp file with tests for COMContainer class.
 * @author  Austin Small.
 */

#include "COMContainerTests.h"

std::string COMContainerTests::runTests(void)
{
    std::string output("");
    
    /** Empty constructor **/
    COMContainer testContainer; // This object will be reused for the remaining unit tests.
    
    if (fabs(testContainer.getX0Pos() - 0.000)  < 0.001 && fabs(testContainer.getX0Vel() - 0.000)   < 0.001 &&
        fabs(testContainer.getY0Pos() - 0.000)  < 0.001 && fabs(testContainer.getY0Vel() - 0.000)   < 0.001 &&
        fabs(testContainer.getR() - 0.000)      < 0.001 && fabs(testContainer.getRWeight() - 0.000) < 0.001 &&
        fabs(testContainer.getTStart() - 0.000) < 0.001 && fabs(testContainer.getTEnd() - 0.000)    < 0.001)
    {
        output.append("Empty constructor test passed!\n");
    }
    else
    {
        output.append("Empty constructor test FAILED!\n");
    }
    
    /** Copy constructor **/
    COMContainer tempContainer;
    
    tempContainer.setX0Pos(1.0);
    tempContainer.setX0Vel(2.0);
    tempContainer.setY0Pos(3.0);
    tempContainer.setY0Vel(4.0);
    tempContainer.setR(5.0);
    tempContainer.setRWeight(6.0);
    tempContainer.setTStart(7.0);
    tempContainer.setTEnd(8.0);
    
    COMContainer testContainerTwo(tempContainer);
    
    if (fabs(testContainerTwo.getX0Pos() - 1.000)  < 0.001 && fabs(testContainerTwo.getX0Vel() - 2.000)   < 0.001 &&
        fabs(testContainerTwo.getY0Pos() - 3.000)  < 0.001 && fabs(testContainerTwo.getY0Vel() - 4.000)   < 0.001 &&
        fabs(testContainerTwo.getR() - 5.000)      < 0.001 && fabs(testContainerTwo.getRWeight() - 6.000) < 0.001 &&
        fabs(testContainerTwo.getTStart() - 7.000) < 0.001 && fabs(testContainerTwo.getTEnd() - 8.000)    < 0.001)
    {
        output.append("Copy constructor test passed!\n");
    }
    else
    {
        output.append("Copy constructor test FAILED!\n");
    }
    
    /** Assignment operator **/
    testContainer.clear();
    tempContainer.clear();
    
    tempContainer.setX0Pos(1.0);
    tempContainer.setX0Vel(2.0);
    tempContainer.setY0Pos(3.0);
    tempContainer.setY0Vel(4.0);
    tempContainer.setR(5.0);
    tempContainer.setRWeight(6.0);
    tempContainer.setTStart(7.0);
    tempContainer.setTEnd(8.0);
    
    testContainer = tempContainer;
    
    if (fabs(testContainerTwo.getX0Pos() - 1.000)  < 0.001 && fabs(testContainerTwo.getX0Vel() - 2.000)   < 0.001 &&
        fabs(testContainerTwo.getY0Pos() - 3.000)  < 0.001 && fabs(testContainerTwo.getY0Vel() - 4.000)   < 0.001 &&
        fabs(testContainerTwo.getR() - 5.000)      < 0.001 && fabs(testContainerTwo.getRWeight() - 6.000) < 0.001 &&
        fabs(testContainerTwo.getTStart() - 7.000) < 0.001 && fabs(testContainerTwo.getTEnd() - 8.000)    < 0.001)
    {
        output.append("Assignment operator test passed!\n");
    }
    else
    {
        output.append("Assignment operator test FAILED!\n");
    }

    /** clear **/
    testContainer.setX0Pos(1.0);
    testContainer.setX0Vel(2.0);
    testContainer.setY0Pos(3.0);
    testContainer.setY0Vel(4.0);
    testContainer.setR(5.0);
    testContainer.setRWeight(6.0);
    testContainer.setTStart(7.0);
    testContainer.setTEnd(8.0);
    
    testContainer.clear();
    
    if (fabs(testContainer.getX0Pos() - 0.000)  < 0.001 && fabs(testContainer.getX0Vel() - 0.000)   < 0.001 &&
        fabs(testContainer.getY0Pos() - 0.000)  < 0.001 && fabs(testContainer.getY0Vel() - 0.000)   < 0.001 &&
        fabs(testContainer.getR() - 0.000)      < 0.001 && fabs(testContainer.getRWeight() - 0.000) < 0.001 &&
        fabs(testContainer.getTStart() - 0.000) < 0.001 && fabs(testContainer.getTEnd() - 0.000)    < 0.001)
    {
        output.append("Clear test passed!\n");
    }
    else
    {
        output.append("Clear test FAILED!\n");
    }
    
    /** setX0Pos and getX0Pos **/
    testContainer.setX0Pos(25.22);
    
    if (fabs(testContainer.getX0Pos() - 25.22) < 0.001)
    {
        output.append("setX0Pos and getX0Pos test passed!\n");
    }
    else
    {
        output.append("setX0Pos and getX0Pos test FAILED!\n");
    }
    
    /** setX0Vel and getX0Vel **/
    testContainer.setX0Vel(134.00);
    
    if (fabs(testContainer.getX0Vel() - 134.00) < 0.001)
    {
        output.append("setX0Vel and getX0Vel test passed!\n");
    }
    else
    {
        output.append("setX0Vel and getX0Vel test FAILED!\n");
    }
    
    /** setY0Pos and getY0Pos **/
    testContainer.setY0Pos(145.2);
    
    if (fabs(testContainer.getY0Pos() - 145.2) < 0.001)
    {
        output.append("setY0Pos and getY0Pos test passed!\n");
    }
    else
    {
        output.append("setY0Pos and getY0Pos test FAILED!\n");
    }
    
    /** setY0Vel and getY0Vel **/
    testContainer.setY0Vel(12.2);
    
    if (fabs(testContainer.getY0Vel() - 12.2) < 0.001)
    {
        output.append("setY0Vel and getY0Vel test passed!\n");
    }
    else
    {
        output.append("setY0Vel and getY0Vel test FAILED!\n");
    }
    
    /** setR and getR **/
    testContainer.setR(2.2);
    
    if (fabs(testContainer.getR() - 2.2) < 0.001)
    {
        output.append("setR and getR test passed!\n");
    }
    else
    {
        output.append("setR and getR test FAILED!\n");
    }
    
    /** setRWeight and getRWeight **/
    testContainer.setRWeight(1.4);
    
    if (fabs(testContainer.getRWeight() - 1.4) < 0.001)
    {
        output.append("setRWeight and getRWeight test passed!\n");
    }
    else
    {
        output.append("setRWeight and getRWeight test FAILED!\n");
    }
    
    /** setTStart and getTStart **/
    testContainer.setTStart(100.2);
    
    if (fabs(testContainer.getTStart() - 100.2) < 0.001)
    {
        output.append("setTStart and getTStart test passed!\n");
    }
    else
    {
        output.append("setTStart and getTStart test FAILED!\n");
    }
    
    /** setTEnd and getTEnd **/
    testContainer.setTEnd(92.0);
    
    if (fabs(testContainer.getTEnd() - 92.0) < 0.001)
    {
        output.append("setTEnd and getTEnd test passed!\n");
    }
    else
    {
        output.append("setTEnd and getTEnd test FAILED!\n");
    }
    
    return output;
}