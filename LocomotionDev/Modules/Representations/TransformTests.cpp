/**
 * @file    TransformTests.cpp
 * @brief   Contains tests for Transform class.
 * @author  Austin Small.
 */

#include "TransformTests.h"

std::string TransformTests::runTests(void)
{
    std::string output("");
    
    /** Empty constructor. **/
    Transform testTransform; // This object will be reused for the remaining unit tests.
    
    if (testTransform(0,0) == 1 && testTransform(0,1) == 0 && testTransform(0,2) == 0 && testTransform(0,3) == 0 &&
        testTransform(1,0) == 0 && testTransform(1,1) == 1 && testTransform(1,2) == 0 && testTransform(1,3) == 0 &&
        testTransform(2,0) == 0 && testTransform(2,1) == 0 && testTransform(2,2) == 1 && testTransform(2,3) == 0 &&
        testTransform(3,0) == 0 && testTransform(3,1) == 0 && testTransform(3,2) == 0 && testTransform(3,3) == 1)
    {
        output.append("Empty constructor test passed!\n");
    }
    else
    {
        output.append("Empty constructor test FAILED!\n");
    }
    
    /** Copy constructor. **/
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            testTransform(i, j, (i + 1) * (j + 1));
        }
    }
    
    Transform tempTransform(testTransform);
    
    if (tempTransform(0,0) == 1 && tempTransform(0,1) == 2 && tempTransform(0,2) == 3  && tempTransform(0,3) == 4 &&
        tempTransform(1,0) == 2 && tempTransform(1,1) == 4 && tempTransform(1,2) == 6  && tempTransform(1,3) == 8 &&
        tempTransform(2,0) == 3 && tempTransform(2,1) == 6 && tempTransform(2,2) == 9  && tempTransform(2,3) == 12 &&
        tempTransform(3,0) == 4 && tempTransform(3,1) == 8 && tempTransform(3,2) == 12 && tempTransform(3,3) == 16)
    {
        output.append("Copy constructor test passed!\n");
    }
    else
    {
        output.append("Copy constructor test FAILED!\n");
    }
    
    /** Assignment operator **/
    testTransform.clear();
    tempTransform.clear();
    
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            testTransform(i, j, (i + 1) * (j + 1));
        }
    }
    
    tempTransform = testTransform;
    
    if (tempTransform(0,0) == 1 && tempTransform(0,1) == 2 && tempTransform(0,2) == 3  && tempTransform(0,3) == 4 &&
        tempTransform(1,0) == 2 && tempTransform(1,1) == 4 && tempTransform(1,2) == 6  && tempTransform(1,3) == 8 &&
        tempTransform(2,0) == 3 && tempTransform(2,1) == 6 && tempTransform(2,2) == 9  && tempTransform(2,3) == 12 &&
        tempTransform(3,0) == 4 && tempTransform(3,1) == 8 && tempTransform(3,2) == 12 && tempTransform(3,3) == 16)
    {
        output.append("Assignment operator test passed!\n");
    }
    else
    {
        output.append("Assignment operator test FAILED!\n");
    }

    /** translate(double x, doubly y, double z) **/
    // Translate on identity matrix.
    
    testTransform.clear();
    testTransform.translate(3.0, 4.0, 5.0);
    
    if (testTransform(0,0) == 1 && testTransform(0,1) == 0 && testTransform(0,2) == 0 && testTransform(0,3) == 3 &&
        testTransform(1,0) == 0 && testTransform(1,1) == 1 && testTransform(1,2) == 0 && testTransform(1,3) == 4 &&
        testTransform(2,0) == 0 && testTransform(2,1) == 0 && testTransform(2,2) == 1 && testTransform(2,3) == 5 &&
        testTransform(3,0) == 0 && testTransform(3,1) == 0 && testTransform(3,2) == 0 && testTransform(3,3) == 1)
    {
        output.append("translate(3.0, 4.0, 5.0) test passed!\n");
    }
    else
    {
        output.append("translate(3.0, 4.0, 5.0) test FAILED!\n");
    }
    
    testTransform.translate(6.0, 4.0, 5.0);
    
    if (testTransform(0,0) == 1 && testTransform(0,1) == 0 && testTransform(0,2) == 0 && testTransform(0,3) == 9  &&
        testTransform(1,0) == 0 && testTransform(1,1) == 1 && testTransform(1,2) == 0 && testTransform(1,3) == 8  &&
        testTransform(2,0) == 0 && testTransform(2,1) == 0 && testTransform(2,2) == 1 && testTransform(2,3) == 10 &&
        testTransform(3,0) == 0 && testTransform(3,1) == 0 && testTransform(3,2) == 0 && testTransform(3,3) == 1)
    {
        output.append("translate(6.0, 4.0, 5.0) test passed!\n");
    }
    else
    {
        output.append("translate(6.0, 4.0, 5.0) test FAILED!\n");
    }
    
    /** translateX(double x) **/
    // Translate on identity matrix.
    testTransform.clear();
    
    testTransform.translateX(9.0);
    
    if (testTransform(0,0) == 1 && testTransform(0,1) == 0 && testTransform(0,2) == 0 && testTransform(0,3) == 9 &&
        testTransform(1,0) == 0 && testTransform(1,1) == 1 && testTransform(1,2) == 0 && testTransform(1,3) == 0 &&
        testTransform(2,0) == 0 && testTransform(2,1) == 0 && testTransform(2,2) == 1 && testTransform(2,3) == 0 &&
        testTransform(3,0) == 0 && testTransform(3,1) == 0 && testTransform(3,2) == 0 && testTransform(3,3) == 1)
    {
        output.append("translateX(9.0) test passed!\n");
    }
    else
    {
        output.append("translateX(9.0) test FAILED!\n");
    }
    
    /** translateY(double y) **/
    // Translate on identity matrix.
    testTransform.clear();
    
    testTransform.translateY(11.0);
    
    if (testTransform(0,0) == 1 && testTransform(0,1) == 0 && testTransform(0,2) == 0 && testTransform(0,3) == 0  &&
        testTransform(1,0) == 0 && testTransform(1,1) == 1 && testTransform(1,2) == 0 && testTransform(1,3) == 11 &&
        testTransform(2,0) == 0 && testTransform(2,1) == 0 && testTransform(2,2) == 1 && testTransform(2,3) == 0  &&
        testTransform(3,0) == 0 && testTransform(3,1) == 0 && testTransform(3,2) == 0 && testTransform(3,3) == 1)
    {
        output.append("translateY(11.0) test passed!\n");
    }
    else
    {
        output.append("translateY(11.0) test FAILED!\n");
    }
    
    /** translateZ(double z) **/
    // Translate on identity matrix.
    testTransform.clear();
    
    testTransform.translateZ(100.0);
    
    if (testTransform(0,0) == 1 && testTransform(0,1) == 0 && testTransform(0,2) == 0 && testTransform(0,3) == 0   &&
        testTransform(1,0) == 0 && testTransform(1,1) == 1 && testTransform(1,2) == 0 && testTransform(1,3) == 0   &&
        testTransform(2,0) == 0 && testTransform(2,1) == 0 && testTransform(2,2) == 1 && testTransform(2,3) == 100 &&
        testTransform(3,0) == 0 && testTransform(3,1) == 0 && testTransform(3,2) == 0 && testTransform(3,3) == 1)
    {
        output.append("translateZ(100.0) test passed!\n");
    }
    else
    {
        output.append("translateZ(100.0) test FAILED!\n");
    }
    
    /** rotateX(double a) **/
    // Rotate on identity matrix.
    testTransform.clear();
    
    testTransform.rotateX(M_PI/6);
    
    if (fabs(testTransform(0,0) - 1.0) < 0.001 && fabs(testTransform(0,1) - 0.0) < 0.001      && fabs(testTransform(0,2) - 0.0) < 0.001      && fabs(testTransform(0,3) - 0.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.0) < 0.001 && fabs(testTransform(1,1) - 0.866025) < 0.001 && fabs(testTransform(1,2) - (-0.5)) < 0.001   && fabs(testTransform(1,3) - 0.0) < 0.001 &&
        fabs(testTransform(2,0) - 0.0) < 0.001 && fabs(testTransform(2,1) - 0.5) < 0.001      && fabs(testTransform(2,2) - 0.866025) < 0.001 && fabs(testTransform(2,3) - 0.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001 && fabs(testTransform(3,1) - 0.0) < 0.001      && fabs(testTransform(3,2) - 0.0) < 0.001      && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("rotateX(pi/6) test passed!\n");
    }
    else
    {
        output.append("rotateX(pi/6) test FAILED!\n");
    }
    
    /** rotateY(double a) **/
    // Rotate on identity matrix.
    testTransform.clear();
    
    testTransform.rotateY(M_PI / 3);
    
    if (fabs(testTransform(0,0) - 0.5) < 0.001         && fabs(testTransform(0,1) - 0.0) < 0.001 && fabs(testTransform(0,2) - 0.866025) < 0.001 && fabs(testTransform(0,3) - 0.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.0) < 0.001         && fabs(testTransform(1,1) - 1.0) < 0.001 && fabs(testTransform(1,2) - 0.0) < 0.001      && fabs(testTransform(1,3) - 0.0) < 0.001 &&
        fabs(testTransform(2,0) - (-0.866025)) < 0.001 && fabs(testTransform(2,1) - 0.0) < 0.001 && fabs(testTransform(2,2) - 0.5) < 0.001      && fabs(testTransform(2,3) - 0.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001         && fabs(testTransform(3,1) - 0.0) < 0.001 && fabs(testTransform(3,2) - 0.0) < 0.001      && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("rotateY(pi / 3) test passed!\n");
    }
    else
    {
        output.append("rotateY(pi / 3) test FAILED!\n");
    }
    
    /** rotateZ(double a) **/
    // Rotate on identity matrix.
    testTransform.clear();
    
    testTransform.rotateZ(M_PI / 4);
    
    if (fabs(testTransform(0,0) - 0.707107) < 0.001 && fabs(testTransform(0,1) - (-0.707107)) < 0.001 && fabs(testTransform(0,2) - 0.0) < 0.001 && fabs(testTransform(0,3) - 0.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.707107) < 0.001 && fabs(testTransform(1,1) - 0.707107) < 0.001    && fabs(testTransform(1,2) - 0.0) < 0.001 && fabs(testTransform(1,3) - 0.0) < 0.001 &&
        fabs(testTransform(2,0) - 0.0) < 0.001      && fabs(testTransform(2,1) - 0.0) < 0.001         && fabs(testTransform(2,2) - 1.0) < 0.001 && fabs(testTransform(2,3) - 0.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001      && fabs(testTransform(3,1) - 0.0) < 0.001         && fabs(testTransform(3,2) - 0.0) < 0.001 && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("rotateZ(pi / 4) test passed!\n");
    }
    else
    {
        output.append("rotateZ(pi / 4) test FAILED!\n");
    }
    
    /** Compositions of translation and rotation. **/
    // RotateX -> RotateY -> RotateZ
    testTransform.clear();
    
    testTransform.rotateX(34.0 * (M_PI / 180)).rotateY(86.0 * (M_PI / 180)).rotateZ(48.0 * (M_PI / 180));
    
    if (fabs(testTransform(0,0) - 0.046676) < 0.001    && fabs(testTransform(0,1) - (-0.242833)) < 0.001 && fabs(testTransform(0,2) - 0.968944) < 0.001 && fabs(testTransform(0,3) - 0.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.051839) < 0.001    && fabs(testTransform(1,1) - 0.969283) < 0.001    && fabs(testTransform(1,2) - 0.240421) < 0.001 && fabs(testTransform(1,3) - 0.0) < 0.001 &&
        fabs(testTransform(2,0) - (-0.997564)) < 0.001 && fabs(testTransform(2,1) - 0.039007) < 0.001    && fabs(testTransform(2,2) - 0.057831) < 0.001 && fabs(testTransform(2,3) - 0.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001         && fabs(testTransform(3,1) - 0.0) < 0.001         && fabs(testTransform(3,2) - 0.0) < 0.001      && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("RotateX -> RotateY -> RotateZ test passed!\n");
    }
    else
    {
        output.append("RotateX -> RotateY -> RotateZ test FAILED!\n");
    }
    
    // RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ
    testTransform.clear();
    
    testTransform.rotateX(34.0 * (M_PI / 180)).rotateY(86.0 * (M_PI / 180)).rotateZ(48.0 * (M_PI / 180)).translateX(34.0).translateY(14.0).translateZ(12.0);
    
    if (fabs(testTransform(0,0) - 0.046676) < 0.001    && fabs(testTransform(0,1) - (-0.242833)) < 0.001 && fabs(testTransform(0,2) - 0.968944) < 0.001 && fabs(testTransform(0,3) - 34.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.051839) < 0.001    && fabs(testTransform(1,1) - 0.969283) < 0.001    && fabs(testTransform(1,2) - 0.240421) < 0.001 && fabs(testTransform(1,3) - 14.0) < 0.001 &&
        fabs(testTransform(2,0) - (-0.997564)) < 0.001 && fabs(testTransform(2,1) - 0.039007) < 0.001    && fabs(testTransform(2,2) - 0.057831) < 0.001 && fabs(testTransform(2,3) - 12.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001         && fabs(testTransform(3,1) - 0.0) < 0.001         && fabs(testTransform(3,2) - 0.0) < 0.001      && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ test passed!\n");
    }
    else
    {
        output.append("RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ test FAILED!\n");
    }
    
    // RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ
    testTransform.clear();
    
    testTransform.rotateX(34.0 * (M_PI / 180)).rotateY(12.0 * (M_PI / 180)).rotateZ(14.0 * (M_PI / 180)).translateX(34.0).translateY(86.0).translateZ(48.0);
    
    if (fabs(testTransform(0,0) - 0.949092) < 0.001    && fabs(testTransform(0,1) - (-0.087753)) < 0.001 && fabs(testTransform(0,2) - 0.302528) < 0.001    && fabs(testTransform(0,3) - 34.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.236635) < 0.001    && fabs(testTransform(1,1) - 0.832538) < 0.001    && fabs(testTransform(1,2) - (-0.500883)) < 0.001 && fabs(testTransform(1,3) - 86.0) < 0.001 &&
        fabs(testTransform(2,0) - (-0.207912)) < 0.001 && fabs(testTransform(2,1) - 0.546973) < 0.001    && fabs(testTransform(2,2) - 0.810921) < 0.001    && fabs(testTransform(2,3) - 48.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001         && fabs(testTransform(3,1) - 0.0) < 0.001         && fabs(testTransform(3,2) - 0.0) < 0.001         && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ test passed!\n");
    }
    else
    {
        output.append("RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ test FAILED!\n");
    }
    
    // RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ
    testTransform.clear();
    
    testTransform.rotateX(12.0 * (M_PI / 180)).rotateY(14.0 * (M_PI / 180)).rotateZ(34.0 * (M_PI / 180)).translateX(34.0).translateY(86.0).translateZ(48.0);
    
    if (fabs(testTransform(0,0) - 0.804412) < 0.001    && fabs(testTransform(0,1) - (-0.505274)) < 0.001 && fabs(testTransform(0,2) - 0.312442) < 0.001    && fabs(testTransform(0,3) - 34.0) < 0.001 &&
        fabs(testTransform(1,0) - 0.542582) < 0.001    && fabs(testTransform(1,1) - 0.839048) < 0.001    && fabs(testTransform(1,2) - (-0.040042)) < 0.001 && fabs(testTransform(1,3) - 86.0) < 0.001 &&
        fabs(testTransform(2,0) - (-0.241922)) < 0.001 && fabs(testTransform(2,1) - 0.201736) < 0.001    && fabs(testTransform(2,2) - 0.949092) < 0.001    && fabs(testTransform(2,3) - 48.0) < 0.001 &&
        fabs(testTransform(3,0) - 0.0) < 0.001         && fabs(testTransform(3,1) - 0.0) < 0.001         && fabs(testTransform(3,2) - 0.0) < 0.001         && fabs(testTransform(3,3) - 1.0) < 0.001)
    {
        output.append("RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ test passed!\n");
    }
    else
    {
        output.append("RotateX -> RotateY -> RotateZ -> TranslateX -> TranslateY -> TranslateZ test FAILED!\n");
    }
    
    /** operator(int i, int j, double val) **/
    testTransform.clear();
    
    testTransform(0,0,1.0); testTransform(0,1,0.0); testTransform(0,2,1.0); testTransform(0,3,3.0);
    testTransform(1,0,2.0); testTransform(1,1,1.0); testTransform(1,2,3.0); testTransform(1,3,9.0);
    testTransform(2,0,4.0); testTransform(2,1,0.0); testTransform(2,2,0.5); testTransform(2,3,4.0);
    testTransform(3,0,0.0); testTransform(3,1,0.0); testTransform(3,2,0.0); testTransform(3,3,1.0);
    
    if ((int) testTransform(0,0) == 1 && (int) testTransform(0,1) == 0 && (int) testTransform(0,2) == 1 && (int) testTransform(0,3) == 3 &&
        (int) testTransform(1,0) == 2 && (int) testTransform(1,1) == 1 && (int) testTransform(1,2) == 3 && (int) testTransform(1,3) == 9 &&
        (int) testTransform(2,0) == 4 && (int) testTransform(2,1) == 0 && (int) testTransform(2,2) == 0 && (int) testTransform(2,3) == 4 &&
        (int) testTransform(3,0) == 0 && (int) testTransform(3,1) == 0 && (int) testTransform(3,2) == 0 && (int) testTransform(3,3) == 1)
    {
        output.append("operator(i, j, val) test passed!\n");
    }
    else
    {
        output.append("operator(i, j, val) test FAILED!\n");
    }
    
    /** operator (int i, int j) - returns reference **/
    testTransform.clear();
    
    testTransform(0,0) = 2.0; testTransform(0,1) = 0.0; testTransform(0,2) = 3.0; testTransform(0,3) = 0.0;
    testTransform(1,0) = 1.0; testTransform(1,1) = 2.0; testTransform(1,2) = 3.0; testTransform(1,3) = 9.0;
    testTransform(2,0) = 0.5; testTransform(2,1) = 0.3; testTransform(2,2) = 4.0; testTransform(2,3) = 11.0;
    testTransform(3,0) = 0.0; testTransform(3,1) = 0.1; testTransform(3,2) = 1.0; testTransform(3,3) = 0.0;
    
    if ((int) testTransform(0,0) == 2 && (int) testTransform(0,1) == 0 && (int) testTransform(0,2) == 3 && (int) testTransform(0,3) == 0  &&
        (int) testTransform(1,0) == 1 && (int) testTransform(1,1) == 2 && (int) testTransform(1,2) == 3 && (int) testTransform(1,3) == 9  &&
        (int) testTransform(2,0) == 0 && (int) testTransform(2,1) == 0 && (int) testTransform(2,2) == 4 && (int) testTransform(2,3) == 11 &&
        (int) testTransform(3,0) == 0 && (int) testTransform(3,1) == 0 && (int) testTransform(3,2) == 1 && (int) testTransform(3,3) == 0)
    {
        output.append("operator (int i, int j) - returns reference test passed!\n");
    }
    else
    {
        output.append("operator (int i, int j) - returns reference test FAILED!\n");
    }
    
    /** operator * **/
    testTransform.clear();
    Transform testTransformTwo;
    
    testTransform(0,0,1.0); testTransform(0,1,0.0); testTransform(0,2,1.0); testTransform(0,3,3.0);
    testTransform(1,0,2.0); testTransform(1,1,1.0); testTransform(1,2,3.0); testTransform(1,3,9.0);
    testTransform(2,0,4.0); testTransform(2,1,0.0); testTransform(2,2,0.5); testTransform(2,3,4.0);
    testTransform(3,0,0.0); testTransform(3,1,0.0); testTransform(3,2,0.0); testTransform(3,3,1.0);
    
    testTransformTwo(0,0,3.0); testTransformTwo(0,1,3.0); testTransformTwo(0,2,0.0); testTransformTwo(0,3,1.0);
    testTransformTwo(1,0,1.0); testTransformTwo(1,1,5.0); testTransformTwo(1,2,3.0); testTransformTwo(1,3,11.0);
    testTransformTwo(2,0,4.0); testTransformTwo(2,1,0.0); testTransformTwo(2,2,0.5); testTransformTwo(2,3,3.0);
    testTransformTwo(3,0,0.0); testTransformTwo(3,1,0.0); testTransformTwo(3,2,0.0); testTransformTwo(3,3,1.0);
    
    Transform resultTransform(testTransform * testTransformTwo);
    
    if (fabs(resultTransform(0,0) - 7.0)  < 0.001  && fabs(resultTransform(0,1) - 3.0) < 0.001  && fabs(resultTransform(0,2) - 0.5) < 0.001  && fabs(resultTransform(0,3) - 7.0) < 0.001  &&
        fabs(resultTransform(1,0) - 19.0) < 0.001 && fabs(resultTransform(1,1) - 11.0) < 0.001 && fabs(resultTransform(1,2) - 4.5) < 0.001  && fabs(resultTransform(1,3) - 31.0) < 0.001 &&
        fabs(resultTransform(2,0) - 14.0) < 0.001 && fabs(resultTransform(2,1) - 12.0) < 0.001 && fabs(resultTransform(2,2) - 0.25) < 0.001 && fabs(resultTransform(2,3) - 9.5) < 0.001  &&
        fabs(resultTransform(3,0) - 0.0)  < 0.001  && fabs(resultTransform(3,1) - 0.0) < 0.001  && fabs(resultTransform(3,2) - 0.0) < 0.001  && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("operator * test passed!\n");
    }
    else
    {
        output.append("operator * test FAILED!\n");
    }
    
    /** inv (Transform t1) **/
    // rotateX.
    testTransform.clear();
    testTransform.rotateX(30 * (M_PI / 180));
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 1.0) < 0.001 && fabs(resultTransform(0,1) - 0.0) < 0.001      && fabs(resultTransform(0,2) - 0.0) < 0.001      && fabs(resultTransform(0,3) - 0.0) < 0.001 &&
        fabs(resultTransform(1,0) - 0.0) < 0.001 && fabs(resultTransform(1,1) - 0.866025) < 0.001 && fabs(resultTransform(1,2) - 0.5) < 0.001      && fabs(resultTransform(1,3) - 0.0) < 0.001 &&
        fabs(resultTransform(2,0) - 0.0) < 0.001 && fabs(resultTransform(2,1) - (-0.5)) < 0.001   && fabs(resultTransform(2,2) - 0.866025) < 0.001 && fabs(resultTransform(2,3) - 0.0) < 0.001 &&
        fabs(resultTransform(3,0) - 0.0) < 0.001 && fabs(resultTransform(3,1) - 0.0) < 0.001      && fabs(resultTransform(3,2) - 0.0) < 0.001      && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) rotateX test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) rotateX test FAILED!\n");
    }
    
    // rotateY.
    testTransform.clear();
    testTransform.rotateY(30 * (M_PI / 180));
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 0.866025) < 0.001 && fabs(resultTransform(0,1) - 0.0) < 0.001 && fabs(resultTransform(0,2) - (-0.5)) < 0.001   && fabs(resultTransform(0,3) - 0.0) < 0.001 &&
        fabs(resultTransform(1,0) - 0.0) < 0.001      && fabs(resultTransform(1,1) - 1.0) < 0.001 && fabs(resultTransform(1,2) - 0.0) < 0.001      && fabs(resultTransform(1,3) - 0.0) < 0.001 &&
        fabs(resultTransform(2,0) - 0.5) < 0.001      && fabs(resultTransform(2,1) - 0.0) < 0.001 && fabs(resultTransform(2,2) - 0.866025) < 0.001 && fabs(resultTransform(2,3) - 0.0) < 0.001 &&
        fabs(resultTransform(3,0) - 0.0) < 0.001      && fabs(resultTransform(3,1) - 0.0) < 0.001 && fabs(resultTransform(3,2) - 0.0) < 0.001      && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) rotateY test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) rotateY test FAILED!\n");
    }
    
    // rotateZ.
    testTransform.clear();
    testTransform.rotateZ(30 * (M_PI / 180));
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 0.866025) < 0.001 && fabs(resultTransform(0,1) - 0.5) < 0.001      && fabs(resultTransform(0,2) - 0.0) < 0.001 && fabs(resultTransform(0,3) - 0.0) < 0.001 &&
        fabs(resultTransform(1,0) - (-0.5))   < 0.001   && fabs(resultTransform(1,1) - 0.866025) < 0.001 && fabs(resultTransform(1,2) - 0.0) < 0.001 && fabs(resultTransform(1,3) - 0.0) < 0.001 &&
        fabs(resultTransform(2,0) - 0.0) < 0.001      && fabs(resultTransform(2,1) - 0.0) < 0.001      && fabs(resultTransform(2,2) - 1.0) < 0.001 && fabs(resultTransform(2,3) - 0.0) < 0.001 &&
        fabs(resultTransform(3,0) - 0.0) < 0.001      && fabs(resultTransform(3,1) - 0.0) < 0.001      && fabs(resultTransform(3,2) - 0.0) < 0.001 && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) rotateZ test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) rotateZ test FAILED!\n");
    }
    
    // Composition of rotations.
    testTransform.clear();
    testTransform.rotateX(78.0 * (M_PI / 180)).rotateY(15.0 * (M_PI / 180)).rotateZ(20.0 * (M_PI / 180));
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 0.907673) < 0.001 && fabs(resultTransform(0,1) - 0.330366) < 0.001    && fabs(resultTransform(0,2) - (-0.258819)) < 0.001  && fabs(resultTransform(0,3) - 0.0) < 0.001 &&
        fabs(resultTransform(1,0) - 0.166786) < 0.001 && fabs(resultTransform(1,1) - 0.28196) < 0.001     && fabs(resultTransform(1,2) - 0.944818) < 0.001     && fabs(resultTransform(1,3) - 0.0) < 0.001 &&
        fabs(resultTransform(2,0) - 0.385112) < 0.001 && fabs(resultTransform(2,1) - (-0.900753)) < 0.001 && fabs(resultTransform(2,2) - 0.200827) < 0.001     && fabs(resultTransform(2,3) - 0.0) < 0.001 &&
        fabs(resultTransform(3,0) - 0.0) < 0.001      && fabs(resultTransform(3,1) - 0.0) < 0.001         && fabs(resultTransform(3,2) - 0.0) < 0.001          && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) composition of rotations test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) composition of rotations test FAILED!\n");
    }
    
    // translateX.
    testTransform.clear();
    testTransform.translateX(15.0);
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 1.0) < 0.001 && fabs(resultTransform(0,1) - 0.0) < 0.001 && fabs(resultTransform(0,2) - 0.0) < 0.001 && fabs(resultTransform(0,3) - (-15.0)) < 0.001 &&
        fabs(resultTransform(1,0) - 0.0) < 0.001 && fabs(resultTransform(1,1) - 1.0) < 0.001 && fabs(resultTransform(1,2) - 0.0) < 0.001 && fabs(resultTransform(1,3) - 0.0) < 0.001     &&
        fabs(resultTransform(2,0) - 0.0) < 0.001 && fabs(resultTransform(2,1) - 0.0) < 0.001 && fabs(resultTransform(2,2) - 1.0) < 0.001 && fabs(resultTransform(2,3) - 0.0) < 0.001     &&
        fabs(resultTransform(3,0) - 0.0) < 0.001 && fabs(resultTransform(3,1) - 0.0) < 0.001 && fabs(resultTransform(3,2) - 0.0) < 0.001 && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) translateX test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) translationX test FAILED!\n");
    }
    
    // translateY.
    testTransform.clear();
    testTransform.translateY(15.0);
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 1.0) < 0.001 && fabs(resultTransform(0,1) - 0.0) < 0.001 && fabs(resultTransform(0,2) - 0.0) < 0.001 && fabs(resultTransform(0,3) - 0.0) < 0.001     &&
        fabs(resultTransform(1,0) - 0.0) < 0.001 && fabs(resultTransform(1,1) - 1.0) < 0.001 && fabs(resultTransform(1,2) - 0.0) < 0.001 && fabs(resultTransform(1,3) - (-15.0)) < 0.001 &&
        fabs(resultTransform(2,0) - 0.0) < 0.001 && fabs(resultTransform(2,1) - 0.0) < 0.001 && fabs(resultTransform(2,2) - 1.0) < 0.001 && fabs(resultTransform(2,3) - 0.0) < 0.001     &&
        fabs(resultTransform(3,0) - 0.0) < 0.001 && fabs(resultTransform(3,1) - 0.0) < 0.001 && fabs(resultTransform(3,2) - 0.0) < 0.001 && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) translateY test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) translationY test FAILED!\n");
    }
    
    // translateZ.
    testTransform.clear();
    testTransform.translateZ(15.0);
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 1.0) < 0.001 && fabs(resultTransform(0,1) - 0.0) < 0.001 && fabs(resultTransform(0,2) - 0.0) < 0.001 && fabs(resultTransform(0,3) - 0.0) < 0.001     &&
        fabs(resultTransform(1,0) - 0.0) < 0.001 && fabs(resultTransform(1,1) - 1.0) < 0.001 && fabs(resultTransform(1,2) - 0.0) < 0.001 && fabs(resultTransform(1,3) - 0.0) < 0.001     &&
        fabs(resultTransform(2,0) - 0.0) < 0.001 && fabs(resultTransform(2,1) - 0.0) < 0.001 && fabs(resultTransform(2,2) - 1.0) < 0.001 && fabs(resultTransform(2,3) - (-15.0)) < 0.001 &&
        fabs(resultTransform(3,0) - 0.0) < 0.001 && fabs(resultTransform(3,1) - 0.0) < 0.001 && fabs(resultTransform(3,2) - 0.0) < 0.001 && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) translateZ test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) translationZ test FAILED!\n");
    }
    
    // Composition of translations.
    testTransform.clear();
    testTransform.translateX(34.0).translateY(-65.4).translateZ(1.3);
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 1.0) < 0.001 && fabs(resultTransform(0,1) - 0.0) < 0.001 && fabs(resultTransform(0,2) - 0.0) < 0.001 && fabs(resultTransform(0,3) - (-34.0)) < 0.001 &&
        fabs(resultTransform(1,0) - 0.0) < 0.001 && fabs(resultTransform(1,1) - 1.0) < 0.001 && fabs(resultTransform(1,2) - 0.0) < 0.001 && fabs(resultTransform(1,3) - 65.4) < 0.001    &&
        fabs(resultTransform(2,0) - 0.0) < 0.001 && fabs(resultTransform(2,1) - 0.0) < 0.001 && fabs(resultTransform(2,2) - 1.0) < 0.001 && fabs(resultTransform(2,3) - (-1.3)) < 0.001  &&
        fabs(resultTransform(3,0) - 0.0) < 0.001 && fabs(resultTransform(3,1) - 0.0) < 0.001 && fabs(resultTransform(3,2) - 0.0) < 0.001 && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) composition of translations test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) composition of translations test FAILED!\n");
    }
    
    // Composition of rotations and translations.
    testTransform.clear();
    testTransform.rotateX(78.0 * (M_PI / 180)).rotateY(15.0 * (M_PI / 180)).rotateZ(20.0 * (M_PI / 180)).translateX(34.0).translateY(-65.4).translateZ(1.3);
    
    resultTransform = inv(testTransform);
    
    if (fabs(resultTransform(0,0) - 0.907673) < 0.001 && fabs(resultTransform(0,1) - 0.330366) < 0.001    && fabs(resultTransform(0,2) - (-0.258819)) < 0.001 && fabs(resultTransform(0,3) - (-8.91849)) < 0.001 &&
        fabs(resultTransform(1,0) - 0.166786) < 0.001 && fabs(resultTransform(1,1) - 0.28196) < 0.001     && fabs(resultTransform(1,2) - 0.944818) < 0.001    && fabs(resultTransform(1,3) - 11.5412) < 0.001    &&
        fabs(resultTransform(2,0) - 0.385112) < 0.001 && fabs(resultTransform(2,1) - (-0.900753)) < 0.001 && fabs(resultTransform(2,2) - 0.200827) < 0.001    && fabs(resultTransform(2,3) - (-72.2642)) < 0.001 &&
        fabs(resultTransform(3,0) - 0.0) < 0.001      && fabs(resultTransform(3,1) - 0.0) < 0.001         && fabs(resultTransform(3,2) - 0.0) < 0.001         && fabs(resultTransform(3,3) - 1.0) < 0.001)
    {
        output.append("inv (Transform t1) composition of rotations and translations test passed!\n");
    }
    else
    {
        output.append("inv (Transform t1) composition of rotations and translations test FAILED!\n");
    }
    
    /** position6D(Transform t1) **/
    // rotateX.
    testTransform.clear();
    
    testTransform.rotateX(2.0 * (M_PI / 180));
    
    std::vector<double> p (position6D(testTransform));
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 0.0) < 0.001 && fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - (2.0 * (M_PI / 180))) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("position6D(Transform t1) rotateX test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) rotateX test FAILED!\n");
    }
    
    // rotateY.
    testTransform.clear();
    
    testTransform.rotateY(50.0 * (M_PI / 180));
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001  && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001  && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - (50.0 * (M_PI / 180))) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("position6D(Transform t1) rotateY test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) rotateY test FAILED!\n");
    }
    
    // rotateZ.
    testTransform.clear();
    
    testTransform.rotateZ(89.0 * (M_PI / 180));
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - (89.0 * (M_PI / 180))) < 0.001)
    {
        output.append("position6D(Transform t1) rotateZ test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) rotateZ test FAILED!\n");
    }
    
    // Composition of rotations.
    testTransform.clear();
    
    testTransform.rotateX(0.5 * (M_PI / 180)).rotateY(0.0 * (M_PI / 180)).rotateZ(89.0 * (M_PI / 180));
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - (0.5 * (M_PI / 180))) < 0.001 &&
        fabs(p[4] - (0.0 * (M_PI / 180))) < 0.001 && fabs(p[5] - (89.0 * (M_PI / 180))) < 0.001)
    {
        output.append("position6D(Transform t1) composition of rotations test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) composition of rotations test FAILED!\n");
    }
    
    // translateX.
    testTransform.clear();
    
    testTransform.translateX(2.0);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 2.0) < 0.001 && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("position6D(Transform t1) translateX test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) translateX test FAILED!\n");
    }
    
    // translateY.
    testTransform.clear();
    
    testTransform.translateY(90.0);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 90.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - 0.0) < 0.001  &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("position6D(Transform t1) translateY test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) translateY test FAILED!\n");
    }
    
    // translateZ.
    testTransform.clear();
    
    testTransform.translateZ(-14.0);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001     && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - (-14.0)) < 0.001 && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001     && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("position6D(Transform t1) translateZ test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) translateZ test FAILED!\n");
    }
    
    // Composition of translations.
    testTransform.clear();
    
    testTransform.translateX(-14.0).translateY(3.0).translateZ(8.0);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - (-14.0)) < 0.001 && fabs(p[1] - 3.0) < 0.001 &&
        fabs(p[2] - 8.0) < 0.001     && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001     && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("position6D(Transform t1) composition of translations test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) composition of translations test FAILED!\n");
    }
    
    // Composition of rotations and translations.
    testTransform.clear();
    
    testTransform.rotateX(0.0 * (M_PI / 180)).rotateY(0.0 * (M_PI / 180)).rotateZ(10.0 * (M_PI / 180)).translateX(-14.0).translateY(3.0).translateZ(8.0);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - (-14.0)) < 0.001 && fabs(p[1] - 3.0) < 0.001 &&
        fabs(p[2] - 8.0) < 0.001     && fabs(p[3] - (0.0 * (M_PI / 180))) < 0.001 &&
        fabs(p[4] - (0.0 * (M_PI / 180))) < 0.001     && fabs(p[5] - (10.0 * (M_PI / 180))) < 0.001)
    {
        output.append("position6D(Transform t1) composition of rotations and translations test passed!\n");
    }
    else
    {
        output.append("position6D(Transform t1) composition of rotations and translations test FAILED!\n");
    }
    
    /** transform6D(std::vector<double> p) **/
    // rotateX.
    testTransform.clear();
    
    std::vector<double> testVector(6);
    testVector[0] = 0.0;
    testVector[1] = 0.0;
    testVector[2] = 0.0;
    testVector[3] = 34.0 * (M_PI / 180);
    testVector[4] = 0.0;
    testVector[5] = 0.0;
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - (34.0 * (M_PI / 180))) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) rotateX test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) rotateX test FAILED!\n");
    }
    
    // rotateY.
    testTransform.clear();
    
    testVector[0] = 0.0;
    testVector[1] = 0.0;
    testVector[2] = 0.0;
    testVector[3] = 0.0;
    testVector[4] = 25.0 * (M_PI / 180);
    testVector[5] = 0.0;
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001  && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001  && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - (25.0 * (M_PI / 180))) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) rotateY test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) rotateY test FAILED!\n");
    }
    
    // rotateZ.
    testTransform.clear();
    
    testVector[0] = 0.0;
    testVector[1] = 0.0;
    testVector[2] = 0.0;
    testVector[3] = 0.0;
    testVector[4] = 0.0;
    testVector[5] = 4.0 * (M_PI / 180);
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - (4.0 * (M_PI / 180))) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) rotateZ test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) rotateZ test FAILED!\n");
    }
    
    // Composition of rotations.
    testTransform.clear();
    
    testVector[0] = 0.0;
    testVector[1] = 0.0;
    testVector[2] = 0.0;
    testVector[3] = 13.0 * (M_PI / 180);
    testVector[4] = 25.0 * (M_PI / 180);
    testVector[5] = 90.0 * (M_PI / 180);
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001  && fabs(p[1] - 0.0) < 0.001  &&
        fabs(p[2] - 0.0) < 0.001  && fabs(p[3] - (13.0 * (M_PI / 180))) < 0.001 &&
        fabs(p[4] - (25.0 * (M_PI / 180))) < 0.001 && fabs(p[5] - (90.0 * (M_PI / 180))) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) composition of rotations test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) composition of rotations test FAILED!\n");
    }
    
    // translateX.
    testTransform.clear();
    
    testVector[0] = 1900.0;
    testVector[1] = 0.0;
    testVector[2] = 0.0;
    testVector[3] = 0.0;
    testVector[4] = 0.0;
    testVector[5] = 0.0;
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 1900.0) < 0.001  && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001     && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001     && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) translateX test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) translateX test FAILED!\n");
    }
    
    // translateY.
    testTransform.clear();
    
    testVector[0] = 0.0;
    testVector[1] = 500.0;
    testVector[2] = 0.0;
    testVector[3] = 0.0;
    testVector[4] = 0.0;
    testVector[5] = 0.0;
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001 && fabs(p[1] - 500.0) < 0.001 &&
        fabs(p[2] - 0.0) < 0.001 && fabs(p[3] - 0.0) < 0.001   &&
        fabs(p[4] - 0.0) < 0.001 && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) translateY test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) translateY test FAILED!\n");
    }
    
    // translateZ.
    testTransform.clear();
    
    testVector[0] = 0.0;
    testVector[1] = 0.0;
    testVector[2] = 62.1;
    testVector[3] = 0.0;
    testVector[4] = 0.0;
    testVector[5] = 0.0;
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 0.0) < 0.001  && fabs(p[1] - 0.0) < 0.001 &&
        fabs(p[2] - 62.1) < 0.001 && fabs(p[3] - 0.0) < 0.001 &&
        fabs(p[4] - 0.0) < 0.001  && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) translateZ test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) translateZ test FAILED!\n");
    }
    
    // Composition of translations.
    testTransform.clear();
    
    testVector[0] = 1900.0;
    testVector[1] = 30.0;
    testVector[2] = 50.0;
    testVector[3] = 0.0;
    testVector[4] = 0.0;
    testVector[5] = 0.0;
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    if (fabs(p[0] - 1900.0) < 0.001 && fabs(p[1] - 30.0) < 0.001 &&
        fabs(p[2] - 50.0) < 0.001   && fabs(p[3] - 0.0) < 0.001  &&
        fabs(p[4] - 0.0) < 0.001    && fabs(p[5] - 0.0) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) composition of translations test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) composition of translations test FAILED!\n");
    }
    
    // Composition of rotations and translations.
    testTransform.clear();
    
    testVector[0] = 1900.0;
    testVector[1] = 30.0;
    testVector[2] = 50.0;
    testVector[3] = 33.0 * (M_PI / 180);
    testVector[4] = 0.0 * (M_PI / 180);
    testVector[5] = 90.0 * (M_PI / 180);
    
    testTransform = transform6D(testVector);
    
    p = position6D(testTransform);
    
    /*std::cout << p[0];
     std::cout << p[1];
     std::cout << p[2];
     std::cout << p[3];
     std::cout << p[4];
     std::cout << p[5];*/
    
    if (fabs(p[0] - 1900.0) < 0.001 && fabs(p[1] - 30.0) < 0.001 &&
        fabs(p[2] - 50.0) < 0.001   && fabs(p[3] - (33.0 * (M_PI / 180))) < 0.001 &&
        fabs(p[4] - (0.0 * (M_PI / 180))) < 0.001    && fabs(p[5] - (90.0 * (M_PI / 180))) < 0.001)
    {
        output.append("transform6D(std::vector<double> p) composition of rotations and translations test passed!\n");
    }
    else
    {
        output.append("transform6D(std::vector<double> p) composition of rotations and translations test FAILED!\n");
    }
    
    // Rotation and translation seq.
    /*
    std::cout<<"\n\n";
    testTransform.clear();
    testTransform.rotateX(90).translate(3.0, 4.0, 5.0);
    
    if ((testTransform(0,0) - 1.0) < 0.001 && (testTransform(0,1) - 0.0) < 0.001 && (testTransform(0,2) - 0.0) < 0.001   && (testTransform(0,3) - 3.0) < 0.001    &&
        (testTransform(1,0) - 0.0) < 0.001 && (testTransform(1,1) - 0.0) < 0.001 && (testTransform(1,2) - (-1.0) < 0.001 && (testTransform(1,3) - (-5.0)) < 0.001 &&
        (testTransform(2,0) - 0.0) < 0.001 && (testTransform(2,1) - 1.0) < 0.001 && (testTransform(2,2) - 0.0) < 0.001   && (testTransform(2,3) - 4.0) < 0.001    &&
        (testTransform(3,0) - 0.0) < 0.001 && (testTransform(3,1) - 0.0) < 0.001 && (testTransform(3,2) - 0.0) < 0.001   && (testTransform(3,3) - 1.0) < 0.001))
    {
        output.append("Rotation and translation seq test passed!\n");
    }
    else
    {
        output.append("Rotation and translation seq  test FAILED!\n");
        //testTransform.print();
    }
    */
    
    // Create and decode 6d vector.
    testTransform.clear();
    
    std::vector<double> testVector2(6);
    testVector2[0] = 3;
    testVector2[1] = 10;
    testVector2[2] = -2;
    testVector2[3] = 80 * (M_PI / 180);
    testVector2[4] = 5 * (M_PI / 180);
    testVector2[5] = 4 * (M_PI / 180);
    
    testTransform = transform6D(testVector2);
    
    std::vector<double> testVector3 (position6D(testTransform));
    
    if (std::abs(testVector3[0] - testVector2[0]) < 0.001 &&
        std::abs(testVector3[1] - testVector2[1]) < 0.001 &&
        std::abs(testVector3[2] - testVector2[2]) < 0.001 &&
        std::abs(testVector3[3] - testVector2[3]) < 0.001 &&
        std::abs(testVector3[4] - testVector2[4]) < 0.001 &&
        std::abs(testVector3[5] - testVector2[5]) < 0.001)
    {
        output.append("Create and decode 6d vector test passed!\n");
    }
    else
    {
        output.append("Create and decode 6d vector test FAILED!\n");
        
        //testTransform.print();
        //for (int i=0; i<6; i++){
        //    std::cout<< testVector[i]<<" "<<testVector2[i]<<"\n";
        //}
    }
    
    return output;
}
