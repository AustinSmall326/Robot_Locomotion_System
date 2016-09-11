/**
 * @file    Vector3Tests.cpp
 * @brief   Contains tests for template class Vector3 (a 3d vector) of generic type V.
 * @author  Austin Small.
 */

#include "Vector3.h"
#include <iostream>

int main()
{
    
    // Empty constructor.
    Vector3<int> testVectorInt1;
    
    if (testVectorInt1.x == 0 && testVectorInt1.y == 0 && testVectorInt1.z == 0)
    {
        std::cout << "Empty constructor test passed!\n";
    }
    else {
        std::cout << "Empty constructor test FAILED!\n";
    }
    
    
    // Constructor with initialization.
    Vector3<int> testVectorInt2 (1, 2, 3);
    
    if (testVectorInt2.x == 1 && testVectorInt2.y == 2 && testVectorInt2.z == 3)
    {
        std::cout << "Constructor with initialization test passed!\n";
    }
    else {
        std::cout << "Constructor with initialization test FAILED!\n";
    }
    
    // Copy constructor.
    Vector3<int> testVectorInt3 (1, 2, 3);
    Vector3<int> testVectorInt4 (testVectorInt3);
    
    if (testVectorInt4.x == 1 && testVectorInt4.y == 2 && testVectorInt4.z == 3)
    {
        std::cout << "Copy constructor test passed!\n";
    }
    else {
        std::cout << "Copy constructor test FAILED!\n";
    }
    
    // Assignment operator.  I don't think the overloaded equality operator in
    // Vector3.h is called.
    Vector3<int> testVectorInt5 (4, 5, 6);
    Vector3<int> testVectorInt6 = testVectorInt5;
    
    if (testVectorInt6.x == 4 && testVectorInt6.y == 5 && testVectorInt6.z == 6)
    {
        std::cout << "Assignment operator test passed!\n";
    }
    else {
        std::cout << "Assignment operator test FAILED!\n";
    }
    
    // Addition of one vector to another (+=).
    Vector3<int> testVectorInt7 (1, 2, 3);
    Vector3<int> testVectorInt8 (2, 3, 4);
    testVectorInt7 += testVectorInt8;

    if (testVectorInt7.x == 3 && testVectorInt7.y == 5 && testVectorInt7.z == 7)
    {
        std::cout << "Addition of one vector to another (+=) test passed!\n";
    }
    else {
        std::cout << "Addition of one vector to another (+=) test FAILED!\n";
    }
    
    // Subtraction of one vector from another (-=).
    Vector3<int> testVectorInt9 (1, 2, 3);
    Vector3<int> testVectorInt10 (2, 3, 4);
    testVectorInt9 -= testVectorInt10;
    
    if (testVectorInt9.x == -1 && testVectorInt9.y == -1 && testVectorInt9.z == -1)
    {
        std::cout << "Subtraction of one vector from another (-=) test passed!\n";
    }
    else {
        std::cout << "Subtraction of one vector from another (-=) test FAILED!\n";
    }
    
    // Multiplication of a vector by a factor (*=).
    Vector3<int> testVectorInt11 (1, 2, 3);
    testVectorInt11 *= 5;
    
    if (testVectorInt11.x == 5 && testVectorInt11.y == 10 && testVectorInt11.z == 15)
    {
        std::cout << "Multiplication of a vector by a factor (*=) test passed!\n";
    }
    else {
        std::cout << "Multiplication of a vector by a factor (*=) test FAILED!\n";
    }

    // Division of a vector by a factor (/=).
    Vector3<int> testVectorInt12 (1, 4, 4);
    testVectorInt12 /= 2;
    
    if (testVectorInt12.x == 0 && testVectorInt12.y == 2 && testVectorInt12.z == 2)
    {
        std::cout << "Division of a vector by a factor (/=) test passed!\n";
    }
    else {
        std::cout << "Division of a vector by a factor (/=) test FAILED!\n";
    }
    
    // Addition of one vector to another (+).
    Vector3<int> testVectorInt13 (7, 6, 4);
    Vector3<int> testVectorInt14 (1, 3, 4);
    Vector3<int> testVectorInt15 = testVectorInt13 + testVectorInt14;
    
    if (testVectorInt15.x == 8 && testVectorInt15.y == 9 && testVectorInt15.z == 8)
    {
        std::cout << "Addition of one vector to another (+) test passed!\n";
    }
    else {
        std::cout << "Addition of one vector to another (+) test FAILED!\n";
    }
    
    // Subtraction of one vector from another (-).
    Vector3<int> testVectorInt16 (7, 6, 4);
    Vector3<int> testVectorInt17 (1, 3, 4);
    Vector3<int> testVectorInt18 = testVectorInt17 - testVectorInt16;
    
    if (testVectorInt18.x == -6 && testVectorInt18.y == -3 && testVectorInt18.z == 0)
    {
        std::cout << "Subtraction of one vector from another (-) test passed!\n";
    }
    else {
        std::cout << "Subtraction of one vector from another (-) test FAILED!\n";
    }
    
    // Negation of a vector.
    Vector3<int> testVectorInt19 (5, 4, 3);
    Vector3<int> testVectorInt20 = -testVectorInt19;
    
    if (testVectorInt20.x == -5 && testVectorInt20.y == -4 && testVectorInt20.z == -3)
    {
        std::cout << "Negation of a vector test passed!\n";
    }
    else {
        std::cout << "Negation of a vector test FAILED!\n";
    }
    
    // Inner product of one vector with another.
    Vector3<int> testVectorInt21 (5, 4, 3);
    Vector3<int> testVectorInt22 (6, 3, 1);
    int innerProductOutput = testVectorInt21 * testVectorInt22;
    
    if (innerProductOutput == 45)
    {
        std::cout << "Inner product of one vector with another test passed!\n";
    }
    else {
        std::cout << "Inner product of one vector with another test FAILED!\n";
    }
    
    // Multiplication of one vector by a factor (*).
    Vector3<int> testVectorInt23 (5, 4, 3);
    Vector3<int> testVectorInt24 = testVectorInt23 * 2;
    
    if (testVectorInt24.x == 10 && testVectorInt24.y == 8 && testVectorInt24.z == 6)
    {
        std::cout << "Multiplication of one vector by a factor (*) test passed!\n";
    }
    else {
        std::cout << "Multiplication of one vector by a factor (*) test FAILED!\n";
    }
    
    // Division of one vector by a factor (/).
    Vector3<int> testVectorInt25 (5, 4, 3);
    Vector3<int> testVectorInt26 = testVectorInt25 / 3;
    
    if (testVectorInt26.x == 1 && testVectorInt26.y == 1 && testVectorInt26.z == 1)
    {
        std::cout << "Division of one vector by a factor (/) test passed!\n";
    }
    else {
        std::cout << "Division of one vector by a factor (/) test FAILED!\n";
    }
    
    // Comparison of two vectors (==).
    Vector3<int> testVectorInt27 (5, 4, 3);
    Vector3<int> testVectorInt28 (5, 4, 3);
    
    if (testVectorInt27 == testVectorInt28)
    {
        std::cout << "Comparison of two equal vectors (==) test passed!\n";
    }
    else {
        std::cout << "Comparison of two equal vectors (==) test FAILED!\n";
    }
    
    Vector3<int> testVectorInt29 (5, 5, 3);
    Vector3<int> testVectorInt30 (5, 4, 3);
    
    if (!(testVectorInt29 == testVectorInt30))
    {
        std::cout << "Comparison of two non-equal vectors (==) test passed!\n";
    }
    else {
        std::cout << "Comparison of two non-equal vectors (==) test FAILED!\n";
    }

    // Comparison of two equal vectors (!=).
    Vector3<int> testVectorInt31 (5, 4, 3);
    Vector3<int> testVectorInt32 (5, 4, 3);
    
    if (!(testVectorInt31 != testVectorInt32))
    {
        std::cout << "Comparison of two equal vectors (!=) test passed!\n";
    }
    else {
        std::cout << "Comparison of two equal vectors (!=) test FAILED!\n";
    }
    
    Vector3<int> testVectorInt33 (5, 5, 3);
    Vector3<int> testVectorInt34 (5, 4, 3);
    
    if (testVectorInt33 != testVectorInt34)
    {
        std::cout << "Comparison of two non-equal vectors (!=) test passed!\n";
    }
    else {
        std::cout << "Comparison of two non-equal vectors (!=) test FAILED!\n";
    }

    // Array-like member access.
    Vector3<int> testVectorInt35 (5, 4, 3);
    
    if (testVectorInt35[0] == 5 && testVectorInt35[1] == 4 && testVectorInt35[2] == 3)
    {
        std::cout << "Array-like member access test passed!\n";
    }
    else {
        std::cout << "Array-like member access test FAILED!\n";
    }
    
    // Calculation of the length of a vector.
    Vector3<int> testVectorInt36 (5, 4, 3);

    if (testVectorInt36.abs() == 7)
    {
        std::cout << "Calculation of the length of a vector test passed!\n";
    }
    else {
        std::cout << "Calculation of the length of a vector test FAILED!\n";
    }
    
    // Calculation of the square length of a vector.
    Vector3<int> testVectorInt37 (5, 4, 3);
    
    if (testVectorInt37.squareAbs() == 50)
    {
        std::cout << "Calculation of the square length of a vector test passed!\n";
    }
    else {
        std::cout << "Calculation of the square length of a vector test FAILED!\n";
    }
    
    // Cross product of one vector with another (^).
    Vector3<int> testVectorInt38 (5, 4, 3);
    Vector3<int> testVectorInt39 (1, 2, 3);
    Vector3<int> testVectorInt40 = testVectorInt38 ^ testVectorInt39;
    
    if (testVectorInt40.x == 6 && testVectorInt40.y == -12 && testVectorInt40.z == 6)
    {
        std::cout << "Cross product of one vector with another (^) test passed!\n";
    }
    else {
        std::cout << "Cross product of one vector with another (^) test FAILED!\n";
    }

    // Cross product of one vector with another (^=).
    Vector3<int> testVectorInt41 (5, 4, 3);
    Vector3<int> testVectorInt42 (1, 2, 3);
    testVectorInt41 ^= testVectorInt42;
    
    if (testVectorInt41.x == 6 && testVectorInt41.y == -12 && testVectorInt41.z == 6)
    {
        std::cout << "Cross product of one vector with another (^=) test passed!\n";
    }
    else {
        std::cout << "Cross product of one vector with another (^=) test FAILED!\n";
    }

    // Normalize a vector to a designated length.
    Vector3<int> testVectorInt43 (1, 3, 4);
    testVectorInt43.normalize(2);
    
    if (testVectorInt43.x == 0 && testVectorInt43.y == 1 && testVectorInt43.z == 1)
    {
        std::cout << "Normalize a vector to a designated length test passed!\n";
    }
    else {
        std::cout << "Normalize a vector to a designated length test FAILED!\n";
    }
    
    // Normalize a vector to default length of 1.
    Vector3<int> testVectorInt44 (1, 3, 4);
    testVectorInt44.normalize();
    
    if (testVectorInt44.x == 0 && testVectorInt44.y == 0 && testVectorInt44.z == 0)
    {
        std::cout << "Normalize a vector to default length of 1 test passed!\n";
    }
    else {
        std::cout << "Normalize a vector to default length of 1 test FAILED!\n";
    }

}

