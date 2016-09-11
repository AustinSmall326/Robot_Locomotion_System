/**
 * @file KinematicsWrapper.h
 * @brief Header file for kinematics wrapper functions
 *
 * @author Alex Baucom for the UPennalizers
 */
 
 #pragma once
 
 #include <vector>
 
 #include "../Representations/Transform.h"
 
 //Invert left and right leg (together).
 std::vector<double> InvertLegs(const Transform lT, const Transform rT);

 //Invert left leg
 std::vector<double> InvertLegL(const Transform t);
 
 //Invert right leg
 std::vector<double> InvertLegR(const Transform t);

 //Invert left arm
 std::vector<double> InvertArmL(const Transform t);
 
 //Invert right arm
 std::vector<double> InvertArmR(const Transform t);
 
 //Invert head
 std::vector<double> InvertHead(const Transform t);

 //Forward kinematics left and right leg (together).
 std::vector<Transform> ForwardLegs(const std::vector<double> lp, const std::vector<double> rp);

 //Forward kinematics left leg
 Transform ForwardLegL(const std::vector<double> p);
 
 //Forward kinematics right leg
 Transform ForwardLegR(const std::vector<double> p);
 
 //Forward kinematics left arm
 Transform ForwardArmL(const std::vector<double> p);
 
 //Forward kinematics right arn
 Transform ForwardArmR(std::vector<double> p);
 
 //Forward kinematics head
 Transform ForwardHead(const std::vector<double> p);
 
 //Find COM
 std::vector<double> calcCOM(const std::vector<double> p);
 
 
