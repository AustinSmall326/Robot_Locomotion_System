/**
 * @file KinematicsWrapper.cpp
 * @brief Cpp file for kinematics wrapper functions
 *
 * @author Alex Baucom for the UPennalizers
 */
 
 #include "KinematicsWrapper.h"
 #include "KMat.hpp"
 #include "NAOKinematics.h"
 #include "robotConsts.h"
 #include <iostream>

/** @brief   Perform inverse kinematics of left and right legs (together).
 *  @param   lT     Transform for left leg.
 *  @param   rT     Transform for right leg.
 *  @return A vector with angle values in the order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll,
 *                                                                RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
 */
std::vector<double> InvertLegs(const Transform lT, const Transform rT)
{
    // Perform inverse kinematics.
    std::vector<double> anglesLLeg(InvertLegL(lT));
    std::vector<double> anglesRLeg(InvertLegR(rT));
    
    // RHipYawPitch doesn't exist, due to joint coupling.
    anglesLLeg[0] = 2 * anglesLLeg[0];

    std::vector<double> output(anglesLLeg);
    
    for (int i = 1; i < 6; i++)
    {
        output.push_back(anglesRLeg[i]);
    }
    
    return output;
}

/** @brief   Perform inverse kinematics of left leg
 *  @details Shifts some data types to call the kinematics library functions
 *  @param t A transform specifying the location and orientation of the foot with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 *  @return A vector with angle values in the order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
 */
 std::vector<double> InvertLegL(const Transform t) {
    
     //Get 6DOF from transform
     std::vector<double> p(6);
     p = position6D(t);
     
     //Create kinematics objects and move data into kmatTable
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
	 NAOKinematics::KMatTransf::makeTransformation(T, p[0], p[1], p[2], p[3], p[4], p[5]);
     
     //Call inverse function with new transformation table
     std::vector<std::vector<float> > result;
     result=nkin.inverseLeftLeg(T);
          
     //Repackage result
     std::vector<double> ans(6);
     
     if(!result.empty()){
		for(unsigned j=0; j<result[0].size(); j++){
			ans[j] = static_cast<double>(result[0][j]);
		}
     }
     
     return ans;    
 }
 
 /** @brief   Perform inverse kinematics of right leg
  *  @details Shifts some data types to call the kinematics library functions
  *  @param t A transform specifying the location and orientation of the foot with respect to the torso frame
  *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a>
  *  @return A vector with angle values in the order: RHipYawPitch, RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
  */
 std::vector<double> InvertLegR(const Transform t) {
    
     //Get 6DOF from transform
     std::vector<double> p(6);
     p = position6D(t);
     
     //Create kinematics objects and move data into kmatTable
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     NAOKinematics::KMatTransf::makeTransformation(T, p[0], p[1], p[2], p[3], p[4], p[5]);
     
     //Call inverse function with new transformation table
     std::vector<std::vector<float> > result;
     result=nkin.inverseRightLeg(T);
     
     //Repackage result
     std::vector<double> ans(6);
     if(!result.empty()){
		for(unsigned j=0; j<result[0].size(); j++){
			ans[j] = static_cast<double>(result[0][j]);
		}
     }
     
     return ans;    
 }
 
 /** @brief   Perform inverse kinematics of left arm
 *  @details Shifts some data types to call the kinematics library functions
 *  @param t A transform specifying the location and orientation of the hand with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 *  @return A vector with angle values in the order: LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll
 */
 std::vector<double> InvertArmL(const Transform t) {
    
     //Get 6DOF from transform
     std::vector<double> p(6);
     p = position6D(t);
     
     //Create kinematics objects and move data into kmatTable
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     NAOKinematics::KMatTransf::makeTransformation(T, p[0], p[1], p[2], p[3], p[4], p[5]);
     
     //Call inverse function with new transformation table
     std::vector<std::vector<float> > result;
     result=nkin.inverseLeftHand(T);
     
     //Repackage result
     std::vector<double> ans(4);
     if(!result.empty()){
		for(unsigned j=0; j<ans.size(); j++){
			ans[j] = static_cast<double>(result[0][j]);
		}
     }
     
     return ans;
 }
 
 /** @brief   Perform inverse kinematics of right arm
 *  @details Shifts some data types to call the kinematics library functions
 *  @param t A transform specifying the location and orientation of the hand with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 *  @return A vector with angle values in the order: RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll
 */
 std::vector<double> InvertArmR(const Transform t) {
    
     //Get 6DOF from transform
     std::vector<double> p(6);
     p = position6D(t);
     
     //Create kinematics objects and move data into kmatTable
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     NAOKinematics::KMatTransf::makeTransformation(T, p[0], p[1], p[2], p[3], p[4], p[5]);
     
     //Call inverse function with new transformation table
     std::vector<std::vector<float> > result;
     result=nkin.inverseRightHand(T);
     
     //Repackage result
     std::vector<double> ans(4);
     if(!result.empty()){
		for(unsigned j=0; j<ans.size(); j++){
			ans[j] = static_cast<double>(result[0][j]);
		}
     }
     return ans;    
 }
 
 /** @brief   Perform inverse kinematics of head
 *  @details Shifts some data types to call the kinematics library functions
 *  @param t A transform specifying the location and orientation of the top camera with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 *  @return A vector with angle values in the order: HeadYaw,HeadPitch
 */
 std::vector<double> InvertHead(const Transform t) {
    
     //Get 6DOF from transform
     std::vector<double> p(6);
     p = position6D(t);
     
     //Create kinematics objects and move data into kmatTable
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     NAOKinematics::KMatTransf::makeTransformation(T, p[0], p[1], p[2], p[3], p[4], p[5]);
     
     //Call inverse function with new transformation table
     std::vector<std::vector<float> > result;
     result=nkin.inverseHead(T,1,1);  //Ones are to select using angles only and wrt to top camera
     
     //Repackage result
     std::vector<double> ans(2);
     if(!result.empty()){
		for(unsigned j=0; j<result[0].size(); j++){
			ans[j] = static_cast<double>(result[0][j]);
		}
     }
     return ans;    
 }
 
/** @brief  Perform forward kinematics of left and right legs (together).
 *  @param  lp A vector with angle values in the order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
 *  @param  rp A vector with angle values in the order:              RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
 *  @return A vector with transforms for the left and right foot.  First element is Transform for left foot, and second
 *          element is Transform for right foot.
 */

std::vector<Transform> ForwardLegs(const std::vector<double> lp, const std::vector<double> rp)
{
    // Generate transform for left foot.
    std::vector<double> lpCopy(lp);
    lpCopy[0] = lpCopy[0] / 2;
    
    Transform leftTransform(ForwardLegL(lpCopy));
    
    // Generate transform for right foot.
    std::vector<double> rpCopy(rp);
    rpCopy.insert(rpCopy.begin(), lpCopy[0]);
    
    Transform rightTransform(ForwardLegR(rpCopy));
    
    std::vector<Transform> output;
    output.push_back(leftTransform);
    output.push_back(rightTransform);
    
    return output;
}

 /** @brief  Perform forward kinematics of left leg
 *  @details Shifts some data types to call the kinematics library functions
 *  @param p A vector with angle values in the order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
 *  @return A transform specifying the location and orientation of the foot with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 */
 Transform ForwardLegL(const std::vector<double> p){
	 //Repackage input arguement
     std::vector<float> angles(6);
     for(unsigned j=0; j<p.size(); j++){
        angles[j] = static_cast<float>(p[j]);
     }

     //Create and call FK objects
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     nkin.setChain(KDeviceLists::CHAIN_L_LEG, angles);
     T = nkin.getForwardEffector((NAOKinematics::Effectors) KDeviceLists::CHAIN_L_LEG);
     
     //Change data type to transform
     Transform t;
     KMath::KMat::GenMatrix<double, 3, 1> Rotation;
     KMath::KMat::GenMatrix<double, 3, 1> Translation;
     Translation = T.getTranslation();
     Rotation = T.getEulerAngles();

     std::vector<double> a(6);
     a[0] = Translation(0,0);
     a[1] = Translation(1,0);
     a[2] = Translation(2,0);
     a[3] = Rotation(0,0);
     a[4] = Rotation(1,0);
     a[5] = Rotation(2,0);
     
     t=transform6D(a);
     return t;     
 }
 
 /** @brief  Perform forward kinematics of right leg
 *  @details Shifts some data types to call the kinematics library functions
 *  @param p A vector with angle values in the order: RHipYawPitch, RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
 *  @return A transform specifying the location and orientation of the foot with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 */
 Transform ForwardLegR(const std::vector<double> p)
 {
     //Repackage input arguement
     std::vector<float> angles(6);
     for(unsigned j=0; j<p.size(); j++){
        angles[j] = static_cast<float>(p[j]);
     }
     
     //Create and call FK objects
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     nkin.setChain(KDeviceLists::CHAIN_R_LEG, angles);
     T = nkin.getForwardEffector((NAOKinematics::Effectors) KDeviceLists::CHAIN_R_LEG);
     
     //Change data type to transform
     Transform t;
     KMath::KMat::GenMatrix<double, 3, 1> Rotation;
     KMath::KMat::GenMatrix<double, 3, 1> Translation;
     Translation = T.getTranslation();
     Rotation = T.getEulerAngles();
     
     std::vector<double> a(6);
     a[0] = Translation(0,0);
     a[1] = Translation(1,0);
     a[2] = Translation(2,0);
     a[3] = Rotation(0,0);
     a[4] = Rotation(1,0);
     a[5] = Rotation(2,0);

     t=transform6D(a);
     return t;     
 }
 
 /** @brief  Perform forward kinematics of left arm
 *  @details Shifts some data types to call the kinematics library functions
 *  @param p A vector with angle values in the order: LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll
 *  @return A transform specifying the location and orientation of the hand with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 */
 Transform ForwardArmL(const std::vector<double> p){
     
     //Repackage input arguement
     std::vector<float> angles(5);
     for(unsigned j=0; j<p.size(); j++){
        angles[j] = static_cast<float>(p[j]);
     }
	 angles[4] = 0;
     
     //Create and call FK objects
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     nkin.setChain(KDeviceLists::CHAIN_L_ARM, angles);
     T = nkin.getForwardEffector((NAOKinematics::Effectors) KDeviceLists::CHAIN_L_ARM);
     
     //Change data type to transform
     Transform t;
     KMath::KMat::GenMatrix<double, 3, 1> Rotation;
     KMath::KMat::GenMatrix<double, 3, 1> Translation;
     Translation = T.getTranslation();
     Rotation = T.getEulerAngles();

     std::vector<double> a(6);
     a[0] = Translation(0,0);
     a[1] = Translation(1,0);
     a[2] = Translation(2,0);
     a[3] = Rotation(0,0);
     a[4] = Rotation(1,0);
     a[5] = Rotation(2,0);

     t=transform6D(a);
     return t;     
 }
 
 /** @brief  Perform forward kinematics of right arm
 *  @details Shifts some data types to call the kinematics library functions
 *  @param p A vector with angle values in the order: RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll
 *  @return A transform specifying the location and orientation of the hand with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 */
 Transform ForwardArmR(std::vector<double> p){
     
     //Repackage input arguement
     std::vector<float> angles(5);
     for(unsigned j=0; j<p.size(); j++){
        angles[j] = static_cast<float>(p[j]);
     }
	 angles[4] = 0;   
		
     //Create and call FK objects
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     nkin.setChain(KDeviceLists::CHAIN_R_ARM, angles);
     T = nkin.getForwardEffector((NAOKinematics::Effectors) KDeviceLists::CHAIN_R_ARM);	 

     
     //Change data type to transform
     Transform t;
     KMath::KMat::GenMatrix<double, 3, 1> Rotation;
     KMath::KMat::GenMatrix<double, 3, 1> Translation;
     Translation = T.getTranslation();
     Rotation = T.getEulerAngles();

     std::vector<double> a(6);
     a[0] = Translation(0,0);
     a[1] = Translation(1,0);
     a[2] = Translation(2,0);
     a[3] = Rotation(0,0);
     a[4] = Rotation(1,0);
     a[5] = Rotation(2,0);

     t=transform6D(a);
     return t;     
 }
 
 /** @brief  Perform forward kinematics of head
 *  @details Shifts some data types to call the kinematics library functions
 *  @param p A vector with angle values in the order: HeadYaw,HeadPitch
 *  @return A transform specifying the location and orientation of the head with respect to the torso frame
 *              Frames are defined in the <a href="doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links"> aldebaran documentation</a> 
 */
 Transform ForwardHead(const std::vector<double> p){
     
	 
     //Repackage input arguement
     std::vector<float> angles(2);
     for(unsigned j=0; j<p.size(); j++){
        angles[j] = static_cast<float>(p[j]);
     }
     
	 
     //Create and call FK objects
     NAOKinematics nkin;
     NAOKinematics::kmatTable T;
     nkin.setChain(KDeviceLists::CHAIN_HEAD, angles);
     T = nkin.getForwardEffector((NAOKinematics::Effectors) KDeviceLists::CHAIN_HEAD);
	 
     //Change data type to transform
     Transform t;
     KMath::KMat::GenMatrix<double, 3, 1> Rotation;
     KMath::KMat::GenMatrix<double, 3, 1> Translation;
     Translation = T.getTranslation();
     Rotation = T.getEulerAngles();
	 
     std::vector<double> a(6);
     a[0] = Translation(0,0);
     a[1] = Translation(1,0);
     a[2] = Translation(2,0);
     a[3] = Rotation(0,0);
     a[4] = Rotation(1,0);
     a[5] = Rotation(2,0);

     t=transform6D(a);
     return t;     
 }
 
 /** @brief  Finds location of the COM
 *  @details Shifts some data types to call the kinematics library functions
 *  @param p A vector with angle values in the order: HeadYaw,HeadPitch, LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll, 
 *           LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
 *           RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll, 
 *           RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll - See KDeviceLists for more info
 *  @return A 3x1 vector with the xyz position of the COM
 */

 std::vector<double> calcCOM (const std::vector<double> argP){
     // Insert 0.0 placeholder element since library is expecting RHipYawPitch.
     std::vector<double> p(argP);
     p.insert(p.begin() + 16, 0.0);
     
     //Repackage input arguement
     std::vector<float> angles(24);
     for(unsigned j=0; j<p.size(); j++){
        angles[j] = static_cast<float>(p[j]);
     }
     
     //Create and call FK objects
     NAOKinematics nkin;
     KVecDouble3 T;
     nkin.setJoints(angles);
     T = nkin.calculateCenterOfMass();
     
     //Change data type to vector
     std::vector<double> a(3);
     a[0] = T(0,0);
     a[1] = T(1,0);
     a[2] = T(2,0);
     return a;     
 }
 
