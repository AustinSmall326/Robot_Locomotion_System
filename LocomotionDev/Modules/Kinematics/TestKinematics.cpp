/**
 * @file TestKinematics.cpp
 * @brief Cpp file for ktesting functions
 *
 * @author Alex Baucom and Austin Small for the UPennalizers
 */

#include "TestKinematics.h"
 
std::string TestKinematics::runTests(void)
{
    std::string output("");
    
    // Determine transforms for robot when standing up (all joint angles set to 0.1).
    Transform leftFootT;
    Transform rightFootT;
    Transform leftHandT;
    Transform rightHandT;
    Transform headT;
    
    std::vector<double> leftFootP(6);
    std::vector<double> rightFootP(6);
    std::vector<double> leftHandP(4);
    std::vector<double> rightHandP(4);
    std::vector<double> headP(2);
     
    // Store 0.1 as default angle.
    for (int i = 0; i < 6; i++)
    {
        leftFootP[i]  = 0.1;
        rightFootP[i] = 0.1;
    }
     
    for (int i = 0; i < 4; i++)
    {
        // A positive angle is not defined for the left elbow roll.
        if (i == 3)
        {
            leftHandP[i]  = -0.1;
        }
        else
        {
            leftHandP[i]  = 0.1;
        }
         
        rightHandP[i] = 0.1;
    }
     
    for (int i = 0; i < 2; i++)
    {
        headP[i] = 0.1;
    }
     
    leftFootT  = ForwardLegL(leftFootP);
    rightFootT = ForwardLegR(rightFootP);
    leftHandT  = ForwardArmL(leftHandP);
    rightHandT = ForwardArmR(rightHandP);
    headT      = ForwardHead(headP);
    
    /** ForwardLegL and InvertLegL **/
    // Test with all angles set to +/- 0.1.
    std::vector<double> p(6);
    
    p = InvertLegL(leftFootT);
     
    bool success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - leftFootP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
    
    if (success)
    {
        output.append("ForwardLegL and InvertLegL Generic Test Success!\n");
    }
    else
    {
        output.append("ForwardLegL and InvertLegL Generic Test FAILURE.\n");
    }
     
    // Test with all angles set to positive maximum.
    double eps = 0.001;
     
    leftFootP[0] = 42.44  * M_PI / 180 - eps;
    leftFootP[1] = 45.29  * M_PI / 180 - eps;
    leftFootP[2] = 27.73  * M_PI / 180 - eps;
    leftFootP[3] = 121.04 * M_PI / 180 - eps;
    leftFootP[4] = 52.86  * M_PI / 180 - eps;
    leftFootP[5] = 44.06  * M_PI / 180 - eps;
     
    leftFootT  = ForwardLegL(leftFootP);
    p = InvertLegL(leftFootT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - leftFootP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardLegL and InvertLegL Max Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardLegL and InvertLegL Max Angles Test FAILURE.\n");
    }
     
    // Test with all angles set to negative maximum.
    leftFootP[0] = -65.62 * M_PI / 180 + eps;
    leftFootP[1] = -21.74 * M_PI / 180 + eps;
    leftFootP[2] = -88.0  * M_PI / 180 + eps;
    leftFootP[3] = -5.29  * M_PI / 180 + eps;
    leftFootP[4] = -68.15 * M_PI / 180 + eps;
    leftFootP[5] = -22.79 * M_PI / 180 + eps;
     
    leftFootT  = ForwardLegL(leftFootP);
    p = InvertLegL(leftFootT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - leftFootP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardLegL and InvertLegL Min Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardLegL and InvertLegL Min Angles Test FAILURE.\n");
    }
     
    /** ForwardLegR and InvertLegR **/
    // Test with all angles set to +/- 0.1.
    p = InvertLegR(rightFootT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - rightFootP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardLegR and InvertLegR Generic Test Success!\n");
    }
    else
    {
        output.append("ForwardLegR and InvertLegR Generic Test FAILURE.\n");
    }
     
    // Test with all angles set to positive maximum.
    rightFootP[0] = 42.44  * M_PI / 180 - eps;
    rightFootP[1] = 21.74  * M_PI / 180 - eps;
    rightFootP[2] = 27.73  * M_PI / 180 - eps;
    rightFootP[3] = 121.47 * M_PI / 180 - eps;
    rightFootP[4] = 52.9   * M_PI / 180 - eps; // Note that the published maximum value is 53.40.
    rightFootP[5] = 22.80  * M_PI / 180 - eps;
     
    rightFootT  = ForwardLegR(rightFootP);
    p = InvertLegR(rightFootT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - rightFootP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardLegR and InvertLegR Max Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardLegR and InvertLegR Max Angles Test FAILURE.\n");
    }
     
    // Test with all angles set to negative maximum.
    rightFootP[0] = -65.62 * M_PI / 180 + eps;
    rightFootP[1] = -45.29 * M_PI / 180 + eps;
    rightFootP[2] = -88.0  * M_PI / 180 + eps;
    rightFootP[3] = -5.90  * M_PI / 180 + eps;
    rightFootP[4] = -67.97 * M_PI / 180 + eps; // Note that the published maximum value is 53.40.
    rightFootP[5] = -44.06 * M_PI / 180 + eps;
     
    rightFootT  = ForwardLegR(rightFootP);
    p = InvertLegR(rightFootT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - rightFootP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardLegR and InvertLegR Min Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardLegR and InvertLegR Min Angles Test FAILURE.\n");
    }

    /** ForwardArmL and InvertArmL **/
    // Test with all angles set to +/- 0.1.
    std::vector<double> pArmNew(4);
    pArmNew = InvertArmL(leftHandT);
     
    success = true;
     
    for (int i = 0; i < pArmNew.size(); i++)
    {
        if (fabs(pArmNew[i] - leftHandP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardArmL and InvertArmL Test Success!\n");
    }
    else
    {
        output.append("ForwardArmL and InvertArmL Test FAILURE.\n");
    }
     
    // Test with all angles set to positive maximum.
    leftHandP[0] = 119.5 * M_PI / 180 - eps;
    leftHandP[1] = 76.0  * M_PI / 180 - eps;
    leftHandP[2] = 119.5 * M_PI / 180 - eps;
    leftHandP[3] = -2.0  * M_PI / 180 - eps;
     
    leftHandT  = ForwardArmL(leftHandP);
    p = InvertArmL(leftHandT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - leftHandP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardArmL and InvertArmL Max Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardArmL and InvertArmL Max Angles Test FAILURE.\n");
    }
     
    // Test with all angles set to negative maximum.
    leftHandP[0] = -119.5 * M_PI / 180 + eps;
    leftHandP[1] = -18.0  * M_PI / 180 + eps;
    leftHandP[2] = -119.5 * M_PI / 180 + eps;
    leftHandP[3] = -88.5  * M_PI / 180 + eps;
     
    leftHandT  = ForwardArmL(leftHandP);
    p = InvertArmL(leftHandT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - leftHandP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardArmL and InvertArmL Min Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardArmL and InvertArmL Min Angles Test FAILURE.\n");
    }

    /** ForwardArmR and InvertArmR **/
    // Test with all angles set to +/- 0.1.
    pArmNew = InvertArmR(rightHandT);
     
    success = true;
     
    for (int i = 0; i < pArmNew.size(); i++)
    {
        if (fabs(pArmNew[i] - rightHandP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardArmR and InvertArmR Generic Test Success!\n");
    }
    else
    {
        output.append("ForwardArmR and InvertArmR Generic Test FAILURE.\n");
    }
     
    // Test with all angles set to positive maximum.
    rightHandP[0] = 119.5 * M_PI / 180 - eps;
    rightHandP[1] = 18.0  * M_PI / 180 - eps;
    rightHandP[2] = 119.5 * M_PI / 180 - eps;
    rightHandP[3] = 88.5  * M_PI / 180 - eps;
     
    rightHandT  = ForwardArmR(rightHandP);
    p = InvertArmR(rightHandT);
     
    success = true;
    
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - rightHandP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardArmR and InvertArmR Max Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardArmR and InvertArmR Max Angles Test FAILURE.\n");
    }
     
    // Test with all angles set to negative maximum.
    rightHandP[0] = -119.5 * M_PI / 180 + eps;
    rightHandP[1] = -76.0  * M_PI / 180 + eps;
    rightHandP[2] = -119.5 * M_PI / 180 + eps;
    rightHandP[3] = 2.0    * M_PI / 180 + eps;
     
    rightHandT  = ForwardArmR(rightHandP);
    p = InvertArmR(rightHandT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - rightHandP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardArmR and InvertArmR Max Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardArmR and InvertArmR Max Angles Test FAILURE.\n");
    }
     
    /** ForwardHead and InvertHead **/
    // Test with all angles set to +/- 0.1.
    std::vector<double> headPNew(2);
    headPNew = InvertHead(headT);
     
    success = true;
     
    for (int i = 0; i < headPNew.size(); i++)
    {
        if (fabs(headPNew[i] - headP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardHead and InvertHead Generic Test Success!\n");
    }
    else
    {
        output.append("ForwardHead and InvertHead Generic Test FAILURE.\n");
    }
     
    // Test with all angles set to positive maximum.
    headP[0] = 119.5 * M_PI / 180 - eps;
    headP[1] = 29.5  * M_PI / 180 - eps;
     
    headT  = ForwardHead(headP);
    p = InvertHead(headT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - headP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardHead and InvertHead Max Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardHead and InvertHead Max Angles Test FAILURE.\n");
    }
     
    // Test with all angles set to negative maximum.
    headP[0] = -119.5 * M_PI / 180 + eps;
    headP[1] = -38.5  * M_PI / 180 + eps;
     
    headT  = ForwardHead(headP);
    p = InvertHead(headT);
     
    success = true;
     
    for (int i = 0; i < p.size(); i++)
    {
        if (fabs(p[i] - headP[i]) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
     
    if (success)
    {
        output.append("ForwardHead and InvertHead Min Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardHead and InvertHead Min Angles Test FAILURE.\n");
    }
    
    /** ForwardLegs and InvertLegs **/
    // Test with all angles set to +/- 0.1.
    std::vector<double> leftP(6);
    std::vector<double> rightP(5);
    
    for (int i = 0; i < 6; i++)
    {
        if (i == 0)
        {
            
        }
        
        leftP[i] = 0.1;
    }
    
    for (int i = 0; i < 5; i++)
    {
        rightP[i] = 0.1;
    }
    
    std::vector<Transform> legsTVect(ForwardLegs(leftP, rightP));
    std::vector<double> legAnglesNew(InvertLegs(legsTVect[0], legsTVect[1]));
    
    success = true;
    
    for (int i = 0; i < legAnglesNew.size(); i++)
    {
        if (fabs(legAnglesNew[i] - 0.1) < 0.001)
        {
            continue;
        }
        else
        {
            success = false;
            break;
        }
    }
    
    if (success)
    {
        output.append("ForwardLegs and InvertLegs Generic Angles Test Success!\n");
    }
    else
    {
        output.append("ForwardLegs and InvertLegs Generic Angles Test FAILURE.\n");
    }
    
    /** calcCOM **/
    // Check for a reasonable value for COM position from resting pose.
    std::vector<double> comAllAngles(24);
     
    comAllAngles[0]  =  0.00000; // HEAD_YAW
    comAllAngles[1]  =  0.00000; // HEAD_PITCH
     
    comAllAngles[2]  =  1.57000; // L_ARM_SHOULDER_PITCH
    comAllAngles[3]  =  0.30000; // L_ARM_SHOULDER_ROLL
    comAllAngles[4]  = -0.30000; // L_ARM_ELBOW_YAW
    comAllAngles[5]  = -0.33000; // L_ARM_ELBOW_ROLL
    comAllAngles[6]  =  0.00000; // L_ARM_WRIST_YAW

    comAllAngles[7]  = -0.30000; // L_LEG_HIP_YAW_PITCH
    comAllAngles[8]  =  0.00000; // L_LEG_HIP_ROLL
    comAllAngles[9]  = -0.70000; // L_LEG_HIP_PITCH
    comAllAngles[10]  = 2.15000; // L_LEG_KNEE_PITCH
    comAllAngles[11] = -1.20000; // L_LEG_ANKLE_PITCH
    comAllAngles[12] =  0.00000; // L_LEG_ANKLE_ROLL
     
    comAllAngles[13] =  0.00000; // R_LEG_HIP_YAW_PITCH
    comAllAngles[14] =  0.00000; // R_LEG_HIP_ROLL
    comAllAngles[15] = -0.70000; // R_LEG_HIP_PITCH
    comAllAngles[16] =  2.15000; // R_LEG_KNEE_PITCH
    comAllAngles[17] = -1.20000; // R_LEG_ANKLE_PITCH
    comAllAngles[18] =  0.00000; // R_LEG_ANKLE_ROLL
     
    comAllAngles[19] =  1.57000; // R_ARM_SHOULDER_PITCH
    comAllAngles[20] = -0.30000; // R_ARM_SHOULDER_ROLL
    comAllAngles[21] =  0.30000; // R_ARM_ELBOW_YAW
    comAllAngles[22] =  0.33000; // R_ARM_ELBOW_ROLL
    comAllAngles[23] =  0.00000; // R_ARM_WRIST_YAW
     
    std::vector<double> comLocus(3);
     
    comLocus = calcCOM(comAllAngles);
    
    output.append("Center of mass location:");
    output.append("\n");

    output.append("x: ");
    output.append(boost::lexical_cast<std::string>(comLocus[0]));
    output.append("\n");
    
    output.append("y: ");
    output.append(boost::lexical_cast<std::string>(comLocus[1]));
    output.append("\n");

    output.append("z: ");
    output.append(boost::lexical_cast<std::string>(comLocus[2]));
    output.append("\n");

    return output;
}