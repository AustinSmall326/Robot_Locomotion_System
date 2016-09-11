{\rtf1\ansi\ansicpg1252\cocoartf1404\cocoasubrtf470
{\fonttbl\f0\froman\fcharset0 TimesNewRomanPSMT;}
{\colortbl;\red255\green255\blue255;}
\margl1440\margr1440\vieww15960\viewh15240\viewkind0
\deftab720
\pard\pardeftab720\partightenfactor0

\f0\fs24 \cf0 \expnd0\expndtw0\kerning0
------------------------\
Directory Overview\
------------------------\
\
The Kinematics directory contains various classes for performing abstracted kinematics operations for the Nao robot.\
\
KinematicsDefines:\
\
	This file contains definitions for variables used in the Kinematics library.\
\
KinematicsWrapper:\
\
	KinematicsWrapper is the highest level file in this folder.  It uses Transform and std::vector objects to pass\
	into the NAOKinematics library.\
\
	Note:\
\
		In practice, it can be advantageous to first get an initial transform for the legs by calling the ForwardLeg\
		functions with a vector of 0\'92s.  This will return a transform that gives the location and orientation of the foot with \
		respect to the torso for a straight leg.  Then, any necessary motions can be computed using this initial transform,\
		which might provide an easier reference than the torso.\
\
KMat:\
\
	A matrix library.\
\
NAOKinematics:\
\
	This file contains code that actually performs the forward and inverse kinematics.\
\
robotConsts:\
\
	Contains enumerations for the Nao\'92s body parts.\
\
TestKinematics:\
\
	This class contains unit tests for the Kinematics library, interfacing through KinematicsWrapper at the highest level.\
\
--------------------------------------------\
Further References / Documentation\
--------------------------------------------\
\
	*	Kofinas N., Orfanoudakis E., Lagoudakis M.: [Complete Analytical Inverse Kinematics for NAO]\
		(http://www.nikofinas.com/Publications/Complete_Analytical_Inverse_Kinematics_for_NAO.pdf), \
		Proceedings of the 13th International Conference on Autonomous Robot Systems and Competitions (ROBOTICA), \
		Lisbon, Portugal, April 2013, pp. 1-6.\
	*	Kofinas N.: [Forward and Inverse Kinematics for the NAO Humanoid Robot]\
		(http://www.nikofinas.com/Projects/KofinasThesis.pdf), Diploma Thesis, Department of Electronic and Computer Engineering, \
		Technical University of Crete, July 2012.	\
}