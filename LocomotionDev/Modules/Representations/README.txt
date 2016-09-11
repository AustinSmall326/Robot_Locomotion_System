{\rtf1\ansi\ansicpg1252\cocoartf1404\cocoasubrtf470
{\fonttbl\f0\froman\fcharset0 TimesNewRomanPSMT;}
{\colortbl;\red255\green255\blue255;}
\margl1440\margr1440\vieww26000\viewh18280\viewkind0
\deftab720
\pard\pardeftab720\partightenfactor0

\f0\fs24 \cf0 \expnd0\expndtw0\kerning0
-------------------------\
Directory Overview\
-------------------------\
\
The Representations directory contains various classes for abstracting information related to a footstep.\
\
Transform:\
\
	The Transform class stores a transformation matrix, and allows various operations on that matrix.  This can \
	be useful in storing the position/orientation of the center of mass (COM) of the robot, as well as end-effectors\
	such as the hands, feet, and head.\
\
Trajectory:\
\
	The Trajectory class stores Transform objects for the position/orientation of the robot\'92s COM and swing foot end\
	effector for a given time.  \
\
COMContainer:\
\
	The COMContainer class stores various parameters used to characterize the LIPM motion of the robot\'92s COM\
	for a given footstep.\
\
Point:\
\
	The Point class stores all information relevant to a given planted footstep (the LIPM parameters are stored in \
	a COMContainer, and Trajectory objects are stored for each time-step relevant to that planted foot-step).\
\
--------------------\
Class Hierarchy\
--------------------\
\
			Point\
		           //       \\\\\
           COMContainer         Trajectory\
                                       	       \\\\\
	                                       Transform}