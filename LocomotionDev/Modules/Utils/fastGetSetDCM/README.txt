About:

	The fastGetSetDCM module is an independent module loaded into NAOQI that 
	sets the Nao’s joint position and hardness based on values stored in shared 
	memory.

Testing Results:

	This module was tested by linearly interpolating joint angle and stiffness
	for various joints in order to evaluate timing.  The values were properly
	interpolated for each time step, with no repeat values (which was a problem
	in our previous system).



