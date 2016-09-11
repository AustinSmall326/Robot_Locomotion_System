function [] = Keyframe_Generator_Deload()
% Define global variables.
global timerHandle

% Turn off timer.
stop(timerHandle);
delete(timerHandle);

end