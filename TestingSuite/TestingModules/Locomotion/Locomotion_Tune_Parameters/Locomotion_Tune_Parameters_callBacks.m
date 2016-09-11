function Locomotion_Tune_Parameters_callBacks(src, ~, callBackName) 
    switch callBackName
        case 'timer'
            timer();
        case 'updateValueCallback'
            updateValue();
        case 'startMotionPBCallback'
            startMotion();
        case 'stopMotionPBCallback'
            stopMotion();
    end 
end

% Callback for timer function.
function timer()
    % Establish global variables.
    global robotIPTB CurrentValueDispTB selectParamPopup

    selectedParamID     = get(selectParamPopup, 'Value');
    selectedParamList   = get(selectParamPopup, 'String');
    selectedParam       = selectedParamList{selectedParamID};
    
    robotIP = get(robotIPTB, 'string');
    command = strcat('StepHandler_GETPARAMVALUE_', selectedParam);       
    data = char(TCPComm(command, robotIP));

    set(CurrentValueDispTB, 'String', data);
end

% Callback for update value pushbutton.
function updateValue()
    % Establish global variables.
    global robotIPTB selectParamPopup UpdateValueEditTB
    
    selectedParamID     = get(selectParamPopup, 'Value');
    selectedParamList   = get(selectParamPopup, 'String');
    selectedParam       = selectedParamList{selectedParamID};

    newParamValue       = get(UpdateValueEditTB, 'String');
    
    command = strcat('StepHandler_SETPARAMVALUE_', selectedParam, '_', newParamValue);
    robotIP = get(robotIPTB, 'string');
    data = char(TCPComm(command, robotIP));
end

% Callback for start motion pushbutton.
function startMotion()
    % Establish global variables.
    global robotIPTB 
   
    command = 'StepHandler_STARTTESTMOTION';
    robotIP = get(robotIPTB, 'string');
    TCPComm(command, robotIP);
end

% Callback for stop motion pushbutton.
function stopMotion()
    % Establish global variables.
    global robotIPTB 
   
    command = 'StepHandler_STOPTESTMOTION';
    robotIP = get(robotIPTB, 'string');
    TCPComm(command, robotIP);
end