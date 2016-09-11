function Locomotion_Unit_Tests_callBacks(src, ~, callBackName) 
    switch callBackName
        case 'pbCallback'
            pbCallback();
    end 
end

% Callback for run push button.
function pbCallback()
    % Establish global variables.
    global testResultsTB selectUnitTestPopup robotIPTB

    unitTestName = get(selectUnitTestPopup,'String');
    unitTestID   = get(selectUnitTestPopup, 'Value');
    robotIP      = get(robotIPTB, 'string');

    command = strcat('StepHandler_RUNTESTS_', unitTestName(unitTestID));
    
    message = char(TCPComm(command{1, 1}, robotIP));
    message = sprintf(message);

    set(testResultsTB, 'String', message);
end