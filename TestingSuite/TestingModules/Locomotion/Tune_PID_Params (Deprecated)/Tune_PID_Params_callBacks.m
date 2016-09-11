function Tune_PID_Params_callBacks(src, ~, callBackName) 
    switch callBackName
        case 'pbCallback'
            pbCallback();
    end 
end

% Callback for run push button.
function pbCallback()
    % Establish global variables.
    global selectJointPopup newJointValueEditTB durationValueEditTB pEditTB ...
           iEditTB dEditTB PIDModePopup robotIPTB plotHandle

    % Generate string message to send to robot.
    command = strcat('tunePIDParam_TESTPARAM_');
    
    % Joint index.
    command = strcat(command, num2str(get(selectJointPopup, 'Value') - 1), '_');
    
    % Joint angle.
    command = strcat(command, num2str(get(newJointValueEditTB, 'String')), '_');

    % Movement duration.
    command = strcat(command, get(durationValueEditTB, 'String'), '_');
    
    % PID state.
    command = strcat(command, num2str((get(PIDModePopup, 'Value') - 1)), '_');
    
    % Gain.
    pGain = get(pEditTB, 'String');
    iGain = get(iEditTB, 'String');
    dGain = get(dEditTB, 'String');
    command = strcat(command, pGain, '_', iGain, '_', dGain, '_');
    
    robotIP = get(robotIPTB, 'string');
    data = char(TCPComm(command, robotIP))

    % Check if data is invalid.
    if (strcmp(data, '') == 1 || strcmp(data, 'Failure') == 1)
        return;
    end
    
    % Parse return message.
    idx        = strfind(data, '_');
    numPackets = str2num(data(1 : (idx - 1)));
    data       = data((idx + 1) : end);
    
    idx           = strfind(data, '_');
    numDataPoints = str2num(data(1 : (idx - 1)));
    data          = data((idx + 1) : end);
    
    actualJointPosVect   = [];
    expectedJointPosVect = [];
    timeStampVect        = [];
    
    for i = 1 : (numPackets + 1)
        if (i > 1)
            % Extract number of data points.
            idx           = strfind(data, '_');
            numDataPoints = str2num(data(1 : (idx - 1)));
            data          = data((idx + 1) : end);
        end
        
        for j = 1 : numDataPoints
            % Extract actual joint position.
            idx            = strfind(data, '_');
            actualJointPos = str2num(data(1 : (idx - 1)));
            data           = data((idx + 1) : end);

            % Extract expected joint position.
            idx              = strfind(data, '_');
            expectedJointPos = str2num(data(1 : (idx - 1)));
            data             = data((idx + 1) : end);

            % Extract time stamp.
            idx       = strfind(data, '_');
            timeStamp = str2num(data(1 : (idx - 1)));
            data      = data((idx + 1) : end);

            actualJointPosVect   = horzcat(actualJointPosVect,   [actualJointPos]);
            expectedJointPosVect = horzcat(expectedJointPosVect, [expectedJointPos]);
            timeStampVect        = horzcat(timeStampVect,        [timeStamp]);
        end
        
        if (i == numPackets)
            break;
        end
        
        command = strcat('tunePIDParam_REQUESTTESTPARAMDATA_');
        disp('extradata')
        data = char(TCPComm(command, robotIP))
    end
    
    size(timeStampVect)
    timeStampVect        = (timeStampVect - timeStampVect(1)) / 1000;
    actualJointPosVect   = actualJointPosVect * 180 / pi;
    expectedJointPosVect = expectedJointPosVect * 180 / pi;
    
    % Plot results.
    cla;
    axes(plotHandle);
    
    hold on;
    plot(timeStampVect, actualJointPosVect);
    plot(timeStampVect, expectedJointPosVect);
    legend('Actual Joint Position', 'Expected Joint Position');
    ylim([-180 180]);
end