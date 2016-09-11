function Locomotion_Visual_Unit_Tests_callBacks(src, ~, callBackName) 
    switch callBackName
        case 'pbCallback'
            pbCallback();
        case 'leftCallback'
            leftCallback();
        case 'rightCallback'
            rightCallback();
        case 'popupOrientationCallback'
            popupOrientationCallback();
    end 
end

% Callback for run push button.
function pbCallback()
    % Establish global variables.
    global selectUnitTestPopup robotIPTB plotHandle visualTestStruct currentPlotIdx ...
        
    % Empty visualTestStruct.
    for i = 1:size(visualTestStruct, 2)
        visualTestStruct(1) = [];
    end
    
    unitTestName = get(selectUnitTestPopup, 'String');
    unitTestID   = get(selectUnitTestPopup, 'Value');
    robotIP      = get(robotIPTB, 'string');

    command = strcat('StepHandler_RUNTESTS_', unitTestName(unitTestID));
    
    data = char(TCPComm(command{1, 1}, robotIP));

    %% Parse data.
    % Parse number of tests.
    idx      = strfind(data, '_');
    numTests = str2num(data(1 : (idx - 1)));
    data     = data((idx + 1) : end);
    
    for i = 1:numTests
        % Determine test name.
        idx      = strfind(data, '_');
        testName = data(1 : (idx - 1));
        data     = data((idx + 1) : end);
        
        visualTestStruct(1, i).TestName = testName;
        
        % Determine number of data points.
        idx       = strfind(data, '_');
        numPoints = str2num(data(1 : (idx - 1)));
        data      = data((idx + 1) : end);
        
        XPoints = [];
        YPoints = [];
        ZPoints = [];
        
        for j = 1:numPoints
            idx     = strfind(data, '_');
            XPoint  = str2num(data(1 : (idx - 1)));
            data    = data((idx + 1) : end);
            XPoints = horzcat(XPoints, [XPoint]);
            
            idx     = strfind(data, '_');
            YPoint  = str2num(data(1 : (idx - 1)));
            data    = data((idx + 1) : end);
            YPoints = horzcat(YPoints, [YPoint]);
            
            idx     = strfind(data, '_');
            ZPoint  = str2num(data(1 : (idx - 1)));
            data    = data((idx + 1) : end);
            ZPoints = horzcat(ZPoints, [ZPoint]);
        end
        
        visualTestStruct(1, i).XPoints = XPoints;
        visualTestStruct(1, i).YPoints = YPoints;
        visualTestStruct(1, i).ZPoints = ZPoints;
    end
        
    % If there is at least one test, plot its data.
    if (size(visualTestStruct, 2) > 0)
        axes(plotHandle);
        plot3(visualTestStruct(1).XPoints, visualTestStruct(1).YPoints, visualTestStruct(1).ZPoints, 'x');
        axis equal;
        grid on;
        title(visualTestStruct(1).TestName);
        currentPlotIdx = 1;
        popupOrientationCallback();
    end
end

function leftCallback()
    % Define global variables.
    global visualTestStruct plotHandle currentPlotIdx 
    
    % Check if visualTestStruct contains more than one record.
    if ((size(visualTestStruct, 2) > 1) && (currentPlotIdx > 1))
        currentPlotIdx = currentPlotIdx - 1;
        axes(plotHandle);
        plot3(visualTestStruct(currentPlotIdx).XPoints, visualTestStruct(currentPlotIdx).YPoints, visualTestStruct(currentPlotIdx).ZPoints, 'x');
        axis equal;
        grid on;
        title(visualTestStruct(currentPlotIdx).TestName);
        popupOrientationCallback();
    end    
end

function rightCallback()
    % Define global variables.
    global visualTestStruct plotHandle currentPlotIdx
    
    % Check if visualTestStruct contains more than one record.
    if ((size(visualTestStruct, 2) > 1) && (currentPlotIdx < size(visualTestStruct, 2)))
        currentPlotIdx = currentPlotIdx + 1;
        axes(plotHandle);
        plot3(visualTestStruct(currentPlotIdx).XPoints, visualTestStruct(currentPlotIdx).YPoints, visualTestStruct(currentPlotIdx).ZPoints, 'x');
        axis equal;
        grid on;
        title(visualTestStruct(currentPlotIdx).TestName);
        popupOrientationCallback();
    end 
end

function popupOrientationCallback()
    % Define global variables.
    global visualTestStruct plotHandle selectOrientationPopup
    
    orientationList = get(selectOrientationPopup, 'string');
    orientationID   = get(selectOrientationPopup, 'value');
    orientation     = orientationList{orientationID};
    
    if (size(visualTestStruct, 2) > 0)
        axes(plotHandle);
        
        if (strcmp(orientation, 'Perspective View') == 1)
            az = -45;
            el =  45;
            view(az, el);
        elseif (strcmp(orientation, 'XY Axis') == 1)
            az = 0;
            el = 90;
            view(az, el);
        elseif (strcmp(orientation, 'YZ Axis') == 1)
            az = -90;
            el =  0;
            view(az, el);
        elseif (strcmp(orientation, 'XZ Axis') == 1)
            az = 0;
            el = 0;
            view(az, el);
        end
    end
end