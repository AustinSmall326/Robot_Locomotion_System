function Keyframe_Generator_callBacks(src, ~, callBackName)
    switch callBackName
        case 'sliderCallback'
            sliderCallback(src);
        case 'motionlbClick'
            motionlbClick();
        case 'motionPlus'
            motionAddCallback();
        case 'motionMinus'
            motionRemoveCallback();
        case 'motionEdit'
            motionEditCallback();
        case 'keyframeslbClick'
            keyframeslbClick();
        case 'keyframesPlus'
            keyframesAddCallback();
        case 'keyframesMinus'
            keyframesRemoveCallback();
        case 'keyframesUp'
            keyframesUpCallback();
        case 'keyframesDown'
            keyframesDownCallback();  
        case 'keyframesEdit'
            keyframesEditCallback();
        case 'interpolationlbClick'
            interpolationlbClick();
        case 'interpolationDurationCallback'
            interpolationDurationCallback();
        case 'updateEditBoxes'
            updateEditBoxes();
        case 'loadFromEditBoxes'
            loadFromEditBoxes();
        case 'loosenRobot'
            loosenRobot();
        case 'stiffenRobot'
            stiffenRobot();
        case 'executeMotionCallback'
            executeMotionCallback();
        case 'editBoxCallback'
            editBoxCallback();
        case 'timer'
            timerCallback();
        case 'loosenHead'
            loosenHead();
        case 'loosenRArm'
            loosenRArm();
        case 'loosenLArm'
            loosenLArm();
        case 'loosenRLeg'
            loosenRLeg();
        case 'loosenLLeg'
            loosenLLeg();
    end 
end

% Callback for slider button.
function sliderCallback(sliderHandle)
    global sliderPanel sliderPanelX sliderPanelY sliderPanelHeight sliderPanelWidth sliderPanelYBtm ...
           overviewPanelHeight estPanelTextHeight
    
    sliderVal = get(sliderHandle, 'value');
    verticalTraverse = (sliderVal / 100) * (sliderPanelHeight - (overviewPanelHeight - estPanelTextHeight));

    sliderPanelY = sliderPanelYBtm - verticalTraverse;
    
    setpixelposition(sliderPanel, [sliderPanelX, sliderPanelY, sliderPanelWidth, sliderPanelHeight]);
end

% Callback for motion list box click event.
function motionlbClick()
    % Establish global variables.
    global motionStruct
    
    % Check if there are any elements in list box.  If not, return from 
    % function.
    if (size(motionStruct, 2) < 1)
        return;
    end
    
    Keyframe_Generator_Update_State(false, false, true, true);
end

% Callback for motion add button.
function motionAddCallback()
    % Establish global variables.
    global motionStruct motionTB
    
    % Add new motion to struct.
    motionStruct(size(motionStruct, 2) + 1).name = get(motionTB, 'String');

    % Update state of environment.
    % If this is the first motion, then simulate a click to populate motion
    % to environment.
    if (size(motionStruct, 2) == 1)
        Keyframe_Generator_Update_State(true, true, false, false);
    else
    	Keyframe_Generator_Update_State(true, false, false, false);
    end
end

% Callback for montion remove button.
function motionRemoveCallback()
    % Establish global variables.
    global motionStruct motionlb
    
    % Remove selected motion from struct.
    % First check if there is an element to remove.
    if (size(motionStruct, 2) > 0)
        motionStruct(get(motionlb, 'Value')) = [];
        
        % Handle newly selected item of listbox.
        currIndex = get(motionlb, 'Value');
        currIndex = currIndex - 1;
        
        if (currIndex < 1)
            currIndex = 1;
        end
        
        set(motionlb, 'Value', currIndex);
        
        % Update state of environment.
        % First call is set to true, to simulate click on newly selected
        % item.
        Keyframe_Generator_Update_State(true, true, false, false);
    end
end

% Callback for click to motion edit pushbutton.
function motionEditCallback()
    % Establish global variables.
    global motionStruct motionTB motionlb
    
    % Make sure there is at least one motion in motionStruct.
    if (size(motionStruct, 2) < 1)
        return;
    end
    
    % Update name of motion in motion struct.
    selectedMotionIndex = get(motionlb, 'Value');
    motionStruct(selectedMotionIndex).name = get(motionTB, 'String');
    
    % Update state of environment.
    Keyframe_Generator_Update_State(true, false, false, false);
end

% Callback for click to keyframes list box.
function keyframeslbClick()
    % Establish global variables.
    global motionStruct motionlb
    
    % Check that at least one motion exists.
    if (size(motionStruct, 2) > 0)
        % Check if the selected motion has any keyframes.
        selectedMotionIndex = get(motionlb, 'Value');
        
        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            % Update environment state.
        	Keyframe_Generator_Update_State(false, false, true, true);
        else
            % Do nothing.
            return;
        end
    else
        % Do nothing.
        return;
    end
end

% Callback for keyframes add button.
function keyframesAddCallback()
    % Establish global variables.
    global motionStruct motionlb keyframesTB
    
    % Check that at least one motion exists.
    if (size(motionStruct, 2) > 0)
        % Check if the selected motion has any keyframes.
        selectedMotionIndex = get(motionlb, 'Value');
        
        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            keyframeStruct = motionStruct(selectedMotionIndex).motions;
        else
            % Create a new keyframe/motion structure.
            keyframeStruct = struct('name', {}, 'interpolationType', {}, ...
                                    'interpolationDuration', {}, ...
                                    'angles', {}, 'stiffness', {});
        end
                
        keyframeStruct(1, size(keyframeStruct, 2) + 1).name = get(keyframesTB, 'String');
        
        % Initiaize all joint angles and stiffness to 0.
        keyframeStruct(1, end).angles    = cell(1, 23);
        keyframeStruct(1, end).stiffness = cell(1, 23);
        
        for i = 1:23
            keyframeStruct(1, end).angles{i}    = '0.0';
            keyframeStruct(1, end).stiffness{i} = '0.0';
        end
        
        % Initialize interpolation style as linear with a 1 second
        % duration.
        keyframeStruct(1, end).interpolationType     = 'Linear';
        keyframeStruct(1, end).interpolationDuration = '1.0';
        
        motionStruct(selectedMotionIndex).motions = keyframeStruct;
        
        % Update state of environment.
        % If this is the first keyframe, then simulate a click to populate
        % keyframe to environment.
        if (size(keyframeStruct) > 1)
            Keyframe_Generator_Update_State(false, false, true, false);
        else
        	Keyframe_Generator_Update_State(false, false, true, true);
        end
    else
        % Do nothing.
        return;
    end
end

% Callback for keyframes remove button.
function keyframesRemoveCallback()
    % Establish global variables.
    global motionStruct keyframeslb motionlb
    
    % Check that at least one motion exists.
    if (size(motionStruct, 2) > 0)
        % Check if the selected motion has any keyframes.
        selectedMotionIndex = get(motionlb, 'Value');
        
        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            selectedKeyframeIndex = get(keyframeslb, 'Value');
            
            keyframeStruct = motionStruct(selectedMotionIndex).motions;
            
            keyframeStruct(selectedKeyframeIndex) = [];
            
            % Handle newly selected item of listbox.
            currIndex = get(keyframeslb, 'Value');
            currIndex = currIndex - 1;
        
            if (currIndex < 1)
                currIndex = 1;
            end
        
            set(keyframeslb, 'Value', currIndex);
            
            motionStruct(selectedMotionIndex).motions = keyframeStruct;

            % Update state of environment.
            Keyframe_Generator_Update_State(false, false, true, true);
        else
            % Do nothing.
            return;
        end
    else
        % Do nothing.
        return;
    end
end

% Callback for keyframes up button.
function keyframesUpCallback()
    % Establish global variables.
    global motionStruct motionlb keyframeslb
    
    % Make sure there is at least one motion.
    if (size(motionStruct, 2) < 1)
        return;
    end
    
    % Make sure there are at least two keyframes.  Otherwise, clicking this
    % button would be pointless.
    selectedMotionIndex = get(motionlb, 'Value');
    
    if (size(motionStruct(selectedMotionIndex).motions, 2) >= 2)
        % Make sure that the user is not trying to move the first item in
        % the listbox up (another pointless operation).
        selectedKeyframeIndex = get(keyframeslb, 'Value');
        
        if (selectedKeyframeIndex == 1)
            return;
        end    
        
        % Swap keyframes in motionStruct.
        motionStruct(selectedMotionIndex).motions(end + 1) = motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex - 1);
        motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex - 1) = motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex);
        motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex) = motionStruct(selectedMotionIndex).motions(end);
        motionStruct(selectedMotionIndex).motions(end) = []; 

        % Update list box value (selected item).
        set(keyframeslb, 'Value', get(keyframeslb, 'Value') - 1);
        
        % Update state of environment.
        Keyframe_Generator_Update_State(false, false, true, false);
    end
end

% Callback for keyframes down button.
function keyframesDownCallback()
    % Establish global variables.
    global motionStruct motionlb keyframeslb
    
    % Make sure there is at least one motion.
    if (size(motionStruct, 2) < 1)
        return;
    end
    
    % Make sure there are at least two keyframes.  Otherwise, clicking this
    % button would be pointless.
    selectedMotionIndex = get(motionlb, 'Value');
    
    if (size(motionStruct(selectedMotionIndex).motions, 2) >= 2)
        % Make sure that the user is not trying to move the first item in
        % the listbox up (another pointless operation).
        selectedKeyframeIndex = get(keyframeslb, 'Value');
        
        if (selectedKeyframeIndex == size(motionStruct(selectedMotionIndex).motions, 2))
            return;
        end    
        
        % Swap keyframes in motionStruct.
        motionStruct(selectedMotionIndex).motions(end + 1) = motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex + 1);
        motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex + 1) = motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex);
        motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex) = motionStruct(selectedMotionIndex).motions(end);
        motionStruct(selectedMotionIndex).motions(end) = []; 

        % Update list box value (selected item).
        set(keyframeslb, 'Value', get(keyframeslb, 'Value') + 1);
        
        % Update state of environment.
        Keyframe_Generator_Update_State(false, false, true, false);
    end
end

% Callback for click to keyframes edit pushbutton.
function keyframesEditCallback()
    % Establish global variables.
    global motionStruct motionlb keyframesTB keyframeslb
    
    % Make sure there is at least one motion in motionStruct.
    if (size(motionStruct, 2) < 1)
        return;
    end
    
    selectedMotionIndex = get(motionlb, 'Value');
    
    % Make sure there is at least one keyframe.
    if (size(motionStruct(selectedMotionIndex).motions, 2) < 1)
        return;
    end
    
    % Update name of keyframe in motion struct.
    selectedKeyframeIndex = get(keyframeslb, 'Value');
    
    motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).name = get(keyframesTB, 'String');
    
    % Update state of environment.
    Keyframe_Generator_Update_State(false, false, true, false);
end

% Callback for click to interpolation listbox.
function interpolationlbClick()
    % Establish global variables.
    global motionStruct motionlb keyframeslb interpolationlb
    
    % Check if any motions exist.
    if (size(motionStruct) > 0)
        % Check if there are any keyframes for selected motion.
        selectedMotionIndex = get(motionlb, 'Value');

        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            selectedKeyframeIndex = get(keyframeslb, 'Value');
            
            % Interpolation type.
            interpTypeIndex = get(interpolationlb, 'Value');
            interpTypes     = get(interpolationlb, 'String');
            interpType      = interpTypes{interpTypeIndex};
            
            motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).interpolationType = interpType;
            Keyframe_Generator_Update_State(false, false, false, false);
        end
    end
end

% Callback for value change of interpolation duration textbox.
function interpolationDurationCallback()
% Establish global variables.
    global motionStruct motionlb keyframeslb interpolationEditTB
    
    % Check if any motions exist.
    if (size(motionStruct) > 0)
        % Check if there are any keyframes for selected motion.
        selectedMotionIndex = get(motionlb, 'Value');

        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            selectedKeyframeIndex = get(keyframeslb, 'Value');
            
            % Interpolation duration.
            motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).interpolationDuration = get(interpolationEditTB, 'String');
            Keyframe_Generator_Update_State(false, false, false, false);
        end
    end
end

% Callback for update edit boxes.
function updateEditBoxes()
    % Establish global variables.
    global editHandles displayHandles
    
    for i = 1:23
        set(editHandles{i, 1}, 'String', get(displayHandles{i, 1}, 'String'));
        set(editHandles{i, 2}, 'String', get(displayHandles{i, 2}, 'String'));
    end
    
    % Callback to indicate that values in edit boxes have changed.
    editBoxCallback();
end

% Callback for load from edit boxes.
function loadFromEditBoxes()
    % Establish global variables.
    global editHandles robotIPTB
  
    actuatorCommandString = '1_1_';

    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end

        actuatorCommandString = strcat(actuatorCommandString, get(editHandles{i, 1}, 'String'), '_');
        actuatorCommandString = strcat(actuatorCommandString, get(editHandles{i, 2}, 'String'), '_');
    end

    actuatorCommandString = strcat(actuatorCommandString, '1.0', '_');
    actuatorCommandString = strcat(actuatorCommandString, 'Linear', '_');

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_ITERPVALUES_', actuatorCommandString), robotIP);
end

% Callback for loosen robot button.
function loosenRobot()
    % Establish global variables.
    global editHandles robotIPTB
  
    actuatorCommandString = '';

    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end

        actuatorCommandString = strcat(actuatorCommandString, get(editHandles{i, 1}, 'String'), '_');
        actuatorCommandString = strcat(actuatorCommandString, '0.0', '_');
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', actuatorCommandString), robotIP);
end

% Callback for stiffen robot button.
function stiffenRobot()
    % Establish global variables.
    global displayHandles robotIPTB
  
    actuatorCommandString = '';

    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end

        actuatorCommandString = strcat(actuatorCommandString, get(displayHandles{i, 1}, 'String'), '_');
        actuatorCommandString = strcat(actuatorCommandString, '1.0', '_');
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', actuatorCommandString), robotIP);
end

% Callback for edit boxes.
function editBoxCallback()
    % Establish global variables.
    global motionStruct motionlb keyframeslb editHandles
    
    % Check if there are any motions.
    if (size(motionStruct) > 0)
        selectedMotionIndex = get(motionlb, 'Value');
        
        % Check if there are any keyframe.
        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            selectedKeyframeIndex = get(keyframeslb, 'Value');
                                     
            % Update joint angles and stiffness in struct.
            for i = 1:23
                % Skip wrist joints.
                if (i == 5 || i == 10)
                    continue;
                end
                
                motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).angles{i}    = get(editHandles{i, 1}, 'String');
                motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).stiffness{i} = get(editHandles{i, 2}, 'String');
            end
                        
            Keyframe_Generator_Update_State(false, false, false, false);
        end
    end
end

% Callback for execute motion button.
function executeMotionCallback()
    % Establish global variables.
    global motionStruct motionlb robotIPTB
    
    % Check if any motions exist.
    if (size(motionStruct) > 0)
        % Check if there are any keyframes for selected motion.
        selectedMotionIndex = get(motionlb, 'Value');

        if (size(motionStruct(selectedMotionIndex).motions) > 0)
            for i = 1 : size(motionStruct(selectedMotionIndex).motions, 2)
                actuatorCommandString = strcat(num2str(size(motionStruct(selectedMotionIndex).motions, 2)), '_', num2str(i), '_');
                
                for j = 1:23
                    % Ignore wrist yaw.
                    if (j == 5 || j == 10)
                        continue;
                    end
        
                    actuatorCommandString = strcat(actuatorCommandString, motionStruct(selectedMotionIndex).motions(i).angles{j}, '_');
                    actuatorCommandString = strcat(actuatorCommandString, motionStruct(selectedMotionIndex).motions(i).stiffness{j}, '_');
                end
                
                actuatorCommandString = strcat(actuatorCommandString, motionStruct(selectedMotionIndex).motions(i).interpolationDuration, '_');
                actuatorCommandString = strcat(actuatorCommandString, motionStruct(selectedMotionIndex).motions(i).interpolationType, '_');
            
                robotIP = get(robotIPTB, 'string');
                TCPComm(strcat('keyframeGen_ITERPVALUES_', actuatorCommandString), robotIP);
            end
        end
    end
end

% Callback for timer.
function timerCallback()
    % Establish global variables.
    global displayHandles robotIPTB

    robotIP = get(robotIPTB, 'string');
    data = char(TCPComm('keyframeGen_GETVALUES_', robotIP));

    % Check if an error occurred.    
    if (size(data) == 0)
        for i = 1:23
            % Ignore wrist yaw.
            if (i == 5 || i == 10)
                continue;
            end

            set(displayHandles{i, 1}, 'String' , '-');
            set(displayHandles{i, 2}, 'String' , '-');
        end
        
        return;
    end
    
    % Parse the data.
    currentValuesCell = cell(23, 2);
    
    for i = 1:23
        firstIdx = strfind(data, '_');
        firstVal = data(1 : (firstIdx - 1));
        
        if (size(firstVal, 2) > 7)
            firstVal = firstVal(1:7);
        end
        
        data = data((firstIdx + 1) : end);
        
        secondIdx = strfind(data, '_');
        secondVal = data(1 : (secondIdx - 1));
        
        if (size(secondVal, 2) > 7)
            secondVal = secondVal(1:7);
        end
        
        data = data((secondIdx + 1) : end);
        
        currentValuesCell{i, 1} = firstVal;
        currentValuesCell{i, 2} = secondVal;
    end

    % Update display fields with data.
     for i = 1:23
         % Ignore wrist yaw.
         if (i == 5 || i == 10)
             continue;
         end
         
         set(displayHandles{i, 1}, 'String' , currentValuesCell{i, 1});
         set(displayHandles{i, 2}, 'String' , currentValuesCell{i, 2});
     end
end

% Loosen head callback.
function loosenHead()
    % Establish global variables.
    global displayHandles robotIPTB
    
    jointPosStiffString = '';
    
    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end
        
        jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 1}, 'String'), '_');
        
        if (i == 22 || i == 23)
        	jointPosStiffString = strcat(jointPosStiffString, '0.0', '_');
        else
        	jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 2}, 'String'), '_');
        end
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', jointPosStiffString), robotIP);
end

% Loosen right arm callback.
function loosenRArm()
    % Establish global variables.
    global displayHandles robotIPTB
    
    jointPosStiffString = '';
    
    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end
        
        jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 1}, 'String'), '_');
        
        if (i >= 6 && i <= 9)
        	jointPosStiffString = strcat(jointPosStiffString, '0.0', '_');
        else
        	jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 2}, 'String'), '_');
        end
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', jointPosStiffString), robotIP);
end

% Loosen left arm callback.
function loosenLArm()
    % Establish global variables.
    global displayHandles robotIPTB
    
    jointPosStiffString = '';
    
    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end
        
        jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 1}, 'String'), '_');
        
        if (i >= 1 && i <= 4)
        	jointPosStiffString = strcat(jointPosStiffString, '0.0', '_');
        else
        	jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 2}, 'String'), '_');
        end
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', jointPosStiffString), robotIP);
end

% Loosen right leg callback.
function loosenRLeg()
    % Establish global variables.
    global displayHandles robotIPTB
    
    jointPosStiffString = '';
    
    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end
        
        jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 1}, 'String'), '_');
        
        if (i >= 17 && i <= 21)
        	jointPosStiffString = strcat(jointPosStiffString, '0.0', '_');
        else
        	jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 2}, 'String'), '_');
        end
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', jointPosStiffString), robotIP);
end

% Loosen left leg callback.
function loosenLLeg()
    % Establish global variables.
    global displayHandles robotIPTB
    
    jointPosStiffString = '';
    
    for i = 1:23
        % Ignore wrist yaw.
        if (i == 5 || i == 10)
            continue;
        end
        
        jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 1}, 'String'), '_');
        
        if (i >= 11 && i <= 16)
        	jointPosStiffString = strcat(jointPosStiffString, '0.0', '_');
        else
        	jointPosStiffString = strcat(jointPosStiffString, get(displayHandles{i, 2}, 'String'), '_');
        end
    end

    robotIP = get(robotIPTB, 'string');
    TCPComm(strcat('keyframeGen_SETVALUES_', jointPosStiffString), robotIP);
end