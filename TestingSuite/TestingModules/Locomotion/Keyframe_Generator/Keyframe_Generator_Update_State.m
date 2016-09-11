function [] = Keyframe_Generator_Update_State(updateMotionLevel, simulateClickMotion, ...
                                              updateKeyframeLevel, simulateClickKeyframes)
    % Establish global variables.
    global motionlb motionTB motionStruct ...
           keyframeslb keyframesTB ...
           editHandles interpolationlb interpolationEditTB
    
    % Changes have been made to motionStruct.  Save those changes to the
    % .mat file.
    save('TestingModules/Locomotion/Keyframe_Generator/motions.mat', 'motionStruct');

    if (updateMotionLevel)
        % Manage items in motion list box.
        set(motionlb, 'String', {motionStruct.name});

        % Mange display text for motion text box.
        set(motionTB, 'String', 'New Motion Name');
        
        % On first call, simulate click on currently selected item to 
        % propate event to entire module.
        if (simulateClickMotion)
            eventData = 'Dummy var';
            Keyframe_Generator_callBacks(motionlb, eventData, 'motionlbClick');
        end
    elseif (updateKeyframeLevel)
        % Manage items in keyframes list box.
        % Check if any motions exist.
        if (size(motionStruct) > 0)
            % Check if there are any keyframes for selected motion.
            selectedMotionIndex = get(motionlb, 'Value');

            if (size(motionStruct(selectedMotionIndex).motions) > 0)
                set(keyframeslb, 'String', {motionStruct(selectedMotionIndex).motions.name});
            else
                set(keyframeslb, 'String', {});
            end
            
            if (simulateClickKeyframes)
                % Check if any keyframes exist.
                if (size(motionStruct(selectedMotionIndex).motions) > 0)
                    % Get selected keyframe index.
                    selectedKeyframeIndex = get(keyframeslb, 'Value');

                    % Update interpolation parameters.
                    % Update selected value of interpolation listbox.
                    interpolationCellArr = get(interpolationlb, 'String');
                                        
                    for i = 1:size(interpolationCellArr)
                        if (strcmp(motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).interpolationType, ...
                                   interpolationCellArr{i}) == 1)
                            set(interpolationlb, 'Value', i);
                        end
                    end
                    
                    % Update duration edit text box.
                    set(interpolationEditTB, 'String', motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).interpolationDuration);
                    
                    % Update/load data to edit text boxes.
                    for i = 1:23
                        set(editHandles{i, 1}, 'String', motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).angles{i});
                        set(editHandles{i, 2}, 'String', motionStruct(selectedMotionIndex).motions(selectedKeyframeIndex).stiffness{i});
                    end            
                else
                    for i = 1:23
                        set(editHandles{i, 1}, 'String', '');
                        set(editHandles{i, 2}, 'String', '');
                    end
                end
            end
        else
            set(keyframeslb, 'String', {});
        end
       
        % Mange display text for motion text box.
        set(keyframesTB, 'String', 'New Keyframe Name');
    end
end