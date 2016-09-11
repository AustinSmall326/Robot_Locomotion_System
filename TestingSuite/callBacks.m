function callBacks(src, ~, callBackName)          
    switch callBackName
        case 'rbCallback'
            rbCallback(src);
        case 'loadButtonCallback'
        	loadButtonCallback();
    end 
end

% Callback for radio buttons.
function rbCallback(rbHandle)
    % Establish global variables.
    global testlb foldersLocomotionCell foldersVisionCell foldersLocalizationCell 
    
    % Find all test files relevant to selected module (Locomotion,
    % Localization, Vision).
    moduleName = get(rbHandle, 'string');
    
    switch moduleName
        case 'Locomotion'
            availableTestModules = strcat(foldersLocomotionCell, '.m');
        case 'Vision'
            availableTestModules = strcat(foldersVisionCell, '.m');
        case 'Localization'
            availableTestModules = strcat(foldersLocalizationCell, '.m');
    end
    
    % Load names into test module list box.
    if (size(availableTestModules, 1) > 0)
        set(testlb, 'string', availableTestModules);
    else
        set(testlb, 'string', {});
    end     
end

% Callback for load button, which is tasked with loading the state selected
% in the environment configuration panel.
function loadButtonCallback()
    % Establish global variables.
    global testlb moduleLoaded deloadModuleHandle

    % Check if a module is currently loaded.  If so, unload that module
    % before proceeding.
    if (moduleLoaded)
        deloadModuleHandle();
    end
    
    moduleLoaded = true;
                         
    % Determine which function is to be loaded.
    % At this point, a radio button by default must be selected, so we only
    % need to check that there are items in the list box, which would
    % guarantee that one is selected.
    listBoxStrings = get(testlb, 'string');
    
    if (size(listBoxStrings) < 1)
        disp('There are no test files to load.');
        return;
    end
    
    % Verify that IP addresses are valid.
    %expression = '\d\d\d\.\d\d\d\.\d\d\d\.\d\d\d';
    %regexp(get(computerIPTB, 'String'), expression)
    
    selectedItem   = listBoxStrings{get(testlb, 'Value')};
    functionName   = selectedItem(1:(end - 2));
    
    % Call function to load test module.
    funcHandle = str2func(functionName);
    functionDeloadName = strcat(functionName, '_Deload');
    deloadModuleHandle = str2func(functionDeloadName);
    funcHandle();
end