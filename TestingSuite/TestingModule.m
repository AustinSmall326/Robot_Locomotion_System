%% Declare global variables (needed by callBacks).
global selectModuleBG testlb robotIPTB fig figWidth figHeight ...
       mainConfigPanelWidth mainConfigPanelHeight panelPadding lbPadding ...
       estPanelTextHeight foldersLocomotionCell foldersVisionCell foldersLocalizationCell ...
       moduleLoaded
   
addpath('Utils/TCPComputerComm');

% Find all folders in locomotion, vision, and localization.
foldersLocomotionStruct   = dir('TestingModules/Locomotion');
foldersVisionStruct       = dir('TestingModules/Vision');
foldersLocalizationStruct = dir('TestingModues/Localization');

foldersLocomotionCell   = {foldersLocomotionStruct.name};
foldersVisionCell       = {foldersVisionStruct.name};
foldersLocalizationCell = {foldersLocalizationStruct.name};

foldersLocomotionCell   = foldersLocomotionCell([foldersLocomotionStruct.isdir]); % Remove all non-folder names.
foldersVisionCell       = foldersVisionCell([foldersVisionStruct.isdir]);
foldersLocalizationCell = foldersLocalizationCell([foldersLocalizationStruct.isdir]);

% Filter out any folders with names containing . or ..
% Locomotion.
filterVect = [];

for i = 1 : size(foldersLocomotionCell, 2)
    if (strcmp(foldersLocomotionCell{i}, '.') == 1)
        continue;
    elseif (strcmp(foldersLocomotionCell{i}, '..') == 1)
    	continue;
    else
        filterVect = horzcat(filterVect, [i]);
    end
end

foldersLocomotionCell = foldersLocomotionCell(filterVect);

% Vision.
filterVect = [];

for i = 1 : size(foldersVisionCell, 2)
    if (strcmp(foldersVisionCell{i}, '.') == 1)
        continue;
    elseif (strcmp(foldersVisionCell{i}, '..') == 1)
    	continue;
    else
        filterVect = horzcat(filterVect, [i]);
    end
end

foldersVisionCell = foldersVisionCell(filterVect);

% Localization.
filterVect = [];

for i = 1 : size(foldersLocalizationCell, 2)
    if (strcmp(foldersLocalizationCell{i}, '.') == 1)
        continue;
    elseif (strcmp(foldersLocalizationCell{i}, '..') == 1)
    	continue;
    else
        filterVect = horzcat(filterVect, [i]);
    end
end

foldersLocalizationCell = foldersLocalizationCell(filterVect);

for i = 1 : size(foldersLocomotionCell, 2)
    addpath(strcat('TestingModules/Locomotion/', foldersLocomotionCell{i}));
end

for i = 1 : size(foldersVisionCell, 2)
    addpath(strcat('TestingModules/Vision/', foldersVisionCell{i}));
end

for i = 1 : size(foldersLocalizationCell, 2)
    addpath(strcat('TestingModules/Localization/', foldersLocalizationCell{i}));
end
        
%% Set up 'Configure Testing Environment' UIPanel
% Relevant parameters.
figWidth = 850;
figHeight = 550;
figX = 300;
figY = 300;

panelPadding = 15; 
estPanelTextHeight = 20;
rbPadding = 15;
lbPadding = 10;
tbPadding = 10;

mainConfigPanelWidth  = figWidth - 2 * panelPadding + 1;
mainConfigPanelHeight = 150;
mainConfigPanelX      = panelPadding + 1;
mainConfigPanelY      = figHeight - (mainConfigPanelHeight + panelPadding) + estPanelTextHeight / 2;

selectModuleBGWidth  = 150;
selectModuleBGHeight = mainConfigPanelHeight - 2 * panelPadding;
selectModuleBGX      = panelPadding;
selectModuleBGY      = panelPadding;

selectTestPanelWidth  = 200;
selectTestPanelHeight = mainConfigPanelHeight - 2 * panelPadding;
selectTestPanelX      = selectModuleBGX + selectModuleBGWidth + panelPadding;
selectTestPanelY      = panelPadding;

enterIPPanelWidth  = 250;
enterIPPanelHeight = 20 + 2 * tbPadding + estPanelTextHeight / 2 + 1;
enterIPPanelX      = selectTestPanelX + selectTestPanelWidth + panelPadding;
enterIPPanelY      = panelPadding + mainConfigPanelHeight - 2 * panelPadding - enterIPPanelHeight;

rLocomotionWidth  = 100;
rLocomotionHeight = 20;
rLocomotionX      = rbPadding;
rLocomotionY      = selectModuleBGHeight - rLocomotionHeight - rbPadding - estPanelTextHeight / 2;

rLocalizationWidth  = 100;
rLocalizationHeight = 20;
rLocalizationX      = rbPadding;
rLocalizationY      = (0/3) * selectModuleBGHeight + rbPadding;

rVisionWidth  = 100;
rVisionHeight = 20;
rVisionX      = rbPadding;
rVisionY      = rLocalizationY + rbPadding + (rLocomotionY - rLocalizationY - rbPadding) / 2 - rbPadding / 2; 

testlbWidth   = selectTestPanelWidth - 2 * lbPadding - 3;
testlbHeight  = selectTestPanelHeight - 2 * lbPadding - estPanelTextHeight / 2 - 1;
testlbX       = lbPadding;
testlbY       = lbPadding;

robotIPTBWidth   = enterIPPanelWidth - 2 * tbPadding - 2;
robotIPTBHeight  = 20;
robotIPTBX       = tbPadding;
robotIPTBY       = enterIPPanelHeight - 2 * tbPadding - 2 * robotIPTBHeight + tbPadding + robotIPTBHeight - estPanelTextHeight / 2;

loadTestPBWidth  = 100;
loadTestPBHeight = 20;
loadTestPBX      = enterIPPanelX + enterIPPanelWidth + panelPadding;
loadTestPBY      = mainConfigPanelHeight - loadTestPBHeight - panelPadding - estPanelTextHeight / 2;

% Figure window.
fig = figure('Name', 'UPenn Robotic Soccer Testing Module', ...
             'Position', [figX figY figWidth figHeight]);
set(fig,'Resize','off'); % Disable resizing.

% Primary configuration panel.
mainConfigPanel = uipanel('Title', 'Configure Testing Environment', ...
                          'FontSize', 12);
setpixelposition(mainConfigPanel, [mainConfigPanelX mainConfigPanelY ...
                                   mainConfigPanelWidth mainConfigPanelHeight]);

% Select module button group.                               
selectModuleBG = uibuttongroup('Parent', mainConfigPanel,'Title', 'Select Module To Test', ...
                            'FontSize',12);
setpixelposition(selectModuleBG, [selectModuleBGX selectModuleBGY ...
                                  selectModuleBGWidth selectModuleBGHeight]);


rLocomotion = uicontrol(selectModuleBG,'Style',...
                        'radiobutton',...
                        'String','Locomotion', ...
                        'Position',[rLocomotionX rLocomotionY rLocomotionWidth rLocomotionHeight],...
                        'HandleVisibility','off', 'callback', ...
                        {@callBacks, 'rbCallback'});                                
                                 
rVision = uicontrol(selectModuleBG,'Style',...
                    'radiobutton',...
                    'String','Vision',...
                    'Position',[rVisionX rVisionY rVisionWidth rVisionHeight],...
                    'HandleVisibility','off', 'callback', ...
                    {@callBacks, 'rbCallback'});    
                
rLocalization = uicontrol(selectModuleBG,'Style',...
                          'radiobutton',...
                          'String','Localization',...
                          'Position',[rLocalizationX rLocalizationY rLocalizationWidth rLocalizationHeight],...
                          'HandleVisibility','off', 'callback', ...
                          {@callBacks, 'rbCallback'});                 
                          
% Select test UIPanel                                 
selectTestPanel = uipanel('Parent', mainConfigPanel,'Title', 'Select Test To Load', ...
                          'FontSize',12);
setpixelposition(selectTestPanel, [selectTestPanelX selectTestPanelY ...
                                   selectTestPanelWidth selectTestPanelHeight]);
 
testlb = uicontrol('Parent', selectTestPanel,'Style','listbox', ...
                   'String',{}, ...
                   'Position',[testlbX testlbY testlbWidth testlbHeight], ...
                   'FontSize', 10, 'Value',1);
               
% Enter IP addresses UIPanel
enterIPPanel = uipanel('Parent', mainConfigPanel, ...
                       'Title', 'Enter Computer and Robot IP', ...
                       'FontSize',12);
setpixelposition(enterIPPanel, [enterIPPanelX enterIPPanelY ...
                                enterIPPanelWidth enterIPPanelHeight]);
                           
robotIPTB = uicontrol('Parent', enterIPPanel, 'style','edit', 'String', 'Robot IP Address', ...
                      'BackgroundColor', 'white');
setpixelposition(robotIPTB, [robotIPTBX, robotIPTBY, robotIPTBWidth, robotIPTBHeight]);

% Load Test Button
loadTestBtn = uicontrol('Parent', mainConfigPanel, ...
                        'Style','pushbutton','String','Load Test', ...
                        'callback', {@callBacks, 'loadButtonCallback'});            
setpixelposition(loadTestBtn, [loadTestPBX, loadTestPBY, loadTestPBWidth, loadTestPBHeight]);

%% Set up modular test environment.
% Call the callback for a press to the Locomotion radio button (since it is
% selected by default.
eventData = 'Dummy var';
callBacks(rLocomotion, eventData, 'rbCallback');

% Track loaded modules.
moduleLoaded = false;