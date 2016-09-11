function [] = Locomotion_Tune_Parameters()
               
% Declare global variables.
global fig figHeight mainConfigPanelWidth mainConfigPanelHeight ...
       panelPadding CurrentValueDispTB selectParamPopup UpdateValueEditTB

% Set current figure.
set(0, 'CurrentFigure', fig);

estPanelTextHeight = 20;
pbPadding = 25;
tbPadding = 12;

% Test Module
testModulePanelWidth  = mainConfigPanelWidth;
testModulePanelHeight = figHeight - mainConfigPanelHeight - 2 * panelPadding;
testModulePanelX      = panelPadding + 1;
testModulePanelY      = panelPadding;

testModulePanel = uipanel('Title', 'Locomotion Tune Parameters', ...
                          'FontSize', 12);
setpixelposition(testModulePanel, [testModulePanelX testModulePanelY ...
                                   testModulePanelWidth testModulePanelHeight]);

% Start motion.
startMotionPBWidth  = 100;
startMotionPBHeight = 35;
startMotionPBX      = (testModulePanelWidth - 2 * startMotionPBWidth - pbPadding) / 2;
startMotionPBY      = testModulePanelHeight - startMotionPBHeight - pbPadding;

startMotionBtn = uicontrol('Parent', testModulePanel, ...
                           'Style','pushbutton','String','Start Motion', ...
                           'callback', {@Locomotion_Tune_Parameters_callBacks, 'startMotionPBCallback'});            
setpixelposition(startMotionBtn, [startMotionPBX, startMotionPBY, startMotionPBWidth, startMotionPBHeight]);

% Stop motion.
stopMotionPBWidth  = 100;
stopMotionPBHeight = 35;
stopMotionPBX      = (testModulePanelWidth - stopMotionPBWidth - startMotionPBWidth - pbPadding) / 2 + startMotionPBWidth + pbPadding;
stopMotionPBY      = testModulePanelHeight - stopMotionPBHeight - pbPadding;

stopMotionBtn = uicontrol('Parent', testModulePanel, ...
                          'Style','pushbutton','String','Stop Motion', ...
                          'callback', {@Locomotion_Tune_Parameters_callBacks, 'stopMotionPBCallback'});            
setpixelposition(stopMotionBtn, [stopMotionPBX, stopMotionPBY, stopMotionPBWidth, stopMotionPBHeight]);

%% Parameters Panel.
paramPanelWidth  = 300;
paramPanelHeight = 4 * 20 + 5 * tbPadding + estPanelTextHeight;
paramPanelX      = (testModulePanelWidth - paramPanelWidth) / 2;
paramPanelY      = testModulePanelHeight - stopMotionPBHeight - 2 * pbPadding - paramPanelHeight + 15;

paramPanel = uipanel('Parent', testModulePanel, ...
                     'Title', 'Parameters', ...
                     'FontSize', 12);
setpixelposition(paramPanel, [paramPanelX paramPanelY ...
                              paramPanelWidth paramPanelHeight]);

% Parameters popup.
popupLabelTBX       = tbPadding;
popupLabelTBY       = 3 * 20 + 4 * tbPadding;
popupLabelTBWidth   = 100;
popupLabelTBHeight  = 20;

popupLabelTB = uicontrol('Parent', paramPanel, 'style','text', 'string', 'Select Param:', ...
                         'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(popupLabelTB, [popupLabelTBX, popupLabelTBY, ...
                                popupLabelTBWidth, popupLabelTBHeight]);   
       
selectParamPopupWidth  = paramPanelWidth - 2 * tbPadding - popupLabelTBWidth - 2;
selectParamPopupHeight = 20;
selectParamPopupY      = 3 * 20 + 4 * tbPadding;
selectParamPopupX      = 2 * tbPadding + popupLabelTBWidth - 4;

selectParamPopup = uicontrol('Parent', paramPanel, 'Style', 'popup',...
                             'String', {'STEP-HEIGHT', ...
                                        'SWING-LIFT-START-PHASE', ...
                                        'SWING-LIFT-END-PHASE', ...
                                        'STEP-FREQUENCY', ...
                                        'RWEIGHT'});
              
setpixelposition(selectParamPopup, [selectParamPopupX selectParamPopupY ...
                                    selectParamPopupWidth selectParamPopupHeight]);

% Updated parameter value.                                
UpdateValueLabelTBX       = tbPadding;
UpdateValueLabelTBY       = 2 * 20 + 3 * tbPadding;
UpdateValueLabelTBWidth   = 100;
UpdateValueLabelTBHeight  = 20;

UpdateValueLabelTB = uicontrol('Parent', paramPanel, 'style','text', 'string', 'Update Value :', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(UpdateValueLabelTB, [UpdateValueLabelTBX, UpdateValueLabelTBY, ...
                                      UpdateValueLabelTBWidth, UpdateValueLabelTBHeight]);   
                          
UpdateValueEditTBX      = 2 * tbPadding + UpdateValueLabelTBWidth;
UpdateValueEditTBY      = 2 * 20 + 3 * tbPadding;
UpdateValueEditTBWidth  = paramPanelWidth - 3 * tbPadding - UpdateValueLabelTBWidth;
UpdateValueEditTBHeight = 20;

UpdateValueEditTB = uicontrol('Parent', paramPanel, 'style','edit', ...
                              'BackgroundColor', 'white'); 

setpixelposition(UpdateValueEditTB, [UpdateValueEditTBX, UpdateValueEditTBY, ...
                                     UpdateValueEditTBWidth, UpdateValueEditTBHeight]);   

updateValuePBWidth  = UpdateValueEditTBWidth;
updateValuePBHeight = 20;
updateValuePBX      = UpdateValueEditTBX;
updateValuePBY      = 1 * 20 + 2 * tbPadding;

updateValueBtn = uicontrol('Parent', paramPanel, ...
                           'Style','pushbutton','String','Update', ...
                           'callback', {@Locomotion_Tune_Parameters_callBacks, 'updateValueCallback'});            
setpixelposition(updateValueBtn, [updateValuePBX, updateValuePBY, updateValuePBWidth, updateValuePBHeight]);

% Current Parameter Value.
CurrentValueLabelTBX       = tbPadding;
CurrentValueLabelTBY       = 0 * 20 + 1 * tbPadding;
CurrentValueLabelTBWidth   = 100;
CurrentValueLabelTBHeight  = 20;

CurrentValueLabelTB = uicontrol('Parent', paramPanel, 'style','text', 'string', 'Current Value :', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(CurrentValueLabelTB, [CurrentValueLabelTBX, CurrentValueLabelTBY, ...
                                       CurrentValueLabelTBWidth, CurrentValueLabelTBHeight]);   
                                                   
CurrentValueDispTBX       = UpdateValueEditTBX;
CurrentValueDispTBY       = 0 * 20 + 1 * tbPadding;
CurrentValueDispTBWidth   = UpdateValueEditTBWidth;
CurrentValueDispTBHeight  = 20;

CurrentValueDispTB = uicontrol('Parent', paramPanel, 'style','text', ...
                               'BackgroundColor', [.77 .77 .77]);
setpixelposition(CurrentValueDispTB, [CurrentValueDispTBX, CurrentValueDispTBY, ...
                                      CurrentValueDispTBWidth, CurrentValueDispTBHeight]); 
                                  
% Create a timer to update current actuator position/hardness values.
timerHandle = timer('ExecutionMode', 'fixedRate', ...     % Run timer repeatedly
                    'Period', 1, ...                    % Initial period is 1 sec.
                    'TimerFcn', {@Locomotion_Tune_Parameters_callBacks, 'timer'}); % Specify callback
start(timerHandle);
end