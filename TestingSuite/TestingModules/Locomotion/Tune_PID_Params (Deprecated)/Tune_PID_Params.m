function [] = Tune_PID_Params()
               
% Declare global variables.
global fig figHeight mainConfigPanelWidth mainConfigPanelHeight ...
       panelPadding estPanelTextHeight selectJointPopup newJointValueEditTB ...
       durationValueEditTB pEditTB iEditTB dEditTB PIDModePopup plotHandle

tbPadding = 13;

% Set current figure.
set(0, 'CurrentFigure', fig);

% Test Module
testModulePanelWidth  = mainConfigPanelWidth;
testModulePanelHeight = figHeight - mainConfigPanelHeight - 2 * panelPadding;
testModulePanelX      = panelPadding + 1;
testModulePanelY      = panelPadding;

testModulePanel = uipanel('Title', 'Tune PID Parameters Module', ...
                          'FontSize', 12);
setpixelposition(testModulePanel, [testModulePanelX testModulePanelY ...
                                   testModulePanelWidth testModulePanelHeight]);

%% Test setup panel.
testSetupPanelWidth  = 300;
testSetupPanelHeight = testModulePanelHeight - 2 * panelPadding;
testSetupPanelX      = panelPadding + 1;
testSetupPanelY      = panelPadding;

testSetupPanel = uipanel('Parent', testModulePanel, 'Title', 'Set Up PID Test', ...
                         'FontSize', 12);
setpixelposition(testSetupPanel, [testSetupPanelX testSetupPanelY ...
                                  testSetupPanelWidth testSetupPanelHeight]);

% Select joint text label.
selectJointLabelTBWidth  = 75;
selectJointLabelTBHeight = 20;
selectJointLabelTBX      = tbPadding;
selectJointLabelTBY      = testSetupPanelHeight - estPanelTextHeight - tbPadding - selectJointLabelTBHeight;


selectJointLabelTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'Select Joint: ', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(selectJointLabelTB, [selectJointLabelTBX selectJointLabelTBY ...
                                      selectJointLabelTBWidth selectJointLabelTBHeight]);   
                              
% Select joint popup.
popupPadding = tbPadding;

selectJointPopupWidth  = testSetupPanelWidth - tbPadding - 2 * popupPadding - selectJointLabelTBWidth;
selectJointPopupHeight = 20;
selectJointPopupX      = selectJointLabelTBX + selectJointLabelTBWidth + popupPadding;
selectJointPopupY      = selectJointLabelTBY;


selectJointPopup = uicontrol('Parent', testSetupPanel, 'Style', 'popup',...
                             'String', {'L ARM SHOULDER PITCH', ...
                                        'L ARM SHOULDER ROLL', ...
                                        'L ARM ELBOW YAW', ...
                                        'L ARM ELBOW ROLL', ...
                                        'L ARM WRIST YAW', ...
                                        'R ARM SHOULDER PITCH', ...
                                        'R ARM SHOULDER ROLL', ...
                                        'R ARM ELBOW YAW', ...
                                        'R ARM ELBOW ROLL', ...
                                        'R ARM WRIST YAW', ...
                                        'L LEG HIP YAW PITCH', ...
                                        'L LEG HIP ROLL', ...
                                        'L LEG HIP PITCH', ...
                                        'L LEG KNEE PITCH', ...
                                        'L LEG ANKLE PITCH', ...
                                        'L LEG ANKLE ROLL', ...
                                        'R LEG HIP YAW PITCH', ...
                                        'R LEG HIP ROLL', ...
                                        'R LEG HIP PITCH', ...
                                        'R LEG KNEE PITCH', ...
                                        'R LEG ANKLE PITCH', ...
                                        'R LEG ANKLE ROLL', ...
                                        'HEAD YAW', ...
                                        'HEAD PITCH'});

setpixelposition(selectJointPopup, [selectJointPopupX selectJointPopupY ...
                                    selectJointPopupWidth selectJointPopupHeight]);

% Updated actuator value.
newJointValueTBWidth  = selectJointLabelTBWidth;
newJointValueTBHeight = selectJointLabelTBHeight;
newJointValueTBX      = selectJointLabelTBX;
newJointValueTBY      = selectJointLabelTBY - newJointValueTBHeight - tbPadding;

newJointValueTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'Joint Angle: ', ...
                            'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(newJointValueTB, [newJointValueTBX newJointValueTBY ...
                                   newJointValueTBWidth newJointValueTBHeight]); 
         
newJointValueEditTBWidth   = testSetupPanelWidth - newJointValueTBWidth - 3 * tbPadding;
newJointValueEditTBHeight  = 20;
newJointValueEditTBX       = newJointValueTBX + newJointValueTBWidth + tbPadding;
newJointValueEditTBY       = newJointValueTBY;

newJointValueEditTB = uicontrol('Parent', testSetupPanel, 'style','edit', 'String', 'Value', ...
                                'BackgroundColor', 'white');
setpixelposition(newJointValueEditTB, [newJointValueEditTBX, newJointValueEditTBY, newJointValueEditTBWidth, newJointValueEditTBHeight]);

% Motion duration value.
durationValueTBWidth  = selectJointLabelTBWidth;
durationValueTBHeight = selectJointLabelTBHeight;
durationValueTBX      = selectJointLabelTBX;
durationValueTBY      = newJointValueTBY - newJointValueTBHeight - tbPadding;

durationValueTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'Duration: ', ...
                            'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(durationValueTB, [durationValueTBX durationValueTBY ...
                                   durationValueTBWidth durationValueTBHeight]); 
         
durationValueEditTBWidth   = testSetupPanelWidth - durationValueTBWidth - 3 * tbPadding;
durationValueEditTBHeight  = 20;
durationValueEditTBX       = durationValueTBX + durationValueTBWidth + tbPadding;
durationValueEditTBY       = durationValueTBY;

durationValueEditTB = uicontrol('Parent', testSetupPanel, 'style','edit', 'String', 'Value', ...
                                'BackgroundColor', 'white');
setpixelposition(durationValueEditTB, [durationValueEditTBX, durationValueEditTBY, durationValueEditTBWidth, newJointValueEditTBHeight]);

% PID proportional text box.
pLabelTBWidth  = selectJointLabelTBWidth;
pLabelTBHeight = selectJointLabelTBHeight;
pLabelTBX      = selectJointLabelTBX;
pLabelTBY      = durationValueTBY - pLabelTBHeight - tbPadding;


pLabelTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'P Gain: ', ...
                     'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(pLabelTB, [pLabelTBX pLabelTBY ...
                            pLabelTBWidth pLabelTBHeight]);   
                          
pEditTBWidth   = testSetupPanelWidth - pLabelTBWidth - 3 * tbPadding;
pEditTBHeight  = 20;
pEditTBX       = pLabelTBX + pLabelTBWidth + tbPadding;
pEditTBY       = pLabelTBY;

pEditTB = uicontrol('Parent', testSetupPanel, 'style','edit', 'String', 'Value', ...
                    'BackgroundColor', 'white');
setpixelposition(pEditTB, [pEditTBX, pEditTBY, pEditTBWidth, pEditTBHeight]);

% PID integral text box.
iLabelTBWidth  = selectJointLabelTBWidth;
iLabelTBHeight = selectJointLabelTBHeight;
iLabelTBX      = selectJointLabelTBX;
iLabelTBY      = pLabelTBY - pLabelTBHeight - tbPadding;

iLabelTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'I Gain: ', ...
                     'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(iLabelTB, [iLabelTBX iLabelTBY ...
                            iLabelTBWidth iLabelTBHeight]);   
    
iEditTBWidth   = testSetupPanelWidth - pLabelTBWidth - 3 * tbPadding;
iEditTBHeight  = 20;
iEditTBX       = iLabelTBX + iLabelTBWidth + tbPadding;
iEditTBY       = iLabelTBY;

iEditTB = uicontrol('Parent', testSetupPanel, 'style','edit', 'String', 'Value', ...
                    'BackgroundColor', 'white');
setpixelposition(iEditTB, [iEditTBX, iEditTBY, iEditTBWidth, iEditTBHeight]);

% PID differential text box.
dLabelTBWidth  = selectJointLabelTBWidth;
dLabelTBHeight = selectJointLabelTBHeight;
dLabelTBX      = selectJointLabelTBX;
dLabelTBY      = iLabelTBY - pLabelTBHeight - tbPadding;

dLabelTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'D Gain: ', ...
                     'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(dLabelTB, [dLabelTBX dLabelTBY ...
                            dLabelTBWidth dLabelTBHeight]);  

dEditTBWidth   = testSetupPanelWidth - pLabelTBWidth - 3 * tbPadding;
dEditTBHeight  = 20;
dEditTBX       = dLabelTBX + dLabelTBWidth + tbPadding;
dEditTBY       = dLabelTBY;

dEditTB = uicontrol('Parent', testSetupPanel, 'style','edit', 'String', 'Value', ...
                    'BackgroundColor', 'white');
setpixelposition(dEditTB, [dEditTBX, dEditTBY, dEditTBWidth, dEditTBHeight]);

% PID mode selection.
PIDModeLabelTBWidth  = selectJointLabelTBWidth;
PIDModeLabelTBHeight = selectJointLabelTBHeight;
PIDModeLabelTBX      = selectJointLabelTBX;
PIDModeLabelTBY      = dLabelTBY - PIDModeLabelTBHeight - tbPadding;

PIDModeLabelTB = uicontrol('Parent', testSetupPanel, 'style','text', 'string', 'PID Mode: ', ...
                           'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(PIDModeLabelTB, [PIDModeLabelTBX PIDModeLabelTBY ...
                                  PIDModeLabelTBWidth PIDModeLabelTBHeight]); 

PIDModePopupWidth  = testSetupPanelWidth - tbPadding - 2 * popupPadding - selectJointLabelTBWidth;
PIDModePopupHeight = 20;
PIDModePopupX      = PIDModeLabelTBX + PIDModeLabelTBWidth + popupPadding;
PIDModePopupY      = PIDModeLabelTBY;


PIDModePopup = uicontrol('Parent', testSetupPanel, 'Style', 'popup', ...
                         'String', {'P', ...
                                    'I', ...
                                    'D', ...
                                    'PI', ...
                                    'ID', ...
                                    'PD', ...
                                    'PID'});

setpixelposition(PIDModePopup, [PIDModePopupX PIDModePopupY ...
                                PIDModePopupWidth PIDModePopupHeight]); 
                            
% Execute PID Test push button.
pbPadding = tbPadding;

executePIDPBWidth  = 150;
executePIDPBHeight = 30;
executePIDPBX      = testSetupPanelWidth / 2 - executePIDPBWidth / 2;
executePIDPBY      = PIDModeLabelTBY - PIDModeLabelTBHeight - 2 * pbPadding;

executePIDPB = uicontrol('Parent', testSetupPanel, ...
                         'Style', 'pushbutton', 'String', 'Execute PID Test', ...
                         'callback', {@Tune_PID_Params_callBacks, 'pbCallback'});            
setpixelposition(executePIDPB, [executePIDPBX, executePIDPBY, executePIDPBWidth, executePIDPBHeight]);

%% Plot results.



dispPlotPanelWidth  = testModulePanelWidth - testSetupPanelWidth - 3 * panelPadding;
dispPlotPanelHeight = testModulePanelHeight - 2 * panelPadding;
dispPlotPanelX      = testSetupPanelX + testSetupPanelWidth + panelPadding;
dispPlotPanelY      = panelPadding;

dispPlotPanel = uipanel('Parent', testModulePanel, 'Title', 'Visualize Results', ...
                        'FontSize', 12, 'BackgroundColor', 'white');
setpixelposition(dispPlotPanel, [dispPlotPanelX dispPlotPanelY ...
                                 dispPlotPanelWidth dispPlotPanelHeight]);

plotHandle = subplot(1, 1, 1, 'Parent', dispPlotPanel);
title('Joint Angle vs. Time');
xlabel('Time (s)');
ylabel('Joint Angle (Degrees)');

end