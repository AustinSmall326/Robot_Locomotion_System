function [] = Keyframe_Generator()
% Define global variables.
global sliderPanel sliderPanelX sliderPanelY sliderPanelHeight sliderPanelWidth sliderPanelYBtm ...
       overviewPanelHeight fig figHeight mainConfigPanelWidth ...
       mainConfigPanelHeight panelPadding lbPadding estPanelTextHeight ...
       timerHandle editHandles displayHandles motionStruct motionlb motionTB ...
       keyframeslb keyframesTB interpolationlb interpolationEditTB

% Protect global variables.
oldPanelPadding = panelPadding;       
oldLBPadding = lbPadding;

tbPadding = 5;

% Set current figure.
set(0, 'CurrentFigure', fig);

% Test Module
testModulePanelWidth  = mainConfigPanelWidth;
testModulePanelHeight = figHeight - mainConfigPanelHeight - 2 * panelPadding;
testModulePanelX      = panelPadding + 1;
testModulePanelY      = panelPadding;

testModulePanel = uipanel('Title', 'Keyframe Generator', ...
                          'FontSize', 12);
setpixelposition(testModulePanel, [testModulePanelX testModulePanelY ...
                                   testModulePanelWidth testModulePanelHeight]);
                                                       
%% Motion panel.
motionPanelWidth  = 150;
motionPanelHeight = (testModulePanelHeight - 3 * panelPadding) / 2;
motionPanelX      = panelPadding;
motionPanelY      = motionPanelHeight + 2 * panelPadding;

motionPanel = uipanel('Parent', testModulePanel, 'Title', 'Motion', ...
                      'FontSize', 12);
setpixelposition(motionPanel, [motionPanelX motionPanelY ...
                               motionPanelWidth motionPanelHeight]);
                  
% Motion panel buttons.
motionAddPBWidth  = (motionPanelWidth - 4 * lbPadding) / 3 - 1;
motionAddPBHeight = 20;
motionAddPBX      = lbPadding;
motionAddPBY      = lbPadding;

motionAddBtn = uicontrol('Parent', motionPanel, ...
                        'Style', 'pushbutton', 'String', '+', ...
                        'callback', {@Keyframe_Generator_callBacks, 'motionPlus'});            
setpixelposition(motionAddBtn, [motionAddPBX, motionAddPBY, motionAddPBWidth, motionAddPBHeight]);

motionRemPBWidth  = (motionPanelWidth - 4 * lbPadding) / 3 - 1;
motionRemPBHeight = 20;
motionRemPBX      = motionAddPBWidth + 2 * lbPadding;
motionRemPBY      = lbPadding;

motionRemBtn = uicontrol('Parent', motionPanel, ...
                         'Style', 'pushbutton', 'String', '-', ...
                         'callback', {@Keyframe_Generator_callBacks, 'motionMinus'});            
setpixelposition(motionRemBtn, [motionRemPBX, motionRemPBY, motionRemPBWidth, motionRemPBHeight]);

motionEditPBWidth  = (motionPanelWidth - 4 * lbPadding) / 3 - 1;
motionEditPBHeight = 20;
motionEditPBX      = motionRemPBX + motionRemPBWidth + lbPadding;
motionEditPBY      = lbPadding;

motionEditBtn = uicontrol('Parent', motionPanel, ...
                          'FontName', 'wingdings', ...
                          'Style', 'pushbutton', 'String', char(9999), ...
                          'callback', {@Keyframe_Generator_callBacks, 'motionEdit'});            
setpixelposition(motionEditBtn, [motionEditPBX, motionEditPBY, motionEditPBWidth, motionEditPBHeight]);

% Motion panel edit text box.
motiontbWidth   = motionPanelWidth - 2 * lbPadding - 3;
motiontbHeight  = 20 + 1;
motiontbX       = lbPadding;
motiontbY       = 2 * lbPadding + motionRemPBHeight;

motionTB = uicontrol('Parent', motionPanel, 'style','edit', 'String', 'New Motion Name', ...
                     'BackgroundColor', 'white');
setpixelposition(motionTB, [motiontbX, motiontbY, motiontbWidth, motiontbHeight]);

% Motion panel listbox.
motionlbWidth   = motionPanelWidth - 2 * lbPadding - 3;
motionlbHeight  = motionPanelHeight - motionRemPBHeight - 4 * lbPadding - estPanelTextHeight / 2 - motiontbHeight - 2;
motionlbX       = lbPadding;
motionlbY       = 3 * lbPadding + motionRemPBHeight + motiontbHeight;

motionlb = uicontrol('Parent', motionPanel, 'Style', 'listbox', ...
                     'String', {}, ...
                     'Position', [motionlbX motionlbY motionlbWidth motionlbHeight], ...
                     'FontSize', 10, 'Value', 1, ...
                     'callback', {@Keyframe_Generator_callBacks, 'motionlbClick'});            

                 
%% Keyframes panel.
keyframesPanelWidth  = 150;
keyframesPanelHeight = (testModulePanelHeight - 3 * panelPadding) / 2;
keyframesPanelX      = panelPadding;
keyframesPanelY      = panelPadding;

keyframesPanel = uipanel('Parent', testModulePanel, 'Title', 'Keyframes', ...
                         'FontSize', 12);
setpixelposition(keyframesPanel, [keyframesPanelX keyframesPanelY ...
                                  keyframesPanelWidth keyframesPanelHeight]);
  
% Keyframes panel buttons.
keyframesAddPBWidth  = (keyframesPanelWidth - 6 * lbPadding) / 5 - 1;
keyframesAddPBHeight = 20;
keyframesAddPBX      = lbPadding;
keyframesAddPBY      = lbPadding;

keyframesAddBtn = uicontrol('Parent', keyframesPanel, ...
                            'Style', 'pushbutton', 'String', '+', ...
                            'callback', {@Keyframe_Generator_callBacks, 'keyframesPlus'});            
setpixelposition(keyframesAddBtn, [keyframesAddPBX, keyframesAddPBY, keyframesAddPBWidth, keyframesAddPBHeight]);

keyframesRemPBWidth  = (keyframesPanelWidth - 6 * lbPadding) / 5 - 1;
keyframesRemPBHeight = 20;
keyframesRemPBX      = keyframesAddPBWidth + 2 * lbPadding;
keyframesRemPBY      = lbPadding;

keyframesRemBtn = uicontrol('Parent', keyframesPanel, ...
                            'Style', 'pushbutton', 'String', '-', ...
                            'callback', {@Keyframe_Generator_callBacks, 'keyframesMinus'});            
setpixelposition(keyframesRemBtn, [keyframesRemPBX, keyframesRemPBY, keyframesRemPBWidth, keyframesRemPBHeight]);                              
                     
keyframesUpPBWidth  = (keyframesPanelWidth - 6 * lbPadding) / 5 - 1;
keyframesUpPBHeight = 20;
keyframesUpPBX      = 2 * keyframesAddPBWidth + 3 * lbPadding;
keyframesUpPBY      = lbPadding;

keyframesUpBtn = uicontrol('Parent', keyframesPanel, ...
                           'FontName', 'Wingdings', ...
                           'Style', 'pushbutton', 'String', char(8679), ...
                           'callback', {@Keyframe_Generator_callBacks, 'keyframesUp'});            
setpixelposition(keyframesUpBtn, [keyframesUpPBX, keyframesUpPBY, keyframesUpPBWidth, keyframesUpPBHeight]);                              
                     
keyframesDownPBWidth  = (keyframesPanelWidth - 6 * lbPadding) / 5 - 1;
keyframesDownPBHeight = 20;
keyframesDownPBX      = 3 * keyframesAddPBWidth + 4 * lbPadding;
keyframesDownPBY      = lbPadding;

keyframesDownBtn = uicontrol('Parent', keyframesPanel, ...
                             'FontName', 'Wingdings', ...
                             'Style', 'pushbutton', 'String', char(8681), ...
                             'callback', {@Keyframe_Generator_callBacks, 'keyframesDown'});            
setpixelposition(keyframesDownBtn, [keyframesDownPBX, keyframesDownPBY, keyframesDownPBWidth, keyframesDownPBHeight]);                              
                     
keyframesEditPBWidth  = (keyframesPanelWidth - 6 * lbPadding) / 5 - 1;
keyframesEditPBHeight = 20;
keyframesEditPBX      = 4 * keyframesAddPBWidth + 5 * lbPadding;
keyframesEditPBY      = lbPadding;

keyframesEditBtn = uicontrol('Parent', keyframesPanel, ...
                             'FontName', 'Wingdings', ...
                             'Style', 'pushbutton', 'String', char(9999), ...
                             'callback', {@Keyframe_Generator_callBacks, 'keyframesEdit'});            
setpixelposition(keyframesEditBtn, [keyframesEditPBX, keyframesEditPBY, keyframesEditPBWidth, keyframesEditPBHeight]);                              
                    
% Keyframes panel edit text box.
keyframestbWidth   = keyframesPanelWidth - 2 * lbPadding - 3;
keyframestbHeight  = 20 + 1;
keyframestbX       = lbPadding;
keyframestbY       = 2 * lbPadding + keyframesRemPBHeight;

keyframesTB = uicontrol('Parent', keyframesPanel, 'style','edit', 'String', 'New Keyframe Name', ...
                        'BackgroundColor', 'white');
setpixelposition(keyframesTB, [keyframestbX, keyframestbY, keyframestbWidth, keyframestbHeight]);

% Keyframes panel listbox.
keyframeslbWidth   = keyframesPanelWidth - 2 * lbPadding - 3;
keyframeslbHeight  = keyframesPanelHeight - keyframesRemPBHeight - 4 * lbPadding - estPanelTextHeight / 2 - keyframestbHeight - 2;
keyframeslbX       = lbPadding;
keyframeslbY       = 3 * lbPadding + keyframesRemPBHeight + keyframestbHeight;

keyframeslb = uicontrol('Parent', keyframesPanel, 'Style', 'listbox', ...
                        'String', {}, ...
                        'Position', [keyframeslbX keyframeslbY keyframeslbWidth keyframeslbHeight], ...
                        'FontSize', 10, 'Value', 1, ...
                        'callback', {@Keyframe_Generator_callBacks, 'keyframeslbClick'});
                              
%% Interpolation panel.
interpolationPanelWidth  = 150;
interpolationPanelHeight = 100;
interpolationPanelX      = keyframesPanelWidth + 2 * panelPadding;
interpolationPanelY      = testModulePanelHeight - interpolationPanelHeight - panelPadding;

interpolationPanel = uipanel('Parent', testModulePanel, 'Title', 'Interpolation', ...
                             'FontSize', 12);
setpixelposition(interpolationPanel, [interpolationPanelX interpolationPanelY ...
                                      interpolationPanelWidth interpolationPanelHeight]);

% Interpolation panel listbox.
interpolationlbWidth   = interpolationPanelWidth - 2 * lbPadding - 3;
interpolationlbHeight  = interpolationPanelHeight - 3 * lbPadding - estPanelTextHeight / 2 - 1 - 20;
interpolationlbX       = lbPadding;
interpolationlbY       = 2 * lbPadding + 20;

interpolationlb = uicontrol('Parent', interpolationPanel, 'Style', 'listbox', ...
                            'String', {'Linear'}, ...
                            'Position', [interpolationlbX interpolationlbY interpolationlbWidth interpolationlbHeight], ...
                            'FontSize', 10, 'Value', 1, ...
                            'callback', {@Keyframe_Generator_callBacks, 'interpolationlbClick'});            
                    
% Interpolation panel textbox.
interpolationDisptbWidth   = (interpolationPanelWidth - 3 * lbPadding) / 2 - 1;
interpolationDisptbHeight  = 20 - 1;
interpolationDisptbX       = lbPadding;
interpolationDisptbY       = lbPadding + 1;

interpolationDispTB = uicontrol('Parent', interpolationPanel, 'style','text', 'String', 'Duration');
setpixelposition(interpolationDispTB, [interpolationDisptbX, interpolationDisptbY, ...
                                       interpolationDisptbWidth, interpolationDisptbHeight]);   

interpolationEdittbWidth   = (interpolationPanelWidth - 3 * lbPadding) / 2 - 1;
interpolationEdittbHeight  = 20 - 1;
interpolationEdittbX       = 2 * lbPadding + interpolationEdittbWidth;
interpolationEdittbY       = lbPadding + 1;

interpolationEditTB = uicontrol('Parent', interpolationPanel, 'style','edit', ...
                                'BackgroundColor', 'white', 'String', '1.0', ...
                                'callback', {@Keyframe_Generator_callBacks, 'interpolationDurationCallback'});            
            
setpixelposition(interpolationEditTB, [interpolationEdittbX, interpolationEdittbY, ...
                                       interpolationEdittbWidth, interpolationEdittbHeight]);   

%% Commands panel.
commandsPanelWidth  = 150;
commandsPanelHeight = testModulePanelHeight - interpolationPanelHeight - 3 * panelPadding;
commandsPanelX      = keyframesPanelWidth + 2 * panelPadding;
commandsPanelY      = panelPadding;

commandsPanel = uipanel('Parent', testModulePanel, 'Title', 'Commands', ...
                             'FontSize', 12);
setpixelposition(commandsPanel, [commandsPanelX commandsPanelY ...
                                 commandsPanelWidth commandsPanelHeight]);

% Update edit boxes button.
commandsUpdatePBWidth  = commandsPanelWidth - 2 * lbPadding - 2;
commandsUpdatePBHeight = 20;
commandsUpdatePBX      = lbPadding;
commandsUpdatePBY      = commandsPanelHeight - 1 * commandsUpdatePBHeight - 1 * lbPadding - (estPanelTextHeight / 2) - 1;

commandsUpdateBtn = uicontrol('Parent', commandsPanel, ...
                              'Style', 'pushbutton', 'String', 'Update Edit Boxes', ...
                              'callback', {@Keyframe_Generator_callBacks, 'updateEditBoxes'});            
setpixelposition(commandsUpdateBtn, [commandsUpdatePBX, commandsUpdatePBY, commandsUpdatePBWidth, commandsUpdatePBHeight]);

% Load from edit boxes button.
commandsLoadPBWidth  = commandsPanelWidth - 2 * lbPadding - 2;
commandsLoadPBHeight = 20;
commandsLoadPBX      = lbPadding;
commandsLoadPBY      = commandsPanelHeight - 2 * commandsUpdatePBHeight - 2 * lbPadding - (estPanelTextHeight / 2) - 1;

commandsLoadBtn = uicontrol('Parent', commandsPanel, ...
                            'Style', 'pushbutton', 'String', 'Load From Edit Boxes', ...
                            'callback', {@Keyframe_Generator_callBacks, 'loadFromEditBoxes'});            
setpixelposition(commandsLoadBtn, [commandsLoadPBX, commandsLoadPBY, commandsLoadPBWidth, commandsLoadPBHeight]);

% Loosen robot button.
commandsLoosenPBWidth  = commandsPanelWidth - 2 * lbPadding - 2;
commandsLoosenPBHeight = 20;
commandsLoosenPBX      = lbPadding;
commandsLoosenPBY      = commandsPanelHeight - 3 * commandsUpdatePBHeight - 3 * lbPadding - (estPanelTextHeight / 2) - 1;

commandsLoosenBtn = uicontrol('Parent', commandsPanel, ...
                              'Style', 'pushbutton', 'String', 'Loosen Robot', ...
                              'callback', {@Keyframe_Generator_callBacks, 'loosenRobot'});            
setpixelposition(commandsLoosenBtn, [commandsLoosenPBX, commandsLoosenPBY, commandsLoosenPBWidth, commandsLoosenPBHeight]);

% Stiffen robot button.
commandsStiffenPBWidth  = commandsPanelWidth - 2 * lbPadding - 2;
commandsStiffenPBHeight = 20;
commandsStiffenPBX      = lbPadding;
commandsStiffenPBY      = commandsPanelHeight - 4 * commandsUpdatePBHeight - 4 * lbPadding - (estPanelTextHeight / 2) - 1;

commandsStiffenBtn = uicontrol('Parent', commandsPanel, ...
                                'Style', 'pushbutton', 'String', 'Stiffen Robot', ...
                                'callback', {@Keyframe_Generator_callBacks, 'stiffenRobot'});            
setpixelposition(commandsStiffenBtn, [commandsStiffenPBX, commandsStiffenPBY, commandsStiffenPBWidth, commandsStiffenPBHeight]);

% Execute motion button.
commandsExecutePBWidth  = commandsPanelWidth - 2 * lbPadding - 2;
commandsExecutePBHeight = 20;
commandsExecutePBX      = lbPadding;
commandsExecutePBY      = commandsPanelHeight - 5 * commandsUpdatePBHeight - 5 * lbPadding - (estPanelTextHeight / 2) - 1;

commandsExecuteBtn = uicontrol('Parent', commandsPanel, ...
                               'Style', 'pushbutton', 'String', 'Execute Motion', ...
                               'callback', {@Keyframe_Generator_callBacks, 'executeMotionCallback'});            
setpixelposition(commandsExecuteBtn, [commandsExecutePBX, commandsExecutePBY, commandsExecutePBWidth, commandsExecutePBHeight]);

% Generate lua keyframes file button.
commandsGenPBWidth  = commandsPanelWidth - 2 * lbPadding - 2;
commandsGenPBHeight = 20;
commandsGenPBX      = lbPadding;
commandsGenPBY      = commandsPanelHeight - 6 * commandsUpdatePBHeight - 6 * lbPadding - (estPanelTextHeight / 2) - 1;

commandsGenBtn = uicontrol('Parent', commandsPanel, ...
                           'Style', 'pushbutton', 'String', 'Generate Lua File');
                        %'callback', {@callBacks, 'loadButtonCallback'});            
setpixelposition(commandsGenBtn, [commandsGenPBX, commandsGenPBY, commandsGenPBWidth, commandsGenPBHeight]);
           
%% Overview panel.
textPadding = 3;

overviewPanelWidth  = testModulePanelWidth - commandsPanelWidth - keyframesPanelWidth - 4 * panelPadding;
overviewPanelHeight = testModulePanelHeight - 2 * panelPadding;
overviewPanelX      = keyframesPanelWidth + commandsPanelWidth + 3 * panelPadding;
overviewPanelY      = panelPadding;

overviewPanel = uipanel('Parent', testModulePanel, 'Title', 'NAO Overview', ...
                        'FontSize', 12);
setpixelposition(overviewPanel, [overviewPanelX overviewPanelY ...
                                 overviewPanelWidth overviewPanelHeight]);

panelPadding = 6;
           
% Slider container panel.
sliderWidth = 20;

sliderPanelWidth  = overviewPanelWidth + 4 - sliderWidth;

LLegPanelHeight = 7 * 20 + 7 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
RLegPanelHeight = 6 * 20 + 6 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
LArmPanelHeight = 5 * 20 + 5 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
RArmPanelHeight = 5 * 20 + 5 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
HeadPanelHeight = 3 * 20 + 3 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
sliderPanelHeight = LLegPanelHeight + RLegPanelHeight + LArmPanelHeight + RArmPanelHeight + ...
                    HeadPanelHeight + 5 * panelPadding;

sliderPanelX      = 1;
sliderPanelYBtm   = 1;
sliderPanelY      = sliderPanelYBtm - (sliderPanelHeight - (overviewPanelHeight - estPanelTextHeight));
    
sliderHeight = overviewPanelHeight - estPanelTextHeight + 1;
sliderX      = sliderPanelX + sliderPanelWidth - 7;
sliderY      = 0;

sliderPanel = uipanel('Parent', overviewPanel, 'BorderType', 'none');
setpixelposition(sliderPanel, [sliderPanelX sliderPanelY sliderPanelWidth sliderPanelHeight]);

sld = uicontrol('Parent', overviewPanel, ...
                'Style', 'slider',...
                'Min', 0, 'Max', 100, 'Value', 100,...
                'Position', [sliderX sliderY sliderWidth sliderHeight], ...
                'callback', {@Keyframe_Generator_callBacks, 'sliderCallback'});
            
hListener = addlistener(sld, 'Value', 'PostSet', @(s,e) Keyframe_Generator_callBacks(sld, e, 'sliderCallback'));

% LLeg panel.
LLegPanelWidth  = sliderPanelWidth - 2 * panelPadding;
LLegPanelHeight = 7 * 20 + 7 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
LLegPanelX      = panelPadding;
LLegPanelY      = panelPadding;

LLegPanel = uipanel('Parent', sliderPanel, 'Title', 'Left Leg', ...
                    'FontSize', 12);
setpixelposition(LLegPanel, [LLegPanelX LLegPanelY ...
                             LLegPanelWidth LLegPanelHeight]);

% Sensor panel.
LLegSensorPanelX      = panelPadding + 100 + panelPadding;
LLegSensorPanelY      = 2 * panelPadding + 20 + 2;
LLegSensorPanelWidth  = (LLegPanelWidth - 4 * panelPadding - 100) / 2 - 1;
LLegSensorPanelHeight = 6 * 20 + 7 * tbPadding + estPanelTextHeight + 2;


LLegSensorPanel = uipanel('Parent', LLegPanel, 'Title', 'Sensor', ...
                    'FontSize', 12);
setpixelposition(LLegSensorPanel, [LLegSensorPanelX LLegSensorPanelY ...
                                   LLegSensorPanelWidth LLegSensorPanelHeight]);

% Stiffness panel.
LLegStiffnessPanelX      = LLegSensorPanelX + LLegSensorPanelWidth + panelPadding;
LLegStiffnessPanelY      = 2 * panelPadding + 20 + 2;
LLegStiffnessPanelWidth  = (LLegPanelWidth - 4 * panelPadding - 100) / 2 - 1;
LLegStiffnessPanelHeight = 6 * 20 + 7 * tbPadding + estPanelTextHeight + 2;

LLegStiffnessPanel = uipanel('Parent', LLegPanel, 'Title', 'Stiffness', ...
                             'FontSize', 12);
setpixelposition(LLegStiffnessPanel, [LLegStiffnessPanelX LLegStiffnessPanelY ...
                                      LLegStiffnessPanelWidth LLegStiffnessPanelHeight]);

% Loosen left leg button.
loosenLLegPBWidth  = LLegSensorPanelWidth + LLegStiffnessPanelWidth + panelPadding - 2;
loosenLLegPBHeight = 20;
loosenLLegPBX      = LLegSensorPanelX;
loosenLLegPBY      = panelPadding + 2;

loosenLLegBtn = uicontrol('Parent', LLegPanel, ...
                          'Style', 'pushbutton', 'String', 'Loosen Left Leg', ...
                          'callback', {@Keyframe_Generator_callBacks, 'loosenLLeg'});            
setpixelposition(loosenLLegBtn, [loosenLLegPBX, loosenLLegPBY, loosenLLegPBWidth, loosenLLegPBHeight]);
     
% Sensor boxes.
% Hip pitch.
LLegHipPitchSensorEditTBX       = tbPadding;
LLegHipPitchSensorEditTBY       = 5 * 20 + 6 * tbPadding + 2;
LLegHipPitchSensorEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 1;
LLegHipPitchSensorEditTBHeight  = 20;

LLegHipPitchSensorEditTB = uicontrol('Parent', LLegSensorPanel, 'style','edit', ...
                                     'BackgroundColor', 'white', ...
                                     'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            

setpixelposition(LLegHipPitchSensorEditTB, [LLegHipPitchSensorEditTBX, LLegHipPitchSensorEditTBY, ...
                                            LLegHipPitchSensorEditTBWidth, LLegHipPitchSensorEditTBHeight]);   

LLegHipPitchSensorDispTBX       = LLegHipPitchSensorEditTBX + LLegHipPitchSensorEditTBWidth + tbPadding;
LLegHipPitchSensorDispTBY       = 5 * 20 + 6 * tbPadding + 2;
LLegHipPitchSensorDispTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipPitchSensorDispTBHeight  = 20;

LLegHipPitchSensorDispTB = uicontrol('Parent', LLegSensorPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegHipPitchSensorDispTB, [LLegHipPitchSensorDispTBX, LLegHipPitchSensorDispTBY, ...
                                            LLegHipPitchSensorDispTBWidth, LLegHipPitchSensorDispTBHeight]);   

% Hip roll.
LLegHipRollSensorEditTBX       = tbPadding;
LLegHipRollSensorEditTBY       = 4 * 20 + 5 * tbPadding + 2;
LLegHipRollSensorEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipRollSensorEditTBHeight  = 20;

LLegHipRollSensorEditTB = uicontrol('Parent', LLegSensorPanel, 'style','edit', ...
                                    'BackgroundColor', 'white', ...
                                    'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegHipRollSensorEditTB, [LLegHipRollSensorEditTBX, LLegHipRollSensorEditTBY, ...
                                           LLegHipRollSensorEditTBWidth, LLegHipRollSensorEditTBHeight]);   

LLegHipRollSensorDispTBX       = LLegHipRollSensorEditTBX + LLegHipRollSensorEditTBWidth + tbPadding;
LLegHipRollSensorDispTBY       = 4 * 20 + 5 * tbPadding + 2;
LLegHipRollSensorDispTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipRollSensorDispTBHeight  = 20;

LLegHipRollSensorDispTB = uicontrol('Parent', LLegSensorPanel, 'style','text', ...
                                    'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegHipRollSensorDispTB, [LLegHipRollSensorDispTBX, LLegHipRollSensorDispTBY, ...
                                           LLegHipRollSensorDispTBWidth, LLegHipRollSensorDispTBHeight]);   
                                       
% Hip yaw pitch.
LLegHipYawPitchSensorEditTBX       = tbPadding;
LLegHipYawPitchSensorEditTBY       = 3 * 20 + 4 * tbPadding + 2;
LLegHipYawPitchSensorEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipYawPitchSensorEditTBHeight  = 20;

LLegHipYawPitchSensorEditTB = uicontrol('Parent', LLegSensorPanel, 'style','edit', ...
                                        'BackgroundColor', 'white', ...
                                        'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegHipYawPitchSensorEditTB, [LLegHipYawPitchSensorEditTBX, LLegHipYawPitchSensorEditTBY, ...
                                               LLegHipYawPitchSensorEditTBWidth, LLegHipYawPitchSensorEditTBHeight]);   

LLegHipYawPitchSensorDispTBX       = LLegHipYawPitchSensorEditTBX + LLegHipYawPitchSensorEditTBWidth + tbPadding;
LLegHipYawPitchSensorDispTBY       = 3 * 20 + 4 * tbPadding + 2;
LLegHipYawPitchSensorDispTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipYawPitchSensorDispTBHeight  = 20;

LLegHipYawPitchSensorDispTB = uicontrol('Parent', LLegSensorPanel, 'style','text', ...
                                        'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegHipYawPitchSensorDispTB, [LLegHipYawPitchSensorDispTBX, LLegHipYawPitchSensorDispTBY, ...
                                               LLegHipYawPitchSensorDispTBWidth, LLegHipYawPitchSensorDispTBHeight]);   
           
% Knee pitch.
LLegKneePitchSensorEditTBX       = tbPadding;
LLegKneePitchSensorEditTBY       = 2 * 20 + 3 * tbPadding + 2;
LLegKneePitchSensorEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegKneePitchSensorEditTBHeight  = 20;

LLegKneePitchSensorEditTB = uicontrol('Parent', LLegSensorPanel, 'style','edit', ...
                                      'BackgroundColor', 'white', ...
                                      'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegKneePitchSensorEditTB, [LLegKneePitchSensorEditTBX, LLegKneePitchSensorEditTBY, ...
                                             LLegKneePitchSensorEditTBWidth, LLegKneePitchSensorEditTBHeight]);   

LLegKneePitchSensorDispTBX       = LLegKneePitchSensorEditTBX + LLegKneePitchSensorEditTBWidth + tbPadding;
LLegKneePitchSensorDispTBY       = 2 * 20 + 3 * tbPadding + 2;
LLegKneePitchSensorDispTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegKneePitchSensorDispTBHeight  = 20;

LLegKneePitchSensorDispTB = uicontrol('Parent', LLegSensorPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegKneePitchSensorDispTB, [LLegKneePitchSensorDispTBX, LLegKneePitchSensorDispTBY, ...
                                             LLegKneePitchSensorDispTBWidth, LLegKneePitchSensorDispTBHeight]);   
                                          
% Ankle pitch.
LLegAnklePitchSensorEditTBX       = tbPadding;
LLegAnklePitchSensorEditTBY       = 20 + 2 * tbPadding + 2;
LLegAnklePitchSensorEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnklePitchSensorEditTBHeight  = 20;

LLegAnklePitchSensorEditTB = uicontrol('Parent', LLegSensorPanel, 'style','edit', ...
                                       'BackgroundColor', 'white', ...
                                       'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegAnklePitchSensorEditTB, [LLegAnklePitchSensorEditTBX, LLegAnklePitchSensorEditTBY, ...
                                              LLegAnklePitchSensorEditTBWidth, LLegAnklePitchSensorEditTBHeight]);   

LLegAnklePitchSensorDispTBX       = LLegAnklePitchSensorEditTBX + LLegAnklePitchSensorEditTBWidth + tbPadding;
LLegAnklePitchSensorDispTBY       = 20 + 2 * tbPadding + 2;
LLegAnklePitchSensorDispTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnklePitchSensorDispTBHeight  = 20;

LLegAnklePitchSensorDispTB = uicontrol('Parent', LLegSensorPanel, 'style','text', ...
                                       'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegAnklePitchSensorDispTB, [LLegAnklePitchSensorDispTBX, LLegAnklePitchSensorDispTBY, ...
                                              LLegAnklePitchSensorDispTBWidth, LLegAnklePitchSensorDispTBHeight]);   
    
% Ankle roll.
LLegAnkleRollSensorEditTBX       = tbPadding;
LLegAnkleRollSensorEditTBY       = tbPadding + 2;
LLegAnkleRollSensorEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnkleRollSensorEditTBHeight  = 20;

LLegAnkleRollSensorEditTB = uicontrol('Parent', LLegSensorPanel, 'style','edit', ...
                                      'BackgroundColor', 'white', ...
                                      'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegAnkleRollSensorEditTB, [LLegAnkleRollSensorEditTBX, LLegAnkleRollSensorEditTBY, ...
                                             LLegAnkleRollSensorEditTBWidth, LLegAnkleRollSensorEditTBHeight]);   

LLegAnkleRollSensorDispTBX       = LLegAnkleRollSensorEditTBX + LLegAnkleRollSensorEditTBWidth + tbPadding;
LLegAnkleRollSensorDispTBY       = tbPadding + 2;
LLegAnkleRollSensorDispTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnkleRollSensorDispTBHeight  = 20;

LLegAnkleRollSensorDispTB = uicontrol('Parent', LLegSensorPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegAnkleRollSensorDispTB, [LLegAnkleRollSensorDispTBX, LLegAnkleRollSensorDispTBY, ...
                                             LLegAnkleRollSensorDispTBWidth, LLegAnkleRollSensorDispTBHeight]);   

% Stiffness boxes.
% Hip pitch.
LLegHipPitchStiffnessEditTBX       = tbPadding;
LLegHipPitchStiffnessEditTBY       = 5 * 20 + 6 * tbPadding + 2;
LLegHipPitchStiffnessEditTBWidth   = (LLegSensorPanelWidth - 3 * tbPadding) / 2 - 1;
LLegHipPitchStiffnessEditTBHeight  = 20;

LLegHipPitchStiffnessEditTB = uicontrol('Parent', LLegStiffnessPanel, 'style','edit', ...
                                        'BackgroundColor', 'white', ...
                                        'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegHipPitchStiffnessEditTB, [LLegHipPitchStiffnessEditTBX, LLegHipPitchStiffnessEditTBY, ...
                                               LLegHipPitchStiffnessEditTBWidth, LLegHipPitchStiffnessEditTBHeight]);   

LLegHipPitchStiffnessDispTBX       = LLegHipPitchStiffnessEditTBX + LLegHipPitchStiffnessEditTBWidth + tbPadding;
LLegHipPitchStiffnessDispTBY       = 5 * 20 + 6 * tbPadding + 2;
LLegHipPitchStiffnessDispTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipPitchStiffnessDispTBHeight  = 20;

LLegHipPitchStiffnessDispTB = uicontrol('Parent', LLegStiffnessPanel, 'style','text', ...
                                        'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegHipPitchStiffnessDispTB, [LLegHipPitchStiffnessDispTBX, LLegHipPitchStiffnessDispTBY, ...
                                               LLegHipPitchStiffnessDispTBWidth, LLegHipPitchStiffnessDispTBHeight]);   

% Hip roll.
LLegHipRollStiffnessEditTBX       = tbPadding;
LLegHipRollStiffnessEditTBY       = 4 * 20 + 5 * tbPadding + 2;
LLegHipRollStiffnessEditTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipRollStiffnessEditTBHeight  = 20;

LLegHipRollStiffnessEditTB = uicontrol('Parent', LLegStiffnessPanel, 'style','edit', ...
                                       'BackgroundColor', 'white', ...
                                       'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegHipRollStiffnessEditTB, [LLegHipRollStiffnessEditTBX, LLegHipRollStiffnessEditTBY, ...
                                              LLegHipRollStiffnessEditTBWidth, LLegHipRollStiffnessEditTBHeight]);   

LLegHipRollStiffnessDispTBX       = LLegHipRollStiffnessEditTBX + LLegHipRollStiffnessEditTBWidth + tbPadding;
LLegHipRollStiffnessDispTBY       = 4 * 20 + 5 * tbPadding + 2;
LLegHipRollStiffnessDispTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipRollStiffnessDispTBHeight  = 20;

LLegHipRollStiffnessDispTB = uicontrol('Parent', LLegStiffnessPanel, 'style','text', ...
                                    'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegHipRollStiffnessDispTB, [LLegHipRollStiffnessDispTBX, LLegHipRollStiffnessDispTBY, ...
                                              LLegHipRollStiffnessDispTBWidth, LLegHipRollStiffnessDispTBHeight]);   
                                       
% Hip yaw pitch.
LLegHipYawPitchStiffnessEditTBX       = tbPadding;
LLegHipYawPitchStiffnessEditTBY       = 3 * 20 + 4 * tbPadding + 2;
LLegHipYawPitchStiffnessEditTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipYawPitchStiffnessEditTBHeight  = 20;

LLegHipYawPitchStiffnessEditTB = uicontrol('Parent', LLegStiffnessPanel, 'style','edit', ...
                                           'BackgroundColor', 'white', ...
                                           'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegHipYawPitchStiffnessEditTB, [LLegHipYawPitchStiffnessEditTBX, LLegHipYawPitchStiffnessEditTBY, ...
                                                  LLegHipYawPitchStiffnessEditTBWidth, LLegHipYawPitchStiffnessEditTBHeight]);   

LLegHipYawPitchStiffnessDispTBX       = LLegHipYawPitchStiffnessEditTBX + LLegHipYawPitchStiffnessEditTBWidth + tbPadding;
LLegHipYawPitchStiffnessDispTBY       = 3 * 20 + 4 * tbPadding + 2;
LLegHipYawPitchStiffnessDispTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegHipYawPitchStiffnessDispTBHeight  = 20;

LLegHipYawPitchStiffnessDispTB = uicontrol('Parent', LLegStiffnessPanel, 'style','text', ...
                                        'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegHipYawPitchStiffnessDispTB, [LLegHipYawPitchStiffnessDispTBX, LLegHipYawPitchStiffnessDispTBY, ...
                                                  LLegHipYawPitchStiffnessDispTBWidth, LLegHipYawPitchStiffnessDispTBHeight]);   
                                                  
% Knee pitch.
LLegKneePitchStiffnessEditTBX       = tbPadding;
LLegKneePitchStiffnessEditTBY       = 2 * 20 + 3 * tbPadding + 2;
LLegKneePitchStiffnessEditTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegKneePitchStiffnessEditTBHeight  = 20;

LLegKneePitchStiffnessEditTB = uicontrol('Parent', LLegStiffnessPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegKneePitchStiffnessEditTB, [LLegKneePitchStiffnessEditTBX, LLegKneePitchStiffnessEditTBY, ...
                                                LLegKneePitchStiffnessEditTBWidth, LLegKneePitchStiffnessEditTBHeight]);   

LLegKneePitchStiffnessDispTBX       = LLegKneePitchStiffnessEditTBX + LLegKneePitchStiffnessEditTBWidth + tbPadding;
LLegKneePitchStiffnessDispTBY       = 2 * 20 + 3 * tbPadding + 2;
LLegKneePitchStiffnessDispTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegKneePitchStiffnessDispTBHeight  = 20;

LLegKneePitchStiffnessDispTB = uicontrol('Parent', LLegStiffnessPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegKneePitchStiffnessDispTB, [LLegKneePitchStiffnessDispTBX, LLegKneePitchStiffnessDispTBY, ...
                                                LLegKneePitchStiffnessDispTBWidth, LLegKneePitchStiffnessDispTBHeight]);   
                                          
% Ankle pitch.
LLegAnklePitchStiffnessEditTBX       = tbPadding;
LLegAnklePitchStiffnessEditTBY       = 20 + 2 * tbPadding + 2;
LLegAnklePitchStiffnessEditTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnklePitchStiffnessEditTBHeight  = 20;

LLegAnklePitchStiffnessEditTB = uicontrol('Parent', LLegStiffnessPanel, 'style','edit', ...
                                          'BackgroundColor', 'white', ...
                                          'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegAnklePitchStiffnessEditTB, [LLegAnklePitchStiffnessEditTBX, LLegAnklePitchStiffnessEditTBY, ...
                                                 LLegAnklePitchStiffnessEditTBWidth, LLegAnklePitchStiffnessEditTBHeight]);   

LLegAnklePitchStiffnessDispTBX       = LLegAnklePitchStiffnessEditTBX + LLegAnklePitchStiffnessEditTBWidth + tbPadding;
LLegAnklePitchStiffnessDispTBY       = 20 + 2 * tbPadding + 2;
LLegAnklePitchStiffnessDispTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnklePitchStiffnessDispTBHeight  = 20;

LLegAnklePitchStiffnessDispTB = uicontrol('Parent', LLegStiffnessPanel, 'style','text', ...
                                          'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegAnklePitchStiffnessDispTB, [LLegAnklePitchStiffnessDispTBX, LLegAnklePitchStiffnessDispTBY, ...
                                                 LLegAnklePitchStiffnessDispTBWidth, LLegAnklePitchStiffnessDispTBHeight]);   
    
% Ankle roll.
LLegAnkleRollStiffnessEditTBX       = tbPadding;
LLegAnkleRollStiffnessEditTBY       = tbPadding + 2;
LLegAnkleRollStiffnessEditTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnkleRollStiffnessEditTBHeight  = 20;

LLegAnkleRollStiffnessEditTB = uicontrol('Parent', LLegStiffnessPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LLegAnkleRollStiffnessEditTB, [LLegAnkleRollStiffnessEditTBX, LLegAnkleRollStiffnessEditTBY, ...
                                                LLegAnkleRollStiffnessEditTBWidth, LLegAnkleRollStiffnessEditTBHeight]);   

LLegAnkleRollStiffnessDispTBX       = LLegAnkleRollStiffnessEditTBX + LLegAnkleRollStiffnessEditTBWidth + tbPadding;
LLegAnkleRollStiffnessDispTBY       = tbPadding + 2;
LLegAnkleRollStiffnessDispTBWidth   = (LLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LLegAnkleRollStiffnessDispTBHeight  = 20;

LLegAnkleRollStiffnessDispTB = uicontrol('Parent', LLegStiffnessPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(LLegAnkleRollStiffnessDispTB, [LLegAnkleRollStiffnessDispTBX, LLegAnkleRollStiffnessDispTBY, ...
                                                LLegAnkleRollStiffnessDispTBWidth, LLegAnkleRollStiffnessDispTBHeight]);   
    
% RLeg panel.
RLegPanelWidth  = sliderPanelWidth - 2 * panelPadding;
RLegPanelHeight = 6 * 20 + 6 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
RLegPanelX      = panelPadding;
RLegPanelY      = LLegPanelY + LLegPanelHeight + panelPadding;

RLegPanel = uipanel('Parent', sliderPanel, 'Title', 'Right Leg', ...
                    'FontSize', 12);
setpixelposition(RLegPanel, [RLegPanelX RLegPanelY ...
                             RLegPanelWidth RLegPanelHeight]);

% Sensor panel.
RLegSensorPanelX      = panelPadding + 100 + panelPadding;
RLegSensorPanelY      = 2 * panelPadding + 20 + 2;
RLegSensorPanelWidth  = (RLegPanelWidth - 4 * panelPadding - 100) / 2 - 1;
RLegSensorPanelHeight = 5 * 20 + 6 * tbPadding + estPanelTextHeight + 2;


RLegSensorPanel = uipanel('Parent', RLegPanel, 'Title', 'Sensor', ...
                    'FontSize', 12);
setpixelposition(RLegSensorPanel, [RLegSensorPanelX RLegSensorPanelY ...
                                   RLegSensorPanelWidth RLegSensorPanelHeight]);

% Stiffness panel.
RLegStiffnessPanelX      = RLegSensorPanelX + RLegSensorPanelWidth + panelPadding;
RLegStiffnessPanelY      = 2 * panelPadding + 20 + 2;
RLegStiffnessPanelWidth  = (RLegPanelWidth - 4 * panelPadding - 100) / 2 - 1;
RLegStiffnessPanelHeight = 5 * 20 + 6 * tbPadding + estPanelTextHeight + 2;

RLegStiffnessPanel = uipanel('Parent', RLegPanel, 'Title', 'Stiffness', ...
                             'FontSize', 12);
setpixelposition(RLegStiffnessPanel, [RLegStiffnessPanelX RLegStiffnessPanelY ...
                                      RLegStiffnessPanelWidth RLegStiffnessPanelHeight]);

% Loosen right leg button.
loosenRLegPBWidth  = RLegSensorPanelWidth + RLegStiffnessPanelWidth + panelPadding - 2;
loosenRLegPBHeight = 20;
loosenRLegPBX      = RLegSensorPanelX;
loosenRLegPBY      = panelPadding + 2;

loosenRLegBtn = uicontrol('Parent', RLegPanel, ...
                          'Style', 'pushbutton', 'String', 'Loosen Right Leg', ...
                          'callback', {@Keyframe_Generator_callBacks, 'loosenRLeg'});            
setpixelposition(loosenRLegBtn, [loosenRLegPBX, loosenRLegPBY, loosenRLegPBWidth, loosenRLegPBHeight]);
                                  
% Sensor boxes.
% Hip pitch.
RLegHipPitchSensorEditTBX       = tbPadding;
RLegHipPitchSensorEditTBY       = 4 * 20 + 5 * tbPadding + 2;
RLegHipPitchSensorEditTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 1;
RLegHipPitchSensorEditTBHeight  = 20;

RLegHipPitchSensorEditTB = uicontrol('Parent', RLegSensorPanel, 'style','edit', ...
                                     'BackgroundColor', 'white', ...
                                     'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegHipPitchSensorEditTB, [RLegHipPitchSensorEditTBX, RLegHipPitchSensorEditTBY, ...
                                            RLegHipPitchSensorEditTBWidth, RLegHipPitchSensorEditTBHeight]);   

RLegHipPitchSensorDispTBX       = RLegHipPitchSensorEditTBX + RLegHipPitchSensorEditTBWidth + tbPadding;
RLegHipPitchSensorDispTBY       = 4 * 20 + 5 * tbPadding + 2;
RLegHipPitchSensorDispTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegHipPitchSensorDispTBHeight  = 20;

RLegHipPitchSensorDispTB = uicontrol('Parent', RLegSensorPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegHipPitchSensorDispTB, [RLegHipPitchSensorDispTBX, RLegHipPitchSensorDispTBY, ...
                                            RLegHipPitchSensorDispTBWidth, RLegHipPitchSensorDispTBHeight]);   

% Hip roll.
RLegHipRollSensorEditTBX       = tbPadding;
RLegHipRollSensorEditTBY       = 3 * 20 + 4 * tbPadding + 2;
RLegHipRollSensorEditTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegHipRollSensorEditTBHeight  = 20;

RLegHipRollSensorEditTB = uicontrol('Parent', RLegSensorPanel, 'style','edit', ...
                                    'BackgroundColor', 'white', ...
                                    'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegHipRollSensorEditTB, [RLegHipRollSensorEditTBX, RLegHipRollSensorEditTBY, ...
                                           RLegHipRollSensorEditTBWidth, RLegHipRollSensorEditTBHeight]);   

RLegHipRollSensorDispTBX       = RLegHipRollSensorEditTBX + RLegHipRollSensorEditTBWidth + tbPadding;
RLegHipRollSensorDispTBY       = 3 * 20 + 4 * tbPadding + 2;
RLegHipRollSensorDispTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegHipRollSensorDispTBHeight  = 20;

RLegHipRollSensorDispTB = uicontrol('Parent', RLegSensorPanel, 'style','text', ...
                                    'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegHipRollSensorDispTB, [RLegHipRollSensorDispTBX, RLegHipRollSensorDispTBY, ...
                                           RLegHipRollSensorDispTBWidth, RLegHipRollSensorDispTBHeight]);   
                                              
% Knee pitch.
RLegKneePitchSensorEditTBX       = tbPadding;
RLegKneePitchSensorEditTBY       = 2 * 20 + 3 * tbPadding + 2;
RLegKneePitchSensorEditTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegKneePitchSensorEditTBHeight  = 20;

RLegKneePitchSensorEditTB = uicontrol('Parent', RLegSensorPanel, 'style','edit', ...
                                      'BackgroundColor', 'white', ...
                                      'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegKneePitchSensorEditTB, [RLegKneePitchSensorEditTBX, RLegKneePitchSensorEditTBY, ...
                                             RLegKneePitchSensorEditTBWidth, RLegKneePitchSensorEditTBHeight]);   

RLegKneePitchSensorDispTBX       = RLegKneePitchSensorEditTBX + RLegKneePitchSensorEditTBWidth + tbPadding;
RLegKneePitchSensorDispTBY       = 2 * 20 + 3 * tbPadding + 2;
RLegKneePitchSensorDispTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegKneePitchSensorDispTBHeight  = 20;

RLegKneePitchSensorDispTB = uicontrol('Parent', RLegSensorPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegKneePitchSensorDispTB, [RLegKneePitchSensorDispTBX, RLegKneePitchSensorDispTBY, ...
                                             RLegKneePitchSensorDispTBWidth, RLegKneePitchSensorDispTBHeight]);   
                                          
% Ankle pitch.
RLegAnklePitchSensorEditTBX       = tbPadding;
RLegAnklePitchSensorEditTBY       = 20 + 2 * tbPadding + 2;
RLegAnklePitchSensorEditTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnklePitchSensorEditTBHeight  = 20;

RLegAnklePitchSensorEditTB = uicontrol('Parent', RLegSensorPanel, 'style','edit', ...
                                       'BackgroundColor', 'white', ...
                                       'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegAnklePitchSensorEditTB, [RLegAnklePitchSensorEditTBX, RLegAnklePitchSensorEditTBY, ...
                                              RLegAnklePitchSensorEditTBWidth, RLegAnklePitchSensorEditTBHeight]);   

RLegAnklePitchSensorDispTBX       = RLegAnklePitchSensorEditTBX + RLegAnklePitchSensorEditTBWidth + tbPadding;
RLegAnklePitchSensorDispTBY       = 20 + 2 * tbPadding + 2;
RLegAnklePitchSensorDispTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnklePitchSensorDispTBHeight  = 20;

RLegAnklePitchSensorDispTB = uicontrol('Parent', RLegSensorPanel, 'style','text', ...
                                       'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegAnklePitchSensorDispTB, [RLegAnklePitchSensorDispTBX, RLegAnklePitchSensorDispTBY, ...
                                              RLegAnklePitchSensorDispTBWidth, RLegAnklePitchSensorDispTBHeight]);   
    
% Ankle roll.
RLegAnkleRollSensorEditTBX       = tbPadding;
RLegAnkleRollSensorEditTBY       = tbPadding + 2;
RLegAnkleRollSensorEditTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnkleRollSensorEditTBHeight  = 20;

RLegAnkleRollSensorEditTB = uicontrol('Parent', RLegSensorPanel, 'style','edit', ...
                                      'BackgroundColor', 'white', ...
                                      'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegAnkleRollSensorEditTB, [RLegAnkleRollSensorEditTBX, RLegAnkleRollSensorEditTBY, ...
                                             RLegAnkleRollSensorEditTBWidth, RLegAnkleRollSensorEditTBHeight]);   

RLegAnkleRollSensorDispTBX       = RLegAnkleRollSensorEditTBX + RLegAnkleRollSensorEditTBWidth + tbPadding;
RLegAnkleRollSensorDispTBY       = tbPadding + 2;
RLegAnkleRollSensorDispTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnkleRollSensorDispTBHeight  = 20;

RLegAnkleRollSensorDispTB = uicontrol('Parent', RLegSensorPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegAnkleRollSensorDispTB, [RLegAnkleRollSensorDispTBX, RLegAnkleRollSensorDispTBY, ...
                                             RLegAnkleRollSensorDispTBWidth, RLegAnkleRollSensorDispTBHeight]);   

% Stiffness boxes.
% Hip pitch.
RLegHipPitchStiffnessEditTBX       = tbPadding;
RLegHipPitchStiffnessEditTBY       = 4 * 20 + 5 * tbPadding + 2;
RLegHipPitchStiffnessEditTBWidth   = (RLegSensorPanelWidth - 3 * tbPadding) / 2 - 1;
RLegHipPitchStiffnessEditTBHeight  = 20;

RLegHipPitchStiffnessEditTB = uicontrol('Parent', RLegStiffnessPanel, 'style','edit', ...
                                        'BackgroundColor', 'white', ...
                                        'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegHipPitchStiffnessEditTB, [RLegHipPitchStiffnessEditTBX, RLegHipPitchStiffnessEditTBY, ...
                                               RLegHipPitchStiffnessEditTBWidth, RLegHipPitchStiffnessEditTBHeight]);   

RLegHipPitchStiffnessDispTBX       = RLegHipPitchStiffnessEditTBX + RLegHipPitchStiffnessEditTBWidth + tbPadding;
RLegHipPitchStiffnessDispTBY       = 4 * 20 + 5 * tbPadding + 2;
RLegHipPitchStiffnessDispTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegHipPitchStiffnessDispTBHeight  = 20;

RLegHipPitchStiffnessDispTB = uicontrol('Parent', RLegStiffnessPanel, 'style','text', ...
                                        'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegHipPitchStiffnessDispTB, [RLegHipPitchStiffnessDispTBX, RLegHipPitchStiffnessDispTBY, ...
                                               RLegHipPitchStiffnessDispTBWidth, RLegHipPitchStiffnessDispTBHeight]);   

% Hip roll.
RLegHipRollStiffnessEditTBX       = tbPadding;
RLegHipRollStiffnessEditTBY       = 3 * 20 + 4 * tbPadding + 2;
RLegHipRollStiffnessEditTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegHipRollStiffnessEditTBHeight  = 20;

RLegHipRollStiffnessEditTB = uicontrol('Parent', RLegStiffnessPanel, 'style','edit', ...
                                       'BackgroundColor', 'white', ...
                                       'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegHipRollStiffnessEditTB, [RLegHipRollStiffnessEditTBX, RLegHipRollStiffnessEditTBY, ...
                                              RLegHipRollStiffnessEditTBWidth, RLegHipRollStiffnessEditTBHeight]);   

RLegHipRollStiffnessDispTBX       = RLegHipRollStiffnessEditTBX + RLegHipRollStiffnessEditTBWidth + tbPadding;
RLegHipRollStiffnessDispTBY       = 3 * 20 + 4 * tbPadding + 2;
RLegHipRollStiffnessDispTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegHipRollStiffnessDispTBHeight  = 20;

RLegHipRollStiffnessDispTB = uicontrol('Parent', RLegStiffnessPanel, 'style','text', ...
                                    'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegHipRollStiffnessDispTB, [RLegHipRollStiffnessDispTBX, RLegHipRollStiffnessDispTBY, ...
                                              RLegHipRollStiffnessDispTBWidth, RLegHipRollStiffnessDispTBHeight]);   
                                                             
% Knee pitch.
RLegKneePitchStiffnessEditTBX       = tbPadding;
RLegKneePitchStiffnessEditTBY       = 2 * 20 + 3 * tbPadding + 2;
RLegKneePitchStiffnessEditTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegKneePitchStiffnessEditTBHeight  = 20;

RLegKneePitchStiffnessEditTB = uicontrol('Parent', RLegStiffnessPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegKneePitchStiffnessEditTB, [RLegKneePitchStiffnessEditTBX, RLegKneePitchStiffnessEditTBY, ...
                                                RLegKneePitchStiffnessEditTBWidth, RLegKneePitchStiffnessEditTBHeight]);   

RLegKneePitchStiffnessDispTBX       = RLegKneePitchStiffnessEditTBX + RLegKneePitchStiffnessEditTBWidth + tbPadding;
RLegKneePitchStiffnessDispTBY       = 2 * 20 + 3 * tbPadding + 2;
RLegKneePitchStiffnessDispTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegKneePitchStiffnessDispTBHeight  = 20;

RLegKneePitchStiffnessDispTB = uicontrol('Parent', RLegStiffnessPanel, 'style','text', ...
                                         'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegKneePitchStiffnessDispTB, [RLegKneePitchStiffnessDispTBX, RLegKneePitchStiffnessDispTBY, ...
                                                RLegKneePitchStiffnessDispTBWidth, RLegKneePitchStiffnessDispTBHeight]);   
                                          
% Ankle pitch.
RLegAnklePitchStiffnessEditTBX       = tbPadding;
RLegAnklePitchStiffnessEditTBY       = 20 + 2 * tbPadding + 2;
RLegAnklePitchStiffnessEditTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnklePitchStiffnessEditTBHeight  = 20;

RLegAnklePitchStiffnessEditTB = uicontrol('Parent', RLegStiffnessPanel, 'style','edit', ...
                                          'BackgroundColor', 'white', ...
                                          'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegAnklePitchStiffnessEditTB, [RLegAnklePitchStiffnessEditTBX, RLegAnklePitchStiffnessEditTBY, ...
                                                 RLegAnklePitchStiffnessEditTBWidth, RLegAnklePitchStiffnessEditTBHeight]);   

RLegAnklePitchStiffnessDispTBX       = RLegAnklePitchStiffnessEditTBX + RLegAnklePitchStiffnessEditTBWidth + tbPadding;
RLegAnklePitchStiffnessDispTBY       = 20 + 2 * tbPadding + 2;
RLegAnklePitchStiffnessDispTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnklePitchStiffnessDispTBHeight  = 20;

RLegAnklePitchStiffnessDispTB = uicontrol('Parent', RLegStiffnessPanel, 'style','text', ...
                                          'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegAnklePitchStiffnessDispTB, [RLegAnklePitchStiffnessDispTBX, RLegAnklePitchStiffnessDispTBY, ...
                                                 RLegAnklePitchStiffnessDispTBWidth, RLegAnklePitchStiffnessDispTBHeight]);   
    
% Ankle roll.
RLegAnkleRollStiffnessEditTBX       = tbPadding;
RLegAnkleRollStiffnessEditTBY       = tbPadding + 2;
RLegAnkleRollStiffnessEditTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnkleRollStiffnessEditTBHeight  = 20;

RLegAnkleRollStiffnessEditTB = uicontrol('Parent', RLegStiffnessPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RLegAnkleRollStiffnessEditTB, [RLegAnkleRollStiffnessEditTBX, RLegAnkleRollStiffnessEditTBY, ...
                                                RLegAnkleRollStiffnessEditTBWidth, RLegAnkleRollStiffnessEditTBHeight]);   

RLegAnkleRollStiffnessDispTBX       = RLegAnkleRollStiffnessEditTBX + RLegAnkleRollStiffnessEditTBWidth + tbPadding;
RLegAnkleRollStiffnessDispTBY       = tbPadding + 2;
RLegAnkleRollStiffnessDispTBWidth   = (RLegStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RLegAnkleRollStiffnessDispTBHeight  = 20;

RLegAnkleRollStiffnessDispTB = uicontrol('Parent', RLegStiffnessPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(RLegAnkleRollStiffnessDispTB, [RLegAnkleRollStiffnessDispTBX, RLegAnkleRollStiffnessDispTBY, ...
                                                RLegAnkleRollStiffnessDispTBWidth, RLegAnkleRollStiffnessDispTBHeight]);   
    
% LArm panel.
LArmPanelWidth  = sliderPanelWidth - 2 * panelPadding;
LArmPanelHeight = 5 * 20 + 5 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
LArmPanelX      = panelPadding;
LArmPanelY      = RLegPanelY + RLegPanelHeight + panelPadding;

LArmPanel = uipanel('Parent', sliderPanel, 'Title', 'Left Arm', ...
                    'FontSize', 12);
setpixelposition(LArmPanel, [LArmPanelX LArmPanelY ...
                             LArmPanelWidth LArmPanelHeight]);

% Sensor panel.
LArmSensorPanelX      = panelPadding + 100 + panelPadding;
LArmSensorPanelY      = 2 * panelPadding + 20 + 2;
LArmSensorPanelWidth  = (LArmPanelWidth - 4 * panelPadding - 100) / 2 - 1;
LArmSensorPanelHeight = 4 * 20 + 5 * tbPadding + estPanelTextHeight + 2;


LArmSensorPanel = uipanel('Parent', LArmPanel, 'Title', 'Sensor', ...
                    'FontSize', 12);
setpixelposition(LArmSensorPanel, [LArmSensorPanelX LArmSensorPanelY ...
                                   LArmSensorPanelWidth LArmSensorPanelHeight]);

% Stiffness panel.
LArmStiffnessPanelX      = LArmSensorPanelX + LArmSensorPanelWidth + panelPadding;
LArmStiffnessPanelY      = 2 * panelPadding + 20 + 2;
LArmStiffnessPanelWidth  = (LArmPanelWidth - 4 * panelPadding - 100) / 2 - 1;
LArmStiffnessPanelHeight = 4 * 20 + 5 * tbPadding + estPanelTextHeight + 2;

LArmStiffnessPanel = uipanel('Parent', LArmPanel, 'Title', 'Stiffness', ...
                             'FontSize', 12);
setpixelposition(LArmStiffnessPanel, [LArmStiffnessPanelX LArmStiffnessPanelY ...
                                      LArmStiffnessPanelWidth LArmStiffnessPanelHeight]);

% Loosen left arm button.
loosenLArmPBWidth  = LArmSensorPanelWidth + LArmStiffnessPanelWidth + panelPadding - 2;
loosenLArmPBHeight = 20;
loosenLArmPBX      = LArmSensorPanelX;
loosenLArmPBY      = panelPadding + 2;

loosenLArmBtn = uicontrol('Parent', LArmPanel, ...
                          'Style', 'pushbutton', 'String', 'Loosen Left Arm', ...
                          'callback', {@Keyframe_Generator_callBacks, 'loosenLArm'});            
setpixelposition(loosenLArmBtn, [loosenLArmPBX, loosenLArmPBY, loosenLArmPBWidth, loosenLArmPBHeight]);
                              
% Sensor boxes.
% Shoulder pitch.
LArmShoulderPitchSensorEditTBX       = tbPadding;
LArmShoulderPitchSensorEditTBY       = 3 * 20 + 4 * tbPadding + 2;
LArmShoulderPitchSensorEditTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
LArmShoulderPitchSensorEditTBHeight  = 20;

LArmShoulderPitchSensorEditTB = uicontrol('Parent', LArmSensorPanel, 'style','edit', ...
                                          'BackgroundColor', 'white', ...
                                          'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmShoulderPitchSensorEditTB, [LArmShoulderPitchSensorEditTBX, LArmShoulderPitchSensorEditTBY, ...
                                                 LArmShoulderPitchSensorEditTBWidth, LArmShoulderPitchSensorEditTBHeight]);   

LArmShoulderPitchSensorDispTBX       = LArmShoulderPitchSensorEditTBX + LArmShoulderPitchSensorEditTBWidth + tbPadding;
LArmShoulderPitchSensorDispTBY       = 3 * 20 + 4 * tbPadding + 2;
LArmShoulderPitchSensorDispTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LArmShoulderPitchSensorDispTBHeight  = 20;

LArmShoulderPitchSensorDispTB = uicontrol('Parent', LArmSensorPanel, 'style','text', ...
                                          'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmShoulderPitchSensorDispTB, [LArmShoulderPitchSensorDispTBX, LArmShoulderPitchSensorDispTBY, ...
                                                 LArmShoulderPitchSensorDispTBWidth, LArmShoulderPitchSensorDispTBHeight]);   
                                       
% Shoulder roll.
LArmShoulderRollSensorEditTBX       = tbPadding;
LArmShoulderRollSensorEditTBY       = 2 * 20 + 3 * tbPadding + 2;
LArmShoulderRollSensorEditTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
LArmShoulderRollSensorEditTBHeight  = 20;

LArmShoulderRollSensorEditTB = uicontrol('Parent', LArmSensorPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmShoulderRollSensorEditTB, [LArmShoulderRollSensorEditTBX, LArmShoulderRollSensorEditTBY, ...
                                                LArmShoulderRollSensorEditTBWidth, LArmShoulderRollSensorEditTBHeight]);   

LArmShoulderRollSensorDispTBX       = LArmShoulderRollSensorEditTBX + LArmShoulderRollSensorEditTBWidth + tbPadding;
LArmShoulderRollSensorDispTBY       = 2 * 20 + 3 * tbPadding + 2;
LArmShoulderRollSensorDispTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LArmShoulderRollSensorDispTBHeight  = 20;

LArmShoulderRollSensorDispTB = uicontrol('Parent', LArmSensorPanel, 'style','text', ...
                                         'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmShoulderRollSensorDispTB, [LArmShoulderRollSensorDispTBX, LArmShoulderRollSensorDispTBY, ...
                                                LArmShoulderRollSensorDispTBWidth, LArmShoulderRollSensorDispTBHeight]);   

% Elbow yaw.
LArmElbowYawSensorEditTBX       = tbPadding;
LArmElbowYawSensorEditTBY       = 1 * 20 + 2 * tbPadding + 2;
LArmElbowYawSensorEditTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
LArmElbowYawSensorEditTBHeight  = 20;

LArmElbowYawSensorEditTB = uicontrol('Parent', LArmSensorPanel, 'style','edit', ...
                                     'BackgroundColor', 'white', ...
                                     'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmElbowYawSensorEditTB, [LArmElbowYawSensorEditTBX, LArmElbowYawSensorEditTBY, ...
                                            LArmElbowYawSensorEditTBWidth, LArmElbowYawSensorEditTBHeight]);   

LArmElbowYawSensorDispTBX       = LArmElbowYawSensorEditTBX + LArmElbowYawSensorEditTBWidth + tbPadding;
LArmElbowYawSensorDispTBY       = 1 * 20 + 2 * tbPadding + 2;
LArmElbowYawSensorDispTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LArmElbowYawSensorDispTBHeight  = 20;

LArmElbowYawSensorDispTB = uicontrol('Parent', LArmSensorPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmElbowYawSensorDispTB, [LArmElbowYawSensorDispTBX, LArmElbowYawSensorDispTBY, ...
                                            LArmElbowYawSensorDispTBWidth, LArmElbowYawSensorDispTBHeight]);   

% Elbow roll.
LArmElbowRollSensorEditTBX       = tbPadding;
LArmElbowRollSensorEditTBY       = 0 * 20 + 1 * tbPadding + 2;
LArmElbowRollSensorEditTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
LArmElbowRollSensorEditTBHeight  = 20;

LArmElbowRollSensorEditTB = uicontrol('Parent', LArmSensorPanel, 'style','edit', ...
                                      'BackgroundColor', 'white', ...
                                      'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmElbowRollSensorEditTB, [LArmElbowRollSensorEditTBX, LArmElbowRollSensorEditTBY, ...
                                             LArmElbowRollSensorEditTBWidth, LArmElbowRollSensorEditTBHeight]);   

LArmElbowRollSensorDispTBX       = LArmElbowRollSensorEditTBX + LArmElbowRollSensorEditTBWidth + tbPadding;
LArmElbowRollSensorDispTBY       = 0 * 20 + 1 * tbPadding + 2;
LArmElbowRollSensorDispTBWidth   = (LArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
LArmElbowRollSensorDispTBHeight  = 20;

LArmElbowRollSensorDispTB = uicontrol('Parent', LArmSensorPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmElbowRollSensorDispTB, [LArmElbowRollSensorDispTBX, LArmElbowRollSensorDispTBY, ...
                                             LArmElbowRollSensorDispTBWidth, LArmElbowRollSensorDispTBHeight]);   
                                                          
% Stiffness boxes.
% Shoulder pitch.
LArmShoulderPitchStiffnessEditTBX       = tbPadding;
LArmShoulderPitchStiffnessEditTBY       = 3 * 20 + 4 * tbPadding + 2;
LArmShoulderPitchStiffnessEditTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
LArmShoulderPitchStiffnessEditTBHeight  = 20;

LArmShoulderPitchStiffnessEditTB = uicontrol('Parent', LArmStiffnessPanel, 'style','edit', ...
                                             'BackgroundColor', 'white', ...
                                             'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmShoulderPitchStiffnessEditTB, [LArmShoulderPitchStiffnessEditTBX, LArmShoulderPitchStiffnessEditTBY, ...
                                                 LArmShoulderPitchStiffnessEditTBWidth, LArmShoulderPitchStiffnessEditTBHeight]);   

LArmShoulderPitchStiffnessDispTBX       = LArmShoulderPitchStiffnessEditTBX + LArmShoulderPitchStiffnessEditTBWidth + tbPadding;
LArmShoulderPitchStiffnessDispTBY       = 3 * 20 + 4 * tbPadding + 2;
LArmShoulderPitchStiffnessDispTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LArmShoulderPitchStiffnessDispTBHeight  = 20;

LArmShoulderPitchStiffnessDispTB = uicontrol('Parent', LArmStiffnessPanel, 'style','text', ...
                                             'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmShoulderPitchStiffnessDispTB, [LArmShoulderPitchStiffnessDispTBX, LArmShoulderPitchStiffnessDispTBY, ...
                                                    LArmShoulderPitchStiffnessDispTBWidth, LArmShoulderPitchStiffnessDispTBHeight]);   
                                       
% Shoulder roll.
LArmShoulderRollStiffnessEditTBX       = tbPadding;
LArmShoulderRollStiffnessEditTBY       = 2 * 20 + 3 * tbPadding + 2;
LArmShoulderRollStiffnessEditTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
LArmShoulderRollStiffnessEditTBHeight  = 20;

LArmShoulderRollStiffnessEditTB = uicontrol('Parent', LArmStiffnessPanel, 'style','edit', ...
                                            'BackgroundColor', 'white', ...
                                            'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmShoulderRollStiffnessEditTB, [LArmShoulderRollStiffnessEditTBX, LArmShoulderRollStiffnessEditTBY, ...
                                                   LArmShoulderRollStiffnessEditTBWidth, LArmShoulderRollStiffnessEditTBHeight]);   

LArmShoulderRollStiffnessDispTBX       = LArmShoulderRollStiffnessEditTBX + LArmShoulderRollStiffnessEditTBWidth + tbPadding;
LArmShoulderRollStiffnessDispTBY       = 2 * 20 + 3 * tbPadding + 2;
LArmShoulderRollStiffnessDispTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LArmShoulderRollStiffnessDispTBHeight  = 20;

LArmShoulderRollStiffnessDispTB = uicontrol('Parent', LArmStiffnessPanel, 'style','text', ...
                                            'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmShoulderRollStiffnessDispTB, [LArmShoulderRollStiffnessDispTBX, LArmShoulderRollStiffnessDispTBY, ...
                                                   LArmShoulderRollStiffnessDispTBWidth, LArmShoulderRollStiffnessDispTBHeight]);   

% Elbow yaw.
LArmElbowYawStiffnessEditTBX       = tbPadding;
LArmElbowYawStiffnessEditTBY       = 1 * 20 + 2 * tbPadding + 2;
LArmElbowYawStiffnessEditTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
LArmElbowYawStiffnessEditTBHeight  = 20;

LArmElbowYawStiffnessEditTB = uicontrol('Parent', LArmStiffnessPanel, 'style','edit', ...
                                        'BackgroundColor', 'white', ...
                                        'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmElbowYawStiffnessEditTB, [LArmElbowYawStiffnessEditTBX, LArmElbowYawStiffnessEditTBY, ...
                                               LArmElbowYawStiffnessEditTBWidth, LArmElbowYawStiffnessEditTBHeight]);   

LArmElbowYawStiffnessDispTBX       = LArmElbowYawStiffnessEditTBX + LArmElbowYawStiffnessEditTBWidth + tbPadding;
LArmElbowYawStiffnessDispTBY       = 1 * 20 + 2 * tbPadding + 2;
LArmElbowYawStiffnessDispTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LArmElbowYawStiffnessDispTBHeight  = 20;

LArmElbowYawStiffnessDispTB = uicontrol('Parent', LArmStiffnessPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmElbowYawStiffnessDispTB, [LArmElbowYawStiffnessDispTBX, LArmElbowYawStiffnessDispTBY, ...
                                               LArmElbowYawStiffnessDispTBWidth, LArmElbowYawStiffnessDispTBHeight]);   

% Elbow roll.
LArmElbowRollStiffnessEditTBX       = tbPadding;
LArmElbowRollStiffnessEditTBY       = 0 * 20 + 1 * tbPadding + 2;
LArmElbowRollStiffnessEditTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
LArmElbowRollStiffnessEditTBHeight  = 20;

LArmElbowRollStiffnessEditTB = uicontrol('Parent', LArmStiffnessPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(LArmElbowRollStiffnessEditTB, [LArmElbowRollStiffnessEditTBX, LArmElbowRollStiffnessEditTBY, ...
                                             LArmElbowRollStiffnessEditTBWidth, LArmElbowRollStiffnessEditTBHeight]);   

LArmElbowRollStiffnessDispTBX       = LArmElbowRollStiffnessEditTBX + LArmElbowRollStiffnessEditTBWidth + tbPadding;
LArmElbowRollStiffnessDispTBY       = 0 * 20 + 1 * tbPadding + 2;
LArmElbowRollStiffnessDispTBWidth   = (LArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
LArmElbowRollStiffnessDispTBHeight  = 20;

LArmElbowRollStiffnessDispTB = uicontrol('Parent', LArmStiffnessPanel, 'style','text', ...
                                         'BackgroundColor', [.77 .77 .77]);
setpixelposition(LArmElbowRollStiffnessDispTB, [LArmElbowRollStiffnessDispTBX, LArmElbowRollStiffnessDispTBY, ...
                                                LArmElbowRollStiffnessDispTBWidth, LArmElbowRollStiffnessDispTBHeight]);   
                           
% RArm panel.
RArmPanelWidth  = sliderPanelWidth - 2 * panelPadding;
RArmPanelHeight = 5 * 20 + 5 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
RArmPanelX      = panelPadding;
RArmPanelY      = LArmPanelY + LArmPanelHeight + panelPadding;

RArmPanel = uipanel('Parent', sliderPanel, 'Title', 'Right Arm', ...
                    'FontSize', 12);
setpixelposition(RArmPanel, [RArmPanelX RArmPanelY ...
                             RArmPanelWidth RArmPanelHeight]);

% Sensor panel.
RArmSensorPanelX      = panelPadding + 100 + panelPadding;
RArmSensorPanelY      = 2 * panelPadding + 20 + 2;
RArmSensorPanelWidth  = (RArmPanelWidth - 4 * panelPadding - 100) / 2 - 1;
RArmSensorPanelHeight = 4 * 20 + 5 * tbPadding + estPanelTextHeight + 2;


RArmSensorPanel = uipanel('Parent', RArmPanel, 'Title', 'Sensor', ...
                    'FontSize', 12);
setpixelposition(RArmSensorPanel, [RArmSensorPanelX RArmSensorPanelY ...
                                   RArmSensorPanelWidth RArmSensorPanelHeight]);

% Stiffness panel.
RArmStiffnessPanelX      = RArmSensorPanelX + RArmSensorPanelWidth + panelPadding;
RArmStiffnessPanelY      = 2 * panelPadding + 20 + 2;
RArmStiffnessPanelWidth  = (RArmPanelWidth - 4 * panelPadding - 100) / 2 - 1;
RArmStiffnessPanelHeight = 4 * 20 + 5 * tbPadding + estPanelTextHeight + 2;

RArmStiffnessPanel = uipanel('Parent', RArmPanel, 'Title', 'Stiffness', ...
                             'FontSize', 12);
setpixelposition(RArmStiffnessPanel, [RArmStiffnessPanelX RArmStiffnessPanelY ...
                                      RArmStiffnessPanelWidth RArmStiffnessPanelHeight]);
           
% Loosen right arm button.
loosenRArmPBWidth  = RArmSensorPanelWidth + RArmStiffnessPanelWidth + panelPadding - 2;
loosenRArmPBHeight = 20;
loosenRArmPBX      = RArmSensorPanelX;
loosenRArmPBY      = panelPadding + 1;

loosenRArmBtn = uicontrol('Parent', RArmPanel, ...
                          'Style', 'pushbutton', 'String', 'Loosen Right Arm', ...
                          'callback', {@Keyframe_Generator_callBacks, 'loosenRArm'});            
setpixelposition(loosenRArmBtn, [loosenRArmPBX, loosenRArmPBY, loosenRArmPBWidth, loosenRArmPBHeight]);                                 
                                  
% Sensor boxes.
% Shoulder pitch.
RArmShoulderPitchSensorEditTBX       = tbPadding;
RArmShoulderPitchSensorEditTBY       = 3 * 20 + 4 * tbPadding + 2;
RArmShoulderPitchSensorEditTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
RArmShoulderPitchSensorEditTBHeight  = 20;

RArmShoulderPitchSensorEditTB = uicontrol('Parent', RArmSensorPanel, 'style','edit', ...
                                          'BackgroundColor', 'white', ...
                                          'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmShoulderPitchSensorEditTB, [RArmShoulderPitchSensorEditTBX, RArmShoulderPitchSensorEditTBY, ...
                                                 RArmShoulderPitchSensorEditTBWidth, RArmShoulderPitchSensorEditTBHeight]);   

RArmShoulderPitchSensorDispTBX       = RArmShoulderPitchSensorEditTBX + RArmShoulderPitchSensorEditTBWidth + tbPadding;
RArmShoulderPitchSensorDispTBY       = 3 * 20 + 4 * tbPadding + 2;
RArmShoulderPitchSensorDispTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RArmShoulderPitchSensorDispTBHeight  = 20;

RArmShoulderPitchSensorDispTB = uicontrol('Parent', RArmSensorPanel, 'style','text', ...
                                          'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmShoulderPitchSensorDispTB, [RArmShoulderPitchSensorDispTBX, RArmShoulderPitchSensorDispTBY, ...
                                                 RArmShoulderPitchSensorDispTBWidth, RArmShoulderPitchSensorDispTBHeight]);   
                                       
% Shoulder roll.
RArmShoulderRollSensorEditTBX       = tbPadding;
RArmShoulderRollSensorEditTBY       = 2 * 20 + 3 * tbPadding + 2;
RArmShoulderRollSensorEditTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
RArmShoulderRollSensorEditTBHeight  = 20;

RArmShoulderRollSensorEditTB = uicontrol('Parent', RArmSensorPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmShoulderRollSensorEditTB, [RArmShoulderRollSensorEditTBX, RArmShoulderRollSensorEditTBY, ...
                                                RArmShoulderRollSensorEditTBWidth, RArmShoulderRollSensorEditTBHeight]);   

RArmShoulderRollSensorDispTBX       = RArmShoulderRollSensorEditTBX + RArmShoulderRollSensorEditTBWidth + tbPadding;
RArmShoulderRollSensorDispTBY       = 2 * 20 + 3 * tbPadding + 2;
RArmShoulderRollSensorDispTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RArmShoulderRollSensorDispTBHeight  = 20;

RArmShoulderRollSensorDispTB = uicontrol('Parent', RArmSensorPanel, 'style','text', ...
                                         'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmShoulderRollSensorDispTB, [RArmShoulderRollSensorDispTBX, RArmShoulderRollSensorDispTBY, ...
                                                RArmShoulderRollSensorDispTBWidth, RArmShoulderRollSensorDispTBHeight]);   

% Elbow yaw.
RArmElbowYawSensorEditTBX       = tbPadding;
RArmElbowYawSensorEditTBY       = 1 * 20 + 2 * tbPadding + 2;
RArmElbowYawSensorEditTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
RArmElbowYawSensorEditTBHeight  = 20;

RArmElbowYawSensorEditTB = uicontrol('Parent', RArmSensorPanel, 'style','edit', ...
                                     'BackgroundColor', 'white', ...
                                     'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmElbowYawSensorEditTB, [RArmElbowYawSensorEditTBX, RArmElbowYawSensorEditTBY, ...
                                            RArmElbowYawSensorEditTBWidth, RArmElbowYawSensorEditTBHeight]);   

RArmElbowYawSensorDispTBX       = RArmElbowYawSensorEditTBX + RArmElbowYawSensorEditTBWidth + tbPadding;
RArmElbowYawSensorDispTBY       = 1 * 20 + 2 * tbPadding + 2;
RArmElbowYawSensorDispTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RArmElbowYawSensorDispTBHeight  = 20;

RArmElbowYawSensorDispTB = uicontrol('Parent', RArmSensorPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmElbowYawSensorDispTB, [RArmElbowYawSensorDispTBX, RArmElbowYawSensorDispTBY, ...
                                            RArmElbowYawSensorDispTBWidth, RArmElbowYawSensorDispTBHeight]);   

% Elbow roll.
RArmElbowRollSensorEditTBX       = tbPadding;
RArmElbowRollSensorEditTBY       = 0 * 20 + 1 * tbPadding + 2;
RArmElbowRollSensorEditTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 1;
RArmElbowRollSensorEditTBHeight  = 20;

RArmElbowRollSensorEditTB = uicontrol('Parent', RArmSensorPanel, 'style','edit', ...
                                      'BackgroundColor', 'white', ...
                                      'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmElbowRollSensorEditTB, [RArmElbowRollSensorEditTBX, RArmElbowRollSensorEditTBY, ...
                                             RArmElbowRollSensorEditTBWidth, RArmElbowRollSensorEditTBHeight]);   

RArmElbowRollSensorDispTBX       = RArmElbowRollSensorEditTBX + RArmElbowRollSensorEditTBWidth + tbPadding;
RArmElbowRollSensorDispTBY       = 0 * 20 + 1 * tbPadding + 2;
RArmElbowRollSensorDispTBWidth   = (RArmSensorPanelWidth - 3 * tbPadding) / 2 - 2;
RArmElbowRollSensorDispTBHeight  = 20;

RArmElbowRollSensorDispTB = uicontrol('Parent', RArmSensorPanel, 'style','text', ...
                                      'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmElbowRollSensorDispTB, [RArmElbowRollSensorDispTBX, RArmElbowRollSensorDispTBY, ...
                                             RArmElbowRollSensorDispTBWidth, RArmElbowRollSensorDispTBHeight]);   
                                                          
% Stiffness boxes.
% Shoulder pitch.
RArmShoulderPitchStiffnessEditTBX       = tbPadding;
RArmShoulderPitchStiffnessEditTBY       = 3 * 20 + 4 * tbPadding + 2;
RArmShoulderPitchStiffnessEditTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
RArmShoulderPitchStiffnessEditTBHeight  = 20;

RArmShoulderPitchStiffnessEditTB = uicontrol('Parent', RArmStiffnessPanel, 'style','edit', ...
                                             'BackgroundColor', 'white', ...
                                             'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmShoulderPitchStiffnessEditTB, [RArmShoulderPitchStiffnessEditTBX, RArmShoulderPitchStiffnessEditTBY, ...
                                                 RArmShoulderPitchStiffnessEditTBWidth, RArmShoulderPitchStiffnessEditTBHeight]);   

RArmShoulderPitchStiffnessDispTBX       = RArmShoulderPitchStiffnessEditTBX + RArmShoulderPitchStiffnessEditTBWidth + tbPadding;
RArmShoulderPitchStiffnessDispTBY       = 3 * 20 + 4 * tbPadding + 2;
RArmShoulderPitchStiffnessDispTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RArmShoulderPitchStiffnessDispTBHeight  = 20;

RArmShoulderPitchStiffnessDispTB = uicontrol('Parent', RArmStiffnessPanel, 'style','text', ...
                                             'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmShoulderPitchStiffnessDispTB, [RArmShoulderPitchStiffnessDispTBX, RArmShoulderPitchStiffnessDispTBY, ...
                                                    RArmShoulderPitchStiffnessDispTBWidth, RArmShoulderPitchStiffnessDispTBHeight]);   
                                       
% Shoulder roll.
RArmShoulderRollStiffnessEditTBX       = tbPadding;
RArmShoulderRollStiffnessEditTBY       = 2 * 20 + 3 * tbPadding + 2;
RArmShoulderRollStiffnessEditTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
RArmShoulderRollStiffnessEditTBHeight  = 20;

RArmShoulderRollStiffnessEditTB = uicontrol('Parent', RArmStiffnessPanel, 'style','edit', ...
                                            'BackgroundColor', 'white', ...
                                            'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmShoulderRollStiffnessEditTB, [RArmShoulderRollStiffnessEditTBX, RArmShoulderRollStiffnessEditTBY, ...
                                                   RArmShoulderRollStiffnessEditTBWidth, RArmShoulderRollStiffnessEditTBHeight]);   

RArmShoulderRollStiffnessDispTBX       = RArmShoulderRollStiffnessEditTBX + RArmShoulderRollStiffnessEditTBWidth + tbPadding;
RArmShoulderRollStiffnessDispTBY       = 2 * 20 + 3 * tbPadding + 2;
RArmShoulderRollStiffnessDispTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RArmShoulderRollStiffnessDispTBHeight  = 20;

RArmShoulderRollStiffnessDispTB = uicontrol('Parent', RArmStiffnessPanel, 'style','text', ...
                                            'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmShoulderRollStiffnessDispTB, [RArmShoulderRollStiffnessDispTBX, RArmShoulderRollStiffnessDispTBY, ...
                                                   RArmShoulderRollStiffnessDispTBWidth, RArmShoulderRollStiffnessDispTBHeight]);   

% Elbow yaw.
RArmElbowYawStiffnessEditTBX       = tbPadding;
RArmElbowYawStiffnessEditTBY       = 1 * 20 + 2 * tbPadding + 2;
RArmElbowYawStiffnessEditTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
RArmElbowYawStiffnessEditTBHeight  = 20;

RArmElbowYawStiffnessEditTB = uicontrol('Parent', RArmStiffnessPanel, 'style','edit', ...
                                        'BackgroundColor', 'white', ...
                                        'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmElbowYawStiffnessEditTB, [RArmElbowYawStiffnessEditTBX, RArmElbowYawStiffnessEditTBY, ...
                                               RArmElbowYawStiffnessEditTBWidth, RArmElbowYawStiffnessEditTBHeight]);   

RArmElbowYawStiffnessDispTBX       = RArmElbowYawStiffnessEditTBX + RArmElbowYawStiffnessEditTBWidth + tbPadding;
RArmElbowYawStiffnessDispTBY       = 1 * 20 + 2 * tbPadding + 2;
RArmElbowYawStiffnessDispTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RArmElbowYawStiffnessDispTBHeight  = 20;

RArmElbowYawStiffnessDispTB = uicontrol('Parent', RArmStiffnessPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmElbowYawStiffnessDispTB, [RArmElbowYawStiffnessDispTBX, RArmElbowYawStiffnessDispTBY, ...
                                               RArmElbowYawStiffnessDispTBWidth, RArmElbowYawStiffnessDispTBHeight]);   

% Elbow roll.
RArmElbowRollStiffnessEditTBX       = tbPadding;
RArmElbowRollStiffnessEditTBY       = 0 * 20 + 1 * tbPadding + 2;
RArmElbowRollStiffnessEditTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
RArmElbowRollStiffnessEditTBHeight  = 20;

RArmElbowRollStiffnessEditTB = uicontrol('Parent', RArmStiffnessPanel, 'style','edit', ...
                                         'BackgroundColor', 'white', ...
                                         'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(RArmElbowRollStiffnessEditTB, [RArmElbowRollStiffnessEditTBX, RArmElbowRollStiffnessEditTBY, ...
                                             RArmElbowRollStiffnessEditTBWidth, RArmElbowRollStiffnessEditTBHeight]);   

RArmElbowRollStiffnessDispTBX       = RArmElbowRollStiffnessEditTBX + RArmElbowRollStiffnessEditTBWidth + tbPadding;
RArmElbowRollStiffnessDispTBY       = 0 * 20 + 1 * tbPadding + 2;
RArmElbowRollStiffnessDispTBWidth   = (RArmStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
RArmElbowRollStiffnessDispTBHeight  = 20;

RArmElbowRollStiffnessDispTB = uicontrol('Parent', RArmStiffnessPanel, 'style','text', ...
                                         'BackgroundColor', [.77 .77 .77]);
setpixelposition(RArmElbowRollStiffnessDispTB, [RArmElbowRollStiffnessDispTBX, RArmElbowRollStiffnessDispTBY, ...
                                                RArmElbowRollStiffnessDispTBWidth, RArmElbowRollStiffnessDispTBHeight]);   
                           
% Head panel.
HeadPanelWidth  = sliderPanelWidth - 2 * panelPadding;
HeadPanelHeight = 3 * 20 + 3 * tbPadding + estPanelTextHeight + 2 + 3 * panelPadding + 7;
HeadPanelX      = panelPadding;
HeadPanelY      = RArmPanelY + RArmPanelHeight + panelPadding;

HeadPanel = uipanel('Parent', sliderPanel, 'Title', 'Head', ...
                    'FontSize', 12);
setpixelposition(HeadPanel, [HeadPanelX HeadPanelY ...
                             HeadPanelWidth HeadPanelHeight]);                           
                           
% Sensor panel.
HeadSensorPanelX      = panelPadding + 100 + panelPadding;
HeadSensorPanelY      = 2 * panelPadding + 20 + 3; 
HeadSensorPanelWidth  = (HeadPanelWidth - 4 * panelPadding - 100) / 2 - 1;
HeadSensorPanelHeight = 2 * 20 + 3 * tbPadding + estPanelTextHeight + 2;


HeadSensorPanel = uipanel('Parent', HeadPanel, 'Title', 'Sensor', ...
                    'FontSize', 12);
setpixelposition(HeadSensorPanel, [HeadSensorPanelX HeadSensorPanelY ...
                                   HeadSensorPanelWidth HeadSensorPanelHeight]);

% Stiffness panel.
HeadStiffnessPanelX      = HeadSensorPanelX + HeadSensorPanelWidth + panelPadding;
HeadStiffnessPanelY      = 2 * panelPadding + 20 + 3;
HeadStiffnessPanelWidth  = (HeadPanelWidth - 4 * panelPadding - 100) / 2 - 1;
HeadStiffnessPanelHeight = 2 * 20 + 3 * tbPadding + estPanelTextHeight + 2;

HeadStiffnessPanel = uipanel('Parent', HeadPanel, 'Title', 'Stiffness', ...
                             'FontSize', 12);
setpixelposition(HeadStiffnessPanel, [HeadStiffnessPanelX HeadStiffnessPanelY ...
                                      HeadStiffnessPanelWidth HeadStiffnessPanelHeight]);

% Loosen head button.
loosenHeadPBWidth  = HeadSensorPanelWidth + HeadStiffnessPanelWidth + panelPadding - 2;
loosenHeadPBHeight = 20;
loosenHeadPBX      = HeadSensorPanelX;
loosenHeadPBY      = panelPadding + 2;

loosenHeadBtn = uicontrol('Parent', HeadPanel, ...
                          'Style', 'pushbutton', 'String', 'Loosen Head', ...
                          'callback', {@Keyframe_Generator_callBacks, 'loosenHead'});            
setpixelposition(loosenHeadBtn, [loosenHeadPBX, loosenHeadPBY, loosenHeadPBWidth, loosenHeadPBHeight]);
                          
% Sensor boxes.                                  
% Head pitch.
HeadPitchSensorEditTBX      = tbPadding;
HeadPitchSensorEditTBY      = 1 * 20 + 2 * tbPadding + 2;
HeadPitchSensorEditTBWidth  = (HeadSensorPanelWidth - 3 * tbPadding) / 2 - 1;
HeadPitchSensorEditTBHeight = 20;

HeadPitchSensorEditTB = uicontrol('Parent', HeadSensorPanel, 'style','edit', ...
                                  'BackgroundColor', 'white', ...
                                  'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(HeadPitchSensorEditTB, [HeadPitchSensorEditTBX, HeadPitchSensorEditTBY, ...
                                         HeadPitchSensorEditTBWidth, HeadPitchSensorEditTBHeight]);   

HeadPitchSensorDispTBX       = HeadPitchSensorEditTBX + HeadPitchSensorEditTBWidth + tbPadding;
HeadPitchSensorDispTBY       = 1 * 20 + 2 * tbPadding + 2;
HeadPitchSensorDispTBWidth   = (HeadSensorPanelWidth - 3 * tbPadding) / 2 - 2;
HeadPitchSensorDispTBHeight  = 20;

HeadPitchSensorDispTB = uicontrol('Parent', HeadSensorPanel, 'style','text', ...
                                  'BackgroundColor', [.77 .77 .77]);
setpixelposition(HeadPitchSensorDispTB, [HeadPitchSensorDispTBX, HeadPitchSensorDispTBY, ...
                                         HeadPitchSensorDispTBWidth, HeadPitchSensorDispTBHeight]);   
                           
% Head yaw.
HeadYawSensorEditTBX      = tbPadding;
HeadYawSensorEditTBY      = 0 * 20 + 1 * tbPadding + 2;
HeadYawSensorEditTBWidth  = (HeadSensorPanelWidth - 3 * tbPadding) / 2 - 1;
HeadYawSensorEditTBHeight = 20;

HeadYawSensorEditTB = uicontrol('Parent', HeadSensorPanel, 'style','edit', ...
                                'BackgroundColor', 'white', ...
                                'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(HeadYawSensorEditTB, [HeadYawSensorEditTBX, HeadYawSensorEditTBY, ...
                                       HeadYawSensorEditTBWidth, HeadYawSensorEditTBHeight]);   

HeadYawSensorDispTBX       = HeadYawSensorEditTBX + HeadYawSensorEditTBWidth + tbPadding;
HeadYawSensorDispTBY       = 0 * 20 + 1 * tbPadding + 2;
HeadYawSensorDispTBWidth   = (HeadSensorPanelWidth - 3 * tbPadding) / 2 - 2;
HeadYawSensorDispTBHeight  = 20;

HeadYawSensorDispTB = uicontrol('Parent', HeadSensorPanel, 'style','text', ...
                                'BackgroundColor', [.77 .77 .77]);
setpixelposition(HeadYawSensorDispTB, [HeadYawSensorDispTBX, HeadYawSensorDispTBY, ...
                                       HeadYawSensorDispTBWidth, HeadYawSensorDispTBHeight]);   
                           
% Stiffness boxes.
% Head pitch.
HeadPitchStiffnessEditTBX      = tbPadding;
HeadPitchStiffnessEditTBY      = 1 * 20 + 2 * tbPadding + 2;
HeadPitchStiffnessEditTBWidth  = (HeadStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
HeadPitchStiffnessEditTBHeight = 20;

HeadPitchStiffnessEditTB = uicontrol('Parent', HeadStiffnessPanel, 'style','edit', ...
                                     'BackgroundColor', 'white', ...
                                     'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(HeadPitchStiffnessEditTB, [HeadPitchStiffnessEditTBX, HeadPitchStiffnessEditTBY, ...
                                         HeadPitchStiffnessEditTBWidth, HeadPitchStiffnessEditTBHeight]);   

HeadPitchStiffnessDispTBX       = HeadPitchStiffnessEditTBX + HeadPitchStiffnessEditTBWidth + tbPadding;
HeadPitchStiffnessDispTBY       = 1 * 20 + 2 * tbPadding + 2;
HeadPitchStiffnessDispTBWidth   = (HeadStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
HeadPitchStiffnessDispTBHeight  = 20;

HeadPitchStiffnessDispTB = uicontrol('Parent', HeadStiffnessPanel, 'style','text', ...
                                     'BackgroundColor', [.77 .77 .77]);
setpixelposition(HeadPitchStiffnessDispTB, [HeadPitchStiffnessDispTBX, HeadPitchStiffnessDispTBY, ...
                                         HeadPitchStiffnessDispTBWidth, HeadPitchStiffnessDispTBHeight]);   
                           
% Head yaw.
HeadYawStiffnessEditTBX      = tbPadding;
HeadYawStiffnessEditTBY      = 0 * 20 + 1 * tbPadding + 2;
HeadYawStiffnessEditTBWidth  = (HeadStiffnessPanelWidth - 3 * tbPadding) / 2 - 1;
HeadYawStiffnessEditTBHeight = 20;

HeadYawStiffnessEditTB = uicontrol('Parent', HeadStiffnessPanel, 'style','edit', ...
                                   'BackgroundColor', 'white', ...
                                   'callback', {@Keyframe_Generator_callBacks, 'editBoxCallback'});            
setpixelposition(HeadYawStiffnessEditTB, [HeadYawStiffnessEditTBX, HeadYawStiffnessEditTBY, ...
                                          HeadYawStiffnessEditTBWidth, HeadYawStiffnessEditTBHeight]);   

HeadYawStiffnessDispTBX       = HeadYawStiffnessEditTBX + HeadYawStiffnessEditTBWidth + tbPadding;
HeadYawStiffnessDispTBY       = 0 * 20 + 1 * tbPadding + 2;
HeadYawStiffnessDispTBWidth   = (HeadStiffnessPanelWidth - 3 * tbPadding) / 2 - 2;
HeadYawStiffnessDispTBHeight  = 20;

HeadYawStiffnessDispTB = uicontrol('Parent', HeadStiffnessPanel, 'style','text', ...
                                   'BackgroundColor', [.77 .77 .77]);
setpixelposition(HeadYawStiffnessDispTB, [HeadYawStiffnessDispTBX, HeadYawStiffnessDispTBY, ...
                                          HeadYawStiffnessDispTBWidth, HeadYawStiffnessDispTBHeight]);   

% Left justified text labels.
% Head pitch.
HeadPitchLabelTBX       = tbPadding;
HeadPitchLabelTBY       = 2 * 20 + 2 * panelPadding + 2 * tbPadding + 3;
HeadPitchLabelTBWidth   = 100;
HeadPitchLabelTBHeight  = 20;

HeadPitchLabelTB = uicontrol('Parent', HeadPanel, 'style','text', 'string', 'Head Pitch', ...
                             'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(HeadPitchLabelTB, [HeadPitchLabelTBX, HeadPitchLabelTBY, ...
                                    HeadPitchLabelTBWidth, HeadPitchLabelTBHeight]);   
                                
% Head yaw.
HeadYawLabelTBX       = tbPadding;
HeadYawLabelTBY       = 1 * 20 + 2 * panelPadding + 1 * tbPadding + 3;
HeadYawLabelTBWidth   = 100;
HeadYawLabelTBHeight  = 20;

HeadYawLabelTB = uicontrol('Parent', HeadPanel, 'style','text', 'string', 'Head Yaw', ...
                           'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(HeadYawLabelTB, [HeadYawLabelTBX, HeadYawLabelTBY, ...
                                  HeadYawLabelTBWidth, HeadYawLabelTBHeight]);   
                      
% Right arm shoulder pitch.
RArmShoulderPitchLabelTBX       = tbPadding;
RArmShoulderPitchLabelTBY       = 4 * 20 + 2 * panelPadding + 4 * tbPadding + 3;
RArmShoulderPitchLabelTBWidth   = 100;
RArmShoulderPitchLabelTBHeight  = 20;

RArmShoulderPitchLabelTB = uicontrol('Parent', RArmPanel, 'style','text', 'string', 'Shoulder Pitch', ...
                                     'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RArmShoulderPitchLabelTB, [RArmShoulderPitchLabelTBX, RArmShoulderPitchLabelTBY, ...
                                            RArmShoulderPitchLabelTBWidth, RArmShoulderPitchLabelTBHeight]);   
                              
% Right arm shoulder roll.
RArmShoulderRollLabelTBX       = tbPadding;
RArmShoulderRollLabelTBY       = 3 * 20 + 2 * panelPadding + 3 * tbPadding + 3;
RArmShoulderRollLabelTBWidth   = 100;
RArmShoulderRollLabelTBHeight  = 20;

RArmShoulderRollLabelTB = uicontrol('Parent', RArmPanel, 'style','text', 'string', 'Shoulder Roll', ...
                                    'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RArmShoulderRollLabelTB, [RArmShoulderRollLabelTBX, RArmShoulderRollLabelTBY, ...
                                           RArmShoulderRollLabelTBWidth, RArmShoulderRollLabelTBHeight]);   
                                        
% Right arm elbow yaw.
RArmElbowYawLabelTBX       = tbPadding;
RArmElbowYawLabelTBY       = 2 * 20 + 2 * panelPadding + 2 * tbPadding + 3;
RArmElbowYawLabelTBWidth   = 100;
RArmElbowYawLabelTBHeight  = 20;

RArmElbowYawLabelTB = uicontrol('Parent', RArmPanel, 'style','text', 'string', 'Elbow Yaw', ...
                                'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RArmElbowYawLabelTB, [RArmElbowYawLabelTBX, RArmElbowYawLabelTBY, ...
                                       RArmElbowYawLabelTBWidth, RArmElbowYawLabelTBHeight]);   
                    
% Right arm elbow roll.
RArmElbowRollLabelTBX       = tbPadding;
RArmElbowRollLabelTBY       = 1 * 20 + 2 * panelPadding + 1 * tbPadding + 3;
RArmElbowRollLabelTBWidth   = 100;
RArmElbowRollLabelTBHeight  = 20;

RArmElbowRollLabelTB = uicontrol('Parent', RArmPanel, 'style','text', 'string', 'Elbow Roll', ...
                                 'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RArmElbowRollLabelTB, [RArmElbowRollLabelTBX, RArmElbowRollLabelTBY, ...
                                        RArmElbowRollLabelTBWidth, RArmElbowRollLabelTBHeight]);   

% Left arm shoulder pitch.
LArmShoulderPitchLabelTBX       = tbPadding;
LArmShoulderPitchLabelTBY       = 4 * 20 + 2 * panelPadding + 4 * tbPadding + 3;
LArmShoulderPitchLabelTBWidth   = 100;
LArmShoulderPitchLabelTBHeight  = 20;

LArmShoulderPitchLabelTB = uicontrol('Parent', LArmPanel, 'style','text', 'string', 'Shoulder Pitch', ...
                                     'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LArmShoulderPitchLabelTB, [LArmShoulderPitchLabelTBX, LArmShoulderPitchLabelTBY, ...
                                            LArmShoulderPitchLabelTBWidth, LArmShoulderPitchLabelTBHeight]);   
                              
% Left arm shoulder roll.
LArmShoulderRollLabelTBX       = tbPadding;
LArmShoulderRollLabelTBY       = 3 * 20 + 2 * panelPadding + 3 * tbPadding + 3;
LArmShoulderRollLabelTBWidth   = 100;
LArmShoulderRollLabelTBHeight  = 20;

LArmShoulderRollLabelTB = uicontrol('Parent', LArmPanel, 'style','text', 'string', 'Shoulder Roll', ...
                                    'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LArmShoulderRollLabelTB, [LArmShoulderRollLabelTBX, LArmShoulderRollLabelTBY, ...
                                           LArmShoulderRollLabelTBWidth, LArmShoulderRollLabelTBHeight]);   
                                        
% Left arm elbow yaw.
LArmElbowYawLabelTBX       = tbPadding;
LArmElbowYawLabelTBY       = 2 * 20 + 2 * panelPadding + 2 * tbPadding + 3;
LArmElbowYawLabelTBWidth   = 100;
LArmElbowYawLabelTBHeight  = 20;

LArmElbowYawLabelTB = uicontrol('Parent', LArmPanel, 'style','text', 'string', 'Elbow Yaw', ...
                                'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LArmElbowYawLabelTB, [LArmElbowYawLabelTBX, LArmElbowYawLabelTBY, ...
                                       LArmElbowYawLabelTBWidth, LArmElbowYawLabelTBHeight]);   
                    
% Left arm elbow roll.
LArmElbowRollLabelTBX       = tbPadding;
LArmElbowRollLabelTBY       = 1 * 20 + 2 * panelPadding + 1 * tbPadding + 3;
LArmElbowRollLabelTBWidth   = 100;
LArmElbowRollLabelTBHeight  = 20;

LArmElbowRollLabelTB = uicontrol('Parent', LArmPanel, 'style','text', 'string', 'Elbow Roll', ...
                                 'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LArmElbowRollLabelTB, [LArmElbowRollLabelTBX, LArmElbowRollLabelTBY, ...
                                        LArmElbowRollLabelTBWidth, LArmElbowRollLabelTBHeight]);   
                                                 
% Right leg hip pitch.
RLegHipPitchLabelTBX       = tbPadding;
RLegHipPitchLabelTBY       = 5 * 20 + 2 * panelPadding + 5 * tbPadding + 3;
RLegHipPitchLabelTBWidth   = 100;
RLegHipPitchLabelTBHeight  = 20;

RLegHipPitchLabelTB = uicontrol('Parent', RLegPanel, 'style','text', 'string', 'Hip Pitch', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RLegHipPitchLabelTB, [RLegHipPitchLabelTBX, RLegHipPitchLabelTBY, ...
                                      RLegHipPitchLabelTBWidth, RLegHipPitchLabelTBHeight]);   
 
% Right leg hip roll.
RLegHipRollLabelTBX       = tbPadding;
RLegHipRollLabelTBY       = 4 * 20 + 2 * panelPadding + 4 * tbPadding + 3;
RLegHipRollLabelTBWidth   = 100;
RLegHipRollLabelTBHeight  = 20;

RLegHipRollLabelTB = uicontrol('Parent', RLegPanel, 'style','text', 'string', 'Hip Roll', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RLegHipRollLabelTB, [RLegHipRollLabelTBX, RLegHipRollLabelTBY, ...
                                      RLegHipRollLabelTBWidth, RLegHipRollLabelTBHeight]);  
       
% Right leg knee pitch.
RLegKneePitchLabelTBX       = tbPadding;
RLegKneePitchLabelTBY       = 3 * 20 + 2 * panelPadding + 3 * tbPadding + 3;
RLegKneePitchLabelTBWidth   = 100;
RLegKneePitchLabelTBHeight  = 20;

RLegKneePitchLabelTB = uicontrol('Parent', RLegPanel, 'style','text', 'string', 'Knee Pitch', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RLegKneePitchLabelTB, [RLegKneePitchLabelTBX, RLegKneePitchLabelTBY, ...
                                      RLegKneePitchLabelTBWidth, RLegKneePitchLabelTBHeight]);  
       
% Right leg ankle pitch.
RLegAnklePitchLabelTBX       = tbPadding;
RLegAnklePitchLabelTBY       = 2 * 20 + 2 * panelPadding + 2 * tbPadding + 3;
RLegAnklePitchLabelTBWidth   = 100;
RLegAnklePitchLabelTBHeight  = 20;

RLegAnklePitchLabelTB = uicontrol('Parent', RLegPanel, 'style','text', 'string', 'Ankle Pitch', ...
                                  'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RLegAnklePitchLabelTB, [RLegAnklePitchLabelTBX, RLegAnklePitchLabelTBY, ...
                                         RLegAnklePitchLabelTBWidth, RLegAnklePitchLabelTBHeight]);  

% Right leg ankle roll.
RLegAnkleRollLabelTBX       = tbPadding;
RLegAnkleRollLabelTBY       = 1 * 20 + 2 * panelPadding + 1 * tbPadding + 3;
RLegAnkleRollLabelTBWidth   = 100;
RLegAnkleRollLabelTBHeight  = 20;

RLegAnkleRollLabelTB = uicontrol('Parent', RLegPanel, 'style','text', 'string', 'Ankle Roll', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(RLegAnkleRollLabelTB, [RLegAnkleRollLabelTBX, RLegAnkleRollLabelTBY, ...
                                      RLegAnkleRollLabelTBWidth, RLegAnkleRollLabelTBHeight]);  

% Left leg hip pitch.
LLegHipPitchLabelTBX       = tbPadding;
LLegHipPitchLabelTBY       = 6 * 20 + 2 * panelPadding + 6 * tbPadding + 3;
LLegHipPitchLabelTBWidth   = 100;
LLegHipPitchLabelTBHeight  = 20;

LLegHipPitchLabelTB = uicontrol('Parent', LLegPanel, 'style','text', 'string', 'Hip Pitch', ...
                                'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LLegHipPitchLabelTB, [LLegHipPitchLabelTBX, LLegHipPitchLabelTBY, ...
                                       LLegHipPitchLabelTBWidth, LLegHipPitchLabelTBHeight]);   
 
% Left leg hip roll.
LLegHipRollLabelTBX       = tbPadding;
LLegHipRollLabelTBY       = 5 * 20 + 2 * panelPadding + 5 * tbPadding + 3;
LLegHipRollLabelTBWidth   = 100;
LLegHipRollLabelTBHeight  = 20;

LLegHipRollLabelTB = uicontrol('Parent', LLegPanel, 'style','text', 'string', 'Hip Roll', ...
                               'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LLegHipRollLabelTB, [LLegHipRollLabelTBX, LLegHipRollLabelTBY, ...
                                      LLegHipRollLabelTBWidth, LLegHipRollLabelTBHeight]);  

% Left leg hip yaw pitch.
LLegHipYawPitchLabelTBX       = tbPadding;
LLegHipYawPitchLabelTBY       = 4 * 20 + 2 * panelPadding + 4 * tbPadding + 3;
LLegHipYawPitchLabelTBWidth   = 100;
LLegHipYawPitchLabelTBHeight  = 20;

LLegHipYawPitchLabelTB = uicontrol('Parent', LLegPanel, 'style','text', 'string', 'Hip Yaw Pitch', ...
                                   'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LLegHipYawPitchLabelTB, [LLegHipYawPitchLabelTBX, LLegHipYawPitchLabelTBY, ...
                                          LLegHipYawPitchLabelTBWidth, LLegHipYawPitchLabelTBHeight]);  
                 
% Left leg knee pitch.
LLegKneePitchLabelTBX       = tbPadding;
LLegKneePitchLabelTBY       = 3 * 20 + 2 * panelPadding + 3 * tbPadding + 3;
LLegKneePitchLabelTBWidth   = 100;
LLegKneePitchLabelTBHeight  = 20;

LLegKneePitchLabelTB = uicontrol('Parent', LLegPanel, 'style','text', 'string', 'Knee Pitch', ...
                                 'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LLegKneePitchLabelTB, [LLegKneePitchLabelTBX, LLegKneePitchLabelTBY, ...
                                        LLegKneePitchLabelTBWidth, LLegKneePitchLabelTBHeight]);  
                
% Left leg ankle pitch.
LLegAnklePitchLabelTBX       = tbPadding;
LLegAnklePitchLabelTBY       = 2 * 20 + 2 * panelPadding + 2 * tbPadding + 3;
LLegAnklePitchLabelTBWidth   = 100;
LLegAnklePitchLabelTBHeight  = 20;

LLegAnklePitchLabelTB = uicontrol('Parent', LLegPanel, 'style','text', 'string', 'Ankle Pitch', ...
                                  'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LLegAnklePitchLabelTB, [LLegAnklePitchLabelTBX, LLegAnklePitchLabelTBY, ...
                                         LLegAnklePitchLabelTBWidth, LLegAnklePitchLabelTBHeight]);  

% Left leg ankle roll.
LLegAnkleRollLabelTBX       = tbPadding;
LLegAnkleRollLabelTBY       = 1 * 20 + 2 * panelPadding + 1 * tbPadding + 3;
LLegAnkleRollLabelTBWidth   = 100;
LLegAnkleRollLabelTBHeight  = 20;

LLegAnkleRollLabelTB = uicontrol('Parent', LLegPanel, 'style','text', 'string', 'Ankle Roll', ...
                                 'FontSize', 12, 'HorizontalAlignment', 'right');
setpixelposition(LLegAnkleRollLabelTB, [LLegAnkleRollLabelTBX, LLegAnkleRollLabelTBY, ...
                                        LLegAnkleRollLabelTBWidth, LLegAnkleRollLabelTBHeight]);  

%% Keyframe generator state.
% Create a cell containing all edit box handles.
editHandles = {LArmShoulderPitchSensorEditTB,  LArmShoulderPitchStiffnessEditTB;
               LArmShoulderRollSensorEditTB,   LArmShoulderRollStiffnessEditTB;
               LArmElbowYawSensorEditTB,       LArmElbowYawStiffnessEditTB;
               LArmElbowRollSensorEditTB,      LArmElbowRollStiffnessEditTB;
               [],                             [];
               RArmShoulderPitchSensorEditTB,  RArmShoulderPitchStiffnessEditTB;
               RArmShoulderRollSensorEditTB,   RArmShoulderRollStiffnessEditTB;
               RArmElbowYawSensorEditTB,       RArmElbowYawStiffnessEditTB;
               RArmElbowRollSensorEditTB,      RArmElbowRollStiffnessEditTB;
               [],                             [];
               LLegHipYawPitchSensorEditTB,    LLegHipYawPitchStiffnessEditTB;
               LLegHipRollSensorEditTB,        LLegHipRollStiffnessEditTB;
               LLegHipPitchSensorEditTB,       LLegHipPitchStiffnessEditTB;
               LLegKneePitchSensorEditTB,      LLegKneePitchStiffnessEditTB;
               LLegAnkleRollSensorEditTB,      LLegAnkleRollStiffnessEditTB;
               LLegAnklePitchSensorEditTB,     LLegAnklePitchStiffnessEditTB;
               RLegHipRollSensorEditTB,        RLegHipRollStiffnessEditTB;
               RLegHipPitchSensorEditTB,       RLegHipPitchStiffnessEditTB;
               RLegKneePitchSensorEditTB,      RLegKneePitchStiffnessEditTB;
               RLegAnklePitchSensorEditTB,     RLegAnklePitchStiffnessEditTB;
               RLegAnkleRollSensorEditTB,      RLegAnkleRollStiffnessEditTB;
               HeadYawSensorEditTB,            HeadYawStiffnessEditTB;
               HeadPitchSensorEditTB,          HeadPitchStiffnessEditTB};
              
% Create a cell containing all display box handles.
displayHandles = {LArmShoulderPitchSensorDispTB,  LArmShoulderPitchStiffnessDispTB;
                  LArmShoulderRollSensorDispTB,   LArmShoulderRollStiffnessDispTB;
                  LArmElbowYawSensorDispTB,       LArmElbowYawStiffnessDispTB;
                  LArmElbowRollSensorDispTB,      LArmElbowRollStiffnessDispTB;
                  [],                             [];
                  RArmShoulderPitchSensorDispTB,  RArmShoulderPitchStiffnessDispTB;
                  RArmShoulderRollSensorDispTB,   RArmShoulderRollStiffnessDispTB;
                  RArmElbowYawSensorDispTB,       RArmElbowYawStiffnessDispTB;
                  RArmElbowRollSensorDispTB,      RArmElbowRollStiffnessDispTB;
                  [],                             [];
                  LLegHipYawPitchSensorDispTB,    LLegHipYawPitchStiffnessDispTB;
                  LLegHipRollSensorDispTB,        LLegHipRollStiffnessDispTB;
                  LLegHipPitchSensorDispTB,       LLegHipPitchStiffnessDispTB;
                  LLegKneePitchSensorDispTB,      LLegKneePitchStiffnessDispTB;
                  LLegAnkleRollSensorDispTB,      LLegAnkleRollStiffnessDispTB;
                  LLegAnklePitchSensorDispTB,     LLegAnklePitchStiffnessDispTB;
                  RLegHipRollSensorDispTB,        RLegHipRollStiffnessDispTB;
                  RLegHipPitchSensorDispTB,       RLegHipPitchStiffnessDispTB;
                  RLegKneePitchSensorDispTB,      RLegKneePitchStiffnessDispTB;
                  RLegAnklePitchSensorDispTB,     RLegAnklePitchStiffnessDispTB;
                  RLegAnkleRollSensorDispTB,      RLegAnkleRollStiffnessDispTB;
                  HeadYawSensorDispTB,            HeadYawStiffnessDispTB;
                  HeadPitchSensorDispTB,          HeadPitchStiffnessDispTB};

% First define structure of motion structure.
motionStruct = struct('name', {}, 'motions', {});

% Check if a motion matfile exists.
if (exist('TestingModules/Locomotion/Keyframe_Generator/motions.mat') == 2)
    load('TestingModules/Locomotion/Keyframe_Generator/motions.mat');
end

Keyframe_Generator_Update_State(true, true, false, false);
                  
% Create a timer to update current actuator position/hardness values.
timerHandle = timer('ExecutionMode', 'fixedRate', ...     % Run timer repeatedly
                    'Period', 1, ...                    % Initial period is 1 sec.
                    'TimerFcn', {@Keyframe_Generator_callBacks, 'timer'}); % Specify callback
start(timerHandle);

% Protect global variables.
panelPadding = oldPanelPadding;        
lbPadding    = oldLBPadding;

end