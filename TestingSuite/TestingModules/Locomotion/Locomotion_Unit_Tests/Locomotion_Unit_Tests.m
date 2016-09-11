function [] = Locomotion_Unit_Tests()
               
% Declare global variables.
global fig figHeight mainConfigPanelWidth mainConfigPanelHeight ...
       panelPadding testResultsTB selectUnitTestPopup

% Set current figure.
set(0, 'CurrentFigure', fig);

% Test Module
testModulePanelWidth  = mainConfigPanelWidth;
testModulePanelHeight = figHeight - mainConfigPanelHeight - 2 * panelPadding;
testModulePanelX      = panelPadding + 1;
testModulePanelY      = panelPadding;

testModulePanel = uipanel('Title', 'Locomotion Unit Test Module', ...
                          'FontSize', 12);
setpixelposition(testModulePanel, [testModulePanelX testModulePanelY ...
                                   testModulePanelWidth testModulePanelHeight]);
                               
% Select unit test to run.
selectUnitTestWidth  = 400;
selectUnitTestHeight = 50;
selectUnitTestPanelX = (testModulePanelWidth - selectUnitTestWidth) / 2;
selectUnitTestPanelY = testModulePanelHeight - selectUnitTestHeight - panelPadding;
    
selectUnitTestPanel = uipanel('Parent', testModulePanel, 'Title', 'Select Unit Test', ...
                          'FontSize', 12);
setpixelposition(selectUnitTestPanel, [selectUnitTestPanelX selectUnitTestPanelY ...
                                       selectUnitTestWidth selectUnitTestHeight]);

popupPadding = 5;
pbWidth = 100;
           
selectUnitTestPopupX = popupPadding;
selectUnitTestPopupY = 10;
selectUnitTestPopupWidth  = selectUnitTestWidth - 2 * popupPadding - pbWidth - 15;
selectUnitTestPopupHeight = 20;

selectUnitTestPopup = uicontrol('Parent', selectUnitTestPanel, 'Style', 'popup',...
                                'String', {'COMContainerTests', ...
                                           'TransformTests', ...
                                           'TrajectoryTests', ...
                                           'PointTests', ...
                                           'KinematicsTests', ...
                                           'StepHandlerTests'});
              
setpixelposition(selectUnitTestPopup, [selectUnitTestPopupX selectUnitTestPopupY ...
                                       selectUnitTestPopupWidth selectUnitTestPopupHeight]);

% Run Test Button
runTestPBWidth  = 100;
runTestPBHeight = 20;
runTestPBX      = popupPadding + selectUnitTestPopupWidth + popupPadding;
runTestPBY      = 10;    

runTestBtn = uicontrol('Parent', selectUnitTestPanel, ...
                       'Style','pushbutton','String','Run Test', 'callback', ...
                        {@Locomotion_Unit_Tests_callBacks, 'pbCallback'});            
setpixelposition(runTestBtn, [runTestPBX, runTestPBY, runTestPBWidth, runTestPBHeight]);

% Test results.
testResultsTB = uicontrol('Parent', testModulePanel, 'Style','edit', ...
                          'min',0,'max',2,'enable','inactive');
                      
setpixelposition(testResultsTB, [panelPadding, panelPadding, ...
                                 testModulePanelWidth - 2 * panelPadding - 2, selectUnitTestPanelY - 2 * panelPadding]);
end