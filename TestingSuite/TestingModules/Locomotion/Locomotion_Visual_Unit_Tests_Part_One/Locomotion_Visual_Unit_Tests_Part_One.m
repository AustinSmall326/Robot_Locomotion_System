function [] = Locomotion_Visual_Unit_Tests()
               
% Declare global variables.
global fig figHeight mainConfigPanelWidth mainConfigPanelHeight ...
       panelPadding selectUnitTestPopup plotHandle visualTestStruct ...
       selectOrientationPopup

% Set current figure.
set(0, 'CurrentFigure', fig);

% Test Module
testModulePanelWidth  = mainConfigPanelWidth;
testModulePanelHeight = figHeight - mainConfigPanelHeight - 2 * panelPadding;
testModulePanelX      = panelPadding + 1;
testModulePanelY      = panelPadding;

testModulePanel = uipanel('Title', 'Locomotion Visual Unit Test Module', ...
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
                                'String', {'StepHandlerVisualTestsPartOne', ...
                                           'StepHandlerVisualTestsPartTwo'});
              
setpixelposition(selectUnitTestPopup, [selectUnitTestPopupX selectUnitTestPopupY ...
                                       selectUnitTestPopupWidth selectUnitTestPopupHeight]);

% Run Test Button
runTestPBWidth  = 100;
runTestPBHeight = 20;
runTestPBX      = popupPadding + selectUnitTestPopupWidth + popupPadding;
runTestPBY      = 10;    

runTestBtn = uicontrol('Parent', selectUnitTestPanel, ...
                       'Style','pushbutton','String','Run Test', 'callback', ...
                        {@Locomotion_Visual_Unit_Tests_callBacks, 'pbCallback'});            
setpixelposition(runTestBtn, [runTestPBX, runTestPBY, runTestPBWidth, runTestPBHeight]);

% Test results.
dispPlotPanelWidth  = testModulePanelWidth - 2 * panelPadding - 2;
dispPlotPanelHeight = selectUnitTestPanelY - 3 * panelPadding - 20;
dispPlotPanelX      = panelPadding;
dispPlotPanelY      = 2 * panelPadding + 20;

dispPlotPanel = uipanel('Parent', testModulePanel, 'Title', 'Visualize Results', ...
                        'FontSize', 12, 'BackgroundColor', 'white');
setpixelposition(dispPlotPanel, [dispPlotPanelX dispPlotPanelY ...
                                 dispPlotPanelWidth dispPlotPanelHeight]);

plotHandle = subplot(1, 1, 1, 'Parent', dispPlotPanel);

% Navigate test results button.
leftPBWidth  = 40;
leftPBHeight = 20;
leftPBX      = panelPadding;
leftPBY      = panelPadding;    

leftBtn = uicontrol('Parent', testModulePanel, ...
                    'FontName', 'wingdings', ...
                    'String', char(8678), ...
                    'Style','pushbutton', 'callback', ...
                    {@Locomotion_Visual_Unit_Tests_callBacks, 'leftCallback'});            
setpixelposition(leftBtn, [leftPBX, leftPBY, leftPBWidth, leftPBHeight]);

rightPBWidth  = 40;
rightPBHeight = 20;
rightPBX      = 2 * panelPadding + leftPBWidth;
rightPBY      = panelPadding;    

rightBtn = uicontrol('Parent', testModulePanel, ...
                     'FontName', 'wingdings', ...
                     'String', char(8680), ...
                     'Style','pushbutton', 'callback', ...
                     {@Locomotion_Visual_Unit_Tests_callBacks, 'rightCallback'});            
setpixelposition(rightBtn, [rightPBX, rightPBY, rightPBWidth, rightPBHeight]);

% Popup to select 3D plot orientation.
selectOrientationPopupY = panelPadding - 1;
selectOrientationPopupWidth  = 300;
selectOrientationPopupHeight = 20;
selectOrientationPopupX = dispPlotPanelX + dispPlotPanelWidth - selectOrientationPopupWidth + 6;

selectOrientationPopup = uicontrol('Parent', testModulePanel, 'Style', 'popup',...
                                   'String', {'Perspective View', ...
                                              'XY Axis', ...
                                              'YZ Axis', ...
                                              'XZ Axis'}, ...
                                   'callback', {@Locomotion_Visual_Unit_Tests_callBacks, 'popupOrientationCallback'});
              
setpixelposition(selectOrientationPopup, [selectOrientationPopupX selectOrientationPopupY ...
                                          selectOrientationPopupWidth selectOrientationPopupHeight]);





% Declare struct for storing plot data.
visualTestStruct = struct('TestName', {}, 'XPoints', {}, 'YPoints', {}, 'ZPoints', {});

end