function [] = Locomotion_Template()
% Establish global variables.
global fig figHeight mainConfigPanelWidth mainConfigPanelHeight ...
       panelPadding
                              
% Set current figure.
set(0, 'CurrentFigure', fig);

% Test Module
testModulePanelWidth  = mainConfigPanelWidth;
testModulePanelHeight = figHeight - mainConfigPanelHeight - 2 * panelPadding;
testModulePanelX      = panelPadding + 1;
testModulePanelY      = panelPadding;

testModulePanel = uipanel('Title', 'Test Module', ...
                          'FontSize', 12);
setpixelposition(testModulePanel, [testModulePanelX testModulePanelY ...
                                   testModulePanelWidth testModulePanelHeight]);
end