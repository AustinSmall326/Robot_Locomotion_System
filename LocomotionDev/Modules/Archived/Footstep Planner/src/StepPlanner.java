/*
 * File Name:	StepPlanner.java
 * Author:		Austin Small
 * Data:		11/8/2015
 * Description:	This file allows for GUI creation of path trajectories
 * 				and accompanying footsteps for the purpose of easily
 * 				testing walking.
 */
import java.awt.Color;
import java.awt.Container;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.*;        
import javax.swing.SpringLayout;
import java.awt.GridLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.*; 

public class StepPlanner {
	/** Path parameters. **/
	// Total distance parameters (m).
	private static double distanceVal = 0.0;
	private static double distanceMin = 0.0;
	private static double distanceMax = 10.0;
	private static double distanceDelta = 0.1;
	
	// Step length parameters (m).
	private static double lengthVal = 0.01;
	private static double lengthMin = 0.01;
	private static double lengthMax = 0.1;
	private static double lengthDelta = 0.001;
	
	// Step width parameters (m).
	private static double widthVal = 0.0;
	private static double widthMin = 0.0;
	private static double widthMax = 0.2;
	private static double widthDelta = 0.001;
	
	// JPanel canvas.
	private static JPanel canvasPanel;
	
	// Array to store step placement.
	private static double[][] leftSteps = null;
	private static double[][] rightSteps = null;
	
	// Step progression type. (Always lowercase)
	private static String progType = "none";
	
    private static void createAndShowGUI() {
        // Create and set up the window.
        JFrame frame = new JFrame("Step Planner");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        Container contentPane = frame.getContentPane();

        // Create a toolbox label.
        JLabel toolboxLabel = new JLabel("Toolbox");
        contentPane.add(toolboxLabel);
        
        /*** Create toolbox items. ***/
        // Label for path distance.
        JLabel distanceLabel = new JLabel("Path Distance (m)");
        distanceLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 0, 0));
        
        // Incrementer box for path distance.
        SpinnerModel smDistance = new SpinnerNumberModel(distanceVal, distanceMin, distanceMax, distanceDelta); //default value,lower bound,upper bound,increment by
        JSpinner spinnerDistance = new JSpinner(smDistance);
        spinnerDistance.addChangeListener(new ChangeListener() {
        	@Override
        	public void stateChanged(ChangeEvent e) {
        		double changedVal = (double) spinnerDistance.getValue();
        		
        		// Update distance value.
        		if (changedVal < distanceMin) {
        			spinnerDistance.setValue(distanceMin);
        			distanceVal = distanceMin;
        		}
        		else if (changedVal > distanceMax) {
        			spinnerDistance.setValue(distanceMax);
        			distanceVal = distanceMax;
        		}
        		else {
        			distanceVal = changedVal;
        		}
        		
        		recalculateSteps();
        		canvasPanel.repaint();
        	}
        });
        
        // Label for length of footsteps.
        JLabel stepLengthLabel = new JLabel("Step Length (cm)");
        stepLengthLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 0, 0));
        
        // Incrementer box for length of footsteps.
        SpinnerModel smLengthSteps = new SpinnerNumberModel(lengthVal * 100, lengthMin * 100, lengthMax * 100, lengthDelta * 100); //default value,lower bound,upper bound,increment by
        JSpinner spinnerLengthSteps = new JSpinner(smLengthSteps);
        spinnerLengthSteps.addChangeListener(new ChangeListener() {
        	@Override
        	public void stateChanged(ChangeEvent e) {
        		double changedVal = (double) spinnerLengthSteps.getValue() / 100;
        		
        		if (changedVal < lengthMin) {
        			spinnerLengthSteps.setValue(lengthMin);
        			lengthVal = lengthMin;
        		}
        		else if (changedVal > lengthMax) {
        			spinnerLengthSteps.setValue(lengthMax);
        			lengthVal = lengthMax;
        		}
        		else {
        			lengthVal = changedVal;
        		}
        		
        		recalculateSteps();
        		canvasPanel.repaint();
        	}
        });
        
        // Label for width of footsteps.
        JLabel stepWidthLabel = new JLabel("Step Width (cm)");
        stepWidthLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 0, 0));
        
        // Incrementer box for width of footsteps.
        SpinnerModel smWidthSteps = new SpinnerNumberModel(widthVal * 100, widthMin * 100, widthMax * 100, widthDelta * 100); //default value,lower bound,upper bound,increment by
        JSpinner spinnerWidthSteps = new JSpinner(smWidthSteps);
        spinnerWidthSteps.addChangeListener(new ChangeListener() {
        	@Override
        	public void stateChanged(ChangeEvent e) {
        		double changedVal = (double) spinnerWidthSteps.getValue() / 100;
        		
        		if (changedVal < widthMin) {
        			spinnerWidthSteps.setValue(widthMin);
        			widthVal = widthMin;
        		}
        		else if (changedVal > widthMax) {
        			spinnerWidthSteps.setValue(widthMax);
        			widthVal = widthMax;
        		}
        		else {
        			widthVal = changedVal;
        		}
        		
        		recalculateSteps();
        		canvasPanel.repaint();
        	}
        });
        
        // Label for constant foot steps.
        JLabel constantStepLabel = new JLabel("Step Progression");
        constantStepLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 0, 0));
        
        // Radio button for constant foot steps.
        JRadioButton constantStepButton = new JRadioButton("Constant");
        constantStepButton.addChangeListener(new ChangeListener() {
        	@Override
        	public void stateChanged(ChangeEvent e) {
        		if (constantStepButton.isSelected()) {
        			progType = "constant";
        		}
        		
        		recalculateSteps();
        		canvasPanel.repaint();
        	}
        });
        
        // Empty label.
        JLabel emptyLabel = new JLabel("");
        
        // Radio button for linearly progressed foot steps.
        JRadioButton linearStepButton = new JRadioButton("Linear");
        linearStepButton.addChangeListener(new ChangeListener() {
        	@Override
        	public void stateChanged(ChangeEvent e) {
        		if (linearStepButton.isSelected()) {
        			progType = "linear";
        		}
        		
        		recalculateSteps();
        		canvasPanel.repaint();
        	}
        });
        
        // Button group to store radio buttons for step progression.
        ButtonGroup bG = new ButtonGroup();
        bG.add(constantStepButton);
        bG.add(linearStepButton);
        
        // Panel to store toolbox items.
        JPanel panel = new JPanel();
        panel.setBackground(Color.gray);
        panel.setLayout(new GridLayout(5, 2, 10, 0));
        panel.add(distanceLabel);
        panel.add(spinnerDistance);
        panel.add(stepLengthLabel);
        panel.add(spinnerLengthSteps);
        panel.add(stepWidthLabel);
        panel.add(spinnerWidthSteps);
        panel.add(constantStepLabel);
        panel.add(constantStepButton);
        panel.add(emptyLabel);
        panel.add(linearStepButton);
        frame.add(panel);

        // Panel to act as canvas for visualizing planned footsteps.
        canvasPanel = new JPanel() {
        	public void paintComponent(Graphics g) {
        		g.setColor(Color.white);
        		g.fillRect(0, 0, 500, 500);
        		        		
        		if (distanceVal > 0) {
        			g.setColor(Color.black);   
            		g.drawLine(249, 49, 249, 449);
            		
            		if (leftSteps != null  && rightSteps != null) {
                		g.setColor(Color.blue);
                		
                		// Draw right footsteps.
                		for (int i = 0; i < rightSteps.length; i++) {
                			int y = 449 - (int) ((rightSteps[i][0] / distanceVal) * 400);
                			
                			// Scale - pixels / meter.
                			double scale = 400 / distanceVal;
                			
                			int x = 249 + (int) (rightSteps[i][1] * scale);
                			
                			g.fillOval(x - 2, y - 2, 4, 4);
                		}
                		
                		// Draw the left footsteps.
                		for (int i = 0; i < leftSteps.length; i++) {
                			int y = 449 - (int) ((leftSteps[i][0] / distanceVal) * 400);
                			
                			// Scale - pixels / meter.
                			double scale = 400 / distanceVal;
                			
                			int x = 249 + (int) (leftSteps[i][1] * scale);
                			
                			g.fillOval(x - 2, y - 2, 4, 4);
                		}
            		}
        		}
        	}
        };
        
        canvasPanel.setBackground(Color.white);
        canvasPanel.setPreferredSize(new Dimension(500, 500));

        frame.add(canvasPanel);
        
        SpringLayout contentLayout = new SpringLayout();
        contentPane.setLayout(contentLayout);
        

        
        // Constraints to position toolbox label.
        contentLayout.putConstraint(SpringLayout.WEST, toolboxLabel, 125, SpringLayout.WEST, contentPane);
        contentLayout.putConstraint(SpringLayout.NORTH, toolboxLabel, 10, SpringLayout.NORTH, contentPane);
       
        // Constraints to position Panel.
        contentLayout.putConstraint(SpringLayout.NORTH, panel, 20, SpringLayout.NORTH, toolboxLabel);
        contentLayout.putConstraint(SpringLayout.WEST, panel, 10, SpringLayout.WEST, contentPane);
        
        // Constraints to position canvas.
        contentLayout.putConstraint(SpringLayout.NORTH, canvasPanel, 0, SpringLayout.NORTH, toolboxLabel);
        contentLayout.putConstraint(SpringLayout.WEST, canvasPanel, 10, SpringLayout.EAST, panel);

        
        
        
        
        
        
        //Display the window.
        frame.pack();
        frame.setVisible(true);
        frame.setSize(770, 545);
    }
    
    // This method recalculates the step placements.
    private static void recalculateSteps () {
		if (distanceVal > 0) {
			if (lengthVal > 0) {
				if (!progType.equals("none")) {
					if (progType.equals("constant")) {
						// Determine number of steps to realize destination.
						int numCompleteLeftSteps = (int) (distanceVal / lengthVal) + 1;
						boolean extraLeftStep = (distanceVal % lengthVal) > 0;
						int totalLeftSteps = numCompleteLeftSteps;
						if (extraLeftStep) { totalLeftSteps++; }
						
						int numCompleteRightSteps = (int) ((distanceVal - lengthVal / 2) / lengthVal) + 1;
						boolean extraRightStep = ((distanceVal - lengthVal / 2) % lengthVal) > 0;
						boolean firstHalfStep = (distanceVal % (lengthVal / 2)) > 0;
						int totalRightSteps = numCompleteRightSteps;
						if (extraRightStep) { totalRightSteps++; }
						if (firstHalfStep) { totalRightSteps++; }
					
						leftSteps = new double[totalLeftSteps][2];
						rightSteps = new double[totalRightSteps][2];
						
						System.out.println(Integer.toString(totalLeftSteps));
						// Initial and guaranteed step values.
						leftSteps[0][0] = 0.0;
						leftSteps[0][1] = -1 * widthVal / 2;
						rightSteps[0][0] = 0.0;
						rightSteps[0][1] = widthVal / 2;
						
						// Final and guaranteed step values.
						leftSteps[leftSteps.length - 1][0] = distanceVal;
						leftSteps[leftSteps.length - 1][1] = -1 * widthVal / 2;
						rightSteps[rightSteps.length - 1][0] = distanceVal;
						rightSteps[rightSteps.length - 1][1] = widthVal / 2;
						
						// Left steps besides first and last.
						for (int i = 1; i < leftSteps.length - 1; i++) {
							leftSteps[i][0] = leftSteps[i - 1][0] + lengthVal;
							leftSteps[i][1] = -1 * widthVal / 2;
						}
						
						// Right steps besides first and last.
						for (int i = 1; i < rightSteps.length - 1; i++) {
							if (i == 1) { 
								rightSteps[1][0] = lengthVal / 2;
								rightSteps[1][1] = widthVal / 2;
							}
							else {
								rightSteps[i][0] = rightSteps[i - 1][0] + lengthVal;
								rightSteps[i][1] = widthVal / 2;
							}							
						}
					}
					else if (progType.equals("linear")) {
						List<Double> rightStepsY = new ArrayList<Double>();
						
						// Add starting point.
						rightStepsY.add(0.0);
						
						double slope = .01;			// Rate to increase/decrease step length.
						double traversedDistance = 0.0;
						double accStepLength = 0.0;     // Step length while increasing step size.

						for (; accStepLength + slope <= lengthVal && traversedDistance + accStepLength + slope <= (distanceVal / 2); ) {
							accStepLength += slope;
							traversedDistance += accStepLength;
							
							// Add new step to array.
							rightStepsY.add(traversedDistance);
							
							System.out.println(Double.toString(traversedDistance));
						}
						
						double tempLengthVal = lengthVal;
						
						// This condition should only be realized if total step length was not achieved.
						if (accStepLength < lengthVal) { tempLengthVal = accStepLength; }
						
						// Total steps/distance at constant step length.
						int addedSteps = 0;
						
						if (distanceVal > (2 * traversedDistance)) {
							addedSteps = (int) ((distanceVal - 2 * traversedDistance) / lengthVal) + 1;	
						}
						
						for (int i = 0; i < addedSteps; i++) {
							traversedDistance += tempLengthVal;
							
							// Add new step to array.
							rightStepsY.add(traversedDistance);
						}

							
						// Total steps/distance while decreasing step size.
						accStepLength = tempLengthVal;
						
						for (; traversedDistance + accStepLength - slope <= distanceVal && accStepLength - slope > 0; ) {
							accStepLength -= slope;
							traversedDistance += accStepLength;
							
							// Add new step to array.
							rightStepsY.add(traversedDistance);
						}
						
						for (; (distanceVal - traversedDistance) > accStepLength; ) {
							traversedDistance += accStepLength;
							
							// Add new step to array.
							rightStepsY.add(traversedDistance);
						}
						
						
						
						// If the robot hasn't reached final feet placements yet, add final destination.
						if (traversedDistance < distanceVal) { 
							rightStepsY.add(distanceVal);
						}
						
						// Convert ArrayLists into an array.
						rightSteps = new double[rightStepsY.size()][2];
						
						for (int i = 0; i < rightStepsY.size(); i++) {
							rightSteps[i][0] = rightStepsY.get(i);
							rightSteps[i][1] = widthVal / 2;
						}
						
						leftSteps = new double[rightStepsY.size()][2];
					}
				}
			}
		}
    }
    
    public static void main(String[] args) {
        //Schedule a job for the event-dispatching thread:
        //creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                createAndShowGUI();
            }
        });
    }
}