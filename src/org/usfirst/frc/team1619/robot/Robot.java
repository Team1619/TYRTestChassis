
package org.usfirst.frc.team1619.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	private final double[] DEFAULT_ARRAY = new double[0];
	private final String[] KEYS = {"centerX", "centerY", "area",  "height", "width"};
	private final String TABLE_NAME = "GRIP/contours";
	
	private Joystick rightStick;
	private JoystickButton followButton; 
	
	
	private CANTalon leftDriveMotor1;
	private CANTalon leftDriveMotor2;
	private CANTalon rightDriveMotor1;
	private CANTalon rightDriveMotor2;
	private static final int leftDrive1ID = 1;
	private static final int leftDrive2ID = 2;
	private static final int rightDrive1ID = 3;
	private static final int rightDrive2ID = 4;
	private RobotDrive drive;
	
	NetworkTable camTable;
	
	private boolean turning;
	private double[][] currentValues;
	
	private int imageRecievePeriod = 100;
	private int imageRecieveValue = 0;
	
	public Robot() {
		rightStick = new Joystick(0);
    	
		followButton = new JoystickButton(rightStick, 1);
		
    	leftDriveMotor1 = new CANTalon(leftDrive1ID);
    	leftDriveMotor2 = new CANTalon(leftDrive2ID);
    	rightDriveMotor1 = new CANTalon(rightDrive1ID);
    	rightDriveMotor2 = new CANTalon(rightDrive2ID);
    	
    	drive = new RobotDrive(leftDriveMotor1, leftDriveMotor2, rightDriveMotor1, rightDriveMotor2);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
    	
    	camTable = NetworkTable.getTable(TABLE_NAME);
    	
    	 turning = false;
	}
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    }

    public void disabledPeriodic() {
    	if (imageRecieveValue > imageRecievePeriod) {
    		currentValues = getTableVals();
        	if (currentValues[0].length > 0) {
        		System.out.println(currentValues[0][0]);
        	}
        	imageRecieveValue = 0;
    	}
    	imageRecieveValue++;
    	
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        if (followButton.get()) {
        	currentValues = getTableVals();
        	turnToContour();
        }
        else {
        	turning = false;
        	driveZ(rightStick);	
        }
    	
        if (imageRecieveValue > imageRecievePeriod) {
    		currentValues = getTableVals();
        	if (currentValues[0].length > 0) {
        		System.out.println(currentValues[0][0]);
        	}
        	imageRecieveValue = 0;
    	}
    	imageRecieveValue++;
        
        SmartDashboard.putNumber("Left Encoder", leftDriveMotor1.getEncPosition());
        SmartDashboard.putNumber("Right Encoder", rightDriveMotor1.getEncPosition());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    private void driveZ(GenericHID input) {
    	drive.arcadeDrive(input.getY(), input.getTwist());
    }
    
    @SuppressWarnings("unused")
	private void driveX(GenericHID input) {
    	drive.arcadeDrive(input.getY(), input.getX());
    }
    
    /**
     * 
     * @param speed between 1 and 0
     */
    private void driveTurn(double speed) {
    	drive.arcadeDrive(0, speed);
    }
    
    private double[][] getTableVals() {
    	double[][] vals = new double[5][];
    	for (int i = 0; i < KEYS.length; i++) {
    		vals[i] = camTable.getNumberArray(KEYS[i], DEFAULT_ARRAY);
    	}
    	
    	return vals;
    }
    
    private void turnToContour() {
    	if (currentValues[0].length == 1) {
    		if (currentValues[0][0] > 170) {
    			driveTurn(0.25 + ((currentValues[0][0]-170)/170) * 0.3);
    		}
    		else {
    			driveTurn(-(0.25 + ((170 - currentValues[0][0])/170) * 0.3));
    		}
    	}
    	else {
    		driveTurn(-0.3);
    	}
    }
}
