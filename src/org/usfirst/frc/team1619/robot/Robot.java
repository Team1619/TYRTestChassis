
package org.usfirst.frc.team1619.robot;

import edu.wpi.first.wpilibj.CANSpeedController.ControlMode;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
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
	
	private static final double ARM_MOTOR = 0.2;
	
	private final double[] DEFAULT_ARRAY = new double[0];
	private final String[] KEYS = {"centerX", "centerY", "area",  "height", "width"};
	private final String TABLE_NAME = "GRIP/contours";
	
	private final boolean TWIST_DRIVE = true;
	
	private Joystick rightStick;
	private JoystickButton followButton;
	
	private Joystick leftStick;
	private JoystickButton spareForward;
	private JoystickButton spareReverse;
	
	
	private CANTalon leftDriveMotor1;
	private CANTalon leftDriveMotor2;
	private CANTalon rightDriveMotor1;
	private CANTalon rightDriveMotor2;
	private CANTalon armMotor1;
	private CANTalon armMotor2;
	private CANTalon spareMotor;
	
	private static final int leftDrive1ID = 1;
	private static final int leftDrive2ID = 2;
	private static final int rightDrive1ID = 3;
	private static final int rightDrive2ID = 4;
	private static final int armMotor1ID = 11;
	private static final int armMotor2ID = 12;
	private static final int spareMotorID = 5;
	
	private RobotDrive drive;
	
	NetworkTable camTable;
	
	private double[][] currentValues;
	
	private int imageReceivePeriod = 50;
	private int imageReceiveValue = 0;
	
	private double prevImageX = 0;
	
	public Robot() {
		rightStick = new Joystick(0);
		leftStick = new Joystick(1);
    	
		followButton = new JoystickButton(rightStick, 1);
		
		spareForward = new JoystickButton(rightStick, 6);
		spareReverse = new JoystickButton(rightStick, 7);
		
    	leftDriveMotor1 = new CANTalon(leftDrive1ID);
    	leftDriveMotor2 = new CANTalon(leftDrive2ID);
    	rightDriveMotor1 = new CANTalon(rightDrive1ID);
    	rightDriveMotor2 = new CANTalon(rightDrive2ID);
    	armMotor1 = new CANTalon(armMotor1ID);
    	armMotor2 = new CANTalon(armMotor2ID);
    	spareMotor = new CANTalon(spareMotorID);
    	
    	drive = new RobotDrive(leftDriveMotor1, leftDriveMotor2, rightDriveMotor1, rightDriveMotor2);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
    	
    	armMotor1.setInverted(false);
    	armMotor2.setInverted(true);
//    	armMotor2.changeControlMode(TalonControlMode.Follower);
//    	armMotor2.set(armMotor1ID);
    	
    	spareMotor.setInverted(false);
    	
    	camTable = NetworkTable.getTable(TABLE_NAME);
	}
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    }

    public void disabledPeriodic() {
    	displayVals();
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
    	armManual(leftStick);
    	
    	spareMotor(rightStick, spareForward, spareReverse);
    	
    	drive(rightStick);
    	
        displayVals();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    private void displayVals() {
    	if (imageReceiveValue > imageReceivePeriod) {
    		double[][] tableVals = getTableVals();
            if (tableVals[0].length > 0) {
            	for (int i = 0; i < tableVals.length; i++) {
                	SmartDashboard.putNumber(KEYS[i], tableVals[i][0]);	
                }
            }
            imageReceiveValue -= imageReceivePeriod;
    	}
    	imageReceiveValue++;
    	
    	SmartDashboard.putNumber("Left Encoder", leftDriveMotor1.getEncPosition());
        SmartDashboard.putNumber("Right Encoder", rightDriveMotor1.getEncPosition());
    }
    
    private void drive(GenericHID input) {
    	if (followButton.get()) {
        	currentValues = getTableVals();
        	turnToContour();
        }
        else {        
	    	if (TWIST_DRIVE) {
	    		driveZ(input);
	    	}
	    	else {
	    		driveX(input);
	    	}
        }
    }
    
    private void driveZ(GenericHID input) {
    	drive.arcadeDrive(input.getY(), input.getTwist());
    }
    
	private void driveX(GenericHID input) {
    	drive.arcadeDrive(input.getY(), input.getX());
    }
    
    private void armManual(GenericHID input) {
    	armMotor1.set(input.getY() * ARM_MOTOR);
    	armMotor2.set(input.getY() * ARM_MOTOR);
    }
    
    private void spareMotor(GenericHID input, JoystickButton forward, JoystickButton reverse) {
    	double thrott = (input.getThrottle());
    	if(forward.get()) {
    		spareMotor.set(thrott);
    	}
    	else if(reverse.get()) {
    		spareMotor.set(-thrott);
    	}
    	else {
    		spareMotor.set(0);
    	}
    }
    
    /**
     * 
     * @param turnSpeed between 1 and 0
     */
    private void driveTurn(double turnSpeed) {
    	drive.arcadeDrive(0, turnSpeed);
    }
    
    private double[][] getTableVals() {
    	double[][] vals = new double[5][];
    	for (int i = 0; i < KEYS.length; i++) {
    		vals[i] = camTable.getNumberArray(KEYS[i], DEFAULT_ARRAY);
    	}
    	
    	return vals;
    }
    
    private void turnToContour() {
    	if (currentValues[0].length >= 1) {
    		if (currentValues[0][0] > 185) {
    			driveTurn(((currentValues[0][0]-170)/170) * 0.5);
    		}
    		else if (currentValues[0][0] < 155) {
    			driveTurn(-(((170 - currentValues[0][0])/170) * 0.5));
    		}
    		
    		if (currentValues[0][0] != prevImageX) {
        		System.out.println(currentValues[0][0]);
        	}
        	prevImageX = currentValues[0][0];
    	}
    	else {
    		driveTurn(-0.0);
    	}
    	
    }
}
