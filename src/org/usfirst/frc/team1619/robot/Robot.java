
package org.usfirst.frc.team1619.robot;

import java.util.Arrays;

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

	private final double ARM_MOTOR = 1.0;
	private final int ARM_ENC_TICKS_REV = 1200;
	private final double ARM_P = 0.0018;
	private final double ARM_I = 0.00001;
	private final double ARM_D = 0;

	private static final double[] DEFAULT_ARRAY = new double[0];

	private static final String[] CONTOUR_KEYS = {"centerX", "centerY", "area",  "height", "width"};
	private static final String CONTOUR_TABLE_NAME = "GRIP/contours";

	private static final String[] LINE_KEYS = {"length", "x1", "x2", "y1", "y2", "angle"};
	private static final String LINE_TABLE_NAME = "GRIP/lines";
	private static final double cameraPictureHeight = 240;
	public static final int x1Coord = 1;
	public static final int y1Coord = 2;
	public static final int x2Coord = 3;
	public static final int y2Coord = 4;

	private static final boolean TWIST_DRIVE = true;

	private Joystick rightStick;
	private JoystickButton followButton;

	private Joystick leftStick;
	private JoystickButton armPIDButton;
	private JoystickButton resetArmEncButton;


	private CANTalon leftDriveMotor1;
	private CANTalon leftDriveMotor2;
	private CANTalon rightDriveMotor1;
	private CANTalon rightDriveMotor2;
	private CANTalon armMotor1;
	// private CANTalon armMotor2;
	private CANTalon spareMotor;

	private static final int leftDrive1ID = 1;
	private static final int leftDrive2ID = 2;
	private static final int rightDrive1ID = 3;
	private static final int rightDrive2ID = 4;
	private static final int armMotor1ID = 11;
	// private static final int armMotor2ID = 12;
	private static final int spareMotorID = 5;

	private RobotDrive drive;

	NetworkTable contourTable;
	NetworkTable lineTable;

	private double[][] currentValues;

	private int imageReceivePeriod = 50;
	private int imageReceiveValue = 0;

	private TestBotPID armPID;

	public Robot() {
		rightStick = new Joystick(0);
		leftStick = new Joystick(1);

		followButton = new JoystickButton(rightStick, 1);
		armPIDButton = new JoystickButton(leftStick, 1);
		resetArmEncButton = new JoystickButton(leftStick, 2);

    	leftDriveMotor1 = new CANTalon(leftDrive1ID);
    	leftDriveMotor2 = new CANTalon(leftDrive2ID);
    	rightDriveMotor1 = new CANTalon(rightDrive1ID);
    	rightDriveMotor2 = new CANTalon(rightDrive2ID);
    	armMotor1 = new CANTalon(armMotor1ID);
    	spareMotor = new CANTalon(spareMotorID);

    	drive = new RobotDrive(leftDriveMotor1, leftDriveMotor2, rightDriveMotor1, rightDriveMotor2);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);

    	armMotor1.setInverted(false);

    	spareMotor.setInverted(false);

    	contourTable = NetworkTable.getTable(CONTOUR_TABLE_NAME);
    	lineTable = NetworkTable.getTable(LINE_TABLE_NAME);

    	armPID = new TestBotPID(ARM_P, ARM_I, ARM_D);
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
    	if(armPIDButton.get()) {
    		armPID.setTarget(leftStick.getY() * (ARM_ENC_TICKS_REV / 4));
        	armMotor1.set(armPID.get(armMotor1.getEncPosition()));
    	}
    	else if(resetArmEncButton.get()) {
    		armMotor1.setEncPosition(0);
    	}
    	else {
    		armManual(leftStick);
    	}
//    	armManual(leftStick);

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
    		double[][] contourTableVals = getContourTableVals();
            if (contourTableVals[0].length > 0) {
                SmartDashboard.putString("Contour", Arrays.toString(CONTOUR_KEYS));
            	for (int i = 0; i < contourTableVals.length; i++) {
                	SmartDashboard.putString("Contour" + i, Arrays.toString(contourTableVals[i]));
                }
            }

            double[][] lineTableVals = getLineTableVals();
            if (lineTableVals[0].length > 0) {
                SmartDashboard.putString("Line", Arrays.toString(LINE_KEYS));
            	for (int i = 0; i < lineTableVals.length; i++) {
                	SmartDashboard.putString(LINE_KEYS[i], Arrays.toString(lineTableVals[i]));
                }
            }

            imageReceiveValue -= imageReceivePeriod;
    	}
    	imageReceiveValue++;

    	SmartDashboard.putNumber("Left Encoder", leftDriveMotor1.getEncPosition());
        SmartDashboard.putNumber("Right Encoder", rightDriveMotor1.getEncPosition());
        SmartDashboard.putNumber("Arm Encoder", armMotor1.getEncPosition());
    }

    private void drive(GenericHID input) {
    	if (followButton.get()) {
        	currentValues = getContourTableVals();
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
    }

    private void driveTurn(double turnSpeed) {
    	drive.arcadeDrive(0, turnSpeed);
    }

    private double[][] getContourTableVals() {
    	return getTableVals(CONTOUR_KEYS, contourTable);
    }

    private double[][] getLineTableVals() {
    	return getTableVals(LINE_KEYS, lineTable);
    }

    private double[][] getTableVals(String[] keys, NetworkTable table) {
    	double[][] vals = new double[keys.length][];

    	for (int i = 0; i < keys.length; i++) {
    		vals[i] = table.getNumberArray(keys[i], DEFAULT_ARRAY);
    	}

    	double[][] organisedVals = new double[(vals[0].length)][keys.length];
    	for( int l = 0; l < (vals[0].length); l++ )
    	{
    		for( int w = 0; w < keys.length; w++ )
        	{
        		organisedVals[l][w] = vals[w][l];
        	}
    	}

    	return organisedVals;
    }

    private void turnToContour() {
    	if (currentValues[0].length >= 1) {
    		if (currentValues[0][0] > 185) {
    			driveTurn(((currentValues[0][0]-170)/170) * 0.5);
    		}
    		else if (currentValues[0][0] < 155) {
    			driveTurn(-(((170 - currentValues[0][0])/170) * 0.5));
    		}
    	}
    	else {
    		driveTurn(-0.0);
    	}

    }

    @SuppressWarnings("unused")
	private void makeQuadrilateral() {
    	double[][] perimeter = getLineTableVals();
    	double[][] contourStats = getContourTableVals();
    	double xCenter = contourStats[0][0];
    	// double yCenter = contourStats[0][1];
    	double leftTop = 0;
    	double leftBottom = cameraPictureHeight;
    	double rightTop = 0;
    	double rightBottom = cameraPictureHeight;
    	double leftTopX = 0;
    	double leftBottomX = 0;
    	double rightTopX = 0;
    	double rightBottomX = 0;
    	for( double[] d : perimeter)
    	{
    		if(d[x1Coord] < xCenter && d[x2Coord] < xCenter)
    		{
    			if(d[y1Coord] > leftTop)
    			{
    				leftTop = d[y1Coord];
    				leftTopX = d[x1Coord];
    			}
    			if(d[y2Coord] > leftTop)
    			{
    				leftTop = d[y2Coord];
    				leftTopX = d[x2Coord];
    			}
    			if(d[y1Coord] < leftBottom)
    			{
    				leftBottom = d[y1Coord];
    				leftBottomX = d[x1Coord];
    			}
    			if(d[y2Coord] < leftBottom)
    			{
    				leftBottom = d[y2Coord];
    				leftBottomX = d[x2Coord];
    			}
    		}
    		if(d[x1Coord] > xCenter && d[x2Coord] > xCenter)
    		{
    			if(d[y1Coord] > rightTop)
    			{
    				rightTop = d[y1Coord];
    				rightTopX = d[x1Coord];
    			}
    			if(d[y2Coord] > rightTop)
    			{
    				rightTop = d[y2Coord];
    				rightTopX = d[x2Coord];
    			}
    			if(d[y1Coord] < rightBottom)
    			{
    				rightBottom = d[y1Coord];
    				rightBottomX = d[x1Coord];
    			}
    			if(d[y2Coord] < rightBottom)
    			{
    				rightBottom = d[y2Coord];
    				rightBottomX = d[x2Coord];
    			}
    		}
    	}
    	double[] leftTopCoords = {leftTopX, leftTop};
    	double[] leftBottomCoords = {leftBottomX, leftBottom};
    	double[] rightTopCoords = {rightTopX, leftTop};
    	double[] rightBottomCoords = {rightBottomX, rightBottom};
    	SmartDashboard.putString("leftTopCoords", leftTopCoords.toString());
    	SmartDashboard.putString("leftBottomCoords", leftBottomCoords.toString());
    	SmartDashboard.putString("rightTopCoords", rightTopCoords.toString());
    	SmartDashboard.putString("rightBottomCoords", rightBottomCoords.toString());
    }
}
