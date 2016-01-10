
package org.usfirst.frc.team1619.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public Joystick rightStick;
	
	private CANTalon leftDriveMotor1;
	private CANTalon leftDriveMotor2;
	private CANTalon rightDriveMotor1;
	private CANTalon rightDriveMotor2;
	private static final int leftDrive1ID = 1;
	private static final int leftDrive2ID = 2;
	private static final int rightDrive1ID = 3;
	private static final int rightDrive2ID = 4;
	private RobotDrive drive;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	rightStick = new Joystick(0);
    	
    	leftDriveMotor1 = new CANTalon(leftDrive1ID);
    	leftDriveMotor2 = new CANTalon(leftDrive2ID);
    	rightDriveMotor1 = new CANTalon(rightDrive1ID);
    	rightDriveMotor2 = new CANTalon(rightDrive2ID);
    	
    	drive = new RobotDrive(leftDriveMotor1, leftDriveMotor2, rightDriveMotor1, rightDriveMotor2);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
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
        driveZ(rightStick);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    private void driveZ(GenericHID input) {
    	drive.arcadeDrive(input.getY(), input.getTwist());
    }
    
    private void driveX(GenericHID input) {
    	drive.arcadeDrive(input.getY(), input.getX());
    }
}
