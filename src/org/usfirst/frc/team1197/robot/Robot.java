package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1197.robot.test.DriveHardwareTest;
import org.usfirst.frc.team1197.robot.test.Test;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private Compressor compressor;
	
	private Joystick player1;
	private Joystick player2;
	private Joystick autoBox;
	
	private TorDrive drive;
	protected static RobotMode mode;
	
	private DriveHardwareTest hardwareTest;
	
    public Robot() {
    	mode = RobotMode.DISABLED;
    	
    	compressor = new Compressor();
    	
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	autoBox = new Joystick(2);
   
    	drive = new TorDrive(player1, autoBox);
    	
    	hardwareTest = new DriveHardwareTest(drive.controller.hardware);
    }
    
    public void robotInit() {
    	drive.controller.hardware.init();
    }

    public void autonomous() {
    	
    }

    public void operatorControl() {
    	mode = RobotMode.TELEOP;
    	drive.controller.setClosedLoopConstants(mode);
    	drive.enable();
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY());
//    		SmartDashboard.putNumber("Gyro Value", drive.controller.hardware.getHeading());
    		SmartDashboard.putNumber("Average Encoder Position", drive.controller.hardware.getAverageEncoderPosition());
    	}
    	drive.disable();
    }

    public void test() {
    	mode = RobotMode.TELEOP;
    	drive.controller.setClosedLoopConstants(mode);
    	drive.enable();
    	while(isEnabled()) {
//    		System.out.println("HELLO");
//    		SmartDashboard.putNumber("Gyro Value", drive.controller.hardware.getHeading());
    		Test.setButtons(getButtonA(), getButtonB());
			hardwareTest.run();
    	}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
		mode = RobotMode.DISABLED;
		drive.disable();
	}

	// Getting the left analog stick X-axis value from the xbox controller. 
	public double getLeftX(){
		return player1.getRawAxis(0);
	}

	
	// Getting the left analog stick Y-axis value from the xbox controller. 
	public double getLeftY(){
		return player1.getRawAxis(1);
	}

	// Getting the right analog stick X-axis value from the xbox controller. 
	public double getRightX(){
		return player1.getRawAxis(4);
	}

	// Getting the right trigger value from the xbox controller.
	public double getRightTrigger(){
		return player1.getRawAxis(3);
	}

	// Getting the left bumper button value from the xbox controller. 
	public boolean getShiftButton(){
		return player1.getRawButton(5);
	}

	public boolean getRightBumper(){
		return player1.getRawButton(6);
	}

	public boolean getButtonA(){
		return player1.getRawButton(1);
	}

	public boolean getButtonB(){
		return player1.getRawButton(2);
	}

	public boolean getButtonX(){
		return player1.getRawButton(3);
	}

	public boolean getButtonY(){
		return player1.getRawButton(4);
	}
	
	public boolean isRed(){
		return autoBox.getRawButton(4);
	}

}
