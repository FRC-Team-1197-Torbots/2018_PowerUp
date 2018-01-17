package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import org.usfirst.frc.team1197.robot.test.DriveHardwareTest;
import org.usfirst.frc.team1197.robot.test.Test;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private Compressor compressor;
	
	private Joystick player1;
	private Joystick player2;
	private Joystick autoBox;
	
	private TalonSRX shootakeTalon1;
	private TalonSRX shootakeTalon2;
	
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	
	private TorBantorShootake shootake;
	private TorBantorArm arm;
	
	private TorDrive drive;
	protected static RobotMode mode;
	
	private DriveHardwareTest hardwareTest;
	
    public Robot() {
    	mode = RobotMode.DISABLED;
    	
    	compressor = new Compressor();
    	
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	autoBox = new Joystick(2);
   
    	shootakeTalon1 = new TalonSRX(7);
    	shootakeTalon2 = new TalonSRX(8);
    	
    	armTalon1 = new TalonSRX(9);
    	armTalon2 = new TalonSRX(10);
    	
    	drive = new TorDrive(player1, autoBox);
 
    	shootake = new TorBantorShootake(player2, shootakeTalon1, shootakeTalon2);
    	arm = new TorBantorArm(player2, armTalon1, armTalon2);
    	
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
    		shootake.shootakeUpdate();
    		arm.armUpdate();
//    		SmartDashboard.putNumber("Gyro Value", drive.controller.hardware.getHeading());
//    		SmartDashboard.putNumber("Average Raw Velocity Position", drive.controller.hardware.getAverageRawVelocity());
    	}
    	drive.disable();
    }

    public void test() {
    	mode = RobotMode.TELEOP;
    	drive.controller.setClosedLoopConstants(mode);
    	drive.enable();
    	while(isEnabled()) {
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
