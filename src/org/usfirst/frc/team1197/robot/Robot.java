package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//import org.usfirst.frc.team1197.robot.test.DriveHardwareTest;
//import org.usfirst.frc.team1197.robot.test.Test;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
//	private Compressor compressor;
	
	private Joystick player1;
	private Joystick player2;
	private Joystick autoBox;
	
//	private Solenoid releaser;
//	private Solenoid Pusher;
	
	private TalonSRX shootakeTalon1;
	private TalonSRX shootakeTalon2;
	
//	private TalonSRX puller;
	
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	
//	private Ultrasonic ultra;
	private TorDrive drive;
	protected static RobotMode mode;
	
	private TelBantorShooarm shooArm;
// 	private Climber climber;
	
//	private DriveHardwareTest hardwareTest;
	
    public Robot() {
//    	mode = RobotMode.DISABLED;
    	
//    	compressor = new Compressor();
    	
//    	player1 = new Joystick(0);
    	player2 = new Joystick(2);
//    	autoBox = new Joystick(2);
   
    	shootakeTalon1 = new TalonSRX(5);
    	shootakeTalon2 = new TalonSRX(6);
    	
    	
    	armTalon1 = new TalonSRX(3);
    	armTalon2 = new TalonSRX(4);
//    	ultra = new Ultrasonic(0,1);
//    	ultra.setAutomaticMode(isEnabled());
//    	drive = new TorDrive(player1, autoBox);
//    	puller = new TalonSRX(8);
    	
    	//Pusher = new Solenoid(5);
//    	releaser = new Solenoid(7);
    	
    	shooArm = new TelBantorShooarm(player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2);
//    	shooArm = new TelBantorShooarm(player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, Pusher);
//    	climber = new Climber(releaser, puller, shooArm, player2);
    	
//    	hardwareTest = new DriveHardwareTest(drive.controller.hardware);
    }
    
    public void robotInit() {
//    	drive.controller.hardware.init();
    }

    public void autonomous() {
    	
    }

    public void operatorControl() {
//    	mode = RobotMode.TELEOP;
//    	drive.controller.setClosedLoopConstants(mode);
//    	drive.enable();
    	while(isEnabled()){
//    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
//					getButtonA(), getButtonB(), getButtonX(), getButtonY());
//    		
//    		SmartDashboard.putNumber("Gyro Value", drive.controller.hardware.getHeading());
//    		SmartDashboard.putNumber("Average Encoder Position", drive.controller.hardware.getAverageEncoderPosition());
//    		SmartDashboard.putNumber("Average Raw Velocity Position", drive.controller.hardware.getAverageRawVelocity());
    		shooArm.TorBantorArmAndShooterUpdate();
// 		climber.update();
//    		System.out.println(ultra.getRangeInches());
    	}
//    	drive.disable();
    }

    public void test() {
//    	mode = RobotMode.TELEOP;
//    	drive.controller.setClosedLoopConstants(mode);
//    	drive.enable();
//    	while(isEnabled()) {
//    		Test.setButtons(getButtonA(), getButtonB());
//			hardwareTest.run();
//			
//    	}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
//		mode = RobotMode.DISABLED;
//		drive.disable();
	}

	// Getting the left analog stick X-axis value from the xbox controller. 
	public double getLeftX(){
		return player1.getRawAxis(1);
	}

	
	// Getting the left analog stick Y-axis value from the xbox controller. 
	public double getLeftY(){
		return player1.getRawAxis(2);
	}

	// Getting the right analog stick X-axis value from the xbox controller. 
	public double getRightX(){
		return player1.getRawAxis(5);
	}

	// Getting the right trigger value from the xbox controller.
	public double getRightTrigger(){
		return player1.getRawAxis(4);
	}

	// Getting the left bumper button value from the xbox controller. 
	public boolean getShiftButton(){
		return player1.getRawButton(6);
	}

	public boolean getRightBumper(){
		return player1.getRawButton(7);
	}

	public boolean getButtonA(){
		return player1.getRawButton(2);
	}

	public boolean getButtonB(){
		return player1.getRawButton(3);
	}

	public boolean getButtonX(){
		return player1.getRawButton(4);
	}

	public boolean getButtonY(){
		return player1.getRawButton(5);
	}
	
	public boolean isRed(){
		return autoBox.getRawButton(5);
	}

}
