package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1197.robot.test.DriveHardwareTest;
import org.usfirst.frc.team1197.robot.test.Test;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//import org.usfirst.frc.team1197.robot.test.DriveHardwareTest;
//import org.usfirst.frc.team1197.robot.test.Test;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private Compressor compressor;
	
	private Joystick player1;
	private Joystick player2;
	private Joystick autoBox;
	
//	private Solenoid releaser;
	private Solenoid Pusher;
	
	private VictorSPX shootakeTalon1;
	private VictorSPX shootakeTalon2;
	
//	private TalonSRX puller1;
//	private TalonSRX puller2;
	
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	
	private TorDrive drive;
	protected static RobotMode mode;
	
	private TorBantorShooarm shooArm;
// 	private Climber climber;
	private DigitalInput breakBeam;
	private DriveHardwareTest hardwareTest;
	private AnalogPotentiometer fourtwenty;
	private TorAuto auto;
	
	/*----------------------------------------------------------------------
	* The Tunes for the Beginning PID to hold the arm up
	*/
	private double kF = .005;
	private double kP = 0.005;
	private double kD = 0.00001;
	private double scaleAngle = 80;
	private double switchAngle = 55;
	private double degreeTolerance = 8;//the tolerance for the normal x + sin x up to get within the switch/scale angle before PID controls it
	private double holdAngle = 10;
	private double scaleBackwardsAngle = 120;
	/*----------------------------------------------------------------------
	*/

	
    public Robot() {
    	mode = RobotMode.DISABLED;
    	
    	compressor = new Compressor();
    	
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	autoBox = new Joystick(2);
   
    	armTalon1 = new TalonSRX(7);
    	armTalon2 = new TalonSRX(8);
    	shootakeTalon1 = new VictorSPX(9);
    	shootakeTalon2 = new VictorSPX(10);
//    	puller1 = new TalonSRX(11);
//    	puller2 = new TalonSRX(12);
    	
    	drive = new TorDrive(player1, autoBox);
    	
		breakBeam = new DigitalInput(0);
    	
    	Pusher = new Solenoid(0, 0);
//    	releaser = new Solenoid(0, 2);//for the climber
    	
		fourtwenty = new AnalogPotentiometer(0, 360, 0); //analog number, how much the value changes as it goes over the 0 to 5 voltage range, the initial value of the degree of the potentiometer
		
//    	shooArm = new TorBantorShooarm(player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, 
//    			breakBeam, fourtwenty, scaleAngle, switchAngle, degreeTolerance, kF, kP, kD, 
//    			holdAngle, Pusher, scaleBackwardsAngle);
    	shooArm = new TorBantorShooarm(player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, 
    			fourtwenty, scaleAngle, switchAngle, degreeTolerance, kF, kP, kD, 
    			holdAngle, Pusher, scaleBackwardsAngle);
//    	climber = new Climber(releaser, puller1, puller2, shooArm, player2);
    	
//    	hardwareTest = new DriveHardwareTest(drive.controller.hardware);  
    	
//    	auto = new TorAuto(drive, autoBox, shooArm);
    }
    
    public void robotInit() {
    	drive.controller.hardware.init();
    }

    public void autonomous() {
    	mode = RobotMode.AUTO;
    	drive.controller.setClosedLoopConstants();
    	drive.enable();
    	while(isEnabled() && isAutonomous()) {
    		auto.run();
    	}    
    }

    public void operatorControl() {
    	mode = RobotMode.TELEOP;
    	drive.controller.setClosedLoopConstants();
    	drive.enable();
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY());
    		SmartDashboard.putNumber("Gyro Value", drive.controller.hardware.getHeading());
//    		SmartDashboard.putNumber("Average Encoder Position", drive.controller.hardware.getRightEncoder());
//    		SmartDashboard.putNumber("Average Raw Velocity Position", drive.controller.hardware.getAverageRawVelocity());
    		SmartDashboard.putBoolean("Motion Profiling Active", drive.controller.motionProfilingActive());
    		SmartDashboard.putNumber("Right Encoder Position", drive.controller.hardware.getRightEncoder());
    		SmartDashboard.putNumber("Left Encoder Position", drive.controller.hardware.getLeftEncoder());
    		shooArm.TorBantorArmAndShooterUpdate();
//    		climber.update();
    	}
    	drive.disable();
    }

    public void test() {
    	mode = RobotMode.TELEOP;
       	drive.controller.setClosedLoopConstants();
    	drive.enable();
    	while(isEnabled()) {
    		Test.setButtons(getButtonA(), getButtonB());
			hardwareTest.run();
    	}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
		mode = RobotMode.DISABLED;
//		drive.disable();
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
//		return autoBox.getRawButton(5);
		return true;
	}

}
