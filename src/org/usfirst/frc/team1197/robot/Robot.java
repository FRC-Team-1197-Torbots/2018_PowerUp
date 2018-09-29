package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1197.robot.auto.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	/*
	 *  Variable purpose/documentation in the constructor
	 */
	private Joystick player1; 
	private Joystick player2;
	private Joystick autoBox;
	private Solenoid Pusher;		
	private Solenoid activeIntake;
	private VictorSPX shootakeTalon1;   
	private VictorSPX shootakeTalon2;   
	private TalonSRX armTalon1; 
	private TalonSRX armTalon2; 
	private TorDrive drive;
	private TorBantorShooarm shooArm;
	private DigitalInput breakBeam;
	private AnalogPotentiometer fourtwenty;
	private DriveHardware hardware;
	private LinearTrajectory LinearTest;
	private PivotTrajectory PivotTest;
	private boolean test;
	private boolean lineUpInFrontOfSwitch;
	private String gameData;
	
	private CenterLeftDoubleSwitch CenterLeftDoubleSwitch;
	private CenterRightDoubleSwitch CenterRightDoubleSwitch;

	public static enum LeftAuto {
		Nothing, LeftLeftOppositeSwitch, LeftLeftDoubleScale, LeftRightSingleScale;
		private LeftAuto() {}
	}
	
	public static enum RightAuto {
		Nothing, RightRightOppositeSwitch, RightRightDoubleScale, RightLeftSingleScale;
		private RightAuto() {}
	}
	
	LeftAuto CoolLeftAuto = LeftAuto.Nothing;//we assign it to nothing to mean that it is not assigned yet
	//it is called cool since it is not just a same side one switch drive forward auto
	RightAuto CoolRightAuto = RightAuto.Nothing;//we assign it to nothing to mean that it is not assigned yet
	//it is called cool since it is not just a same side one switch drive forward auto
	

	/*----------------------------------------------------------------------
	 *  Tunable variables for the hold position of the arm
	 */
	private double kF = .005;
	private double kP = 0.025;
	private double kD = 0.0004;
	private double scalekI = .00005;
	private double scaleAngle = 76;
	private double switchAngle = 55;
	private double degreeTolerance = 8;//the tolerance for the normal x + sin x up to get within the switch/scale angle before PID controls it
	private double holdAngle = 10;
	/*----------------------------------------------------------------------
	 */

	public Robot() {
//		UsbCamera intakeCam = CameraServer.getInstance().startAutomaticCapture(0);
//		intakeCam.setBrightness(50);
//		CvSink cvsink1 = new CvSink("Intake Cam");
//		cvsink1.setSource(intakeCam);
//		cvsink1.setEnabled(true);
//
//
//		UsbCamera shootCam = CameraServer.getInstance().startAutomaticCapture(1);
//		shootCam.setBrightness(0);																																																																																
//		CvSink cvsink2 = new CvSink("Shoot Cam");
//		cvsink2.setSource(shootCam);
//		cvsink2.setEnabled(true);



		hardware = new DriveHardware();

		player1 = new Joystick(0); // Player 1 controller (Controls the drive)
		player2 = new Joystick(1); // Player 2 controller (Controls the arm, shooter/intake)
		autoBox = new Joystick(2); // Control panel box that enables us to choose different autos

		armTalon1 = new TalonSRX(7); 		 // TalonSRX to move the arm
		armTalon2 = new TalonSRX(8); 		 // TalonSRX to move the arm
		breakBeam = new DigitalInput(0);     // Breakbeam to stop the intake when the cube is sucked in
		shootakeTalon1 = new VictorSPX(9);   // Intake/Shooter VictorSPX on the arm
		shootakeTalon2 = new VictorSPX(10);  // Intake/Shooter VictorSPX on the arm
		Pusher = new Solenoid(5);         // Solenoid to shoot out the cube
		activeIntake = new Solenoid(4); //solenoid for the active intake
		
		drive = new TorDrive(player1, autoBox, hardware); // TorDrive object used to enable the drive of the robot

		fourtwenty = new AnalogPotentiometer(0, 360, 0); // Analog Potentiometer to control the position of the arm
		// Analog number, how much the value changes as it goes over the 0 to 5 voltage range, the initial value of the degree of the potentiometer

		shooArm = new TorBantorShooarm(player1, player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, 
				breakBeam, fourtwenty, scaleAngle, switchAngle, degreeTolerance, kF, kP, kD, 
				holdAngle, Pusher, activeIntake, scalekI); // TorBantorShooarm object used to enable the arm control + intake/shoot control of the robot

		LinearTest = new LinearTrajectory(hardware, 1, shooArm, 500);
		PivotTest = new PivotTrajectory(hardware, 90, shooArm, 500);
		CenterLeftDoubleSwitch = new CenterLeftDoubleSwitch(hardware, shooArm);
		CenterRightDoubleSwitch = new CenterRightDoubleSwitch(hardware, shooArm);
	}

	public void robotInit() {
		hardware.init();
	}

	public void autonomous() {
		lineUpInFrontOfSwitch = false;
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // Obtaining the switch & scale colors from the FMS
		SmartDashboard.putString("Game Data", gameData);
		if(autoBox != null) { // Checking if the hardware required for auto is connected 
			while(gameData.length() == 0 && isAutonomous()) {
				gameData = DriverStation.getInstance().getGameSpecificMessage();
			}
			if(!autoBox.getRawButton(2) && !autoBox.getRawButton(3)) {
				//center
				
				//we are setting the center classes up with the scale value
				CenterLeftDoubleSwitch.setScaleSide(gameData.charAt(1) == 'L');
				CenterRightDoubleSwitch.setScaleSide(gameData.charAt(1) == 'L');
				//end of it
				
				
				if(gameData.charAt(0) == 'L') {
					//center left
					while(isAutonomous()) {
						CenterLeftDoubleSwitch.run();
					}
				} else {
					//center right
					while(isAutonomous()) {
						CenterRightDoubleSwitch.run();
					}
				}
			} else {
				//this is to do a non center auto
				if(lineUpInFrontOfSwitch) {
					if(autoBox.getRawButton(2)) {
						//right - line up in front of switch
						Timer.delay(4);//we wait time to let another robot pass in front of us first
						
						if(gameData.charAt(0) == 'R') {
							shooArm.pressXStart();
							shooArm.switchShoot();
							shooArm.shootIdle();
							shooArm.pressLeftTrigger();
							hardware.setMotorSpeeds(0.5, 0.5);
							Timer.delay(2);
							hardware.setMotorSpeeds(0, 0);
							shooArm.autoFire();
							while(isAutonomous()) {
								shooArm.TorBantorArmAndShooterUpdate();
							}
						} else {
							hardware.setMotorSpeeds(0.5, 0.5);
							Timer.delay(2);
							hardware.setMotorSpeeds(0, 0);
						}
					} else {
						//left - line up in front of switch
						Timer.delay(4);//we wait time to let another robot pass in front of us first
						
						if(gameData.charAt(0) == 'L') {
							shooArm.pressXStart();
							shooArm.switchShoot();
							shooArm.shootIdle();
							shooArm.pressLeftTrigger();
							hardware.setMotorSpeeds(0.5, 0.5);
							Timer.delay(2);
							hardware.setMotorSpeeds(0, 0);
							shooArm.autoFire();
							while(isAutonomous()) {
								shooArm.TorBantorArmAndShooterUpdate();
							}
						} else {
							//just drive forward
							hardware.setMotorSpeeds(0.5, 0.5);
							Timer.delay(2);
							hardware.setMotorSpeeds(0, 0);
						}
					}
				} else {
					//here are our real side autos that aren't just one same side switch auto's
					if(autoBox.getRawButton(2)) {
						//right auto that is not a line up in front of the switch
						
						//we will set up the state machine first
						if(gameData.charAt(2) == 'R' && gameData.charAt(1) == 'R') {
							CoolRightAuto = RightAuto.RightRightOppositeSwitch;
						} else if(gameData.charAt(1) == 'R') {
							CoolRightAuto = RightAuto.RightRightDoubleScale;
						} else {
							CoolRightAuto = RightAuto.RightLeftSingleScale;
						}
						
						while(isAutonomous()) {
							switch(CoolRightAuto) {
							case Nothing:
								break;
							case RightRightOppositeSwitch:
								break;
							case RightRightDoubleScale:
								break;
							case RightLeftSingleScale:
								break;
							}
						}
						
						
						
					} else {
						//left auto that is not a line up in front of the switch
						
						//we will set up the state machine first
						if(gameData.charAt(2) == 'L' && gameData.charAt(1) == 'L') {
							CoolLeftAuto = LeftAuto.LeftLeftOppositeSwitch;
						} else if(gameData.charAt(1) == 'L') {
							CoolLeftAuto = LeftAuto.LeftLeftDoubleScale;
						} else {
							CoolLeftAuto = LeftAuto.LeftRightSingleScale;
						}
						
						while(isAutonomous()) {
							switch(CoolLeftAuto) {
							case Nothing:
								break;
							case LeftLeftOppositeSwitch:
								break;
							case LeftLeftDoubleScale:
								break;
							case LeftRightSingleScale:
								break;
							}
						}
						
						
						
						
					}
				}
			}
		}
	}

	public void operatorControl() {
		hardware.shiftToLowGear();
		boolean linearTest = false;
		test = false;
		if(!linearTest) {
			while(isEnabled()){
				if(test) {
					SmartDashboard.putNumber("POT VALUE:", (fourtwenty.get()));
					SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
					SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
					SmartDashboard.putBoolean("BREAKBEAM:", breakBeam.get());
					SmartDashboard.putNumber("GET POSITION", hardware.getPosition());
					SmartDashboard.putNumber("Get Heading", hardware.getHeading());
				} else {
					shooArm.setAutoIntake(0.6);
					drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
							getButtonA(), getButtonB(), getButtonX(), getButtonY()); // Enabling the drive ofthe robot
					shooArm.TorBantorArmAndShooterUpdate(); // Enabling arm control
					SmartDashboard.putNumber("POT VALUE:", (fourtwenty.get()));
					SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
					SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
					SmartDashboard.putBoolean("BREAKBEAM:", breakBeam.get());
					SmartDashboard.putNumber("GET POSITION", hardware.getPosition());

				}
			}
		} else {
//			LinearTest.init();
//			while(!LinearTest.isDone()) {
//				SmartDashboard.putNumber("GET POSITION", hardware.getPosition());
//				SmartDashboard.putNumber("Get Heading", hardware.getHeading());
//				SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
//				SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
//				SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
//				SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
//				shooArm.TorBantorArmAndShooterUpdate();
//				LinearTest.run();
//			}
//			SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
//			SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
			PivotTest.init();
			while(!PivotTest.isDone()) {
				SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
				SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
				SmartDashboard.putNumber("GET POSITION", hardware.getPosition());
				SmartDashboard.putNumber("Get Heading", hardware.getHeading());
				SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
				SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
				shooArm.TorBantorArmAndShooterUpdate();
				PivotTest.run();
			}
			SmartDashboard.putBoolean("Linear Finished?:", true);
			SmartDashboard.putBoolean("Pivot Test Finished?:", true);
			shooArm.pressX();
			while(isEnabled()) {
				shooArm.TorBantorArmAndShooterUpdate();
			}
		}
			
	}

	

	public void test() {
		while(isEnabled()) {
			//compressor.start
			SmartDashboard.putNumber("POT VALUE:", (fourtwenty.get()));
			SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
			SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
			SmartDashboard.putBoolean("BREAKBEAM:", breakBeam.get());
			SmartDashboard.putNumber("GET POSITION", hardware.getPosition());
		}
	}


	/*
	 *  The following are a bunch of accessor methods to obtain input from the controller.
	 */
	public double getLeftX(){
		return player1.getRawAxis(0);
	}

	public double getLeftY(){
		return player1.getRawAxis(1);
	}

	public double getRightX(){
		return player1.getRawAxis(4);
	}

	public double getRightTrigger(){
		return player1.getRawAxis(3);
	}

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
}
