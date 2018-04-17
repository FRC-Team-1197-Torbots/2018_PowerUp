package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private TalonSRX shootakeTalon1;   
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
	private double currentPosition;
	private double lastPosition;
	private double lastAngle;
	private double currentAngle;
	private long currentTime;
	private long lastTime;
	private long revTime = 1400;
	private String gameData;
	private double angleError;
	private double angleLastError;
	private double startAngle;
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omega;
	private final double rkP = 0.4;//PD For rotation
	private final double rkD = 0.0002;
	private double endTime;
	private final double shootTime = 2000;
	private CenterLeftDoubleSwitch CenterLeftDoubleSwitch;

	public static enum auto {
		IDLE, FORWARDL, TURNL, FORWARDL2, TURNL2, FORWARDL3, FORWARDR, TURNR, FORWARDR2, TURNR2, FORWARDR3, FIRE, REVDOWN;
		private auto() {}
	}

	/*----------------------------------------------------------------------
	 *  Tunable variables for the hold position of the arm
	 */
	private double kF = .005;
	private double kP = 0.025;
	private double kD = 0.0004;
	private double scalekI = .00005;
	private double scaleAngle = 76;
	private double switchAngle = 50;
	private double humanAngle = 33;
	private double degreeTolerance = 8;//the tolerance for the normal x + sin x up to get within the switch/scale angle before PID controls it
	private double holdAngle = 10;
	/*----------------------------------------------------------------------
	 */

	public Robot() {
//		UsbCamera intakeCam = CameraServer.getInstance().startAutomaticCapture(0);
//		intakeCam.setBrightness(50);
//		intakeCam.setFPS(15);
//		CvSink cvsink1 = new CvSink("Intake Cam");
//		cvsink1.setSource(intakeCam);
//		cvsink1.setEnabled(true);
//
//
//		UsbCamera shootCam = CameraServer.getInstance().startAutomaticCapture(1);
//		shootCam.setBrightness(0);
//		shootCam.setFPS(15);
//		CvSink cvsink2 = new CvSink("Shoot Cam");
//		cvsink2.setSource(shootCam);
//		cvsink2.setEnabled(true);
//
//

		hardware = new DriveHardware();

		player1 = new Joystick(0); // Player 1 controller (Controls the drive)
		player2 = new Joystick(1); // Player 2 controller (Controls the arm, shooter/intake)
		autoBox = new Joystick(2); // Control panel box that enables us to choose different autos

		armTalon1 = new TalonSRX(7); 		 // TalonSRX to move the arm
		armTalon2 = new TalonSRX(8); 		 // TalonSRX to move the arm
		breakBeam = new DigitalInput(0);     // Breakbeam to stop the intake when the cube is sucked in
//		shootakeTalon1 = new VictorSPX(9);   // Intake/Shooter VictorSPX on the arm
		shootakeTalon1 = new TalonSRX(9);
		shootakeTalon2 = new VictorSPX(10);  // Intake/Shooter VictorSPX on the arm
		Pusher = new Solenoid(0, 0);         // Solenoid to shoot out the cube

		drive = new TorDrive(player1, autoBox, hardware); // TorDrive object used to enable the drive of the robot

		fourtwenty = new AnalogPotentiometer(0, 360, 0); // Analog Potentiometer to control the position of the arm
//		// Analog number, how much the value changes as it goes over the 0 to 5 voltage range, the initial value of the degree of the potentiometer
//
		shooArm = new TorBantorShooarm(player1, player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, 
				breakBeam, fourtwenty, scaleAngle, switchAngle, humanAngle, degreeTolerance, kF, kP, kD, 
				holdAngle, Pusher, scalekI); // TorBantorShooarm object used to enable the arm control + intake/shoot control of the robot

		LinearTest = new LinearTrajectory(hardware, -1, shooArm);
		PivotTest = new PivotTrajectory(hardware, 90, shooArm);
		CenterLeftDoubleSwitch = new CenterLeftDoubleSwitch(hardware, shooArm);
	}

	public void robotInit() {
		hardware.init();
	}

	public void autonomous() {	
		//startAngle = hardware.getHeading();
		//shooArm.shootIdle();
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // Obtaining the switch & scale colors from the FMS
		SmartDashboard.putString("Game Data", gameData);
		if(autoBox != null) { // Checking if the hardware required for auto is connected 
			while(gameData.length() == 0 && isAutonomous()) {
				gameData = DriverStation.getInstance().getGameSpecificMessage();
			}
			if(!autoBox.getRawButton(2) && !autoBox.getRawButton(3)) {
				//center
				if(gameData.charAt(0) == 'L') {
					//center left
					while(isAutonomous()) {
						CenterLeftDoubleSwitch.run();
					}
				} else {
					//center right
					
					
					
					
					
					
				}
				
			} else if(autoBox.getRawButton(2)) {
				//right
			} else {
				//left
			}
			
			
			
			
			
			
		}
	}

	public void operatorControl() {
		hardware.shiftToLowGear();
		boolean linearTest = true;
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
			LinearTest.init();
			while(!LinearTest.isDone()) {
				SmartDashboard.putNumber("GET POSITION", hardware.getPosition());
				SmartDashboard.putNumber("Get Heading", hardware.getHeading());
				SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
				SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
				SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
				SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
				LinearTest.run();
			}
			SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
			SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
			PivotTest.init();
			while(!PivotTest.isDone()) {
				SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
				SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
				SmartDashboard.putNumber("GET POSITION", hardware.getPosition());
				SmartDashboard.putNumber("Get Heading", hardware.getHeading());
				SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
				SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
				PivotTest.run();
			}
			SmartDashboard.putBoolean("Linear Finished?:", LinearTest.isDone());
			SmartDashboard.putBoolean("Pivot Test Finished?:", PivotTest.isDone());
			shooArm.pressX();
			
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
