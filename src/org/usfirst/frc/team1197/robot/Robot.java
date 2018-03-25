package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
	private VictorSPX shootakeTalon1;   
	private VictorSPX shootakeTalon2;   
	private TalonSRX armTalon1; 
	private TalonSRX armTalon2; 
	private TorDrive drive;
	private TorBantorShooarm shooArm;
	private DigitalInput breakBeam;
	private AnalogPotentiometer fourtwenty;
	private DriveHardware hardware;
	private TorAuto TorAuto;
	private PivotTrajectory Test;
	private boolean test;
	private boolean goStraight;
	private double currentPosition;
	private double lastPosition;
	private double lastAngle;
	private double currentAngle;
	private long currentTime;
	private long lastTime;
	private long revTime = 1400;
	private String gameData;

	public static enum auto {
		IDLE, FORWARDL, TURNL, FORWARDL2, TURNL2, FORWARDL3, FORWARDR, TURNR, FORWARDR2, TURNR2, FORWARDR3, FIRE, REVDOWN;
		private auto() {}
	}
	public static enum autoLeft {
		IDLE, FORWARDL, TURNL, FIRE, REVDOWN;
	}
	public static enum autoRight {
		IDLE, FORWARDR, TURNR, FIRE, REVDOWN;
	}
	
	/*----------------------------------------------------------------------
	 *  Tunable variables for the hold position of the arm
	 */
	private double kF = .005;
	private double kP = 0.02;
	private double kD = 0.000075;
	private double scaleAngle = 75;
	private double switchAngle = 50;
	private double degreeTolerance = 8;//the tolerance for the normal x + sin x up to get within the switch/scale angle before PID controls it
	private double holdAngle = 10;
	private auto autoRun = auto.IDLE;
	private autoLeft autoLeftRun = autoLeft.IDLE;
	private autoRight autoRightRun = autoRight.IDLE;
	/*----------------------------------------------------------------------
	 */

	public Robot() {
		CameraServer server = CameraServer.getInstance();
		server.startAutomaticCapture("TorCam", 0);
		server.putVideo("BWENAN OPEN YOUR EYES", 640, 480);
		hardware = new DriveHardware();

		player1 = new Joystick(0); // Player 1 controller (Controls the drive)
		player2 = new Joystick(1); // Player 2 controller (Controls the arm, shooter/intake)
		autoBox = new Joystick(2); // Control panel box that enables us to choose different autos

		armTalon1 = new TalonSRX(7); 		 // TalonSRX to move the arm
		armTalon2 = new TalonSRX(8); 		 // TalonSRX to move the arm
		breakBeam = new DigitalInput(0);     // Breakbeam to stop the intake when the cube is sucked in
		shootakeTalon1 = new VictorSPX(9);   // Intake/Shooter VictorSPX on the arm
		shootakeTalon2 = new VictorSPX(10);  // Intake/Shooter VictorSPX on the arm
		Pusher = new Solenoid(0, 0);         // Solenoid to shoot out the cube

		drive = new TorDrive(player1, autoBox, hardware); // TorDrive object used to enable the drive of the robot

		fourtwenty = new AnalogPotentiometer(0, 360, 0); // Analog Potentiometer to control the position of the arm
		// Analog number, how much the value changes as it goes over the 0 to 5 voltage range, the initial value of the degree of the potentiometer

		shooArm = new TorBantorShooarm(player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, 
				breakBeam, fourtwenty, scaleAngle, switchAngle, degreeTolerance, kF, kP, kD, 
				holdAngle, Pusher); // TorBantorShooarm object used to enable the arm control + intake/shoot control of the robot
																			
		TorAuto = new TorAuto(hardware, autoBox, shooArm);
		Test = new PivotTrajectory(hardware, -90, shooArm);
	}

	public void robotInit() {
		hardware.init();
	}

	public void autonomous() {			
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // Obtaining the switch & scale colors from the FMS
		SmartDashboard.putString("Game Data", gameData);															
		//goStraight = false;
		/*if(goStraight) {
			hardware.setMotorSpeeds(0.75, 0.75);
			Timer.delay(2);
			hardware.setMotorSpeeds(0, 0);	
		} 
		else {*/
			if(!autoBox.getRawButton(2) && !autoBox.getRawButton(3)) {
				if(gameData.charAt(0) == 'L') {
					autoRun = auto.FORWARDL;	
				} 
				else {
					autoRun = auto.FORWARDR;
				}
				currentPosition = hardware.getPosition();
				lastPosition = currentPosition;
				currentAngle = hardware.getHeading();
				lastAngle = currentAngle;
				hardware.shiftToLowGear();													
				while(isAutonomous()) {
					currentTime = System.currentTimeMillis();
					shooArm.TorBantorArmAndShooterUpdate();
					currentPosition = hardware.getPosition();
					currentAngle = hardware.getHeading();
					switch(autoRun) {
					case IDLE:
						break;
					case FORWARDL:
						shooArm.pressXStart();
						hardware.setMotorSpeeds(0.3, 0.3);
						autoRun = auto.TURNL;
						break;
					case TURNL:
						if((currentPosition - lastPosition) > 1.2) {
							hardware.setMotorSpeeds(0.35, -0.35);//rightSpeed, leftSpeed
							lastAngle = currentAngle;
							autoRun = auto.FORWARDL2;
						}
						break;
					case FORWARDL2:
						if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
							lastPosition = currentPosition;
							hardware.setMotorSpeeds(0.3, 0.3);
							autoRun = auto.TURNL2;
						}
						break;
					case TURNL2:
						if((currentPosition - lastPosition) > 1.6) {
							lastAngle = currentAngle;
							hardware.setMotorSpeeds(-0.35, 0.35);
							autoRun = auto.FORWARDL3;
						}
						break;
					case FORWARDL3:
						if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
							shooArm.pressLeftTrigger();
							lastPosition = currentPosition;
							hardware.setMotorSpeeds(0.3, 0.3);
							autoRun = auto.FIRE;
						}
						break;
					case FORWARDR:
						shooArm.pressXStart();
						hardware.setMotorSpeeds(0.3, 0.3);
						autoRun = auto.TURNR;
						break;
					case TURNR:
						if((currentPosition - lastPosition) > 1.2) {
							hardware.setMotorSpeeds(-0.35, 0.35);//rightSpeed, leftSpeed
							lastAngle = currentAngle;
							autoRun = auto.FORWARDR2;
						}
						break;
					case FORWARDR2:
						if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
							lastPosition = currentPosition;
							hardware.setMotorSpeeds(0.3, 0.3);
							autoRun = auto.TURNR2;
						}
						break;
					case TURNR2:
						if(currentPosition - lastPosition > 1.2) {
							lastAngle = currentAngle;
							hardware.setMotorSpeeds(0.35, -0.35);
							autoRun = auto.FORWARDR3;
						}
						break;
					case FORWARDR3:
						if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
							shooArm.pressLeftTrigger();
							lastPosition = currentPosition;
							hardware.setMotorSpeeds(0.3, 0.3);
							autoRun = auto.FIRE;
						}
						break;
					case FIRE:
						if((currentPosition - lastPosition) > 1.2) {
							hardware.setMotorSpeeds(0.0, 0.0);
							shooArm.autoFire();
							lastTime = currentTime;
							autoRun = auto.REVDOWN;
						}
						break;
					case REVDOWN:
						if((currentTime - lastTime) > revTime) {
							shooArm.releaseLeftTrigger();
							autoRun = auto.IDLE;
						}
						break;
					}
				}  			
			}
			else {//LEFT OR RIGHT SCALE AUTO'S
				shooArm.scaleShoot();
				shooArm.pressLeftTrigger();
				if(autoBox.getRawButton(2)) {//THE RIGHT TRAJECTORIES
					if(gameData.charAt(1) == 'L') {
						autoRightRun = autoRight.IDLE;
						hardware.setMotorSpeeds(0.75, 0.75);
						Timer.delay(2);
						hardware.setMotorSpeeds(0, 0);	
						
					} else {
						autoRightRun = autoRight.FORWARDR;
					}
					currentPosition = hardware.getPosition();
					lastPosition = currentPosition;
					currentAngle = hardware.getHeading();
					lastAngle = currentAngle;
					hardware.shiftToLowGear();													
					while(isAutonomous()) {
						currentTime = System.currentTimeMillis();
						shooArm.TorBantorArmAndShooterUpdate();
						currentPosition = hardware.getPosition();
						currentAngle = hardware.getHeading();
						switch(autoRightRun) {
						case IDLE:
							break;
						case FORWARDR:
							shooArm.pressYStart();
							hardware.setMotorSpeeds(0.5, 0.5);//right, left
							lastPosition = currentPosition;
							autoRightRun = autoRight.TURNR;
							break;
						case TURNR:
							if(currentPosition - lastPosition > (8.45 - 0.3)) {
								hardware.setMotorSpeeds(0.35, -0.35);
								lastAngle = currentAngle;
								autoRightRun = autoRight.FIRE;
							}
							break;
						case FIRE:
							if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > (90 - 5)) {
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								shooArm.autoFire();
								lastTime = currentTime;
								autoRightRun = autoRight.REVDOWN;
							}
							break;
						case REVDOWN:
							if(currentTime - lastTime > revTime) {
								shooArm.releaseLeftTrigger();
								autoRightRun = autoRight.IDLE;
							}
							break;
						}
					}

					
					
				} else {
					if(gameData.charAt(1) == 'L') {//LEFT
						autoLeftRun = autoLeft.FORWARDL;
					} else {
						autoLeftRun = autoLeft.IDLE;
						hardware.setMotorSpeeds(0.75, 0.75);
						Timer.delay(2);
						hardware.setMotorSpeeds(0, 0);
					}
					currentPosition = hardware.getPosition();
					lastPosition = currentPosition;
					currentAngle = hardware.getHeading();
					lastAngle = currentAngle;
					hardware.shiftToLowGear();													
					while(isAutonomous()) {
						currentTime = System.currentTimeMillis();
						shooArm.TorBantorArmAndShooterUpdate();//THE LEFT TRAJECTORIES
						currentPosition = hardware.getPosition();
						currentAngle = hardware.getHeading();
						switch(autoLeftRun) {
						case IDLE:
							break;
						case FORWARDL:
							shooArm.pressYStart();
							hardware.setMotorSpeeds(0.5, 0.5);//right, left
							lastPosition = currentPosition;
							autoLeftRun = autoLeft.TURNL;
							break;
						case TURNL:
							if(currentPosition - lastPosition > (8.45 - 0.3)) {
								hardware.setMotorSpeeds(-0.35, 0.35);
								lastAngle = currentAngle;
								autoLeftRun = autoLeft.FIRE;
							}
							break;
						case FIRE:
							if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > (90 - 5)) {
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								shooArm.autoFire();
								lastTime = currentTime;
								autoLeftRun = autoLeft.REVDOWN;
							}
							break;
						case REVDOWN:
							if(currentTime - lastTime > revTime) {
								shooArm.releaseLeftTrigger();
								autoLeftRun = autoLeft.IDLE;
							}
							break;
						}
					}
					
				}
			}
		}

	public void operatorControl() {
		shooArm.setAutoIntake(0.6);
		test = false;
		while(isEnabled()){
			if(test) {
				SmartDashboard.putNumber("POT VALUE:", (fourtwenty.get()));
				SmartDashboard.putNumber("RIGHT ENCODER:", hardware.getRightEncoder());
				SmartDashboard.putNumber("LEFT ENCODER:", hardware.getLeftEncoder());
				SmartDashboard.putBoolean("BREAKBEAM:", breakBeam.get());
				SmartDashboard.putNumber("GET POSITION", hardware.getPosition());

			} else {
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
	}

	public void test() {
		while(isEnabled()) {
			//compressor.start
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

	
	