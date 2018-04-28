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
	private final double rkP = 0.5;//PD For rotation
	private final double rkD = 0.0002;
	private double endTime;
	private final double shootTime = 500;
	private double currentTimeDouble;
	private double lastTimeDouble;
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

	private static enum autoLeftSwitch {
		IDLE, MOVE1, MOVE2, MOVE3, FIRE, REVDOWN;
	}
	private static enum autoRightSwitch {
		IDLE, MOVE1, MOVE2, MOVE3, FIRE, REVDOWN;
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
	private double degreeTolerance = 8;//the tolerance for the normal x + sin x up to get within the switch/scale angle before PID controls it
	private double holdAngle = 10;
	private auto autoRun = auto.IDLE;
	private autoLeft autoLeftRun = autoLeft.IDLE;
	private autoRight autoRightRun = autoRight.IDLE;
	private autoLeftSwitch autoLeftSwitchRun = autoLeftSwitch.IDLE;
	private autoRightSwitch autoRightSwitchRun = autoRightSwitch.IDLE;
	/*----------------------------------------------------------------------
	 */

	public Robot() {
		UsbCamera intakeCam = CameraServer.getInstance().startAutomaticCapture(0);
		intakeCam.setBrightness(50);
		CvSink cvsink1 = new CvSink("Intake Cam");
		cvsink1.setSource(intakeCam);
		cvsink1.setEnabled(true);


		UsbCamera shootCam = CameraServer.getInstance().startAutomaticCapture(1);
		shootCam.setBrightness(0);																																																																																
		CvSink cvsink2 = new CvSink("Shoot Cam");
		cvsink2.setSource(shootCam);
		cvsink2.setEnabled(true);



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

		shooArm = new TorBantorShooarm(player1, player2, armTalon1, armTalon2, shootakeTalon1, shootakeTalon2, 
				breakBeam, fourtwenty, scaleAngle, switchAngle, degreeTolerance, kF, kP, kD, 
				holdAngle, Pusher, scalekI); // TorBantorShooarm object used to enable the arm control + intake/shoot control of the robot

		LinearTest = new LinearTrajectory(hardware, -90, shooArm);
		PivotTest = new PivotTrajectory(hardware, 1, shooArm);
	}

	public void robotInit() {
		hardware.init();
	}

	public void autonomous() {	
		startAngle = hardware.getHeading();
		shooArm.shootIdle();
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // Obtaining the switch & scale colors from the FMS
		SmartDashboard.putString("Game Data", gameData);
		while(gameData.length() == 0 && isAutonomous()) {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}

		if(!autoBox.getRawButton(2) && !autoBox.getRawButton(3)) {
			if(gameData.charAt(0) == 'L') {
				autoRun = auto.FORWARDL;	
			} 
			else {
				autoRun = auto.FORWARDR;
			}
			simpleCenterAutoRun();
		} 
		else {//LEFT OR RIGHT SCALE AUTO'S
			shooArm.scaleShoot();
			if(autoBox.getRawButton(2)) {//THE RIGHT TRAJECTORIES
				if(gameData.charAt(1) == 'R') {
					autoRightRun = autoRight.FORWARDR;
					currentPosition = hardware.getPosition();
					lastPosition = currentPosition;
					currentAngle = hardware.getHeading();
					lastAngle = currentAngle;
					hardware.shiftToLowGear();													
					while(isAutonomous()) {
						currentAngle = hardware.getHeading();
						angleError = currentAngle - startAngle;
						currentTime = System.currentTimeMillis();
						shooArm.TorBantorArmAndShooterUpdate();
						currentPosition = hardware.getPosition();
						currentAngle = hardware.getHeading();
						switch(autoRightRun) {
						case IDLE:
							break;
						case FORWARDR:
							shooArm.shootIdle();
							shooArm.pressYStart();
							hardware.setMotorSpeeds(0.5, 0.5);//right, left
							lastPosition = currentPosition;
							angleLastError = angleError;
							autoRightRun = autoRight.TURNR;
							break;
						case TURNR:
							omegaP = angleError * rkP;
							omegaD = (angleError - angleLastError) * (rkD / kF);
							angleLastError = angleError;
							omega = omegaP + omegaD;
							omega *= -1;
							hardware.setMotorSpeeds((0.5 - omega), (0.5 + omega));
							if(currentPosition - lastPosition > (8 - 0.3)) {
								shooArm.pressLeftTrigger();
								hardware.setMotorSpeeds(0.4, -0.4);
								lastAngle = currentAngle;
								autoRightRun = autoRight.FIRE;
							}
							break;
						case FIRE:
							if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > (90 - 5)) {
								hardware.setMotorSpeeds(-0.4, -0.4);
								Timer.delay(3);
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								shooArm.autoFire();
								lastTime = currentTime;
								autoRightRun = autoRight.REVDOWN;
							}	
							break;
						case REVDOWN:
							if(currentTime - lastTime > revTime) {
								autoRightRun = autoRight.IDLE;
							}
							break;
						}
					}
				}
				else if(gameData.charAt(0) == 'R') { //right to right switch
					autoRightSwitchRun = autoRightSwitch.MOVE1;
					currentPosition = hardware.getPosition();
					lastPosition = currentPosition;
					currentAngle = hardware.getHeading();
					lastAngle = currentAngle;
					hardware.shiftToLowGear();		
					while(isAutonomous()) {
						currentAngle = hardware.getHeading();
						angleError = currentAngle - startAngle;
						currentTime = System.currentTimeMillis();
						shooArm.TorBantorArmAndShooterUpdate();
						currentPosition = hardware.getPosition();
						currentAngle = hardware.getHeading();
						switch(autoRightSwitchRun) {
						case IDLE:
							break;
						case MOVE1:
							shooArm.shootIdle();
							shooArm.pressXStart();
							shooArm.switchShoot();
							hardware.setMotorSpeeds(0.5, 0.5);//right left
							lastPosition = currentPosition;
							angleLastError = angleError;
							startAngle = currentAngle;
							autoRightSwitchRun = autoRightSwitch.MOVE2;
							break;
						case MOVE2:
							omegaP = angleError * rkP;
							omegaD = (angleError - angleLastError) * (rkD / kF);
							angleLastError = angleError;
							omega = omegaP + omegaD;
							omega *= -1;
							hardware.setMotorSpeeds((0.5 - omega), (0.5 + omega));
							if(currentPosition - lastPosition > (3.78 - 0.3)) {
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								hardware.setMotorSpeeds(0.35, -0.35);
								lastAngle = currentAngle;
								autoRightSwitchRun = autoRightSwitch.MOVE3;
							}
							break;
						case MOVE3:
							if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > (90 - 5)) {
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								hardware.setMotorSpeeds(0.3, 0.3);
								lastPosition = currentPosition;
								shooArm.pressLeftTrigger();
								autoRightSwitchRun = autoRightSwitch.FIRE;
							}
							break;
						case FIRE:
							if(currentPosition - lastPosition > (0.64 - 0.2)) {//NEED TO GET THIS VALUE
								hardware.setMotorSpeeds(0, 0);
								shooArm.autoFire();
								endTime = currentTime + shootTime;
								autoRightSwitchRun = autoRightSwitch.REVDOWN;
							}
							break;
						case REVDOWN:
							if(currentTime > endTime) {
								shooArm.releaseLeftTrigger();
								hardware.setMotorSpeeds(0, 0);
								autoRightSwitchRun = autoRightSwitch.IDLE;
							}
							break;
						}
					}
				} 
				else {
					autoRightRun = autoRight.IDLE;
					hardware.setMotorSpeeds(0.75, 0.75);
					Timer.delay(2);
					hardware.setMotorSpeeds(0, 0);
				}	
			} 
			else {
				if(gameData.charAt(1) == 'L') {//LEFT
					autoLeftRun = autoLeft.FORWARDL;
					currentPosition = hardware.getPosition();
					lastPosition = currentPosition;
					currentAngle = hardware.getHeading();
					lastAngle = currentAngle;
					hardware.shiftToLowGear();
					while(isAutonomous()) {
						currentAngle = hardware.getHeading();
						angleError = currentAngle - startAngle;
						currentTime = System.currentTimeMillis();
						shooArm.TorBantorArmAndShooterUpdate();//THE LEFT TRAJECTORIES
						currentPosition = hardware.getPosition();
						currentAngle = hardware.getHeading();
						switch(autoLeftRun) {
						case IDLE:
							break;
						case FORWARDL:
							shooArm.shootIdle();
							shooArm.pressYStart();
							hardware.setMotorSpeeds(0.5, 0.5);//right, left
							lastPosition = currentPosition;
							angleLastError = angleError;
							autoLeftRun = autoLeft.TURNL;
							break;
						case TURNL:
							omegaP = angleError * rkP;
							omegaD = (angleError - angleLastError) * (rkD / kF);
							angleLastError = angleError;
							omega = omegaP + omegaD;
							omega *= -1;
							hardware.setMotorSpeeds((0.5 - omega), (0.5 + omega));
							if(currentPosition - lastPosition > (8 - 0.3)) {
								shooArm.pressLeftTrigger();
								hardware.setMotorSpeeds(-0.4, 0.4);
								lastAngle = currentAngle;
								autoLeftRun = autoLeft.FIRE;
							}
							break;
						case FIRE:
							if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > (90 - 5)) {
								hardware.setMotorSpeeds(-0.4, -0.4);
								Timer.delay(3);
								hardware.setMotorSpeeds(0, 0);
								shooArm.autoFire();
								lastTime = currentTime;
								autoLeftRun = autoLeft.REVDOWN;
							}
							break;
						case REVDOWN:
							if(currentTime - lastTime > revTime) {
								autoLeftRun = autoLeft.IDLE;
							}
							break;
						}
					}

				} 
				else if(gameData.charAt(0) == 'L') {
					//Left to left switch
					autoLeftSwitchRun = autoLeftSwitch.MOVE1;
					currentPosition = hardware.getPosition();
					lastPosition = currentPosition;
					currentAngle = hardware.getHeading();
					lastAngle = currentAngle;
					hardware.shiftToLowGear();		
					while(isAutonomous()) {
						currentAngle = hardware.getHeading();
						angleError = currentAngle - startAngle;
						currentTime = System.currentTimeMillis();
						shooArm.TorBantorArmAndShooterUpdate();
						currentPosition = hardware.getPosition();
						currentAngle = hardware.getHeading();
						switch(autoLeftSwitchRun) {
						case IDLE:
							break;
						case MOVE1:
							shooArm.shootIdle();
							shooArm.pressXStart();
							shooArm.switchShoot();
							hardware.setMotorSpeeds(0.5, 0.5);//left left
							lastPosition = currentPosition;
							angleLastError = angleError;
							startAngle = currentAngle;
							autoLeftSwitchRun = autoLeftSwitch.MOVE2;
							break;
						case MOVE2:
							omegaP = angleError * rkP;
							omegaD = (angleError - angleLastError) * (rkD / kF);
							angleLastError = angleError;
							omega = omegaP + omegaD;
							omega *= -1;
							hardware.setMotorSpeeds((0.5 - omega), (0.5 + omega));
							if(currentPosition - lastPosition > (3.78 - 0.3)) {
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								hardware.setMotorSpeeds(-0.35, 0.35);
								lastAngle = currentAngle;
								autoLeftSwitchRun = autoLeftSwitch.MOVE3;
							}
							break;
						case MOVE3:
							if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > (90 - 5)) {
								hardware.setMotorSpeeds(0, 0);
								Timer.delay(0.3);
								hardware.setMotorSpeeds(0.3, 0.3);
								lastPosition = currentPosition;
								shooArm.pressLeftTrigger();
								autoLeftSwitchRun = autoLeftSwitch.FIRE;
							}
							break;
						case FIRE:
							if(currentPosition - lastPosition > (0.64 - 0.2)) {//NEED TO GET THIS VALUE
								hardware.setMotorSpeeds(0, 0);
								shooArm.autoFire();
								endTime = currentTime + shootTime;
								autoLeftSwitchRun = autoLeftSwitch.REVDOWN;
							}
							break;
						case REVDOWN:
							if(currentTime > endTime) {
								shooArm.releaseLeftTrigger();
								hardware.setMotorSpeeds(0, 0);
								autoLeftSwitchRun = autoLeftSwitch.IDLE;
							}
							break;
						}
					} 
				} 
				else {
					autoLeftRun = autoLeft.IDLE;
					hardware.setMotorSpeeds(0.75, 0.75);
					Timer.delay(2);
					hardware.setMotorSpeeds(0, 0);
				}
			}
		}
	}

	public void operatorControl() {
		shooArm.setAutoIntake(0.6);
		test = false;
		while(isEnabled()){
			if(test) {
				//compressor.start
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
				SmartDashboard.putNumber("GET HEADING", hardware.getHeading());																																																																													
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

	public void simpleCenterAutoRun() {
		currentPosition = hardware.getPosition();
		lastPosition = currentPosition;
		currentAngle = hardware.getHeading();
		lastAngle = currentAngle;
		hardware.shiftToLowGear();
		shooArm.shootIdle();													
		while(isAutonomous()) {
			currentTimeDouble = Timer.getFPGATimestamp();
			currentTime = System.currentTimeMillis();
			shooArm.TorBantorArmAndShooterUpdate();
			currentPosition = hardware.getPosition();
			currentAngle = hardware.getHeading();
			switch(autoRun) {
			case IDLE:
				break;
			case FORWARDL:
				shooArm.pressXStart();
				shooArm.shootIdle();
				hardware.setMotorSpeeds(0.3, 0.3);
				autoRun = auto.TURNL;
				break;
			case TURNL:
				if((currentPosition - lastPosition) > 1.2) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					hardware.setMotorSpeeds(0.35, -0.35);//rightSpeed, leftSpeed
					lastAngle = currentAngle;
					autoRun = auto.FORWARDL2;
				}
				break;
			case FORWARDL2:
				if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					lastPosition = currentPosition;
					hardware.setMotorSpeeds(0.3, 0.3);
					autoRun = auto.TURNL2;
				}
				break;
			case TURNL2:
				if((currentPosition - lastPosition) > 1.2) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					lastAngle = currentAngle;
					hardware.setMotorSpeeds(-0.35, 0.35);
					autoRun = auto.FORWARDL3;
				}
				break;
			case FORWARDL3:
				if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					shooArm.pressLeftTrigger();
					lastPosition = currentPosition;
					hardware.setMotorSpeeds(0.3, 0.3);
					lastTimeDouble = currentTimeDouble;
					autoRun = auto.FIRE;
				}
				break;
			case FORWARDR:
				shooArm.pressXStart();
				shooArm.shootIdle();
				hardware.setMotorSpeeds(0.3, 0.3);
				autoRun = auto.TURNR;
				break;
			case TURNR:
				if((currentPosition - lastPosition) > 1.2) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					hardware.setMotorSpeeds(-0.35, 0.35);//rightSpeed, leftSpeed
					lastAngle = currentAngle;
					autoRun = auto.FORWARDR2;
				}
				break;
			case FORWARDR2:
				if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					lastPosition = currentPosition;
					hardware.setMotorSpeeds(0.3, 0.3);
					autoRun = auto.TURNR2;
				}
				break;
			case TURNR2:
				if(currentPosition - lastPosition > 1.2) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					lastAngle = currentAngle;
					hardware.setMotorSpeeds(0.35, -0.35);
					autoRun = auto.FORWARDR3;
				}
				break;
			case FORWARDR3:
				if(Math.abs((currentAngle - lastAngle) * (180 / Math.PI)) > 85) {
					hardware.setMotorSpeeds(0, 0);
					Timer.delay(0.3);
					shooArm.pressLeftTrigger();
					lastPosition = currentPosition;
					hardware.setMotorSpeeds(0.3, 0.3);
					lastTimeDouble = currentTimeDouble;
					autoRun = auto.FIRE;
				}
				break;
			case FIRE:
				if((currentPosition - lastPosition) > 1.5 || (currentTimeDouble - lastTimeDouble > 6)) {
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
}


