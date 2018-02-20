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
	private DigitalInput hittingDown;
	private AnalogPotentiometer fourtwenty;
	private DriveHardware hardware;
	private TorAuto TorAuto;
	
	/*----------------------------------------------------------------------
	*  Tunable variables for the hold position of the arm
	*/
	private double kF = .005;
	private double kP = 0.02;
	private double kD = 0.000075;
	private double scaleAngle = 80;
	private double switchAngle = 60;
	private double degreeTolerance = 8;//the tolerance for the normal x + sin x up to get within the switch/scale angle before PID controls it
	private double holdAngle = 10;
	private double scaleBackwardsAngle = 120;
	/*----------------------------------------------------------------------
	*/
	
    public Robot() {
    	CameraServer server = CameraServer.getInstance();
    	server.startAutomaticCapture("TorCam", 0);
    	server.putVideo("BWENAN OPEN YOUR EYES", 640, 480);
    	
    	hardware = new DriveHardware();
    	
    	hittingDown = new DigitalInput(1);
    	
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
    			holdAngle, Pusher, scaleBackwardsAngle, hittingDown); // TorBantorShooarm object used to enable the arm control + intake/shoot control of the robot
    	
    	TorAuto = new TorAuto(hardware, autoBox, shooArm);
    }
    
    public void robotInit() {
    	hardware.init();
    }

    public void autonomous() {
    	TorAuto.run();
    }

    public void operatorControl() {
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
    				getButtonA(), getButtonB(), getButtonX(), getButtonY()); // Enabling the drive ofthe robot
    		shooArm.TorBantorArmAndShooterUpdate(); // Enabling arm control
    	}
    }

    public void test() {
    	while(isEnabled()) {
    		
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
