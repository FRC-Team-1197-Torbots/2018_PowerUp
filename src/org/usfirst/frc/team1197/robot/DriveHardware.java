package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

public class DriveHardware {
	
	private ADXRS450_Gyro gyro;

	private final TalonSRX rightMaster;
	private final TalonSRX rightSlave1;
	private final TalonSRX rightSlave2;
	private final TalonSRX leftMaster;
	private final TalonSRX leftSlave1;
	private final TalonSRX leftSlave2;
	
//	private double leftSpeed;
//	private double rightSpeed;
//	private double threshold = 50;
	
	private final Solenoid solenoid;
	
	/** TUNABLE HARDWARE VALUES **/ 
	private static final double encoderTicksPerMeter = 999.84825; // (units: ticks per meter)
	private static final double approximateSensorSpeed = 924; // measured maximum (units: RPM)
	private static final double quadEncNativeUnits = 512.0; // (units: ticks per revolution)
	
	public static final double trackWidth = 0.654177; // [meters].
	public static final double halfTrackWidth = trackWidth / 2.0; // [meters]
	public static final double backlash = 0.015; // [meters]
	
	public static final double absoluteMaxSpeed = (approximateSensorSpeed * quadEncNativeUnits) / (60 * encoderTicksPerMeter); // [meters/sec] (2018 robot: ~7.886 m/s)
	public static final double absoluteMaxOmega = absoluteMaxSpeed / halfTrackWidth;
	
//	private final double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));
//	private final double kP = 0.0; 
//	private final double kI = 0.0; 
//	private final double kD = 0.0; 
	
	/***************************/
	
	private double heading = 0.0;
	
	public DriveHardware() {
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		
		solenoid = new Solenoid(0, 1);

		leftMaster = new TalonSRX(1);
		leftSlave1 = new TalonSRX(2);
		leftSlave2 = new TalonSRX(3);  
		rightMaster = new TalonSRX(4);
		rightSlave1 = new TalonSRX(5);
		rightSlave2 = new TalonSRX(6);

//		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
//		rightMaster.configNominalOutputForward(+0.0f, 0);
//		rightMaster.configNominalOutputReverse(-0.0f, 0);
//		rightMaster.configPeakOutputForward(+12.0f, 0);
//		rightMaster.configPeakOutputReverse(-12.0f, 0);
//		rightMaster.selectProfileSlot(0, 0);
//		rightMaster.config_kF(0, kF, 0);
//		rightMaster.config_kP(0, kP, 0);
//		rightMaster.config_kI(0, kI, 0);
//		rightMaster.config_kD(0, kD, 0);

		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);

//		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
//		leftMaster.configNominalOutputForward(+0.0f, 0);
//		leftMaster.configNominalOutputReverse(-0.0f, 0);
//		leftMaster.configPeakOutputForward(+12.0f, 0);
//		leftMaster.configPeakOutputReverse(-12.0f, 0);
//		leftMaster.selectProfileSlot(0, 0);
//		leftMaster.config_kF(0, kF, 0);
//		leftMaster.config_kP(0, kP, 0);
//		leftMaster.config_kI(0, kI, 0);
//		leftMaster.config_kD(0, kD, 0);

		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		
		leftMaster.setInverted(true); // Left master must be attached to the farthest CIM from the output shaft
		leftSlave1.setInverted(false); 
		leftSlave2.setInverted(false);
		
		rightMaster.setInverted(false); // Right master must be attached to the farthest CIM from the output shaft
		rightSlave1.setInverted(true); 
		rightSlave2.setInverted(true);

		// 160ms, hard coded in for now because CTR did not add the StatusFrameRate for QuadEncoder
//		rightMaster.setStatusFramePeriod(160, 2, 0);
//		leftMaster.setStatusFramePeriod(160, 2, 0);
		
		resetEncoder();
		resetGyro();
	}

	public void setMotorSpeeds(double rightSpeed, double leftSpeed) {
		SetLeft(leftSpeed);
		SetRight(rightSpeed);
	}

	// Setting the left master Talon's speed to the given parameter
	public void SetLeft(double speed) {
		leftMaster.set(ControlMode.PercentOutput, speed);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed) {
		rightMaster.set(ControlMode.PercentOutput, speed);
	}

	// Getting raw position value from the right encoder
	public double getRightEncoder() {
		return rightMaster.getSelectedSensorPosition(0);
	}

	// Getting raw position value from the left encoder
	public double getLeftEncoder() {
		return leftMaster.getSelectedSensorPosition(0);
	}

	// Getting the average raw velocity from both TalonSRXs
	public double getAverageRawVelocity() {
		return (rightMaster.getSelectedSensorVelocity(0) + leftMaster.getSelectedSensorVelocity(0)) * 0.5;
	}
	
	// Getting the raw velocity from the right TalonSRX
	public double getRightVelocity(){
		return rightMaster.getSelectedSensorVelocity(0);
	}
	
	// Getting the raw velocity from the left TalonSRX
	public double getLeftVelocity(){
		return leftMaster.getSelectedSensorVelocity(0);
	}

	// Getting the average encoder position from both encoders
	public double getAverageEncoderPosition() {
		return (rightMaster.getSelectedSensorPosition(0) + leftMaster.getSelectedSensorPosition(0)) * 0.5;
	}

	// Getting the position from both encoders in meters
	public double getPosition() {
		return (rightMaster.getSelectedSensorPosition(0) + leftMaster.getSelectedSensorPosition(0)) * 0.5 / encoderTicksPerMeter; // [meters]
	}

	// Getting the velocity from both TalonSRXs in meters per second
	public double getVelocity() {
		return (rightMaster.getSelectedSensorVelocity(0) + leftMaster.getSelectedSensorVelocity(0)) * 0.5 * 10 / encoderTicksPerMeter; // [meters/second]
	}

	// Getting the angle in radians from the spartan board
	public double getHeading() {
		heading = (gyro.getAngle() * (Math.PI / 180));
		return heading; // [radians]
	}

	// Getting the angular speed in radian per second from the spartan board
	public double getOmega() {
		return (gyro.getRate() * (Math.PI / 180)); // [radians/second] 
	}

	// Method to set the the linear and angular speed of the robot
//	public void setTargets(double v, double omega) {
//		leftSpeed = (v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter;
//		if(!(leftSpeed == 0)) {
//			if(leftSpeed > 0 && leftSpeed < threshold) {
//				leftSpeed = threshold;
//			}
//			if(leftSpeed < 0 && leftSpeed > -threshold) {
//				leftSpeed = -threshold;
//			}
//		}
//		
//		rightSpeed = (v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter;
//		if(!(rightSpeed == 0)) {
//			if(rightSpeed > 0 && rightSpeed < threshold) {
//				rightSpeed = threshold;
//			}
//			if(rightSpeed < 0 && rightSpeed > -threshold) {
//				rightSpeed = -threshold;
//			}
//		}
		
		
		
//		rightMaster.set(ControlMode.Velocity, rightSpeed);
//		leftMaster.set(ControlMode.Velocity, leftSpeed);
//	}

	// Method to reset the encoder values
	public void resetEncoder() {
		rightMaster.setSelectedSensorPosition(0, 0, 0);
		leftMaster.setSelectedSensorPosition(0, 0, 0);
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		gyro.reset(); 
	}

	// Accessor for the backlash variable
	public double getBacklash() {
		return backlash; // [meters]
	}

	// Accessor for the absoluteMaxSpeed variable
	public double absoluteMaxSpeed() {
		return absoluteMaxSpeed; // [meters/second]
	}

	// Accessor for the approximateSensorSpeed variable
	public double getSensorSpeed() {
		return approximateSensorSpeed;
	}
	
	// Method to shift the drive to low gear
	public void shiftToLowGear() {
		solenoid.set(false);
	}
	
	// Method to shift the drive to high gear
	public void shiftToHighGear() {
		solenoid.set(true);
	}
	
	// Method to initialize 
	public void init(){
		if(gyro == null){ 
			gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		}
		gyro.calibrate();
	}
}
