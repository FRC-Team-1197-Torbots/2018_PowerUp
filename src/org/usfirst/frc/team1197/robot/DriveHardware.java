package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
//Currently configured for the red sensor bot.
public class DriveHardware {
	
	private ADXRS450_Gyro gyro;

	private final TalonSRX rightMaster;
	private final TalonSRX rightSlave1;
	private final TalonSRX leftMaster;
	private final TalonSRX leftSlave1;
	private final Solenoid solenoid;
	
	private static final double encoderTicksPerMeter = 1689.2; // (units: ticks per meter)
	private static final double approximateSensorSpeed = 4357; // measured maximum (units: RPM)
	private static final double quadEncNativeUnits = 512.0; // (units: ticks per revolution)
	
	public static final double trackWidth = 0.5786; // [meters] (0.5786m = 23.13")
	public static final double halfTrackWidth = trackWidth / 2.0; // [meters]
	public static final double backlash = 0.015; // [meters]
	
	public static final double absoluteMaxSpeed = (approximateSensorSpeed * quadEncNativeUnits) 
			/ (60 * encoderTicksPerMeter); // [meters/sec] (2017 robot: ~4.405 m/s)
	public static final double absoluteMaxOmega = absoluteMaxSpeed / halfTrackWidth;
	
	private final double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));
	private final double kP = 1.0; 
	private final double kI = 0.0; 
	private final double kD = 50.0; 
	
	private boolean leftOutputReversed = false;
	private boolean rightOutputReversed = true;
	private double heading = 0.0;

	public DriveHardware() {
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		
		solenoid = new Solenoid(0);
		
		// TalonSRX, temporary fix
		leftMaster = new TalonSRX(5);
		leftSlave1 = new TalonSRX(6);  
		rightMaster = new TalonSRX(3);
		rightSlave1 = new TalonSRX(4);  

		// use only when testing to get approximate sensor speed
		// rightSlave2.configEncoderCodesPerRev(128);
		// leftMaster.configEncoderCodesPerRev(128);

		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightMaster.setSensorPhase(false);
		rightMaster.setInverted(rightOutputReversed); 
		// rightMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		// rightMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		rightMaster.selectProfileSlot(0, 0);
		rightMaster.config_kF(0, kF, 0);
		rightMaster.config_kP(0, kP, 0);
		rightMaster.config_kI(0, kI, 0);
		rightMaster.config_kD(0, kD, 0);

		rightSlave1.follow(rightMaster);

		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		leftMaster.setSensorPhase(true); 
		leftMaster.setInverted(leftOutputReversed); 
		// leftMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		// leftMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftMaster.selectProfileSlot(0, 0);
		leftMaster.config_kF(0, kF, 0);
		leftMaster.config_kP(0, kP, 0);
		leftMaster.config_kI(0, kI, 0);
		leftMaster.config_kD(0, kD, 0);

		leftSlave1.follow(leftMaster);

//		rightMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
//		leftMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
		
		resetEncoder();
		resetGyro();
	}

	public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
		if(leftOutputReversed){
			SetLeft(-leftSpeed);
		}
		else{
			SetLeft(leftSpeed);
		}
		if(rightOutputReversed){
			SetRight(-rightSpeed);
		}
		else{
			SetRight(rightSpeed);
		}
	}

	// Setting the left master Talon's speed to the given parameter
	public void SetLeft(double speed) {
		leftMaster.set(ControlMode.PercentOutput, speed);
	}

	// Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed) {
		rightMaster.set(ControlMode.PercentOutput, speed);
	}

	/*
	 * A method to change the right and left master Talon's control mode to
	 * percentVbus, so we can use it when the robot is in low gear.
	 */
	public void choosePercentVbus() {
//		rightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//		leftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	/*
	 * A method to change the right and left master Talon's control mode to
	 * speed, so we can use it when the robot is in high gear.
	 */
	public void chooseVelocityControl() {
//		rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
//		leftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
	}

	public void chooseMotionProfileControl() {
//		rightMaster.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
//		leftMaster.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
	}

	public double getRightEncoder() {
		return rightMaster.getSelectedSensorPosition(0);
	}

	public double getLeftEncoder() {
		return leftMaster.getSelectedSensorPosition(0);
	}

	public double getAverageRawVelocity() {
		return (rightMaster.getSelectedSensorVelocity(0) + leftMaster.getSelectedSensorVelocity(0)) * 0.5;
	}
	
	public double getRightVelocity(){
		return rightMaster.getSelectedSensorVelocity(0);
	}
	
	public double getLeftVelocity(){
		return leftMaster.getSelectedSensorVelocity(0);
	}

	public double getAverageEncoderPosition() {
		return (rightMaster.getSelectedSensorPosition(0) + leftMaster.getSelectedSensorPosition(0)) * 0.5;
	}

	public double getPosition() {
		return (rightMaster.getSelectedSensorPosition(0) + leftMaster.getSelectedSensorPosition(0)) * 0.5 / encoderTicksPerMeter; // [meters]
	}

	public double getVelocity() {
		return (rightMaster.getSelectedSensorVelocity(0) + leftMaster.getSelectedSensorVelocity(0)) * 0.5 * 10 / encoderTicksPerMeter; // [meters/second]
	}

	public double getHeading() {
		heading = (gyro.getAngle() * (Math.PI / 180));
		return heading; // [radians]
	}

	public double getOmega() {
		return (gyro.getRate() * (Math.PI / 180)); // [radians/second] 
	}

	public void setTargets(double v, double omega) {
		rightMaster.set(ControlMode.MotionProfile, (v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
		leftMaster.set(ControlMode.MotionProfile, (v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
	}

	public void resetEncoder() {
		rightMaster.setSelectedSensorPosition(0, 0, 0);
		leftMaster.setSelectedSensorPosition(0, 0, 0);
	}

	public void resetGyro() {
		gyro.reset(); 
	}

	public double getBacklash() {
		return backlash; // [meters]
	}

	public double absoluteMaxSpeed() {
		return absoluteMaxSpeed; // [meters/second]
	}

	public double getSensorSpeed() {
		return approximateSensorSpeed;
	}
	
	public void shiftToLowGear() {
		solenoid.set(true);
	}
	
	public void shiftToHighGear() {
		solenoid.set(false);
	}
	
	public void init(){
		if(gyro == null){ 
			gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		}
		gyro.calibrate();
	}
}
