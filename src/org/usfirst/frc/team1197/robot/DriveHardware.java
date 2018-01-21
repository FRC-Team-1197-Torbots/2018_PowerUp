package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Currently configured for the red sensor bot.
public class DriveHardware {
	
	private ADXRS450_Gyro gyro;

	private final TalonSRX rightMaster;
	private final TalonSRX rightSlave1;
	private final TalonSRX rightSlave2;
	private final TalonSRX leftMaster;
	private final TalonSRX leftSlave1;
	private final TalonSRX leftSlave2;
	
	private final Solenoid solenoid;
	//5125 - encoder, 46 inches.
	private static final double encoderTicksPerMeter = 4416.08796; // (units: ticks per meter)
	private static final double approximateSensorSpeed = 545.2148; // measured maximum (units: RPM)
	private static final double quadEncNativeUnits = 512.0; // (units: ticks per revolution)
	
	public static final double trackWidth = 0.5842; // [meters].
	public static final double halfTrackWidth = trackWidth / 2.0; // [meters]
	public static final double backlash = 0.015; // [meters]
	
	public static final double absoluteMaxSpeed = (approximateSensorSpeed * quadEncNativeUnits) 
			/ (60 * encoderTicksPerMeter); // [meters/sec] (2017 robot: ~4.405 m/s)
	public static final double absoluteMaxOmega = absoluteMaxSpeed / halfTrackWidth;
	
	private final double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));
	private final double kP = 0.0; 
	private final double kI = 0.0; 
	private final double kD = 0.0; 
	
	private boolean leftOutputReversed = false;
	private boolean rightOutputReversed = true;
	
	private double heading = 0.0;

	// WEST COAST DRIVE GEARBOXES
	// MAKE SURE
	// TO
	// MAKE ONE OF THE MOTORS OPPOSITE!!!!!!!!!!!!!!!!!!!!!!!!!!! (motor that is farthest from the ouput shaft)
	public DriveHardware() {
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		
		solenoid = new Solenoid(0);
		
		rightMaster = new TalonSRX(1);
		rightSlave1 = new TalonSRX(2);
		rightSlave2 = new TalonSRX(3);
		leftMaster = new TalonSRX(4);
		leftSlave1 = new TalonSRX(5);
		leftSlave2 = new TalonSRX(6);  

		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightMaster.setSensorPhase(rightOutputReversed);
		rightMaster.configNominalOutputForward(+0.0f, 0);
		rightMaster.configNominalOutputReverse(-0.0f, 0);
		rightMaster.configPeakOutputForward(+12.0f, 0);
		rightMaster.configPeakOutputReverse(-12.0f, 0);
		rightMaster.selectProfileSlot(0, 0);
		rightMaster.config_kF(0, kF, 0);
		rightMaster.config_kP(0, kP, 0);
		rightMaster.config_kI(0, kI, 0);
		rightMaster.config_kD(0, kD, 0);

		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);

		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		leftMaster.setSensorPhase(leftOutputReversed); 
		leftMaster.configNominalOutputForward(+0.0f, 0);
		leftMaster.configNominalOutputReverse(-0.0f, 0);
		leftMaster.configPeakOutputForward(+12.0f, 0);
		leftMaster.configPeakOutputReverse(-12.0f, 0);
		leftMaster.selectProfileSlot(0, 0);
		leftMaster.config_kF(0, kF, 0);
		leftMaster.config_kP(0, kP, 0);
		leftMaster.config_kI(0, kI, 0);
		leftMaster.config_kD(0, kD, 0);

		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);

		// 160ms, hardcoded in for now because CTR did not add the StatusFrameRate for QuadEncoder
		leftMaster.setStatusFramePeriod(160, 2, 0);
		leftMaster.setStatusFramePeriod(160, 2, 0);
		
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
		rightMaster.set(ControlMode.Velocity, (v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
		leftMaster.set(ControlMode.Velocity, (v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
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
		SmartDashboard.putBoolean("LowGear", true);
		solenoid.set(true);
	}
	
	public void shiftToHighGear() {
		SmartDashboard.putBoolean("LowGear", false);
		solenoid.set(false);
	}
	
	public void init(){
		if(gyro == null){ 
			gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		}
		gyro.calibrate();
	}
}
