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
	
	private final Solenoid solenoid;
	private final double encoderTicksPerMeter = 999.84825; // (units: ticks per meter)
	
	
	
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

		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
		
		leftMaster.setInverted(true); // Left master must be attached to the farthest CIM from the output shaft
		leftSlave1.setInverted(false); 
		leftSlave2.setInverted(false);
		
		rightMaster.setInverted(false); // Right master must be attached to the farthest CIM from the output shaft
		rightSlave1.setInverted(true); 
		rightSlave2.setInverted(true);
		
		resetEncoder();
		resetGyro();
	}

	public void setVelocity(double leftSpeed, double rightSpeed) {
		leftMaster.set(ControlMode.Velocity, leftSpeed);
		rightMaster.set(ControlMode.Velocity, rightSpeed);
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

	// Getting the position from both encoders in meters
	public double getPosition() {
		return (rightMaster.getSelectedSensorPosition(0) + leftMaster.getSelectedSensorPosition(0)) * 0.5 / encoderTicksPerMeter; // [meters]
	}

	// Getting the angle in radians from the spartan board
	public double getHeading() {
		heading = (gyro.getAngle() * (Math.PI / 180));
		return heading; // [radians]
	}

	// Method to reset the encoder values
	public void resetEncoder() {
		rightMaster.setSelectedSensorPosition(0, 0, 0);
		leftMaster.setSelectedSensorPosition(0, 0, 0);
	}

	// Method to reset the spartan board gyro values
	public void resetGyro() {
		gyro.reset(); 
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
