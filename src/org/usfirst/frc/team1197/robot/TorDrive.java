package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TorDrive
{	
	private DriveHardware hardware;
	private boolean isHighGear = true;
	
	public TorDrive(Joystick stick, Joystick cypress, DriveHardware hardware) {
		this.hardware = hardware;
	}

	public void driving(double throttleAxis, double arcadeSteerAxis, double carSteerAxis, boolean shiftButton,
			boolean rightBumper, boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY) {
		if (isHighGear) {
			ArcadeDrive(throttleAxis, arcadeSteerAxis);
			
			// When you hold down the shiftButton (left bumper), then shift to low gear.
			if (shiftButton) {
				isHighGear = false;
				hardware.shiftToLowGear();
			}
		} else {
			ArcadeDrive(throttleAxis, arcadeSteerAxis);
			
			// When you release the shiftButton (left bumper), then shift to high gear.
			if (!shiftButton) {
				isHighGear = true;
				hardware.shiftToHighGear();
			}
		}
	}

	public void ArcadeDrive(double throttleAxis, double arcadeSteerAxis){
		if (Math.abs(arcadeSteerAxis) <= 0.1) {
			arcadeSteerAxis = 0.0D;
		}
		if (Math.abs(throttleAxis) <= 0.2D) {
			throttleAxis = 0.0D;
		}

		if (arcadeSteerAxis >= 0.0D) {
			arcadeSteerAxis *= arcadeSteerAxis;
		} 
		else {
			arcadeSteerAxis = -(arcadeSteerAxis * arcadeSteerAxis);
		}
		
		if (throttleAxis >= 0.0D) {
			throttleAxis *= throttleAxis;
		} 
		else {
			throttleAxis = -(throttleAxis * throttleAxis);
		}
		
		double rightMotorSpeed;
		double leftMotorSpeed;

		if (throttleAxis > 0.0D) {
			if (arcadeSteerAxis > 0.0D) {
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = Math.max(throttleAxis, arcadeSteerAxis);
			}
			else {
				leftMotorSpeed = Math.max(throttleAxis, -arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
		}
		else {
			if (arcadeSteerAxis > 0.0D) {
				leftMotorSpeed = -Math.max(-throttleAxis, arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
			else {
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = -Math.max(-throttleAxis, -arcadeSteerAxis);
			}
		}
		
		hardware.setMotorSpeeds(-rightMotorSpeed, -leftMotorSpeed);
	}
}