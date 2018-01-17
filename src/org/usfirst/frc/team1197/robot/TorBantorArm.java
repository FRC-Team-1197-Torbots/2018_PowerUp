package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;

public class TorBantorArm {
	private Joystick player2;
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	private long currentTime;
	private long endTime;
	
	private int switchUpPos1Time = 40;
	private int switchUpPos2Time = 40;
	private int switchUpPos3Time = 40;
	
	private int scaleUpPos1Time = 40;
	private int scaleUpPos2Time = 40;
	private int scaleUpPos3Time = 40;
	
	private int switchDownPos1Time = 40;
	private int switchDownPos2Time = 40;
	private int switchDownPos3Time = 40;
	
	private int scaleDownPos1Time = 40;
	private int scaleDownPos2Time = 40;
	private int scaleDownPos3Time = 40;
	
	public static enum SWITCHUP {
		IDLE, POS0, POS1, POS2, POS3;
		private SWITCHUP() {}
	}
	
	public static enum SWITCHDOWN {
		IDLE, POS0, POS1, POS2, POS3;
		private SWITCHDOWN() {}
	}
	
	public static enum SCALEUP {
		IDLE, POS0, POS1, POS2, POS3;
		private SCALEUP() {}
	}
	
	public static enum SCALEDOWN {
		IDLE, POS0, POS1, POS2, POS3;
		private SCALEDOWN() {}
	}
	
	public SWITCHUP switchUpState = SWITCHUP.IDLE;
	public SWITCHDOWN switchDownState = SWITCHDOWN.IDLE;
	public SCALEUP scaleUpState = SCALEUP.IDLE;
	public SCALEDOWN scaleDownState = SCALEDOWN.IDLE;
	
	public TorBantorArm(Joystick player2, TalonSRX armTalon1, TalonSRX armTalon2) {
		this.player2 = player2;
		this.armTalon1 = armTalon1;
		this.armTalon2 = armTalon2;
	}
	
	public void armControl() {
		if(player2.getRawButton(1)) {
			switchUpState = SWITCHUP.POS0;
		}
		else if(player2.getRawButton(3)) {
			switchDownState = SWITCHDOWN.POS0;
		}
		else if(player2.getRawButton(2)){
			scaleUpState = SCALEUP.POS0;
		}
		else if(player2.getRawButton(4)) {
			scaleDownState = SCALEDOWN.POS0;
		}
	}
	
	public void armUpdate() {
		if(player2.getRawButton(5)) {
			switchUpState = SWITCHUP.IDLE;
			switchDownState = SWITCHDOWN.IDLE;
			scaleUpState = SCALEUP.IDLE;
			scaleDownState = SCALEDOWN.IDLE;
			manualOverride();
		}
		else if(switchUpState != SWITCHUP.IDLE) {
			switchUp();
		}
		else if(switchDownState != SWITCHDOWN.IDLE) {
			switchDown();
		}
		else if(scaleUpState != SCALEUP.IDLE) {
			scaleUp();
		}
		else if(scaleDownState != SCALEDOWN.IDLE) {
			scaleDown();
		}
	}
	
	public void switchUp() {
		switch(switchUpState) {
		case IDLE:
			break;
		case POS0:
			endTime = System.currentTimeMillis() + switchUpPos1Time;
			switchUpState = SWITCHUP.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + switchUpPos2Time;
				switchUpState = SWITCHUP.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + switchUpPos3Time;
				switchUpState = SWITCHUP.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				armTalon1.set(ControlMode.PercentOutput, 0.0);
				armTalon2.set(ControlMode.PercentOutput, 0.0);
				switchUpState = SWITCHUP.IDLE;
			}
			break;
		}
	}
	
	public void switchDown() {
		switch(switchDownState) {
		case IDLE:
			break;
		case POS0:
			endTime = System.currentTimeMillis() + switchDownPos1Time;
			switchDownState = SWITCHDOWN.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + switchDownPos2Time;
				switchDownState = SWITCHDOWN.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + switchDownPos3Time;
				switchDownState = SWITCHDOWN.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				armTalon1.set(ControlMode.PercentOutput, 0.0);
				armTalon2.set(ControlMode.PercentOutput, 0.0);
				switchDownState = SWITCHDOWN.IDLE;
			}
			break;
		}
	}
	
	public void scaleUp() {
		switch(scaleUpState) {
		case IDLE:
			break;
		case POS0:
			endTime = System.currentTimeMillis() + scaleUpPos1Time;
			scaleUpState = SCALEUP.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + scaleUpPos2Time;
				scaleUpState = SCALEUP.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + scaleUpPos3Time;
				scaleUpState = SCALEUP.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				armTalon1.set(ControlMode.PercentOutput, 0.0);
				armTalon2.set(ControlMode.PercentOutput, 0.0);
				scaleUpState = SCALEUP.IDLE;
			}
			break;
		}
	}
	
	public void scaleDown() {
		switch(scaleDownState) {
		case IDLE:
			break;
		case POS0:
			endTime = System.currentTimeMillis() + scaleDownPos1Time;
			scaleDownState = SCALEDOWN.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + scaleDownPos2Time;
				scaleDownState = SCALEDOWN.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + scaleDownPos3Time;
				scaleDownState = SCALEDOWN.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			if(endTime < currentTime) {
				armTalon1.set(ControlMode.PercentOutput, 0.0);
				armTalon2.set(ControlMode.PercentOutput, 0.0);
				scaleDownState = SCALEDOWN.IDLE;
			}
			break;
		}
	}
	
	public void manualOverride() {
		armTalon1.set(ControlMode.PercentOutput, player2.getY());
		armTalon2.set(ControlMode.PercentOutput, player2.getY());
	}
}
