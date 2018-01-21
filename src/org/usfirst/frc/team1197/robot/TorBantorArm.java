package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class TorBantorArm {
	private Joystick player2;
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	private long currentTime;
	private long endTime;
	
	private int switchPos1Time = 400;
	private int switchPos2Time = 400;
	private int switchPos3Time = switchPos1Time;
	
	private int scalePos1Time = 600;
	private int scalePos2Time = 600;
	private int scalePos3Time = scalePos1Time;
	
	private double speed = 0;//don't change this
	private double increment = 0.0001;
	private double secondincrement = 0.00005;
	
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
		if(player2.getRawButton(1) && switchUpState == SWITCHUP.IDLE && switchDownState == SWITCHDOWN.IDLE && scaleUpState == SCALEUP.IDLE && scaleDownState == SCALEDOWN.IDLE) {
			switchUpState = SWITCHUP.POS0;
		}
		else if(player2.getRawButton(2) && switchUpState == SWITCHUP.IDLE && switchDownState == SWITCHDOWN.IDLE && scaleUpState == SCALEUP.IDLE && scaleDownState == SCALEDOWN.IDLE) {
			switchDownState = SWITCHDOWN.POS0;
		}
		else if(player2.getRawButton(3) && switchUpState == SWITCHUP.IDLE && switchDownState == SWITCHDOWN.IDLE && scaleUpState == SCALEUP.IDLE && scaleDownState == SCALEDOWN.IDLE){
			scaleUpState = SCALEUP.POS0;
		}
		else if(player2.getRawButton(4) && switchUpState == SWITCHUP.IDLE && switchDownState == SWITCHDOWN.IDLE && scaleUpState == SCALEUP.IDLE && scaleDownState == SCALEDOWN.IDLE) {
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
			endTime = System.currentTimeMillis() + switchPos1Time;
			speed = 0;
			switchUpState = SWITCHUP.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, speed);
			speed += increment;
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + switchPos2Time;
				switchUpState = SWITCHUP.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, speed);
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + 2 * switchPos3Time;
				switchUpState = SWITCHUP.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, speed);
			speed -= secondincrement;
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
			endTime = System.currentTimeMillis() + switchPos1Time;
			speed = 0;
			switchDownState = SWITCHDOWN.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			speed -= increment;
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + switchPos2Time;
				switchDownState = SWITCHDOWN.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + 2 * switchPos3Time;
				switchDownState = SWITCHDOWN.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			speed += secondincrement;
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
			endTime = System.currentTimeMillis() + scalePos1Time;
			speed = 0;
			scaleUpState = SCALEUP.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			speed += increment;
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + scalePos2Time;
				scaleUpState = SCALEUP.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + 2 * scalePos3Time;
				scaleUpState = SCALEUP.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			speed -= secondincrement;
			if(endTime < currentTime) {
				armTalon1.set(ControlMode.PercentOutput, 0.0);
				armTalon2.set(ControlMode.PercentOutput, 0.0);
				scaleUpState = SCALEUP.IDLE;
			}
			break;
		}
	}
	
	public void scaleDown() {
		switch(scaleUpState) {
		case IDLE:
			break;
		case POS0:
			endTime = System.currentTimeMillis() + scalePos1Time;
			speed = 0;
			scaleUpState = SCALEUP.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			speed -= increment;
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + scalePos2Time;
				scaleUpState = SCALEUP.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			if(endTime < currentTime) {
				endTime = System.currentTimeMillis() + 2 * scalePos3Time;
				scaleUpState = SCALEUP.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed);
			armTalon2.set(ControlMode.PercentOutput, -speed);
			speed += secondincrement;
			if(endTime < currentTime) {
				armTalon1.set(ControlMode.PercentOutput, 0.0);
				armTalon2.set(ControlMode.PercentOutput, 0.0);
				scaleUpState = SCALEUP.IDLE;
			}
			break;
		}
	}
	
	public void manualOverride() {
			armTalon1.set(ControlMode.PercentOutput, -player2.getRawAxis(1));
			armTalon2.set(ControlMode.PercentOutput, player2.getRawAxis(1));
	}
}
