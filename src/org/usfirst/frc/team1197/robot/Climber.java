package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber {
	private Solenoid releaser;
	private TalonSRX puller1;
	private TalonSRX puller2;
	private TorBantorShooarm armShoo;
	private Joystick player2;
	
	public Climber(Solenoid releaser, TalonSRX puller1, TalonSRX puller2, TorBantorShooarm armShoo, Joystick player2) {
		this.releaser = releaser;
		this.puller1 = puller1;
		this.armShoo = armShoo;
		this.player2 = player2;
		this.puller2 = puller2;
	}
	
	public static enum climb {
		IDLE, POS0, FIRE, LIFT;
		private climb() {}
	}
	
	climb climbIt = climb.IDLE;
	
	public void update() {
		climbDo();
		if(player2.getRawButton(2) 
				&& player2.getRawButton(5) 
				&& player2.getRawButton(6)) {
			climbIt = climb.POS0;
		}
	}
	
	/*THE TUNES--------------------------------------------------------------------------
	*/
	private long fireTime = 600;
	private long pullTime = 3000;
	private double pullPower = 1;
	private double angleToHold = 15;
	private double waitTime = 500;//in milliseconds
	/*-----------------------------------------------------------------------------------
	*/
	
	private long currentTime;
	private long endTime;
	private boolean isStopped = false;
	
	public void climbDo() {
		currentTime = System.currentTimeMillis();
		switch(climbIt) {
		case IDLE:
			endTime = currentTime + (long)waitTime;
			break;
		case POS0:
			if(!isStopped) {
				armShoo.stop(angleToHold);
				isStopped = true;
			}
			currentTime = System.currentTimeMillis();
			if(currentTime > endTime) {
				endTime = currentTime + fireTime;
				climbIt = climb.FIRE;
			}
			break;
		case FIRE:
			releaser.set(false);
			if(currentTime > endTime) {
				endTime = currentTime + pullTime;
				climbIt = climb.LIFT;
			}
			break;
		case LIFT:
			puller1.set(ControlMode.PercentOutput, pullPower);
			puller2.set(ControlMode.PercentOutput, -pullPower);
			if(currentTime > endTime) {
				climbIt = climb.IDLE;
				puller1.set(ControlMode.PercentOutput, 0);
				puller2.set(ControlMode.PercentOutput, 0);
			}
			break;
		}
	}	
}
