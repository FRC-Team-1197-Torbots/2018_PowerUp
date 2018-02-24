package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber {
	private Solenoid releaser;
	private VictorSPX puller1;
	private VictorSPX puller2;
	private TorBantorShooarm armShoo;
	private Joystick autobox;
	
	public Climber(Solenoid releaser, VictorSPX puller1, 
			VictorSPX puller2, TorBantorShooarm armShoo, Joystick autobox) {
		this.releaser = releaser;
		this.puller1 = puller1;
		this.armShoo = armShoo;
		this.autobox = autobox;
		this.puller2 = puller2;
	}
	
	public static enum climb {
		IDLE, POS0, FIRE, LIFT;
		private climb() {}
	}
	
	climb climbIt = climb.IDLE;
	
	public void update() {
		climbDo();
		if(!autobox.getRawButton(1)) {
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
