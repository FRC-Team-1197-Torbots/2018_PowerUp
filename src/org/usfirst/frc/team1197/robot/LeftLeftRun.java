package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.trajectories.LeftLeft1;
import org.usfirst.frc.team1197.trajectories.LeftLeft2;
import org.usfirst.frc.team1197.trajectories.LeftLeft3;

public class LeftLeftRun {
	private TorDrive drive;
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	private TorBantorShooarm shooArm;
	
	public static enum run {
		IDLE, MOVE1, FIRE, MOVE2, MOVE3;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	
	public LeftLeftRun(TorDrive drive, TorBantorShooarm shooArm) {
		Move1 = new LeftLeft1();
		Move2 = new LeftLeft2();
		Move3 = new LeftLeft3();
		this.drive = drive;
		this.shooArm = shooArm;
	}
	
	public void update() {
		switch(runIt) {
		case IDLE:
			break;
		case MOVE1:
			drive.executeTrajectory(Move1);
			runIt = run.FIRE;
			break;
		case FIRE:
			if(Move1.isComplete) {
				shooArm.autoFire();
				drive.executeTrajectory(Move2);
				runIt = run.MOVE2;
			}
			break;
		case MOVE2:
			if(Move2.isComplete) {
				drive.executeTrajectory(Move3);
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move3.isComplete) {
				runIt = run.IDLE;
			}
			break;
		}
	}
	
	
	public void run() {
		runIt = run.MOVE1;
	}
}
