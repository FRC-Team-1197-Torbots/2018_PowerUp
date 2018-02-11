package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.robot.LeftLeftRun.run;

public class CenterRightRun {
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
	
	
	public CenterRightRun(TorDrive drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		Move1 = new CenterRight1();
		Move2 = new CenterRight2();
		Move3 = new CenterRight3();
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
