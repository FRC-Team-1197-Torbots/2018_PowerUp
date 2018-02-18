package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.trajectories.LeftRight1;

public class LeftRightRun {
	private TorTrajectory Move1;
	private TorBantorShooarm shooArm;
	private TorDrive drive;
	private long currentTime;
	private long endTime;
	private long revTime = 2000;
	private long extendTime = 200;
	
	public static enum run {
		IDLE, MOVE1, REVUP, FIRE, REVDOWN;
		private run() {}
	}
	private run runIt = run.IDLE;
	
	public LeftRightRun(TorDrive drive, TorBantorShooarm shooArm) {
		Move1 = new LeftRight1();
		this.drive = drive;
		this.shooArm = shooArm;
	}
	
	public void update() {
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case MOVE1:
			drive.executeTrajectory(Move1);
			break;
		case REVUP:
			if(Move1.isComplete) {
				shooArm.pressLeftTrigger();
				endTime = currentTime + revTime;
				runIt = run.FIRE;
			}
			break;
		case FIRE:
			if(currentTime > endTime) {
				shooArm.autoFire();
				endTime = currentTime + extendTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				runIt = run.IDLE;
			}
			break;
		}
	}
	public void run() {
		runIt = run.MOVE1;
	}
	
}
