package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.trajectories.CenterRight1;
import org.usfirst.frc.team1197.trajectories.CenterRight2;
import org.usfirst.frc.team1197.trajectories.CenterRight3;

public class CenterRightRun {
	private TorDrive drive;
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	private long currentTime;
	private long endTime;
	private final long extendTime = 200;
	private TorBantorShooarm shooArm;

	public static enum run {
		IDLE, MOVEUP, MOVE1, REVUP, FIRE, REVDOWN, MOVE2, MOVE3;
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
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case MOVEUP:
			shooArm.pressX();
			runIt = run.MOVE1;
		case MOVE1:
			if(shooArm.switchIsPID()) {
				drive.executeTrajectory(Move1);
				runIt = run.REVUP;
			}
			break;
		case REVUP:
			shooArm.pressLeftTrigger();
			runIt = run.FIRE;
			break;
		case FIRE:
			if(Move1.isComplete) {
				shooArm.autoFire();
				endTime = currentTime + extendTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
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
		runIt = run.MOVEUP;
	}
}
