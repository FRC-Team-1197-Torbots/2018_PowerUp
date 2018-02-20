package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.robot.RightRightRun.run;

public class LeftLeftRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 700;
	private long extendTime = 300;
	private boolean isFinished = false;
	
	public static enum run {
		IDLE, SCALEUP, MOVE1, MOVE2, MOVE3, REVUP, FIRE, REVDOWN;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public LeftLeftRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 3.8, 0.87, 4000, 0.0);
		Move2 = new PivotTrajectory(drive, 90, 0.575, 2500, 0.0);
		Move3 = new LinearTrajectory(drive, 0.5, 0.55, 3000, 0.0);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case SCALEUP:
			shooArm.pressXStart();
			runIt = run.MOVE1;
			break;
		case MOVE1:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move1.run();
				runIt = run.MOVE2;
			}
			break;
		case MOVE2:
			if(Move1.isDone() && shooArm.inSwitch()) {
				Move2.run();
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move2.isDone() && shooArm.inSwitch()) {
				Move3.run();
				runIt = run.REVUP;	
			}
			break;
		case REVUP:
			if(Move3.isDone() && shooArm.inSwitch()) {
				shooArm.pressLeftTrigger();
				endTime = currentTime + revTime;
				runIt = run.FIRE;				
			}
			break;
		case FIRE:
			if(currentTime >= endTime) {
				shooArm.autoFire();
				endTime = currentTime + extendTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				runIt = run.IDLE;
				isFinished = true;
			}
			break;
		}
	}
	
	
	public void run() {
		runIt = run.SCALEUP;
		while(!isFinished) {
			update();
		}
	}
}