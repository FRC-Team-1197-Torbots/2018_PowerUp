package org.usfirst.frc.team1197.robot;

public class LeftLeftRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 700;
	private long stopTime = 500;
	private boolean isFinished = false;
	
	public static enum run {
		IDLE, SCALEUP, MOVE1, MOVE2, REVUP, MOVE3, SCALEDOWN, SCALEUP2, FIRE, REVDOWN;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public LeftLeftRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 8.2, 0.8, 1500, 0.0);
		Move2 = new PivotTrajectory(drive, 90, 0.575, 2500, 0.0);
		Move3 = new LinearTrajectory(drive, -0.5, 0.4, 3000, 0.0);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case SCALEUP:
			shooArm.pressYStart();
			runIt = run.MOVE1;
			break;
		case MOVE1:
			Move1.run();
			runIt = run.MOVE2;
			break;
		case MOVE2:
			if(Move1.isDone()) {
				Move2.run();
				runIt = run.REVUP;
			}
			break;
		case REVUP:
			if(Move2.isDone()) {
				shooArm.pressLeftTrigger();
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			endTime = currentTime + stopTime;
			runIt = run.SCALEDOWN;
			break;
		case SCALEDOWN:
			if(Move3.isDone() && currentTime > endTime) {
				shooArm.pressY();
				runIt = run.SCALEUP2;
			}
			break;
		case SCALEUP2:
			if(shooArm.isHold()) {
				shooArm.pressY();
				endTime = currentTime + revTime;
				runIt = run.FIRE;
			}
			break;
		case FIRE:
			if(Move3.isDone() && (currentTime > endTime) && shooArm.inScale()) {
				shooArm.autoFire();
				endTime = currentTime + revTime;
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
