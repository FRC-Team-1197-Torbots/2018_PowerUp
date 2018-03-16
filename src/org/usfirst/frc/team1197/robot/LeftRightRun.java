package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class LeftRightRun {
	private LinearTrajectory Move1;
	private LinearTrajectory Move1L;
	private PivotTrajectory Move2L;
	private LinearTrajectory Move3L;
	private TorBantorShooarm shooArm;
	private boolean isFinished = false;
	private boolean switchLeft = false;
	private long currentTime;
	private long endTime;
	private final long revTime = 1400;
	
	public void switchLeft() {
		switchLeft = true;
	}
	
	public static enum run {
		IDLE, SWITCHUP, MOVE1, Move1R, Move2R, Move3R, FIRE, REVDOWN;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
//	public LinearTrajectory(DriveHardware drive, double distance, 
//	double accelerationFraction, long accelerateTime,
//	double tkP, double tkD, double rkP,
//	double rkD, int lor) {
//
//public PivotTrajectory(DriveHardware drive, double angle, double accelerationFraction, 
//	long accelerateTime, double rkP, double rkD, int lor) {
	
	public LeftRightRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 6, shooArm);
		Move1L = new LinearTrajectory(drive, 3.8, shooArm);
		Move2L = new PivotTrajectory(drive, 90, shooArm);
		Move3L = new LinearTrajectory(drive, 0.55, shooArm);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		switch(runIt) {
		case IDLE:
			break;
		case SWITCHUP:
			shooArm.pressXStart();
			if(switchLeft) {
				runIt = run.Move1R;
			} else {
				runIt = run.MOVE1;
			}
			break;
		case MOVE1:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move1.run();
				isFinished = true;
				runIt = run.IDLE;
				break;
			}
		case Move1R:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move1L.run();
				runIt = run.Move2R;
			}
			break;
		case Move2R:
			if(Move1.isDone()) {
				shooArm.pressX();
				shooArm.switchShoot();
				Move2L.run();
				runIt = run.Move3R;
			}
			break;
		case Move3R:
			if(Move2L.isDone()) {
				Move3L.run();
			}
			break;
		case FIRE:
			if(Move3L.isDone()) {
				shooArm.autoFire();
				endTime = currentTime + revTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
			}
			break;
		}
	}
	
	
	public void run() {
		
		runIt = run.SWITCHUP;
		while(!isFinished) {
			update();
			

			if(Timer.getMatchTime() < 1) {
				isFinished = true;
			}
		}
	}
}