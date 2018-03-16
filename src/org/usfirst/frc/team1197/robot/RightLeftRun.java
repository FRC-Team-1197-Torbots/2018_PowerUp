package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class RightLeftRun {
	private LinearTrajectory Move1;
	private LinearTrajectory Move1R;
	private PivotTrajectory Move2R;
	private LinearTrajectory Move3R;
	private TorBantorShooarm shooArm;
	private double starttime;
	private boolean isFinished = false;
	private boolean switchRight = false;
	private long currentTime;
	private long endTime;
	private final long revTime = 1400;
	
	public void switchRight() {
		switchRight = true;
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
	
	public RightLeftRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 6, shooArm);
		Move1R = new LinearTrajectory(drive, 3.8, shooArm);
		Move2R = new PivotTrajectory(drive, -90, shooArm);
		Move3R = new LinearTrajectory(drive, 0.55, shooArm);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		switch(runIt) {
		case IDLE:
			break;
		case SWITCHUP:
			shooArm.pressXStart();
			if(switchRight) {
				runIt = run.Move1R;
			} else {
				runIt = run.MOVE1;
			}
			break;
		case MOVE1:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move1.run(starttime);
				isFinished = true;
				runIt = run.IDLE;
				break;
			}
		case Move1R:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move1R.run(starttime);
				runIt = run.Move2R;
			}
			break;
		case Move2R:
			if(Move1.isDone()) {
				shooArm.pressX();
				shooArm.switchShoot();
				Move2R.run(starttime);
				runIt = run.Move3R;
			}
			break;
		case Move3R:
			if(Move2R.isDone()) {
				Move3R.run(starttime);
			}
			break;
		case FIRE:
			if(Move3R.isDone()) {
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
	
	
	public void run(double starttime) {
		this.starttime = starttime;
		runIt = run.SWITCHUP;
		while(!isFinished) {
			update();
			

			if(Timer.getFPGATimestamp() - starttime > 14) {
				isFinished = true;
			}
		}
	}
}
