package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class LeftLeftRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private PivotTrajectory Move3;
	private LinearTrajectory Move4;
	private LinearTrajectory Move5;
	private LinearTrajectory Move5L;
	private PivotTrajectory Move6L;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 1400;
	private boolean isFinished = false;
	private boolean isLeft = false;
	
	public void switchLeft() {
		isLeft = true;
	}
	
	public static enum run {
		IDLE, SCALEUP, MOVE1, MOVE2, FIRE, REVDOWN, COMEDOWN, SWITCHRIGHT, 
		SWITCHLEFT, MOVE3, INTAKE, MOVE4, SWITCHFIRE1, 
		SWITCHFIRE2, SWITCHFIRE3, SWITCHFIRE4, MOVE3R, INTAKER, MOVE4R, MOVE5R,
		MOVE6R, FIRER, REVDOWNR;
		private run() {}
	}
	
//	public LinearTrajectory(DriveHardware drive, double distance, 
//	double accelerationFraction, long accelerateTime,
//	double tkP, double tkD, double rkP,
//	double rkD, int lor) {
//
//public PivotTrajectory(DriveHardware drive, double angle, double accelerationFraction, 
//	long accelerateTime, double rkP, double rkD, int lor) {
	
	public run runIt = run.IDLE;
	
	public LeftLeftRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 6.45, shooArm);
		Move2 = new PivotTrajectory(drive, 65, shooArm);
		Move3 = new PivotTrajectory(drive, 102, shooArm);
		Move4 = new LinearTrajectory(drive, 1.85, shooArm);
		Move5 = new LinearTrajectory(drive, .6, shooArm);
		Move5L = new LinearTrajectory(drive, -1.85, shooArm);
		Move6L = new PivotTrajectory(drive, -102, shooArm);
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
			if(shooArm.scaleIsPID() && shooArm.inScale()) {
				Move1.run();
				runIt = run.MOVE2;
			}
			break;
		case MOVE2:
			if(Move1.isDone()) {
				shooArm.pressLeftTrigger();
				Move2.run();
				runIt = run.FIRE;
			}
		case FIRE:
			if(Move2.isDone()) {
				shooArm.autoFire();
				endTime = currentTime + revTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				runIt = run.COMEDOWN;
			}
			break;
		case COMEDOWN:
			shooArm.pressY();
			if(isLeft) {
				runIt = run.SWITCHLEFT;
			} else {
				runIt = run.SWITCHRIGHT;
			}
			break;
		case SWITCHLEFT:
			runIt = run.MOVE3;
			break;
		case SWITCHRIGHT:
			runIt = run.MOVE3R;
			break;
		case MOVE3:
			if(shooArm.isHold()) {
				Move3.run();
				runIt = run.INTAKE;
			}
			break;
		case INTAKE:
			if(Move3.isDone()) {
				shooArm.setAutoIntake(.7);
				shooArm.pressA();
				runIt = run.MOVE4;
			}
			break;
		case MOVE4:
			if(shooArm.isIntake()) {
				Move4.setSpeed(.4);
				Move4.run();
				runIt = run.SWITCHFIRE1;
			}
			break;
		case SWITCHFIRE1:
			if(Move4.isDone() && shooArm.inHold()) {
				shooArm.setAutoIntake(.6);
				shooArm.pressX();
				shooArm.switchShoot();
				shooArm.pressLeftTrigger();
				endTime = currentTime + revTime;
				runIt = run.SWITCHFIRE2;
			}
			break;
		case SWITCHFIRE2:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move5.run();
				runIt = run.SWITCHFIRE3;
			}
			break;
		case SWITCHFIRE3:
			if(currentTime > endTime && Move5.isDone()) {
				shooArm.autoFire();
				endTime = currentTime + revTime;
				runIt = run.SWITCHFIRE4;
			}
			break;
		case SWITCHFIRE4:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				isFinished = true;
				runIt = run.IDLE;
			}
			break;
		case MOVE3R:			
			if(shooArm.isHold()) {
				Move3.run();
				runIt = run.INTAKER;
			}
			break;
		case INTAKER:
			if(Move3.isDone()) {
				shooArm.setAutoIntake(.7);
				shooArm.pressA();
				runIt = run.MOVE4R;
			}
			break;
		case MOVE4R:
			if(shooArm.isIntake()) {
				Move4.setSpeed(.4);
				Move4.run();
				runIt = run.MOVE5R;
			}
			break;
		case MOVE5R:
			if(Move4.isDone() && shooArm.inHold()) {
				Move5L.run();
				shooArm.setAutoIntake(.6);
				shooArm.pressY();
				shooArm.scaleShoot();
				shooArm.pressLeftTrigger();
				endTime = currentTime + revTime;
				runIt = run.MOVE6R;
			}
			break;
		case MOVE6R:
			if(Move5.isDone()) {
				Move6L.run();
				runIt = run.FIRER;
			}
			break;
		case FIRER:
			if(Move6L.isDone() && currentTime > endTime) {
				shooArm.autoFire();
				currentTime = endTime + revTime;
				runIt = run.REVDOWNR;
			}
			break;
		case REVDOWNR:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				isFinished = true;
				runIt = run.IDLE;
			}
			break;
		}
	}
	
	
	public void run() {
		
		runIt = run.SCALEUP;
		while(!isFinished) {
			update();
			

			if(Timer.getMatchTime() < 1) {
				isFinished = true;
			}
		}
	}
}
