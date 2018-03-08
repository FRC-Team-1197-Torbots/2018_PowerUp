package org.usfirst.frc.team1197.robot;

public class LeftLeftRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private PivotTrajectory Move3;
	private LinearTrajectory Move4;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 700;
	private long extendTime = 300;
	private boolean isFinished = false;
	
	public static enum run {
		IDLE, SCALEUP, MOVE1, MOVE2, REVUP, FIRE, REVDOWN, COMEDOWN, MOVE3, INTAKE, MOVE4;
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
		Move3 = new PivotTrajectory(drive, 110, shooArm);
		Move4 = new LinearTrajectory(drive, 1.85, shooArm);
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
				Move2.run();
				runIt = run.REVUP;
			}
			break;
		case REVUP:
			if(Move2.isDone()) {
				shooArm.pressLeftTrigger();
				endTime = currentTime + revTime + extendTime;
				runIt = run.FIRE;				
			}
			break;
		case FIRE:
			if(currentTime >= endTime) {
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
			runIt = run.MOVE3;
			break;
		case MOVE3:
			if(shooArm.isHold()) {
				Move3.run();
				runIt = run.INTAKE;
			}
			break;
		case INTAKE:
			if(Move3.isDone()) {
				shooArm.pressA();
				runIt = run.MOVE4;
			}
			break;
		case MOVE4:
			if(shooArm.isHold()) {
				Move4.run();
				isFinished = true;
				runIt = run.IDLE;
			}
		}
	}
	
	
	public void run() {
		runIt = run.SCALEUP;
		while(!isFinished) {
			update();
		}
	}
}
