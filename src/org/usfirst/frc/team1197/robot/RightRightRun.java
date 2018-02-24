package org.usfirst.frc.team1197.robot;

public class RightRightRun {
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
	
//	public LinearTrajectory(DriveHardware drive, double distance, 
//	double accelerationFraction, long accelerateTime,
//	double tkP, double tkD, double rkP,
//	double rkD, int lor) {
//
//public PivotTrajectory(DriveHardware drive, double angle, double accelerationFraction, 
//	long accelerateTime, double rkP, double rkD, int lor) {
	
	public run runIt = run.IDLE;
	
	public RightRightRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 3.8, 0.87, 4000);
		Move2 = new PivotTrajectory(drive, -90, 0.575, 2500);
		Move3 = new LinearTrajectory(drive, 0.55, 0.6, 4000);
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
			if(Move1.isDone()) {
				Move2.run();
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move2.isDone()) {
				Move3.run();
				runIt = run.REVUP;	
			}
			break;
		case REVUP:
			if(Move3.isDone()) {
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
