package org.usfirst.frc.team1197.robot;

public class LeftRightRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private PivotTrajectory Move4;
	private LinearTrajectory Move5;
	private PivotTrajectory Move6;
	private LinearTrajectory Move7;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 700;
	private long stopTime = 500;
	private boolean isFinished = false;
	
	
	public static enum run {
		IDLE, SWITCHUP, MOVE1, MOVE2, MOVE3, MOVE4, MOVE5, MOVE6, MOVE7, REVUP, FIRE, REVDOWN;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	
	public LeftRightRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 5.46, 0.75, 2500, 0);
		Move2 = new PivotTrajectory(drive, 90, 0.565, 2500, 0.0);
		Move3 = new LinearTrajectory(drive, 6, 0.75, 2500, 0);
		Move4 = new PivotTrajectory(drive, -90, 0.565, 2500, 0.0);
		Move5 = new LinearTrajectory(drive, 2.3, 0.75, 2500, 0.0);
		Move6 = new PivotTrajectory(drive, -90, 0.565, 2500, 0.0);
		Move7 = new LinearTrajectory(drive, 0.0, 0.0, 1, 0.0);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case SWITCHUP:
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
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move2.isDone()) {
				Move3.run();
				runIt = run.MOVE4;
			}
			break;
		case MOVE4:
			if(Move3.isDone()) {
				shooArm.switchShoot();
				Move4.run();
				runIt = run.MOVE5;
			}
			break;
		case MOVE5:
			if(Move4.isDone()) {
				Move5.run();
				runIt = run.MOVE6;
			}
			break;
		case MOVE6:
			if(Move5.isDone()) {
				shooArm.pressY();
				Move6.run();
				runIt = run.REVUP;
			}
			break;
		case REVUP:
			if(Move6.isDone()) {
				shooArm.pressLeftTrigger();
				runIt = run.MOVE7;
			}
			break;
		case MOVE7:
			Move7.run();
			endTime = currentTime + revTime + stopTime;
			runIt = run.FIRE;
			break;
		case FIRE:
			if(Move7.isDone() && (currentTime > endTime) && shooArm.inScale()) {
				shooArm.autoFire();
				endTime = currentTime + revTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				isFinished = true;
				runIt = run.IDLE;
			}
			break;
		}
	}
	
	
	public void run() {
		runIt = run.SWITCHUP;
		while(!isFinished) {
			update();
		}
	}
}
