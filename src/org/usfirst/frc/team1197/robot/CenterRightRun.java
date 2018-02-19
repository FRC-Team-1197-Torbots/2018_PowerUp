package org.usfirst.frc.team1197.robot;

public class CenterRightRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private PivotTrajectory Move4;
	private LinearTrajectory Move5;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 700;
	private boolean isFinished = false;
	
	public static enum run {
		IDLE, SWITCHUP, MOVE1, MOVE2, MOVE3, MOVE4, REVUP, MOVE5, FIRE, REVDOWN;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public CenterRightRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 1.6, 0.6, 1500, 0.0);
		Move2 = new PivotTrajectory(drive, 90, 0.565, 2500, 0.0);
		Move3 = new LinearTrajectory(drive, 1.45, 0.6, 1500, 0.0);
		Move4 = new PivotTrajectory(drive, -90, 0.565, 2500, 0.0);
		Move5 = new LinearTrajectory(drive, 1.2, 0.6, 1500, 0.0);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case SWITCHUP:
			shooArm.pressXStart();
			runIt = run.MOVE1;
			break;
		case MOVE1:
			if(shooArm.switchIsPID()) {
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
				runIt = run.MOVE4;
			}
			break;
		case MOVE4:
			if(Move3.isDone()) {
				shooArm.switchShoot();
				Move4.run();
				runIt = run.REVUP;
			}
			break;
		case REVUP:
			if(Move4.isDone()) {
				shooArm.pressLeftTrigger();
				runIt = run.MOVE5;
			}
			break;
		case MOVE5:
			Move5.run();
			runIt = run.FIRE;
			break;
		case FIRE:
			if(Move5.isDone()) {
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
		runIt = run.SWITCHUP;
		while(!isFinished) {
			update();
		}
	}
}
