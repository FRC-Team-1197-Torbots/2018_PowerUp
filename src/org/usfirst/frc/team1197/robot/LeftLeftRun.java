package org.usfirst.frc.team1197.robot;

public class LeftLeftRun {
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	private long currentTime;
	private long endTime;
	private long extendTime = 200;
	private TorDrive drive;
	private double fob = 1;//forwards or backwards
	private double lor = 1;//left or right
	private TorBantorShooarm shooArm;
	public LeftLeftRun(TorDrive drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(7.32109 * fob);
		Move2 = new PivotTrajectory(77.44615 * lor);
		Move3 = new LinearTrajectory(.24372 * fob);
	}
	
	public static enum run {
		IDLE, MOVE1, MOVE2, MOVE3, FIRE, REVDOWN;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public void update() {
		currentTime = System.currentTimeMillis();
		switch(runIt) {
		case IDLE:
			break;
		case MOVE1:
			drive.executeTrajectory(Move1);
			runIt = run.MOVE2;
			break;
		case MOVE2:
			if(Move1.isComplete) {
				drive.executeTrajectory(Move2);
				shooArm.pressLeftTrigger();
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move2.isComplete) {
				drive.executeTrajectory(Move3);
				runIt = run.FIRE;
			}
			break;
		case FIRE:
			if(Move3.isComplete) {
				shooArm.autoFire();
				endTime = currentTime + extendTime;
				runIt = run.REVDOWN;
			}
			break;
		case REVDOWN:
			if(currentTime > endTime) {
				shooArm.releaseLeftTrigger();
				runIt = run.IDLE;
			}
			break;
		}
	}
	
	public void run() {
		runIt = run.MOVE1;
	}
}
