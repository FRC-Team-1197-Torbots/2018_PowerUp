package org.usfirst.frc.team1197.robot;

public class LeftRightRun {
	private LinearTrajectory Move1;
	private TorBantorShooarm shooArm;
	private boolean isFinished = false;
	
	
	public static enum run {
		IDLE, SWITCHUP, MOVE1;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	
	public LeftRightRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 6, 0.75, 5000, 0);
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		switch(runIt) {
		case IDLE:
			break;
		case SWITCHUP:
			shooArm.pressXStart();
			runIt = run.MOVE1;
			break;
		case MOVE1:
			if(shooArm.switchIsPID() && shooArm.inSwitch()) {
				Move1.run();
				isFinished = true;
				runIt = run.IDLE;
				break;
			}
		}
	}
	
	
	public void run() {
		runIt = run.SWITCHUP;
		while(!isFinished) {
			update();
		}
	}
}
