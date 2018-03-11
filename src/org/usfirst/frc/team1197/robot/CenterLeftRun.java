package org.usfirst.frc.team1197.robot;

public class CenterLeftRun {
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private PivotTrajectory Move4;
	private LinearTrajectory Move5;
	private LinearTrajectory Move6;
	private PivotTrajectory Move7;
	private LinearTrajectory Move8;
	private PivotTrajectory Move9;
	private LinearTrajectory Move10;
	private PivotTrajectory Move11;
	private LinearTrajectory Move12;
	private PivotTrajectory Move13;
	private PivotTrajectory Move1L;
	private LinearTrajectory Move2L;
	private PivotTrajectory Move3L;
	private LinearTrajectory Move4L;
	private PivotTrajectory Move5L;
	private PivotTrajectory Move1R;
	private LinearTrajectory Move2R;
	private PivotTrajectory Move3R;
	private LinearTrajectory Move4R;
	private PivotTrajectory Move5R;
	private TorBantorShooarm shooArm;
	private long currentTime;
	private long endTime;
	private long revTime = 700;
	private long startTime;
	private boolean isFinished = false;
	private boolean isScaleLeft = false;
	private DriveHardware drive;
	
	public void scaleLeft() {
		isScaleLeft = true;
	}
	
	public static enum run {
		IDLE, SWITCHUP, MOVE1, MOVE2, MOVE3, MOVE4, REVUP, MOVE5, MOVE6, 
		MOVE7, MOVE8, MOVE9, MOVE10, MOVE11, MOVE12, MOVE13,
		GOFORWARD, INTAKE, GOBACKWARDS, 
		FIRE, REVDOWN,
		MOVE1L, MOVE2L, MOVE3L, MOVE4L, MOVE5L, FINISHL, 
		MOVE1R, MOVE2R, MOVE3R, MOVE4R, MOVE5R, FINISHR, 
		SCALEFIRE, SCALEREVDOWN;
		private run() {}
	}
	
//	public LinearTrajectory(DriveHardware drive, double distance, 
//	double accelerationFraction, long accelerateTime,
//	double tkP, double tkD, double rkP,
//	double rkD, int lor) {
//
//	public PivotTrajectory(DriveHardware drive, double angle, double accelerationFraction, 
//	long accelerateTime, double rkP, double rkD, int lor) {
	
	public run runIt = run.IDLE;
	
	public CenterLeftRun(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		this.drive = drive;
		Move1 = new LinearTrajectory(drive, 1.362075, shooArm);
		Move2 = new PivotTrajectory(drive, -90, shooArm);
		Move3 = new LinearTrajectory(drive, 1.644269, shooArm);
		Move4 = new PivotTrajectory(drive, 90, shooArm);
		Move5 = new LinearTrajectory(drive, 1.362075, shooArm);
		Move6 = new LinearTrajectory(drive, -1.362075, shooArm);
		Move7 = new PivotTrajectory(drive, -90, shooArm);
		Move8 = new LinearTrajectory(drive, 1.50205065, shooArm);
		Move9 = new PivotTrajectory(drive, 90, shooArm);
		Move10 = new LinearTrajectory(drive, 4.116325, shooArm);
		Move11 = new PivotTrajectory(drive, 90, shooArm);
		Move12 = new LinearTrajectory(drive, 1.50205065, shooArm);
		Move13 = new PivotTrajectory(drive, 90, shooArm);
		Move1L = new PivotTrajectory(drive, 90, shooArm);
		Move2L = new LinearTrajectory(drive, 1.50205065, shooArm);
		Move3L = new PivotTrajectory(drive, 90, shooArm);
		Move4L = new LinearTrajectory(drive, 1.178785, shooArm);
		Move5L = new PivotTrajectory(drive, 56.15046226, shooArm);
		Move1R = new PivotTrajectory(drive, -90, shooArm);
		Move2R = new LinearTrajectory(drive, 4.69533865, shooArm);
		Move3R = new PivotTrajectory(drive, -90, shooArm);
		Move4R = new LinearTrajectory(drive, 1.178785, shooArm);
		Move5R = new PivotTrajectory(drive, -56.15046226, shooArm);
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
				runIt = run.MOVE4;
			}
			break;
		case MOVE4:
			if(Move3.isDone()) {
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
				runIt = run.MOVE6;
			}
			break;
		case MOVE6:
			Move6.run();
			runIt = run.MOVE7;
			break;
		case MOVE7:
			if(Move6.isDone()) {
				shooArm.pressX();
				Move7.run();
				runIt = run.MOVE8;
			}
			break;
		case MOVE8:
			if(Move7.isDone()) {
				Move8.run();
				runIt = run.MOVE9;
			}
			break;
		case MOVE9:
			if(Move8.isDone()) {
				Move9.run();
				runIt = run.MOVE10;
			}
			break;
		case MOVE10:
			if(Move9.isDone()) {
				Move10.run();
				runIt = run.MOVE11;
			}
			break;
		case MOVE11:
			if(Move10.isDone()) {
				Move11.run();
				runIt = run.MOVE12;
			}
			break;
		case MOVE12:
			if(Move11.isDone()) {
				Move12.run();
				runIt = run.MOVE13;
			}
			break;
		case MOVE13:
			if(Move12.isDone()) {
				shooArm.pressA();
				Move13.run();
				runIt = run.GOFORWARD;
			}
			break;
		case GOFORWARD:
			if(Move13.isDone()) {
				startTime = currentTime;
				drive.setMotorSpeeds(0.3, 0.3);
				runIt = run.INTAKE;
			}
			break;
		case INTAKE:
			if(shooArm.inHold()) {
				endTime = (currentTime - startTime) + currentTime;
				drive.setMotorSpeeds(-0.3, -0.3);
				runIt = run.GOBACKWARDS;
			}
			break;
		case GOBACKWARDS:
			if(currentTime > endTime) {
				drive.setMotorSpeeds(0, 0);
				if(isScaleLeft) {
					runIt = run.MOVE1L;
				} else {
					runIt = run.MOVE1R;
				}
			}
			break;
		case MOVE1L:
			Move1L.run();
			runIt = run.MOVE2L;
			break;
		case MOVE2L:
			if(Move1L.isDone()) {
				Move2L.run();
				runIt = run.MOVE3L;
			}
			break;
		case MOVE3L:
			if(Move2L.isDone()) {
				Move3L.run();
				runIt = run.MOVE4L;
			}
			break;
		case MOVE4L:
			if(Move3L.isDone()) {
				shooArm.pressY();
				shooArm.pressLeftTrigger();
				Move4L.run();
				runIt = run.MOVE5L;
			}
			break;
		case MOVE5L:
			if(Move4L.isDone()) {
				Move5L.run();
				runIt = run.FINISHL;
			}
			break;
		case FINISHL:
			if(Move5L.isDone()) {
				runIt = run.SCALEFIRE;
			}
			break;
			
			
			
			
			
		case MOVE1R:
			Move1R.run();
			runIt = run.MOVE2R;
			break;
		case MOVE2R:
			if(Move1R.isDone()) {
				Move2R.run();
				runIt = run.MOVE3R;
			}
			break;
		case MOVE3R:
			if(Move2R.isDone()) {
				Move3R.run();
				runIt = run.MOVE4R;
			}
			break;
		case MOVE4R:
			if(Move3R.isDone()) {
				shooArm.pressY();
				shooArm.pressLeftTrigger();
				Move4R.run();
				runIt = run.MOVE5R;
			}
			break;
		case MOVE5R:
			if(Move4R.isDone()) {
				Move5R.run();
				runIt = run.FINISHR;
			}
			break;
		case FINISHR:
			if(Move5R.isDone()) {
				runIt = run.SCALEFIRE;
			}
			break;	
			

			
			
			
			
		case SCALEFIRE:
			shooArm.autoFire();
			endTime = currentTime + revTime;
			runIt = run.SCALEREVDOWN;
			break;
		case SCALEREVDOWN:
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
