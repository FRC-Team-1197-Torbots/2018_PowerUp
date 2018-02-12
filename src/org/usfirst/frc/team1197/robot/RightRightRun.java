package org.usfirst.frc.team1197.robot;

public class RightRightRun {
	private TorDrive drive;
	
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	private TorTrajectory Move4;
	private TorTrajectory Move5;
	private TorTrajectory Move6;
	private TorTrajectory Move7;
	private TorTrajectory Move8;
	private TorTrajectory Move9;
	private TorTrajectory Move10;
	private TorTrajectory Move11;
	private TorTrajectory Move12;
	
	private TorBantorShooarm shooArm;
	private int lor; //left or right
	private int fob; //forwards or backwards
	
	public static enum run {
		IDLE, MOVE1, MOVE2, MOVE3, MOVE4, MOVE5, FIRE, MOVE6, MOVE7, MOVE8, MOVE9, MOVE10, MOVE11, MOVE12;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public RightRightRun(TorDrive drive, TorBantorShooarm shooArm) {
		lor = 1;
		fob = 1;
		this.drive = drive;
		this.shooArm = shooArm;
		
		Move1 = new LinearTrajectory(1.36208 * fob);
		Move2 = new PivotTrajectory(-90 * lor); // assumed negative was counter clockwise (don't know which is negative or positive)
		Move3 = new LinearTrajectory(1.40373 * fob);
		Move4 = new PivotTrajectory(90 * lor); // assumed positive was clockwise
		Move5 = new LinearTrajectory(1.36208 * fob);
		// DO THE SWITCH
		Move6 = new LinearTrajectory(-1.36208 * fob);
		Move7 = new PivotTrajectory(90 * lor);
		Move8 = new LinearTrajectory(1.40373 * fob);
		Move9 = new PivotTrajectory(-90 * lor);
		Move10 = new LinearTrajectory(4.03203 * fob);
		Move11 = new PivotTrajectory(-90 * lor);
		Move12 = new LinearTrajectory(.89165 * fob);	
	}
	
	public void update() {
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
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move2.isComplete) {
				drive.executeTrajectory(Move3);
				runIt = run.MOVE4;
			}
			break;
		case MOVE4:
			if(Move3.isComplete) {
				drive.executeTrajectory(Move4);
				runIt = run.MOVE5;
			}
			break;
		case MOVE5:
			if(Move4.isComplete) {
				drive.executeTrajectory(Move5);
				runIt = run.FIRE;
			}
			break;
		case FIRE:
			if(Move5.isComplete) {
				shooArm.autoFire();
				drive.executeTrajectory(Move6);
				runIt = run.MOVE6;
			}
			break;
		case MOVE6:
			if(Move6.isComplete) {
				drive.executeTrajectory(Move7);
				runIt = run.MOVE7;
			}
			break;
		case MOVE7:
			if(Move7.isComplete) {
				drive.executeTrajectory(Move8);
				runIt = run.MOVE8;
			}
			break;
		case MOVE8:
			if(Move8.isComplete) {
				drive.executeTrajectory(Move9);
				runIt = run.MOVE9;
			}
			break;
		case MOVE9:
			if(Move9.isComplete) {
				drive.executeTrajectory(Move10);
				runIt = run.MOVE10;
			}
			break;
		case MOVE10:
			if(Move10.isComplete) {
				drive.executeTrajectory(Move11);
				runIt = run.MOVE11;
			}
			break;
		case MOVE11:
			if(Move11.isComplete) {
				drive.executeTrajectory(Move12);
				runIt = run.MOVE12;
			}
			break;
		case MOVE12:
			if(Move12.isComplete) {
				runIt = run.IDLE;
			}
			break;
		}
	}
	
	public void run() {
		runIt = run.MOVE1;
	}
}
