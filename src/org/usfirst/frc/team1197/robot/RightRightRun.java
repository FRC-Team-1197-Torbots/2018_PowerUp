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
	private int lor;//left or right
	private int fob;//forwards or backwards
	
	
	public RightRightRun(TorDrive drive) {
		lor = 1;
		fob = 1;
		this.drive = drive;
		
		Move1 = new LinearTrajectory(1.36208 * fob);
		Move2 = new PivotTrajectory(-90 * lor);//assumed negative was counter clockwise (don't know which is negative or positive)
		Move3 = new LinearTrajectory(1.40373 * fob);
		Move4 = new PivotTrajectory(90 * lor);//assumed positive was clockwise
		Move5 = new LinearTrajectory(1.36208 * fob);
		//DO THE SWITCH
		Move6 = new LinearTrajectory(-1.36208 * fob);
		Move7 = new PivotTrajectory(90 * lor);
		Move8 = new LinearTrajectory(1.40373 * fob);
		Move9 = new PivotTrajectory(-90 * lor);
		Move10 = new LinearTrajectory(4.03203 * fob);
		Move11 = new PivotTrajectory(-90 * lor);
		Move12 = new LinearTrajectory(.89165 * fob);
		
	}
	
	public void run() {
		drive.executeTrajectory(Move1);
		drive.executeTrajectory(Move2);
		drive.executeTrajectory(Move3);
		drive.executeTrajectory(Move4);
		drive.executeTrajectory(Move5);
		//do the switch
		drive.executeTrajectory(Move6);
		drive.executeTrajectory(Move7);
		drive.executeTrajectory(Move8);
		drive.executeTrajectory(Move9);
		drive.executeTrajectory(Move10);
		drive.executeTrajectory(Move11);
		drive.executeTrajectory(Move12);
	}
	
}
