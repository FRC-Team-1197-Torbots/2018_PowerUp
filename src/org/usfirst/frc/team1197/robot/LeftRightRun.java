package org.usfirst.frc.team1197.robot;

public class LeftRightRun {
	private TorDrive drive;
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	public LeftRightRun(TorDrive drive) {
		Move1 = new LeftRight1();
		Move2 = new LeftRight2();
		Move3 = new LeftRight3();
		this.drive = drive;
	}
	
	public void run() {
		drive.executeTrajectory(Move1);
		//do the switch
		drive.executeTrajectory(Move2);
		drive.executeTrajectory(Move3);
	}
}
