package org.usfirst.frc.team1197.robot;

public class CenterRightRun {
	private TorDrive drive;
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	public CenterRightRun(TorDrive drive) {
		this.drive = drive;
		Move1 = new CenterRight1();
		Move2 = new CenterRight2();
		Move3 = new CenterRight3();
	}
	public void run() {
		drive.executeTrajectory(Move1);
		drive.executeTrajectory(Move2);
		drive.executeTrajectory(Move3);
	}
}
