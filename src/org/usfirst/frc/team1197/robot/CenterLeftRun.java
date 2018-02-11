package org.usfirst.frc.team1197.robot;

public class CenterLeftRun {
private TorDrive drive;
private TorTrajectory Move1;
private TorTrajectory Move2;
private TorTrajectory Move3;
	public CenterLeftRun(TorDrive drive) {
		this.drive = drive;
		Move1 = new CenterLeft1();
		Move2 = new CenterLeft2();
		Move3 = new CenterLeft3();
	}
	public void run() {
		drive.executeTrajectory(Move1);
		//to Switch
		drive.executeTrajectory(Move2);
		drive.executeTrajectory(Move3);
		
	}
}
