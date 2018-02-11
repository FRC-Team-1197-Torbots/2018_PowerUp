package org.usfirst.frc.team1197.robot;

public class LeftLeftRun {
	private TorDrive drive;
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	public LeftLeftRun(TorDrive drive) {
		Move1 = new LeftLeft1();
		Move2 = new LeftLeft2();
		Move3 = new LeftLeft3();
		this.drive = drive;
	}
	public void run() {
		drive.executeTrajectory(Move1);
		//do the switch
		drive.executeTrajectory(Move2);
		drive.executeTrajectory(Move3);
	}
}
