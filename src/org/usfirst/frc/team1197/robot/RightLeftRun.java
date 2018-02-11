package org.usfirst.frc.team1197.robot;

public class RightLeftRun {
	private TorDrive drive;
	private TorTrajectory Move1;
	private TorTrajectory Move2;
	private TorTrajectory Move3;
	public RightLeftRun(TorDrive drive) {
		Move1 = new RightLeft1();
		Move2 = new RightLeft2();
		Move3 = new RightLeft3();
		this.drive = drive;
	}
	
	public void run() {
		drive.executeTrajectory(Move1);
		//do the switch
		drive.executeTrajectory(Move2);
		drive.executeTrajectory(Move3);
	}
}
