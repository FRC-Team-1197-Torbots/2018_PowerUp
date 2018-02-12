package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.robot.LeftLeftRun.run;
import org.usfirst.frc.team1197.trajectories.CenterLeft1;
import org.usfirst.frc.team1197.trajectories.CenterLeft2;
import org.usfirst.frc.team1197.trajectories.CenterLeft3;

public class CenterLeftRun {
private TorDrive drive;
private TorTrajectory Move1;
private TorTrajectory Move2;
private TorTrajectory Move3;
private TorBantorShooarm shooArm;

public static enum run {
	IDLE, MOVE1, FIRE, MOVE2, MOVE3;
	private run() {}
}

public run runIt = run.IDLE;


	public CenterLeftRun(TorDrive drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		Move1 = new CenterLeft1();
		Move2 = new CenterLeft2();
		Move3 = new CenterLeft3();
		this.shooArm = shooArm;
	}
	
	public void update() {
		switch(runIt) {
		case IDLE:
			break;
		case MOVE1:
			drive.executeTrajectory(Move1);
			runIt = run.FIRE;
			break;
		case FIRE:
			if(Move1.isComplete) {
				shooArm.autoFire();
				drive.executeTrajectory(Move2);
				runIt = run.MOVE2;
			}
			break;
		case MOVE2:
			if(Move2.isComplete) {
				drive.executeTrajectory(Move3);
				runIt = run.MOVE3;
			}
			break;
		case MOVE3:
			if(Move3.isComplete) {
				runIt = run.IDLE;
			}
			break;
		}
	}
	
	
	public void run() {
		runIt = run.MOVE1;
	}
}
