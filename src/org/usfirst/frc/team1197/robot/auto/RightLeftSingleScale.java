package org.usfirst.frc.team1197.robot.auto;

import org.usfirst.frc.team1197.robot.DriveHardware;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorBantorShooarm;

import edu.wpi.first.wpilibj.Timer;

public class RightLeftSingleScale {
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private PivotTrajectory Move4;
	//fires after move 4
	private PivotTrajectory Move5;
	//turns to intake
	private LinearTrajectory Move6;
	//intaking
	private LinearTrajectory Move7;
	//backs up
	private PivotTrajectory Move8;
	//fires
	
	private double currentTime;
	private double endTime;
	
	public static enum runIt {
		IDLE, START, MOVE1, MOVE2, MOVE3, MOVE4, MOVE5, MOVE6, INTAKE1, MOVE7, MOVE8;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	public RightLeftSingleScale(DriveHardware drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		
		Move1 = new LinearTrajectory(drive, 5.0, shooArm, 2.75);
		Move2 = new PivotTrajectory(drive, 0, shooArm, 1.4);
		Move3 = new LinearTrajectory(drive, 0, shooArm, 2.75);
		Move4 = new PivotTrajectory(drive, 0, shooArm, 1.4);
		Move5 = new PivotTrajectory(drive, 0, shooArm, 1.7);
		Move6 = new LinearTrajectory(drive, 0, shooArm, 1.0);
		Move7 = new LinearTrajectory(drive, 0, shooArm, 1.5);
		Move8 = new PivotTrajectory(drive, 0, shooArm, 1.8);
		
//		Move1 = new LinearTrajectory(drive, 5.0, shooArm, 2.75);
//		Move2 = new PivotTrajectory(drive, -90, shooArm, 1.4);
//		Move3 = new LinearTrajectory(drive, 5.0, shooArm, 2.75);
//		Move4 = new PivotTrajectory(drive, 90, shooArm, 1.4);
//		Move5 = new PivotTrajectory(drive, 150, shooArm, 1.7);
//		Move6 = new LinearTrajectory(drive, 1.7, shooArm, 1.0);
//		Move7 = new LinearTrajectory(drive, -2.0, shooArm, 1.5);
//		Move8 = new PivotTrajectory(drive, -170, shooArm, 1.8);
	}
	
	public void run() {
		currentTime = Timer.getFPGATimestamp();
		shooArm.TorBantorArmAndShooterUpdate();
		shooArm.autoSet(true);
		switch(run1) {
		case IDLE:
			break;
		case START:
			shooArm.pressYStart();
			Move1.init();
			Move1.run();
			run1 = runIt.MOVE1;
			break;
		case MOVE1:
			Move1.run();
			if(Move1.isDone()) {
				Move2.init();
				Move2.run();
				run1 = runIt.MOVE2;
			}
			break;
		case MOVE2:
			Move2.run();
			if(Move2.isDone()) {
				Move3.init();
				Move3.run();
				run1 = runIt.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			if(Move3.isDone()) {
				Move4.init();
				Move4.run();
				shooArm.pressLeftTriggerControl(1.0);
				run1 = runIt.MOVE4;
			}
			break;
		case MOVE4:
			Move4.run();
			if(Move4.isDone()) {
				shooArm.autoFire();
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.75);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.75);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.75);
				Timer.delay(0.3);
				shooArm.pressLeftTriggerControl(0.0);
				shooArm.pressY();
				Move5.init();
				Move5.run();
				run1 = runIt.MOVE5;
			}
			break;
		case MOVE5:
			Move5.run();
			if(Move5.isDone()) {
				shooArm.pressA();
				Move6.init();
				Move6.run();
				run1 = runIt.MOVE6;
			}
			break;
		case MOVE6:
			Move6.run();
			shooArm.pressA();
			if(Move6.isDone() || shooArm.isInside()) {
				drive.setMotorSpeeds(0.3, 0.3);
				endTime = currentTime + 2.25;
				run1 = runIt.INTAKE1;
			}
			break;
		case INTAKE1:
			shooArm.pressA();
			if(shooArm.isInside() || currentTime > endTime) {
				drive.setMotorSpeeds(0, 0);
				Move7.init();
				Move7.run();
				run1 = runIt.MOVE7;
			}
			break;
		case MOVE7:
			Move7.run();
			if(Move7.isDone()) {
				shooArm.pressY();
				Move8.init();
				Move8.run();
				endTime = currentTime + 1.6;
				run1 = runIt.MOVE8;
			}
			break;
		case MOVE8:
			Move8.run();
			if(Move8.isDone() && currentTime > endTime) {
				shooArm.pressLeftTriggerControl(0.7);
				Timer.delay(0.2);
				shooArm.autoFire();
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.7);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.7);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.7);
				Timer.delay(0.2);
				shooArm.pressLeftTriggerControl(0.0);
				shooArm.pressY();
				run1 = runIt.IDLE;
			}
			break;
		}
	}
}
