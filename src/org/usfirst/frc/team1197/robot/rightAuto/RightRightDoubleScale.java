package org.usfirst.frc.team1197.robot.rightAuto;

import org.usfirst.frc.team1197.robot.DriveHardware;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorBantorShooarm;

import edu.wpi.first.wpilibj.Timer;

public class RightRightDoubleScale {
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private PivotTrajectory Move4;
	private LinearTrajectory Move5;
	private LinearTrajectory Move6;
	private PivotTrajectory Move7;
	
	private double currentTime;
	private double endTime;
	
	public static enum runIt {
		IDLE, START, MOVE1, MOVE2, MOVE3, MOVE4, INTAKINGBUG, MOVE5, INTAKE1, MOVE6, MOVE7;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	
	public RightRightDoubleScale(DriveHardware drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		
		Move1 = new LinearTrajectory(drive, 8.0, shooArm, 3.25);
		//goes forward
		Move2 = new PivotTrajectory(drive, -71, shooArm, 1.25);
		//turns left
		Move3 = new LinearTrajectory(drive, -0.3, shooArm, 0.75);
		//backs up then shoots
		Move4 = new PivotTrajectory(drive, -80, shooArm, 1.25);
		//turns left to get in position to intake
		Move5 = new LinearTrajectory(drive, 1.7, shooArm, 1.0);
		//goes forward to intake
		Move6 = new LinearTrajectory(drive, -2.0, shooArm, 1.5);
		//goes backward after intaking
		Move7 = new PivotTrajectory(drive, 95, shooArm, 1.5);
		//turns around then fires
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
			shooArm.pressLeftTriggerControl(0.75);
			if(Move2.isDone()) {
				Move3.init();
				Move3.run();
				shooArm.pressLeftTriggerControl(0.75);
				run1 = runIt.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			shooArm.pressLeftTriggerControl(0.75);
			if(Move3.isDone()) {
				shooArm.autoFire();
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.75);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.75);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.75);
				Timer.delay(0.5);
				shooArm.pressLeftTriggerControl(0.0);
				shooArm.pressY();
				Move4.init();
				Move4.run();
				run1 = runIt.MOVE4;
			}
			break;
		case MOVE4:
			Move4.run();
			if(Move4.isDone()) {
				Move5.init();
				shooArm.pressA();
				endTime = currentTime + 0.05;
				run1 = runIt.INTAKINGBUG;
			}
			break;
		case INTAKINGBUG:
			shooArm.pressA();
			if(currentTime > endTime) {
				run1 = runIt.MOVE5;
			}
			break;
		case MOVE5:
			Move5.run();
			shooArm.pressA();
			if(Move5.isDone()) {
				drive.setMotorSpeeds(0.3, 0.3);
				endTime = currentTime + 2.5;
				run1 = runIt.INTAKE1;
			}
			break;
		case INTAKE1:
			shooArm.pressA();
			if(shooArm.isInside() || currentTime > endTime) {
				drive.setMotorSpeeds(0, 0);
				Move6.init();
				Move6.run();
				run1 = runIt.MOVE6;
			}
			break;
		case MOVE6:
			Move6.run();
			if(Move6.isDone()) {
				shooArm.pressY();
				Move7.init();
				Move7.run();
				endTime = currentTime + 1.6;
				run1 = runIt.MOVE7;
			}
			break;
		case MOVE7:
			Move7.run();
			if(Move7.isDone() && currentTime > endTime) {
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
