package org.usfirst.frc.team1197.robot.auto;

import org.usfirst.frc.team1197.robot.DriveHardware;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorBantorShooarm;

import edu.wpi.first.wpilibj.Timer;

public class RightRightOppositeSwitch {
	private TorBantorShooarm shooArm;
	
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	
	public static enum runIt {
		IDLE, START, MOVE1, MOVE2;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	
	public RightRightOppositeSwitch(DriveHardware drive, TorBantorShooarm shooArm) {
		this.shooArm = shooArm;
		
		Move1 = new LinearTrajectory(drive, 7.9, shooArm, 4.0);
		Move2 = new PivotTrajectory(drive, -30, shooArm, 1.5);
	}
	
	public void run() {
		shooArm.TorBantorArmAndShooterUpdate();
		shooArm.autoSet(true);
		switch(run1) {
		case IDLE:
			break;
		case START:
			shooArm.pressXStart();
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
			shooArm.pressLeftTriggerControl(1.0);
			if(Move2.isDone()) {
				shooArm.autoFire();
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(1.0);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(1.0);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(1.0);
				Timer.delay(0.5);
				shooArm.pressLeftTriggerControl(0.0);
				run1 = runIt.IDLE;
			}
			break;
		}
	}
}
