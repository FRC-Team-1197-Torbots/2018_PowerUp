package org.usfirst.frc.team1197.robot.auto;

import org.usfirst.frc.team1197.robot.DriveHardware;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorBantorShooarm;
import org.usfirst.frc.team1197.robot.TorDerivative;

import edu.wpi.first.wpilibj.Timer;

public class CenterLeftDoubleSwitch {
	//boolean for whether it just stops after the first cube in the switch
	private final boolean oneSwitchDone = false;
	private boolean scaleSideLeft;
	
	
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private PivotTrajectory Move3;
	//goes forward to intake and goes back for the same time with a PID here
	private LinearTrajectory Move4;
	private PivotTrajectory Move5;
	private PivotTrajectory Move6;
	private PivotTrajectory Move7L;
	private PivotTrajectory Move7R;
	private LinearTrajectory Move8;
	private final double rkP = 0.05;//PD For rotation 5
	private final double rkD = 0;//.05
	private final double rkI = 0;//.01
	private final double kF = 0.005;

	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI = 0;

	private double firstAngle;
	private double currentAngle;
	private double angleError;

	private double omega;
	
	private TorDerivative angleDerivative;
	
	public static enum runIt {
		IDLE, START, MOVE1, MOVE2, MOVE3, GOFORWARD,
		MOVE4, MOVE5, MOVE6, MOVE7L, MOVE7R, MOVE8;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	
	public CenterLeftDoubleSwitch(DriveHardware drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 0.6, shooArm, 1.0);
		Move2 = new PivotTrajectory(drive, -30, shooArm, 3);
		Move3 = new PivotTrajectory(drive, 20, shooArm, 3);
		//goes forward to intake and goes back for the same time with a PID here
		Move4 = new LinearTrajectory(drive, -0.6, shooArm, 1);
		Move5 = new PivotTrajectory(drive, -23, shooArm, 1.5);
		Move6 = new PivotTrajectory(drive, 25, shooArm, 1.5);
		Move7L = new PivotTrajectory(drive, -60, shooArm, 1.0);
		Move7R = new PivotTrajectory(drive, 55, shooArm, 1.0);
		//drives forward, crosses auto line, and is now close to the scale
		Move8 = new LinearTrajectory(drive, 2.0, shooArm, 1.5);
		angleDerivative = new TorDerivative(kF);
	}
	
	public void setScaleSide(boolean isLeft) {
		scaleSideLeft = isLeft;
	}
	
	public void run() {
		currentAngle = drive.getHeading();
		shooArm.TorBantorArmAndShooterUpdate();
		switch(run1) {
		case IDLE:
			break;
		case START:
			shooArm.pressXStart();
			shooArm.pressLeftTriggerControl(1.0);
			Move1.init();
			Move1.run();
			run1 = runIt.MOVE1;
			break;
		case MOVE1:
			Move1.run();
			shooArm.pressLeftTriggerControl(1.0);
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
				Move3.init();
				Move3.run();
				shooArm.pressLeftTriggerControl(1.0);
				run1 = runIt.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			shooArm.pressLeftTriggerControl(1.0);
			if(Move3.isDone()) {
				shooArm.pressLeftTriggerControl(1.0);
				if(oneSwitchDone) {
					run1 = runIt.IDLE;
				} else {
					shooArm.pressA();
					firstAngle = drive.getHeading();
					Timer.delay(1);
					drive.setMotorSpeeds(0.35, 0.35);
					angleDerivative.resetValue(drive.getHeading());
					run1 = runIt.GOFORWARD;
				}
			}
			break;
		case GOFORWARD:
			angleError = currentAngle - firstAngle;
			//is in radians so we have to make sure that it goes from -pi to pi and does not have 
			//an absolute value greater than pi in order to be an efficient control system
			if(angleError > Math.PI) {
				angleError -= (2 * Math.PI);
			} else {
				if(angleError < -Math.PI) {
					angleError += (2 * Math.PI);
				}
			}
			
			omegaP = angleError * rkP;
			omegaD = (angleDerivative.estimate(drive.getHeading())) * rkD;
			omegaI += angleError;
			
			omega = omegaP + omegaD + (omegaI * kF * rkI);
			
			drive.setMotorSpeeds(0.35 + omega, 0.35 - omega);
			if(shooArm.isInside()) {
				drive.setMotorSpeeds(0, 0);
				omegaI = 0;
				Move4.init();
				Move4.run();
				shooArm.pressX();
				run1 = runIt.MOVE4;
			}
			break;
		case MOVE4:
			Move4.run();
			if(Move4.isDone()) {
				Move5.init();
				Move5.run();
				run1 = runIt.MOVE5;
			}
			break;
		case MOVE5:
			Move5.run();
			shooArm.pressLeftTriggerControl(0.7);
			if(Move5.isDone()) {
				shooArm.autoFire();
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.7);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.7);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.7);
				Timer.delay(0.5);
				Move6.init();
				Move6.run();
				run1 = runIt.MOVE6;
			}
			break;
		case MOVE6:
			Move6.run();
			if(Move6.isDone()) {
				if(scaleSideLeft) {
					Move7L.init();
					Move7L.run();
					run1 = runIt.MOVE7L;
				} else {
					Move7R.init();
					Move7R.run();
					run1 = runIt.MOVE7R;
				}
			}
			break;
		case MOVE7L:
			Move7L.run();
			if(Move7L.isDone()) {
				Move8.init();
				Move8.run();
				run1 = runIt.MOVE8;
			}
			break;
		case MOVE7R:
			Move7R.run();
			if(Move7R.isDone()) {
				Move8.init();
				Move8.run();
				run1 = runIt.MOVE8;
			}
			break;
		case MOVE8:
			Move8.run();
			if(Move8.isDone()) {
				run1 = runIt.IDLE;
			}
			break;
		}
	}
}
