package org.usfirst.frc.team1197.robot.auto;

import org.usfirst.frc.team1197.robot.DriveHardware;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorBantorShooarm;
import org.usfirst.frc.team1197.robot.TorDerivative;

import edu.wpi.first.wpilibj.Timer;

public class CenterRightDoubleSwitch {
	//boolean for whether it just stops after the first cube in the switch
	private final boolean oneSwitchDone = false;
	
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private PivotTrajectory Move3;
	//goes forward to intake and goes back for the same time with a PID here
	private LinearTrajectory Move4;
	private PivotTrajectory Move5;
	private PivotTrajectory Move6;
	private LinearTrajectory Move7;
	private PivotTrajectory Move8;
	private LinearTrajectory Move9;
	private LinearTrajectory Move10;
	private PivotTrajectory Move11;
	private LinearTrajectory Move12;
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
		MOVE4, MOVE5, MOVE6, MOVE7, MOVE8, MOVE9, MOVE10, MOVE11, MOVE12;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	
	public CenterRightDoubleSwitch(DriveHardware drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 0.6, shooArm, 1.0);
		Move2 = new PivotTrajectory(drive, 20, shooArm, 1.5);
		Move3 = new PivotTrajectory(drive, -29, shooArm, 1.5);
		//goes forward to intake and goes back for the same time with a PID here
		Move4 = new LinearTrajectory(drive, -0.6, shooArm, 1);
		Move5 = new PivotTrajectory(drive, 30, shooArm, 1.5);
		Move6 = new PivotTrajectory(drive, 38, shooArm, 1.5);//positive
		Move7 = new LinearTrajectory(drive, 1.0, shooArm, 1.5);//positive
		Move8 = new PivotTrajectory(drive, -89, shooArm, 2.0);//negative
		//Move 9 is go forwward to intake
		Move9  = new LinearTrajectory(drive, 0.8, shooArm, 1.3);//positive that was 1.2
		Move10 = new LinearTrajectory(drive, -0.6, shooArm, 0.5);//negative
		Move11 = new PivotTrajectory(drive, 42, shooArm, 0.5);//positive
		Move12 = new LinearTrajectory(drive, 1.3, shooArm, 0.6);//positive
		//drives forward, crosses auto line, and is now close to the scale
		angleDerivative = new TorDerivative(kF);
	}
	
	public void run() {
		currentAngle = drive.getHeading();
		shooArm.TorBantorArmAndShooterUpdate();
		shooArm.autoSet(true);
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
				shooArm.pressX();
				run1 = runIt.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			if(Move3.isDone()) {
				if(oneSwitchDone) {
					run1 = runIt.IDLE;
				} else {
					shooArm.pressA();
					firstAngle = drive.getHeading();
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
				shooArm.pressX();
				Move6.init();
				Move6.run();
				run1 = runIt.MOVE6;
			}
			break;
		case MOVE6:
			Move6.run();
			if(Move6.isDone()) {
				Move7.init();
				Move7.run();
				run1 = runIt.MOVE7;
			}
			break;
		case MOVE7:
			Move7.run();
			if(Move7.isDone()) {
				Move8.init();
				Move8.run();
				run1 = runIt.MOVE8;
			}
			break;
		case MOVE8:
			Move8.run();
			if(Move8.isDone()) {
				shooArm.pressA();
				Move9.init();
				Move9.run();
				run1 = runIt.MOVE9;
			}
			break;
		case MOVE9:
			Move9.run();
			if(Move9.isDone() || shooArm.isInside()) {
				Move10.init();
				Move10.run();
				run1 = runIt.MOVE10;
			}
			break;
		case MOVE10:
			Move10.run();
			if(Move10.isDone()) {
				Move11.init();
				Move11.run();
				run1 = runIt.MOVE11;
			}
			break;
		case MOVE11:
			Move11.run();
			if(Move11.isDone()) {
				shooArm.pressLeftTriggerControl(0.27);
				shooArm.pressX();
				Move12.init();
				Move12.run();
				run1 = runIt.MOVE12;
			}
			break;
		case MOVE12:
			Move12.run();
			shooArm.pressLeftTriggerControl(0.27);
			if(Move12.isDone()) {
				shooArm.autoFire();
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.27);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.27);
				shooArm.TorBantorArmAndShooterUpdate();
				shooArm.pressLeftTriggerControl(0.27);
				Timer.delay(0.5);
				run1 = runIt.IDLE;
			}
			break;
		}
	}
}