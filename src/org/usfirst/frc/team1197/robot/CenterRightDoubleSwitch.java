package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class CenterRightDoubleSwitch {
	//boolean for whether it just stops after the first cube in the switch
	private final boolean oneSwitchDone = true;
	
	
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private LinearTrajectory Move4;
	private PivotTrajectory Move5;
	//goes forward to intake and goes back for the same time with a PID here
	private PivotTrajectory Move6;
	private LinearTrajectory Move7;
	private double starttime;
	private double currentTime;
	private double forwardTime;
	private double lastTime;
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
		IDLE, START, MOVE1, MOVE2, MOVE3, 
		MOVE4, MOVE5, GOFORWARD, GOBACK, MOVE6, MOVE7, RELEASE;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	
	public CenterRightDoubleSwitch(DriveHardware drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 0.6, shooArm, 1.0);//.30470
		Move2 = new PivotTrajectory(drive, 8, shooArm, 3);//26.62816
		Move3 = new LinearTrajectory(drive, 2.7, shooArm, 2.25);//2.37121
		Move4 = new LinearTrajectory(drive, -3.0, shooArm, 3);//-2.37121
		Move5 = new PivotTrajectory(drive, -8, shooArm, 3);//-26.62816
		//goes forward to intake and goes back for the same time with a PID here
		Move6 = new PivotTrajectory(drive, 8, shooArm, 3);//26.62816
		Move7 = new LinearTrajectory(drive, 3.0, shooArm, 3);//2.37121
		angleDerivative = new TorDerivative(kF);
	}
	public void run() {
		currentAngle = drive.getHeading();
		currentTime = Timer.getFPGATimestamp();
		shooArm.TorBantorArmAndShooterUpdate();
		switch(run1) {
		case IDLE:
			shooArm.switchShoot();
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
			shooArm.switchShoot();
			shooArm.pressLeftTrigger();
			if(Move2.isDone()) {
				shooArm.switchShoot();
				shooArm.pressLeftTrigger();
				Move3.init();
				Move3.run();
				shooArm.switchShoot();
				run1 = runIt.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			if(Move3.isDone()) {
				shooArm.switchShoot();
				shooArm.pressLeftTrigger();
				shooArm.autoFire();
				if(oneSwitchDone) {
					run1 = runIt.IDLE;
				} else {
					Move4.init();
					Move4.run();
					run1 = runIt.MOVE4;
				}
			}
			break;
		case MOVE4:
			Move4.run();
			if(Move4.isDone()) {
				shooArm.releaseLeftTrigger();
				shooArm.pressX();
				Move5.init();
				Move5.run();
				run1 = runIt.MOVE5;
			}
			break;
		case MOVE5:
			Move5.run();
			if(Move5.isDone()) {
				shooArm.pressA();
				firstAngle = drive.getHeading();
				starttime = currentTime;
				drive.setMotorSpeeds(0.4, 0.4);
				angleDerivative.resetValue(drive.getHeading());
				run1 = runIt.GOFORWARD;
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
			
			drive.setMotorSpeeds(0.4 + omega, 0.4 - omega);
			if(shooArm.isInside()) {
				drive.setMotorSpeeds(0, 0);
				omegaI = 0;
				forwardTime = currentTime - starttime;
				starttime = currentTime;
				run1 = runIt.GOBACK;
			}
			break;
		case GOBACK:
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
			
			drive.setMotorSpeeds(-0.4 + omega, -0.4 - omega);
			
			if(currentTime > (starttime + (forwardTime))) {
				drive.setMotorSpeeds(0, 0);
				Move6.init();
				Move6.run();
				run1 = runIt.MOVE6;
			}
			break;
		case MOVE6:
			Move6.run();
			if(Move6.isDone()) {
				shooArm.pressX();
				Move7.init();
				Move7.run();
				run1 = runIt.MOVE7;
			}
			break;
		case MOVE7:
			Move7.run();
			if(Move7.isDone()) {
				shooArm.switchShoot();
				shooArm.pressLeftTrigger();
				shooArm.autoFire();
				lastTime = currentTime;
				run1 = runIt.RELEASE;
			}
			break;
		case RELEASE:
			if(currentTime > lastTime + 1.5) {
				shooArm.releaseLeftTrigger();
				run1 = runIt.IDLE;
			}
			break;
		}
	}
}
