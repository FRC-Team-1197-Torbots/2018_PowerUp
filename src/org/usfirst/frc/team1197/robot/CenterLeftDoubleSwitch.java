package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class CenterLeftDoubleSwitch {
	//boolean for whether it just stops after the first cube in the switch
	private final boolean oneSwitchDone = true;
	
	
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
	private PivotTrajectory Move4;
	private PivotTrajectory Move5;
	private LinearTrajectory Move6;
	private PivotTrajectory Move7;
	//goes forward to intake and goes back for the same time with a PID here
	private PivotTrajectory Move8;
	private LinearTrajectory Move9;
	private PivotTrajectory Move10;
	private double starttime;
	private double currentTime;
	private double forwardTime;
	private double lastTime;
	private final double rkP = 10;//PD For rotation
	private final double rkD = .05;//.05
	private final double rkI = 0.002;
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
		IDLE, START, MOVE1, MOVE2, MOVE3, MOVE4, MOVE5, 
		MOVE6, MOVE7, GOFORWARD, GOBACK, MOVE8, MOVE9, MOVE10, RELEASE;
		private runIt() {}
	}
	private runIt run1 = runIt.START;
	//can just start it since that it won't run
	//until the void run is called
	
	public CenterLeftDoubleSwitch(DriveHardware drive, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.shooArm = shooArm;
		Move1 = new LinearTrajectory(drive, 0.30470, shooArm, 3);
		Move2 = new PivotTrajectory(drive, -30.15250, shooArm, 3);
		Move3 = new LinearTrajectory(drive, 2.44113, shooArm, 3);
		Move4 = new PivotTrajectory(drive, 21.02266, shooArm, 3);
		Move5 = new PivotTrajectory(drive, -21.02266, shooArm, 3);
		Move6 = new LinearTrajectory(drive, -2.18829, shooArm, 3);
		Move7 = new PivotTrajectory(drive, 30.1520, shooArm, 3);
		//goes forward to intake and goes back for the same time with a PID here
		Move8 = new PivotTrajectory(drive, -30.1520, shooArm, 3);
		Move9 = new LinearTrajectory(drive, 2.18829, shooArm, 3);
		Move10 = new PivotTrajectory(drive, 21.02266, shooArm, 3);
		angleDerivative = new TorDerivative(kF);
	}
	public void run() {
		currentAngle = drive.getHeading();
		currentTime = Timer.getFPGATimestamp();
		shooArm.TorBantorArmAndShooterUpdate();
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
				run1 = runIt .MOVE2;
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
				shooArm.pressLeftTrigger();
				Move4.init();
				Move4.run();
				run1 = runIt.MOVE2;
			}
			break;
		case MOVE4:
			Move4.run();
			if(Move4.isDone()) {
				shooArm.autoFire();
				Timer.delay(0.4);
				//I KNOW I AM NOT SUPPOSED TO DO THIS
				//but, 0.2 seconds of not updating the arm
				//and it will not mess up the timing of everything else
				Move5.init();
				Move5.run();
				run1 = runIt.MOVE5;
			}
			break;
		case MOVE5:
			Move5.run();
			if(Move5.isDone()) {
				shooArm.releaseLeftTrigger();
				if(oneSwitchDone) {
					run1 = runIt.IDLE;
				} else {
					Move6.init();
					Move6.run();
					run1 = runIt.MOVE6;
				}
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
			if(shooArm.inHold()) {
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
				Move8.init();
				Move8.run();
				run1 = runIt.MOVE8;
			}
			break;
		case MOVE8:
			Move8.run();
			if(Move8.isDone()) {
				shooArm.pressX();
				Move9.init();
				Move9.run();
				run1 = runIt.MOVE9;
			}
			break;
		case MOVE9:
			Move9.run();
			if(Move9.isDone()) {
				shooArm.pressLeftTrigger();
				Move10.init();
				Move10.run();
				run1 = runIt.MOVE10;
			}
			break;
		case MOVE10:
			Move10.run();
			if(Move10.isDone()) {
				shooArm.autoFire();
				lastTime = currentTime;
				run1 = runIt.RELEASE;
			}
			break;
		case RELEASE:
			if(currentTime > lastTime + 0.5) {
				shooArm.releaseLeftTrigger();
			}
			break;
		}
	}
}
