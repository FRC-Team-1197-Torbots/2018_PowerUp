package org.usfirst.frc.team1197.robot;

public class CenterRightDoubleSwitch {
	private DriveHardware drive;
	private TorBantorShooarm shooArm;
	private LinearTrajectory Move1;
	private PivotTrajectory Move2;
	private LinearTrajectory Move3;
//	private PivotTrajectory Move4;
//	private PivotTrajectory Move5;
	private LinearTrajectory Move6;
	private PivotTrajectory Move7;
	private LinearTrajectory Move8;
	private PivotTrajectory Move9;
	private final int lor = 1;
	
	private PivotTrajectory Move10;
	private LinearTrajectory Move11;
	private PivotTrajectory Move12;
	
	private PivotTrajectory Move13;
	private LinearTrajectory Move14;
	private PivotTrajectory Move15;
	
	private long starttime;
	private long currentTime;
	private long forwardTime;
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
	
	private boolean isScaleLeft = false;
	
	private TorDerivative angleDerivative;
	
	//forward .30470
	//turn right 26.62816
	//forward 2.37121
	//turn left 21.55395
	//turn right 21.55395
	//2.37121 back
	//turn left 35.71675
	//forward .80399
	//turn right 9.08859
	//forward till it reaches cube
	//backward till it reaches cube

	//scale left
	//turn left 56.35392
	//forward 3.54131
	//turn right 56.35392
	
	//scale right
	//turn right 56.35392
	//forward 3.54131
	//turn left 56.35392
	
	public static enum runIt {
		IDLE, START, MOVE1, MOVE2, MOVE3, MOVE4, 
		MOVE5, MOVE6, MOVE7, MOVE8, MOVE9, GOFORWARD,
		GOBACK, MOVE10, MOVE11, MOVE12, MOVE13, MOVE14, MOVE15;
		private runIt() {}
	}
	
	private runIt run1 = runIt.START;
	
	public void isScaleLeft(boolean scaleLeft) {
		isScaleLeft = scaleLeft;
	}
	
	public CenterRightDoubleSwitch(DriveHardware drive, TorBantorShooarm shooArm) {
		Move1 = new LinearTrajectory(drive, .6, shooArm);
		Move2 = new PivotTrajectory(drive, 26.62816, shooArm);
		Move3 = new LinearTrajectory(drive, 2.57121, shooArm);
//		Move4 = new PivotTrajectory(drive, -21.55395, shooArm);
//		Move5 = new PivotTrajectory(drive, 21.55395, shooArm);
		Move6 = new LinearTrajectory(drive, -2.57121, shooArm);
		Move7 = new PivotTrajectory(drive, -35.71675, shooArm);
		Move8 = new LinearTrajectory(drive, .80399, shooArm);
		Move9 = new PivotTrajectory(drive, 9.08859, shooArm);
		
		//scale Left
		Move10 = new PivotTrajectory(drive, -56.35392, shooArm);
		Move11 = new LinearTrajectory(drive, 3.54131, shooArm);
		Move12 = new PivotTrajectory(drive, 56.35392, shooArm);
		//scale Right
		Move13 = new PivotTrajectory(drive, 56.35392, shooArm);
		Move14 = new LinearTrajectory(drive, 3.54131, shooArm);
		Move15 = new PivotTrajectory(drive, -56.35392, shooArm);
		
		this.drive = drive;
		this.shooArm = shooArm;
	}
	
	public void run() {
		currentAngle = drive.getHeading();
		currentTime = System.currentTimeMillis();
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
				run1 = runIt.MOVE2;
			}
			break;
		case MOVE2:
			Move2.run();
			if(Move2.isDone()) {
				shooArm.pressLeftTrigger();
				Move3.init();
				Move3.run();
				run1 = runIt.MOVE3;
			}
			break;
		case MOVE3:
			Move3.run();
			if(Move3.isDone()) {
				shooArm.autoFire();
				Move6.init();
				Move6.run();
				run1 = runIt.MOVE6;
			}
			break;
		case MOVE4:
			break;
		case MOVE5:
			break;
//		case MOVE4:
//			Move4.run();
//			if(Move4.isDone()) {
//				//I KNOW I AM NOT SUPPOSED TO DO THIS
//				//but, 0.2 seconds of not updating the arm
//				//and it will not mess up the timing of everything else
//				Move5.init();
//				Move5.run();
//				run1 = runIt.MOVE5;
//			}
//			break;
//		case MOVE5:
//			Move5.run();
//			if(Move5.isDone()) {
//				shooArm.releaseLeftTrigger();
//				Move6.init();
//				Move6.run();
//				run1 = runIt.MOVE6;
//			}
//			break;
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
				Move8.init();
				Move8.run();
				run1 = runIt.MOVE8;
			}
		case MOVE8:
			Move8.run();
			if(Move8.isDone()) {
				Move9.init();
				Move9.run();
				run1 = runIt.MOVE9;
			}
			break;
		case MOVE9:
			Move9.run();
			if(Move9.isDone()) {
				shooArm.pressA();
//				firstAngle = drive.getHeading();
//				starttime = currentTime;
//				drive.setMotorSpeeds(0.4, 0.4);
//				angleDerivative.resetValue(drive.getHeading());
//				run1 = runIt.GOFORWARD;
				run1 = runIt.IDLE;
			}
			break;
		case GOFORWARD:
			angleError = currentAngle - firstAngle;
			
			omegaP = angleError * rkP;
			omegaD = (angleDerivative.estimate(drive.getHeading())) * rkD;
			omegaI += angleError;
			
			omega = omegaP + omegaD + (omegaI * kF * rkI);
			omega *= lor;
			
			drive.setMotorSpeeds(0.4 + omega, 0.4 - omega);
			if(shooArm.inHold()) {
				drive.setMotorSpeeds(0, 0);
				omegaI = 0;
				forwardTime = currentTime - starttime;
				run1 = runIt.GOBACK;
			}
			break;
		case GOBACK:
			angleError = currentAngle - firstAngle;
			
			omegaP = angleError * rkP;
			omegaD = (angleDerivative.estimate(drive.getHeading())) * rkD;
			omegaI += angleError;
			
			omega = omegaP + omegaD + (omegaI * kF * rkI);
			omega *= lor;
			
			drive.setMotorSpeeds(-0.4 + omega, -0.4 - omega);
			
			if(currentTime > (starttime + (2 * forwardTime))) {
				drive.setMotorSpeeds(0, 0);
				if(isScaleLeft) {
					Move10.init();
					Move10.run();
					run1 = runIt.MOVE10;
				} else {
					Move13.init();
					Move13.run();
					run1 = runIt.MOVE13;
				}
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
				Move12.init();
				Move12.run();
				run1 = runIt.MOVE12;
			}
			break;
		case MOVE12:
			Move12.run();
			if(Move12.isDone()) {
				run1 = runIt.IDLE;
			}
			break;
		case MOVE13:
			Move13.run();
			if(Move13.isDone()) {
				Move14.init();
				Move14.run();
				run1 = runIt.MOVE14;
			}
			break;
		case MOVE14:
			Move14.run();
			if(Move14.isDone()) {
				Move15.init();
				Move15.run();
				run1 = runIt.MOVE15;
			}
			break;
		case MOVE15:
			Move15.run();
			if(Move15.isDone()) {
				run1 = runIt.IDLE;
			}
			break;
		}
	}
}
