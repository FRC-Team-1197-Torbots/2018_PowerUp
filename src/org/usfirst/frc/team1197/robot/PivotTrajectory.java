package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class PivotTrajectory {
	private DriveHardware drive;
	private double currentAngle;
	private double speed;
	private boolean isFinished = false;
	private double thisAngle;
	private double lasttime;
	private double currentTime;
	private final double kF = 0.005;
	
	private double startAngle;
	
	private final double rkP = 5;//PD For rotation
	private final double rkD = 0.05;//0.000005
	private final double rkI = 0.01;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI;
	private final double lor = -1;
	
	private double angleError;
	
	private double currentVelocity;
	
	private double timeOutTime;
	
	private TorBantorShooarm shooArm;
	
	private TorDerivative derivative;
	
	public static enum run {
		IDLE, GO;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public PivotTrajectory(DriveHardware drive, double angle, TorBantorShooarm shooArm, double timeOutTime) {
		this.drive = drive;
		this.thisAngle = angle;
		thisAngle *= (Math.PI / 180.0);//degrees to radians
		this.shooArm = shooArm;
		this.timeOutTime = timeOutTime;
		derivative = new TorDerivative(kF);
	}

	public boolean isDone() {
		return isFinished;
	}
	
	public void init() {
		isFinished = false;
		runIt = run.GO;
		startAngle = drive.getHeading();
		derivative.resetValue(drive.getHeading());
		lasttime = Timer.getFPGATimestamp();
	}
	
	public void run() {			
		shooArm.TorBantorArmAndShooterUpdate();
		currentAngle = drive.getHeading();
		currentTime = Timer.getFPGATimestamp();
		switch(runIt) {
		case IDLE:
			break;
		case GO:
			angleError = (thisAngle) - ((currentAngle - startAngle));
			omegaI += angleError;
			omegaP = angleError * rkP;
			if(Math.abs(angleError) < 0.5) {
				omegaI = 0;
			}
			if(omegaI > (0.7 / (rkI * kF))) {
				omegaI = (0.7 / (rkI * kF));
			}
			if(omegaI < -(0.7 / (rkI * kF))) {
				omegaI = -(0.7 / (rkI * kF));
			}
			if(omegaP > 0.7) {
				omegaP = 0.7;
			}
			if(omegaD < -0.7) {
				omegaD = -0.7;
			}
			currentVelocity = derivative.estimate(drive.getHeading());//radians per second
			omegaD = (currentVelocity * rkD);
			currentVelocity *= (180 / Math.PI);//degrees per second
			
			speed = omegaP + omegaD + (omegaI * rkI * kF);
			speed *= lor;
			
			drive.setMotorSpeeds(speed, -speed);
				
			if((Math.abs(angleError) <= (1 * (Math.PI / 180.0))
					&& Math.abs(currentVelocity) < 0.125) || 
					(currentTime - lasttime > 2)) {
				drive.setMotorSpeeds(0, 0);
				isFinished = true;
				runIt = run.IDLE;
			}
			break;
		}
	}
}
