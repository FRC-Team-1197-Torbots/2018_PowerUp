package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class LinearTrajectory {
	private DriveHardware drive;
	private double thisdistance;
	private double currentDistance;
	private boolean isFinished = false;
	private final double tkP = 10;//PID for translation
	private final double tkD = 0.01;
	private final double tkI = 0.03;//.0003
	private final double rkP = 5;//PD For rotation
	private final double rkD = .05;//.05
	private final double rkI = 0.01;
	private final double kF = 0.005;
	private final int lor = 1;
	
	//tolerances
	private final double positionTolerance = 0.015;//units: meters
	private final double velocityTolerance = 0.015;//units: meters per second
	private final double headingTolerance = 1 * (Math.PI / 180.0);//units: radians
	
	private double currentVelocity;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI = 0;
	
	private double vP;//velocity proportional
	private double vD;//velocity derivative
	private double vI = 0;
	
	private double omega;
	private double velocity;
	
	private double firstAngle;
	private double currentAngle;
	private double angleError;
	private double error;
	private double startDistance;
	private double lastTime;
	private double currentTime;
	private TorBantorShooarm shooArm;
	private TorDerivative derivative;
	private TorDerivative angleDerivative;
	
	private double timeOutTime;
	
	
	public static enum run {
		IDLE, GO;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public LinearTrajectory(DriveHardware drive, double distance, TorBantorShooarm shooArm, double timeOutTime) {
		this.drive = drive;
		this.thisdistance = distance;
		this.shooArm = shooArm;
		this.timeOutTime = timeOutTime;
		derivative = new TorDerivative(kF);
		angleDerivative = new TorDerivative(kF);
	}
	
	public boolean isDone() {
		return isFinished;
	}
	public void init() {
		isFinished = false;
		runIt = run.GO;
		startDistance = drive.getPosition();
		firstAngle = drive.getHeading();
		currentAngle = drive.getHeading();
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
		angleDerivative.resetValue(angleError);
		derivative.resetValue(drive.getPosition());
		lastTime = Timer.getFPGATimestamp();
	}
		
	
	public void run() {
		shooArm.TorBantorArmAndShooterUpdate();	
		currentAngle = drive.getHeading();
		//we can't fix current angle right now so that it can't be 359 degrees since we need it in this raw value first for the angleError
		//since it is never used other than for finding angleError, there is no need to make sure that it reads -1 degrees rather than 359 degrees
		currentDistance = drive.getPosition();
		currentTime = Timer.getFPGATimestamp();
		switch(runIt) {
		case IDLE:
			break;
		case GO:
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
			
			//since this distance is always positive, we have to multiply by fob for if it is negative
			error = (thisdistance) - (currentDistance - startDistance);//error always positive if approaching
			vI += error;
			if(Math.abs(error) <= positionTolerance) {
				vI = 0;
			}
			if(vI > (0.7 / (tkI * kF))) {
				vI = (0.7 / (tkI * kF));
			}
			if(vI < -(0.7 / (tkI * kF))) {
				vI = -(0.7 / (tkI * kF));
			}
			vP = error * tkP;
			if(vP > 0.7) {
				vP = 0.7;
			}
			if(vP < -0.7) {
				vP = -0.7;
			}
			currentVelocity = derivative.estimate(drive.getPosition());//almost always positive
			//has to be multiplied by -1 so that if it is approaching the target to fast
			//it does not act as a positive. Because, if it was approaching fast, the
			//derivative would be positive
			vD = (currentVelocity) * tkD * (180 / Math.PI);//degrees per second
			velocity = vP + vD + (vI * tkI * kF);
			//velocity is good
			omegaP = angleError * rkP;
			omegaI += angleError;
			if(Math.abs(angleError) < headingTolerance) {
				omegaI = 0;
			}
			if(omegaI > ((0.5) / (rkI * kF))) {
				omegaI = ((0.5) / (rkI * kF));
			}
			if(omegaI < -((0.5) / (rkI * kF))) {
				omegaI = -((0.5) / (rkI * kF));
			}
			
			omegaD = (angleDerivative.estimate(angleError)) * rkD;
			omega = omegaP + omegaD + (omegaI * rkI * kF);
			omega *= lor;
			
			drive.setMotorSpeeds(velocity + omega, velocity - omega);//right, left	
				if((Math.abs(error) <= positionTolerance
						&& Math.abs(angleError) <= headingTolerance
						&& Math.abs(currentVelocity) < velocityTolerance)
						|| (currentTime - lastTime > timeOutTime))
				{
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
