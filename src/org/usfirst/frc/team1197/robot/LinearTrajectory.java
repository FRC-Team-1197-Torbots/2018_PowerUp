package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class LinearTrajectory {
	private DriveHardware drive;
	private double thisdistance;
	private int fob = 1;
	private double currentDistance;
	private boolean isFinished = false;
	private final double tkP = 0.2;//PID for translation
	private final double tkD = 0.02;
	private final double tkI = 0.002;
	private final double rkP = 0.1;//PD For rotation
	private final double rkD = 0.01;
	private final double kF = 0.005;
	private final int lor = 1;
	private double currentVelocity;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	
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
	
	
	public static enum run {
		IDLE, GO;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public LinearTrajectory(DriveHardware drive, double distance, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.thisdistance = distance;
		if(thisdistance < 0) {
			fob = -1;
			thisdistance *= -1;
		}
		else {
			fob = 1;
		}
		this.shooArm = shooArm;
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
		angleDerivative.resetValue(drive.getHeading());
		derivative.resetValue(drive.getPosition());
		lastTime = Timer.getFPGATimestamp();
	}
		
	
	public void run() {
		shooArm.TorBantorArmAndShooterUpdate();
		currentAngle = drive.getHeading();
		currentDistance = drive.getPosition();
		currentTime = Timer.getFPGATimestamp();
		switch(runIt) {
		case IDLE:
			break;
		case GO:
			angleError = currentAngle - firstAngle;
			//since this distance is always positive, we have to multiply by fob for if it is negative
			error = (thisdistance * fob) - (currentDistance - startDistance);//error always positive if approaching
			vI += error;
			if(Math.abs(error) <= 0.0015) {//1.5 cm
				vI = 0;
			}
			if(vI > (0.7 / (tkI * kF))) {
				vI = (0.7 / (tkI * kF));
			}
			if(vI < -(0.7 / (tkI * kF))) {
				vI = -(0.7 / (tkI * kF));
			}
			vP = error * tkP * fob;
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
			vD = (currentVelocity) * tkD * -1 * (180 / Math.PI);//degrees per second
			velocity = vP + vD + (vI * tkI * kF);
			//velocity is good
			omegaP = angleError * rkP;
			omegaD = (angleDerivative.estimate(drive.getHeading())) * (rkD) * -1;
			omega = omegaP + omegaD;
			omega *= lor;
			
			drive.setMotorSpeeds(velocity + omega, velocity - omega);//right, left	
				if((Math.abs(error) <= 0.0015//1.5 cm
						&& Math.abs(angleError) <= 0.5 * (Math.PI / 180.0)//0.5 degrees
						&& Math.abs(currentVelocity) < .0015)//1.5 cm per second
			//time out
						|| (currentTime - lastTime > 2)) {
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
