package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class LinearTrajectory {
	private DriveHardware drive;
	private double thisdistance;
	private int fob = 1;
	private double currentDistance;
	private boolean isFinished = false;
	private final double tkP = 2000;//PID for translation
	private final double tkD = 50000;
	private final double tkI = 50;
	private final double rkP = 5000;//PD For rotation
	private final double rkD = 100;
	private final double kF = 0.005;
	private final int lor = 1;
	
	private final double halfTrackWidth = .352425;//in meters
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
	private long currentTime;
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
	
	public void run(double starttime) {
		isFinished = false;
		runIt = run.GO;
		startDistance = drive.getPosition();
		firstAngle = drive.getHeading();
		angleDerivative.resetValue(drive.getHeading());
		derivative.resetValue(drive.getPosition());
		lastTime = Timer.getFPGATimestamp();
		while(!isFinished) {
			if(Timer.getFPGATimestamp() - starttime > 14) {
				drive.setMotorSpeeds(0, 0);
				isFinished = true;
				break;
			}
			
			shooArm.TorBantorArmAndShooterUpdate();
			currentAngle = drive.getHeading();
			currentDistance = drive.getPosition();
			currentTime = System.currentTimeMillis();
			switch(runIt) {
			case IDLE:
				break;
			case GO:
				angleError = currentAngle - firstAngle;
				error = ((currentDistance - startDistance) * fob) - thisdistance;
				
				vI += error;
				if(Math.abs(error) <= 0.005) {
					vI = 0;
				}
				vP = error  * tkP;
				currentVelocity = derivative.estimate(drive.getPosition());
				vD = (currentVelocity) * tkD;
				velocity = vP + vD + (vI * tkI * kF);
				velocity *= -1;
				velocity *= fob;
				
				omegaP = angleError * rkP;
				omegaD = (angleDerivative.estimate(drive.getHeading())) * (rkD);
				omega = omegaP + omegaD;
				
				omega *= lor;
				omega *= halfTrackWidth;
				
				drive.setVelocity(velocity - omega, velocity + omega);
				
				if(((Math.abs(error) <= 0.005 && Math.abs(angleError) <= 0.5 * (Math.PI / 180.0))
						&& currentVelocity < 1) || (currentTime - lastTime > 2)) {
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
}
