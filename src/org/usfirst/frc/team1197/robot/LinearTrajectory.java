package org.usfirst.frc.team1197.robot;

public class LinearTrajectory {
	private DriveHardware drive;
	private double thisdistance;
	private int fob = 1;
	private double lastDistance;
	private double currentDistance;
	private boolean isFinished = false;
	private double accelerateDistance;
	private final double tkP = 2000;//PD for translation
	private final double tkD = 50000;
	private final double rkP = 5000;//PD For rotation
	private final double rkD = 100;
	private final double kF = 0.005;
	private final int lor = 1;
	
	private final double halfTrackWidth = .352425;//in meters
	private double currentVelocity;
	private double dx;
	private double y1;
	private double y2;
	private long lastVelTime;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	
	private double vP;//velocity proportional
	private double vD;//velocity derivative
	
	
	private double omega;
	private double velocity;
	private final double autonomousSpeed = 0.66;
	private final double decelerateDistance = 0.3;
	
	private double firstAngle;
	private double currentAngle;
	private double angleError;
	private double angleLastError;
	private double error;
	private double lastError;
	private double startDistance;
	private double lastTime;
	private long currentTime;
	private TorBantorShooarm shooArm;
	
	public static enum run {
		IDLE, ACCELERATE, DECELERATE;
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
		accelerateDistance = thisdistance - decelerateDistance;
	}
	
	public boolean isDone() {
		return isFinished;
	}
	
	public void run() {
		isFinished = false;
		runIt = run.ACCELERATE;
		firstAngle = drive.getHeading();
		lastDistance = drive.getPosition();
		startDistance = drive.getPosition();
		angleLastError = 0;
		while(!isFinished) {
			shooArm.TorBantorArmAndShooterUpdate();
			currentAngle = drive.getHeading();
			currentDistance = drive.getPosition();
			currentTime = System.currentTimeMillis();
			switch(runIt) {
			case IDLE:
				break;
			case ACCELERATE:
				angleError = currentAngle - firstAngle;
				
				omegaP = angleError * rkP;
				omegaD = (angleError - angleLastError) * (rkD / kF);
				omega = omegaP + omegaD;
				
				omega *= lor;
				omega *= halfTrackWidth;
				omega *= 0.001;
				
				angleLastError = angleError;
				
				drive.setMotorSpeeds((autonomousSpeed * fob) + omega, (autonomousSpeed * fob) - omega);
				if(((currentDistance - lastDistance) * fob) >= accelerateDistance) {
					lastDistance = currentDistance;
					y1 = drive.getPosition();
					lastVelTime = currentTime;
					lastTime = currentTime;
					runIt = run.DECELERATE;
				}
				break;
			case DECELERATE:
				y2 = drive.getPosition();
				dx = currentTime - lastVelTime;
				currentVelocity = (y2 - y1) / dx;
				y1 = y2;
				
				angleError = currentAngle - firstAngle;
				error = ((currentDistance - startDistance) * fob) - thisdistance;
				
				vP = error  * tkP;
				vD = (error - lastError) * tkD;
				velocity = vP + vD;
				velocity *= -1;
				velocity *= fob;
				
				omegaP = angleError * rkP;
				omegaD = (angleError - angleLastError) * (rkD / kF);
				omega = omegaP + omegaD;
				
				omega *= lor;
				omega *= halfTrackWidth;
				
				drive.setVelocity(velocity - omega, velocity + omega);
				
				angleLastError = angleError;
				lastError = error;
				
				if(((Math.abs(error) <= 0.005 //0.2 meters is much more accurate than before
						&& Math.abs(angleError) <= 0.5 * (Math.PI / 180.0))
						&& currentVelocity < 0.005) || (currentTime - lastTime > 250)) {//4 degrees is not too much
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
}
