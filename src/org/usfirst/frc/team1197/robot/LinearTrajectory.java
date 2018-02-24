package org.usfirst.frc.team1197.robot;

public class LinearTrajectory {
	private DriveHardware drive;
	private double thisdistance;
	private int fob = 1;
	private double lastDistance;
	private double currentDistance;
	private double speed;
	private boolean isFinished = false;
	private double accelerateDistance;
	private double coastDistance;
	private long accelerateTime;
	private final double tkP = 0.0;//PD for translation
	private final double tkD = 0.0;
	private final double rkP = 0.0;//PD For rotation
	private final double rkD = 0.0;
	private final int lor = 1;
	
	private final double halfTrackWidth = .352425;//in meters
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	
	private double vP;//velocity proportional
	private double vD;//velocity derivative
	
	
	private double omega;
	private double velocity;
	
	private double firstAngle;
	private double currentAngle;
	private double angleError;
	private double angleLastError;
	private double error;
	private double lastError;
	private double x;
	private double startDistance;
	private long startTime;
	private long relativeTime;
	private long currentTime;
	
	public static enum run {
		IDLE, ACCELERATE, COAST, DECELERATE;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public LinearTrajectory(DriveHardware drive, double distance, 
			double accelerationFraction, long accelerateTime) {
		this.drive = drive;
		this.thisdistance = distance;
		if(thisdistance < 0) {
			fob = -1;
			thisdistance *= -1;
		}
		this.accelerateTime = accelerateTime;
		
		accelerateDistance = thisdistance * accelerationFraction;
		coastDistance = thisdistance * (1 - (2 * accelerationFraction));
	}
	
	public boolean isDone() {
		return isFinished;
	}
	
	public void run() {
		runIt = run.ACCELERATE;
		firstAngle = drive.getHeading();
		lastDistance = drive.getPosition();
		startDistance = drive.getPosition();
		startTime = System.currentTimeMillis();
		while(!isFinished) {
			currentAngle = drive.getHeading();
			currentDistance = drive.getPosition();
			currentTime = System.currentTimeMillis();
			relativeTime = currentTime - startTime;
			x = ((relativeTime * 2 * Math.PI) / accelerateTime) - Math.PI;
			switch(runIt) {
			case IDLE:
				break;
			case ACCELERATE:
				speed = (x + Math.sin(x) + Math.PI) / (2 * Math.PI);
				drive.setMotorSpeeds(speed * fob, speed * fob);
				if(((currentDistance - lastDistance) * fob) >= accelerateDistance) {
					lastDistance = currentDistance;
					runIt = run.COAST;
				}
				break;
			case COAST:
				if(((currentDistance - lastDistance) * fob) >= coastDistance) {
					lastDistance = currentDistance;
					startTime = currentTime;
					angleLastError = currentAngle - firstAngle;
					lastError = (currentDistance - startDistance) - thisdistance;
					runIt = run.DECELERATE;
				}
				break;
			case DECELERATE:
				angleError = currentAngle - firstAngle;
				error = (currentDistance - startDistance) - thisdistance;
				
				vP = error  * tkP;
				vD = (error - lastError) * tkD;
				velocity = vP + vD;
				
				omegaP = angleError * rkP;
				omegaD = (angleError - angleLastError) * rkD;
				omega = omegaP + omegaD;
				
				omega *= lor;
				omega *= (Math.PI / 180.0);
				omega *= halfTrackWidth;
				
				drive.setMotorSpeeds(velocity - omega, velocity + omega);
				
				angleLastError = angleError;
				lastError = error;
				
				if((Math.abs((currentDistance - startDistance) - thisdistance) <= 0.2 //0.2 meters is much more accurate than before
						&& Math.abs(currentAngle - firstAngle) <= 4)
						|| (currentTime - startTime > 1500)) {//4 degrees is not too much
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
}
