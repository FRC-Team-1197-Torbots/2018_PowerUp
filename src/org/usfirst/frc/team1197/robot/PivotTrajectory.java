package org.usfirst.frc.team1197.robot;

public class PivotTrajectory {
	private DriveHardware drive;
	private double lastAngle;
	private double currentAngle;
	private double speed;
	private boolean isFinished = false;
	private double accelerateAngle;
	private double coastAngle;
	private double thisAngle;
	private long accelerateTime;
	private double x;
	private long startTime;
	private long relativeTime;
	private long currentTime;
	private final double kF = 0.005;
	
	private double startAngle;

	private final double halfTrackWidth = .352425;//in meters
	
	private final double rkP = 0.0;//PD For rotation
	private final double rkD = 0.0;

	private int lor = 1;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	
	private double angleError;
	private double angleLastError;
	
	
	
	public static enum run {
		IDLE, ACCELERATE, COAST, DECELERATE;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public PivotTrajectory(DriveHardware drive, double angle, double accelerationFraction, 
			long accelerateTime) {
		this.drive = drive;
		this.accelerateTime = accelerateTime;
		this.thisAngle = angle;
		if(thisAngle < 0) {
			lor = -1;
			thisAngle *= -1;
		}
		accelerateAngle = thisAngle * (Math.PI / 180) * accelerationFraction;
		coastAngle = thisAngle * (Math.PI / 180) * (1 - (2 * accelerationFraction));
	}

	public boolean isDone() {
		return isFinished;
	}
	
	public void run() {
		runIt = run.ACCELERATE;
		lastAngle = drive.getHeading();
		startAngle = drive.getHeading();
		startTime = System.currentTimeMillis();
		while(!isFinished) {
			currentAngle = drive.getHeading();
			currentTime = System.currentTimeMillis();
			relativeTime = currentTime - startTime;
			x = ((relativeTime * 2 * Math.PI) / accelerateTime) - Math.PI;
			switch(runIt) {
			case IDLE:
				break;
			case ACCELERATE:
				speed = (x + Math.sin(x) + Math.PI) / (2 * Math.PI);
				drive.setMotorSpeeds(-speed * lor, speed * lor);
				if(((currentAngle - lastAngle) * lor) >= accelerateAngle) {
					lastAngle = currentAngle;
					runIt = run.COAST;
				}
				break;
			case COAST:
				if(((currentAngle - lastAngle) * lor) >= coastAngle) {
					lastAngle = currentAngle;
					startTime = currentTime;
					angleLastError = angleError = ((currentAngle - startAngle) * lor) - thisAngle;
					runIt = run.DECELERATE;
				}
				break;
			case DECELERATE:
				angleError = ((currentAngle - startAngle) * lor) - thisAngle;
				omegaP = angleError * rkP;
				omegaD = angleError = (angleError- angleLastError) * (rkD / kF);
				speed = omegaP + omegaD;

				speed *= lor;
				
				drive.setVelocity(-speed, speed);
				
				angleLastError = angleError;
				if(Math.abs(((currentAngle - startAngle) * lor) - thisAngle) <= 4 || (currentTime - startTime > 1000)) {
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
}
