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
	private double lor = 1;
	private long startTime;
	private long relativeTime;
	private long currentTime;
	
	public static enum run {
		IDLE, ACCELERATE, COAST, DECELERATE;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public PivotTrajectory(DriveHardware drive, double angle, double accelerationFraction, long accelerateTime, double holdBack) {
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
					runIt = run.DECELERATE;
				}
				break;
			case DECELERATE:
				speed = 0;
				drive.setMotorSpeeds(0, 0);
				isFinished = true;
				runIt = run.IDLE;
				break;
			}
		}
	}
}
