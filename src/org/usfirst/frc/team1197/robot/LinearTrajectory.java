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
	private double x;
	private long startTime;
	private long relativeTime;
	private long currentTime;
	
	public static enum run {
		IDLE, ACCELERATE, COAST, DECELERATE;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public LinearTrajectory(DriveHardware drive, double distance, 
			double accelerationFraction, long accelerateTime, double holdBack) {
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
		lastDistance = drive.getPosition();
		startTime = System.currentTimeMillis();
		while(!isFinished) {
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
					runIt = run.DECELERATE;
				}
				break;
			case DECELERATE:
				drive.setMotorSpeeds(0, 0);
				System.out.println("MOVE 1 is DONE");
				isFinished = true;
				runIt = run.IDLE;
				break;
			}
		}
	}
}
