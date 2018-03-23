package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class PivotTrajectory {
	private DriveHardware drive;
	private double currentAngle;
	private double speed;
	private boolean isFinished = false;
	private double accelerateAngle;
	private double thisAngle;
	private long lasttime;
	private long currentTime;
	private final double kF = 0.005;
	
	private double startAngle;
	
	private final double accelerateSpeed = 0.225;
	private final double decelerateAngle = 30 * (Math.PI / 180.0);
	
	private final double rkP = .025 * (180 / Math.PI);//PD For rotation
	private final double rkD = 0.000000 * (180 / Math.PI);//0.000005

	private int lor = 1;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	
	private double angleError;
	private double angleLastError;
	
	private double currentVelocity;
	private double dx;
	private double y1;
	private double y2;
	private long lastVelTime;
	
	private TorBantorShooarm shooArm;
	
	public static enum run {
		IDLE, ACCELERATE, DECELERATE;
		private run() {}
	}
	
	public run runIt = run.IDLE;
	
	public PivotTrajectory(DriveHardware drive, double angle, TorBantorShooarm shooArm) {
		this.drive = drive;
		this.thisAngle = angle;
		if(thisAngle < 0) {
			lor = -1;
			thisAngle *= -1;
		}
		accelerateAngle = thisAngle * (Math.PI / 180) - decelerateAngle;
		thisAngle *= (Math.PI / 180.0);
		this.shooArm = shooArm;
	}

	public boolean isDone() {
		return isFinished;
	}
	
	public void run(double starttime) {
		isFinished = false;
		runIt = run.ACCELERATE;
		startAngle = drive.getHeading();
		lasttime = System.currentTimeMillis();
		while(!isFinished) {
			if(Timer.getFPGATimestamp() - starttime > 14) {
				drive.setMotorSpeeds(0, 0);
				break;
			}
			
			shooArm.TorBantorArmAndShooterUpdate();
			currentAngle = drive.getHeading();
			currentTime = System.currentTimeMillis();
			switch(runIt) {
			case IDLE:
				break;
			case ACCELERATE:
				drive.setMotorSpeeds(-accelerateSpeed * lor, accelerateSpeed * lor);
				if((((currentAngle - startAngle) * lor) >= accelerateAngle) || (currentTime - lasttime > 2000)) {
					lasttime = currentTime;
					y1 = drive.getHeading();
					lastVelTime = currentTime;
					angleLastError = ((currentAngle - startAngle) * lor) - thisAngle;
					runIt = run.DECELERATE;
				}
				break;
			case DECELERATE:
				y2 = drive.getHeading();
				dx = currentTime - lastVelTime;
				currentVelocity = (y2 - y1) / dx;//radians per milisecond
				currentVelocity *= 1000;//radians per second
				currentVelocity *= (180.0 / Math.PI);//degrees per second
				y1 = y2;
				lastVelTime = currentTime;
				
				angleError = ((currentAngle - startAngle) * lor) - thisAngle;
				omegaP = angleError * rkP;
				omegaD = angleError = (angleError - angleLastError) * (rkD / kF);
				speed = omegaP + omegaD;

				speed *= lor;
				
				drive.setMotorSpeeds(speed, -speed);
//				drive.setVelocity(-speed, speed);
				
				angleLastError = angleError;
				
				if((Math.abs(angleError) <= (0.25 * (Math.PI / 180.0))
						&& Math.abs(currentVelocity) < 0.5) || 
						(currentTime - lasttime > 1)) {
//				if((Math.abs(angleError) <= (0.25 * (Math.PI / 180.0))
//						&& Math.abs(currentVelocity) < 0.5)) {
					drive.setMotorSpeeds(0, 0);
					isFinished = true;
					runIt = run.IDLE;
				}
				break;
			}
		}
	}
}
