package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class PivotTrajectory {
	private DriveHardware drive;
	private double currentAngle;
	private double speed;
	private boolean isFinished = false;
	private double thisAngle;
	private double lasttime;
	private long currentTime;
	private final double kF = 0.005;
	
	private double startAngle;
	
	private final double rkP = .025 * (180 / Math.PI);//PD For rotation
	private final double rkD = 0.000005 * (180 / Math.PI);//0.000005
	private final double rkI = 0.00001 * (180 / Math.PI);
	
	private int lor = 1;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	
	private double angleError;
	private double angleLastError;
	
	private double currentVelocity;
	
	private TorBantorShooarm shooArm;
	
	private TorDerivative derivative;
	
	public static enum run {
		IDLE, GO;
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
		thisAngle *= (Math.PI / 180.0);
		this.shooArm = shooArm;
		derivative = new TorDerivative(drive.getHeading());
	}

	public boolean isDone() {
		return isFinished;
	}
	
	public void run(double starttime) {
		isFinished = false;
		runIt = run.GO;
		startAngle = drive.getHeading();
		lasttime = Timer.getFPGATimestamp();
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
			case GO:
				angleError = ((currentAngle - startAngle) * lor) - thisAngle;
				omegaP = angleError * rkP;
				omegaD = angleError = (angleError - angleLastError) * (rkD / kF);
				speed = omegaP + omegaD;

				currentVelocity = derivative.estimate(angleError);//radians per milisecond
				currentVelocity *= 1000;//radians per second
				currentVelocity *= (180.0 / Math.PI);//degrees per second
				
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
