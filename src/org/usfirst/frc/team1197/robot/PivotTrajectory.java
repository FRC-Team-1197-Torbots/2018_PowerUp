package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Timer;

public class PivotTrajectory {
	private DriveHardware drive;
	private double currentAngle;
	private double speed;
	private boolean isFinished = false;
	private double thisAngle;
	private double lasttime;
	private double currentTime;
	private final double kF = 0.005;
	
	private double startAngle;
	
	private final double rkP = .1;//PD For rotation
	private final double rkD = 0.01;//0.000005
	private final double rkI = 0.001;
	
	private int lor = 1;
	
	private double omegaP;//turning proportional
	private double omegaD;//turning derivative
	private double omegaI;
	
	private double angleError;
	
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
		derivative = new TorDerivative(kF);
	}

	public boolean isDone() {
		return isFinished;
	}
	
	public void init() {
		isFinished = false;
		runIt = run.GO;
		startAngle = drive.getHeading();
		derivative.resetValue(drive.getHeading());
		lasttime = Timer.getFPGATimestamp();
	}
	
	public void run(double starttime) {			
		shooArm.TorBantorArmAndShooterUpdate();
		currentAngle = drive.getHeading();
		currentTime = Timer.getFPGATimestamp();
		switch(runIt) {
		case IDLE:
			break;
		case GO:
			angleError = (thisAngle * lor) - ((currentAngle - startAngle));
			omegaI += angleError;
			omegaP = angleError * rkP;
			
			currentVelocity = derivative.estimate(angleError);//radians per second
			currentVelocity *= (180 / Math.PI);//degrees per second
			omegaD = (currentVelocity * rkD * -1);
			
			speed = omegaP + omegaD + (omegaI * rkI * kF);
			
			drive.setMotorSpeeds(speed, -speed);
				
			if((Math.abs(angleError) <= (0.25 * (Math.PI / 180.0))
					&& Math.abs(currentVelocity) < 0.5) || 
					(currentTime - lasttime > 1)) {
				drive.setMotorSpeeds(0, 0);
				isFinished = true;
				runIt = run.IDLE;
			}
			break;
		}
	}
}
