package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

public class TelBantorShooarm {
	private Joystick player2;
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	private TalonSRX shootakeTalon1;
	private TalonSRX shootakeTalon2;
	private AnalogPotentiometer fourtwenty;//it is the POT
	private DigitalInput breakbeam;
	//	private Solenoid Pusher;
	private double scaleAngle;
	private double switchAngle;
	private double degreeTolerance;
	private double kF;
	private double kP;
	private double kD;
	private double error;
	private double lastError;
	private double velocity;
	private double derivative;
	private double proportional;
	private double currentAngle;
	private boolean switchEnable;
	private boolean scaleEnable;

	//buttons on the controller:
	//a is intake 1
	//x is switch 3
	//y is scale 4
	//right trigger is human fire stick 3
	//left trigger is manuel override button 5
	//b is climb 2
	public static enum switchDo {
		IDLE, POS0, POS1, POS2, POS3, PID, POS4, POS5;
		private switchDo() {}
	}
	public static enum scaleDo {
		IDLE, POS0, POS1, POS2, POS3, PID, POS4, POS5;
		private scaleDo() {}
	}
	private static enum shoot {
		IDLE, POS0, MOTOROUT, EXTEND, RETRACT;
		private shoot() {}
	}
	private static enum intake {
		IDLE, POS0, RETRACT, MOTORIN;
		private intake() {}
	}




	public switchDo switchDo1 = switchDo.IDLE;
	public scaleDo scaleDo1 = scaleDo.IDLE;
	public shoot shootIt = shoot.IDLE;
	public intake intakeIt = intake.IDLE;
	private double speed;

	/**********************************
	 TUNE THE ARM FROM HERE
	 */
	private int uod = 1;//up or down for manuel override. Change it from 1 to -1 to change the control on the arm with the right player y
	private int ioo = -1;//in or out variable. Change this to switch around the outtake and intake when it is up. Change it from 1 to -1 to make it so either shoots out or (not wanted) intakes up there
	//ioo is for pressing the BUTTON NOT MANUEL CONTROL

	private int ioop = -1;//in or our variable for the player under manuel control

	//the switch tunes
	private long switchPos1Time = 250;//change this to make it go up higher during the switch
	private long switchPos2Time = 1;
	//this is the time it is at the max speed
	private double switchMaxSpeed = 0.5;//the acceleration increment for the switch
	//the deacceleration increment for the switch
	private double switchPush = 0.18;//the extra speed to push to hit the degreeTolerance from just a proportional distance speed
	private double switchCushion = -0.13;//NEGATIVE the extra cushion from a proportional down 

	//the scale tunes
	private long scalePos1Time = 350;
	private long scalePos2Time = 20;
	private double scaleMaxSpeed = 1;
	private double scalePush = 0.1;
	private double scaleCushion = -0.13;
	
	//the shooting and intake tunes
	private double shootPower = 1;//the power it shoots out at
	private long extendTime = 5;//the time we give to the solenoid to extend and push the cube
	private long revTime = 500;//the time we give for the motors to go from 0 to the speed and back
	private double intakePower = 0.6;
	private double startAngle;
	/**********************************
		TO HERE
	 */

	private long currentTime;
	private long endTime;
	private long startTime;
	private long relativeTime;
	private double lastAngle;
	private double x;
	private boolean stop = false;
	private double lastTime;

	public TelBantorShooarm(Joystick player2, TalonSRX armTalon1, TalonSRX armTalon2, TalonSRX shootakeTalon1, TalonSRX shootakeTalon2, DigitalInput breakbeam, AnalogPotentiometer fourtwenty, double scaleAngle, double switchAngle, double degreeTolerance, double kF, double kP, double kD) {
		//public TelBantorShooarm(Joystick player2, TalonSRX armTalon1, TalonSRX armTalon2, TalonSRX shootakeTalon, TalonSRX shootakeTalon2, Solenoid Pusher) {	
		this.player2 = player2;
		this.armTalon1 = armTalon1;
		this.armTalon2 = armTalon2;
		this.shootakeTalon1 = shootakeTalon1;
		this.shootakeTalon2 = shootakeTalon2;
		//this.Pusher = Pusher;
		this.breakbeam = breakbeam;
		this.fourtwenty = fourtwenty;
		startAngle = fourtwenty.get();
		this.scaleAngle = scaleAngle;
		this.switchAngle = switchAngle;
		this.degreeTolerance = degreeTolerance;
		this.kF = kF;
		this.kP = kP;
		this.kD = kD;
	}

	public void TorBantorArmAndShooterUpdate() {
		switchDo();
		scaleDo();
		shoot();
		intake();
		if(!stop && player2.getRawButton(3) && (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID) && scaleDo1 == scaleDo.IDLE && !player2.getRawButton(5) && intakeIt == intake.IDLE && shootIt == shoot.IDLE) {
			switchEnable = false;
			switchDo1 = switchDo.POS0;
		}
		if(!stop && player2.getRawButton(4) && (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID) && switchDo1 == switchDo.IDLE && !player2.getRawButton(5) && intakeIt == intake.IDLE && shootIt == shoot.IDLE) {
			scaleEnable = false;
			scaleDo1 = 	scaleDo.POS0;
		}
		if(!stop && player2.getRawButton(1) && switchDo1 == switchDo.IDLE && scaleDo1 == scaleDo.IDLE && !player2.getRawButton(5) && intakeIt == intake.IDLE && shootIt == shoot.IDLE) {
			intakeIt = intake.POS0;
		}
		if(!stop && Math.abs(player2.getRawAxis(3)) > 0.2 && (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID) && (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID) && !player2.getRawButton(5) && intakeIt == intake.IDLE && shootIt == shoot.IDLE) {
			shootIt = shoot.POS0;
		}
		manueloverride();
	}

	public void stop() {
		switchDo1 = switchDo.IDLE;
		scaleDo1 = scaleDo.IDLE;
		shootIt = shoot.IDLE;
		intakeIt = intake.IDLE;
		armTalon1.set(ControlMode.PercentOutput, 0);
		armTalon2.set(ControlMode.PercentOutput, 0);
		shootakeTalon1.set(ControlMode.PercentOutput, 0);
		shootakeTalon2.set(ControlMode.PercentOutput, 0);
		stop = true;
	}

	public void switchDo() {
		currentTime = System.currentTimeMillis();
		relativeTime = currentTime - startTime;
		x = (relativeTime * 2 * Math.PI / switchPos1Time) - Math.PI;
		switch(switchDo1) {
		case IDLE:
			break;
		case POS0:
			endTime = currentTime + switchPos1Time;
			speed = 0;
			startTime = System.currentTimeMillis();
			if((fourtwenty.get() - startAngle) > (scaleAngle - (2 * degreeTolerance))) {
				switchDo1 = switchDo.IDLE;
			} else if((fourtwenty.get() - startAngle) > (switchAngle - (2 * degreeTolerance))) {
				switchDo1 = switchDo.POS4;
			} else {
				switchDo1 = switchDo.POS1;
			}
			break;
		case POS1:
			speed = (Math.sin(x) + x + Math.PI) * switchMaxSpeed / (2 * Math.PI);
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos2Time;
				switchDo1 = switchDo.POS2;
			}
			break;
		case POS2:
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos1Time;
				startTime = System.currentTimeMillis();
				lastAngle = switchAngle - (fourtwenty.get() - startAngle);
				switchDo1 = switchDo.POS3;
			}
			break;
		case POS3:
			speed = (switchMaxSpeed * ((switchAngle - ((fourtwenty.get() - startAngle))) / lastAngle)) + switchPush;
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle)) - switchAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				lastError = switchAngle - ((fourtwenty.get() - startAngle));
				switchEnable = true;
				switchDo1 = switchDo.PID;
			}
			break;
		case PID:
			if(switchEnable) {
				switchPIDGO();
				switchDo1 = switchDo.PID;
			} else {
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.IDLE;
			}
			break;
		case POS4:
			speed = (Math.sin(x) + x + Math.PI) * switchMaxSpeed / (2 * Math.PI);
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos2Time;
				switchDo1 = switchDo.POS5;
			}
			break;
		case POS5:
			switchEnable = false;
			speed = (switchMaxSpeed * ((fourtwenty.get() - startAngle) / lastAngle)) + switchCushion;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(Math.abs(fourtwenty.get() - startAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				switchDo1 = switchDo.IDLE;
			}
			break;
		}
	}

	public void scaleDo() {
		currentTime = System.currentTimeMillis();
		relativeTime = currentTime - startTime;
		x = (relativeTime * 2 * Math.PI / scalePos1Time) - Math.PI;
		switch(scaleDo1) {
		case IDLE:
			break;
		case POS0:
			endTime = currentTime + scalePos1Time;
			speed = 0;
			startTime = System.currentTimeMillis();
			if(((fourtwenty.get() - startAngle)) > (scaleAngle - (2 * degreeTolerance))) {
				scaleDo1 = scaleDo.POS4;
			} else if(((fourtwenty.get() - startAngle)) > (switchAngle - (2 * degreeTolerance))) {
				scaleDo1 = scaleDo.IDLE;
			} else {
				scaleDo1 = scaleDo.POS1;
			}
			break;
		case POS1:
			speed = (Math.sin(x) + x + Math.PI) * scaleMaxSpeed / (2 * Math.PI);
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				scaleDo1 = scaleDo.POS2;
			}
			break;
		case POS2:
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos1Time;
				startTime = System.currentTimeMillis();
				lastAngle = scaleAngle - (fourtwenty.get() - startAngle);
				scaleDo1 = scaleDo.POS3;
			}
			break;
		case POS3:
			speed = (scaleMaxSpeed * (scaleAngle - ((fourtwenty.get() - startAngle))) / lastAngle) + scalePush;
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle)) - scaleAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				lastError = scaleAngle - ((fourtwenty.get() - startAngle));
				scaleEnable = true;
				scaleDo1 = scaleDo.PID;
			}
			break;
		case PID:
			if(scaleEnable) {
				scalePIDGO();
				scaleDo1 = scaleDo.PID;
			} else {
				endTime = currentTime + scalePos1Time;
				scaleDo1 = scaleDo.IDLE;
			}
			break;		
		case POS4:
			speed = (Math.sin(x) + x + Math.PI) * scaleMaxSpeed / (2 * Math.PI);
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				scaleDo1 = scaleDo.POS5;
			}
				break;
		case POS5:
			scaleEnable = false;
			speed = (scaleMaxSpeed * ((fourtwenty.get() - startAngle) / lastAngle)) + scaleCushion;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(fourtwenty.get() - startAngle <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				scaleDo1 = scaleDo.IDLE;
			}
			break;
		}
	}


	public void shoot() {
		switch(shootIt) {
		case IDLE:
			break;
		case POS0:
			currentTime = System.currentTimeMillis();
			endTime = currentTime + revTime;
			shootIt = shoot.MOTOROUT;
			break;
		case MOTOROUT:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, shootPower * ioo);
			shootakeTalon2.set(ControlMode.PercentOutput, -shootPower * ioo);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				shootIt = shoot.EXTEND;
			}
			break;
		case EXTEND:
			currentTime = System.currentTimeMillis();
			//			Pusher.set(true);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				shootIt = shoot.RETRACT;
			}
			break;
		case RETRACT:
			currentTime = System.currentTimeMillis();
			//			Pusher.set(false);
			if(currentTime >= endTime) {
				shootakeTalon1.set(ControlMode.PercentOutput, 0);
				shootakeTalon2.set(ControlMode.PercentOutput, 0);	
				shootIt = shoot.IDLE;
			}
			break;
		}
	}

	public void intake() {
		switch(intakeIt) {
		case IDLE:
			break;
		case POS0:
			currentTime = System.currentTimeMillis();
			endTime = currentTime + extendTime;
			if(currentTime - lastTime >= 2000) {
				intakeIt = intake.RETRACT;	
			} else {
				intakeIt = intake.IDLE;
			}
			break;
		case RETRACT:
			currentTime = System.currentTimeMillis();
			//			Pusher.set(false);
			if(currentTime >= endTime) {
				endTime = currentTime + revTime;
				intakeIt = intake.MOTORIN;
			}
			break;
		case MOTORIN:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, -intakePower * ioo);
			shootakeTalon2.set(ControlMode.PercentOutput, intakePower * ioo);
			if(breakbeam.get() || (currentTime >= endTime && player2.getRawButton(1))) {
				shootakeTalon1.set(ControlMode.PercentOutput, 0);
				shootakeTalon2.set(ControlMode.PercentOutput, 0);
				lastTime = currentTime;
				intakeIt = intake.IDLE;
			}
			break;
		}
	}


	public void manueloverride() {
		if(player2.getRawButton(5) && !stop) {
			switchDo1 = switchDo.IDLE;
			scaleDo1 = scaleDo.IDLE;
			shootIt = shoot.IDLE;
			intakeIt = intake.IDLE;
			if(Math.abs(player2.getRawAxis(1)) > 0.2) {
				armTalon1.set(ControlMode.PercentOutput, player2.getRawAxis(1) * uod);
				armTalon2.set(ControlMode.PercentOutput, -player2.getRawAxis(1) * uod);
			} else {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
			}
			if(player2.getRawButton(6)) {
				//				Pusher.set(true);
			} else {
				//				Pusher.set(false);
			}
			if(Math.abs(player2.getRawAxis(5)) > 0.2) {
				shootakeTalon1.set(ControlMode.PercentOutput, player2.getRawAxis(5) * ioop);
				shootakeTalon2.set(ControlMode.PercentOutput, -player2.getRawAxis(5) * ioop);
			} else {
				shootakeTalon1.set(ControlMode.PercentOutput, 0);
				shootakeTalon2.set(ControlMode.PercentOutput, 0);
			}		
		}
	}

	public void switchPIDGO() {
		currentAngle = ((fourtwenty.get() - startAngle));
		error = switchAngle - currentAngle;
		proportional = kP * error;
		derivative = kD * ((error - lastError)) / kF;
		velocity = proportional + derivative;
		armTalon1.set(ControlMode.PercentOutput, -velocity);
		armTalon2.set(ControlMode.PercentOutput, velocity);
		lastError = error;
	}

	public void scalePIDGO() {
		currentAngle = ((fourtwenty.get() - startAngle));
		error = scaleAngle - currentAngle;
		proportional = kP * error;
		derivative = kD * ((error - lastError) / kF);
		velocity = proportional + derivative;
		armTalon1.set(ControlMode.PercentOutput, -velocity);
		armTalon2.set(ControlMode.PercentOutput, velocity);
		lastError = error;
	}
}