package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;//is the breakbeam

public class TorBantorShooarm {
	private Joystick player2;
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	private VictorSPX shootakeTalon1;
	private VictorSPX shootakeTalon2;
	private AnalogPotentiometer fourtwenty;
	private DigitalInput breakbeam;
	private Solenoid Pusher;
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
	private boolean vaultContinue = false;
	private long vaultLastTime;
	private double scaleBackwardsAngle;
	private final double threshold = 10.0;
	
	public static enum switchDo {
		IDLE, POS0, POS1, POS2, POS3, PID, POS4, POS5;
		private switchDo() {}
	}
	
	public static enum scaleDo {
		IDLE, POS0, POS1, POS2, POS3, PID, POS4, POS5;
		private scaleDo() {}
	}
	
	public static enum backwardsScale {
		IDLE, POS0, POS1, POS2, POS3, PID, POS4, POS5;
	}
	
	private static enum shoot {
		IDLE, POS0, EXTEND, RETRACT;
		private shoot() {}
	}
	
	private static enum intake {
		IDLE, START, POS0, RETRACT, MOTORIN;
		private intake() {}
	}
	
	private static enum holder {
		START, PD, STOP;
		private holder() {}
	}
	
	public static enum manual {
		IDLE, GOING, GOINGDOWN;
		private manual() {}
	}
	
	public static enum intakeDown {
		IDLE, START, PD;
		private intakeDown() {}
	}
	
	public static enum vault {
		IDLE, START, GOING;
		private vault() {}
	}
	
	public static enum vaultDown {
		IDLE, START, PD;
		private vaultDown() {}
	}
	
	public switchDo switchDo1 = switchDo.IDLE;
	public scaleDo scaleDo1 = scaleDo.IDLE;
	public shoot shootIt = shoot.IDLE;
	public intake intakeIt = intake.IDLE;
	public holder holdIt = holder.START;
	public manual manualGo = manual.IDLE;
	public intakeDown holdDown = intakeDown.IDLE;
	public backwardsScale backScale = backwardsScale.IDLE;
	public vaultDown vaultDown1 = vaultDown.IDLE;
	public vault vault1 = vault.IDLE;
	private double speed;

	/**********************************
	 TUNABLE ARM VARIABLES
	 */ 

	private double startAngle;//HAS TO BE TUNED TO THE POTENTIOMETER READING WHEN IT IS ALREADY DOWN
	
	private int uod = 1; // up or down for manual override. Change it from 1 to -1 to change the control on the arm with the right player y
	private int ioo = -1; // in or out variable. Change this to switch around the outtake and intake when it is up. Change it from 1 to -1 to make it so either shoots out or (not wanted) intakes up there
	private double manualMax = 0.4; // POSITIVE. The controls on the speeds for the manual override.
	private double manualMin = -0.2; // HAS TO BE NEGATIVE!
	private double manualMaxAngle = 80;
	
	// Switch Variables
	private long switchPos1Time = 300; // change this to make it go up higher during the switch
	private long switchPos2Time = 100; // this is the time it is at the max speed
	private double switchMaxSpeed = 0.9; // the acceleration increment for the switch
	private double switchPush = 0.15; // the extra speed to push to hit the degreeTolerance from just a proportional distance speed
	private double switchCushion = 0; // NEGATIVE the extra cushion from a proportional down 
	private double switchShootPower = 0.2;

	// Scale Variables
	private long scalePos1Time = 450;
	private long scalePos2Time = 50;
	private double scaleMaxSpeed = 1;
	private double scalePush = 0.23;
	private double scaleCushion = -0.02;
	private double scaleShootPower = 1;
	
	// Shooter & Intake Variables
	private double shootPower = 0.6;//the power it shoots out at
	private long extendTime = 200;//the time we give to the solenoid to extend and push the cube
	private long revTime = 500;//the time we give for the motors to go from 0 to the speed and back
	private double intakePower = 0.6;
	private long shootTime = 3 * (extendTime + revTime);//3 times to make sure there is enough time to shoot it out
	
	// Hold PD Constants
	private double holdkP = 0.05;
	private double holdkD = 0.00001;
	
	private int potSwitch = 1;//change this from 1 to 
//	-1 to control the pot. (pot might be mounted on backwards)
	
	/***********************************/

	private long currentTime;
	private long endTime;
	private long startTime;
	private long relativeTime;
	private double lastAngle;
	private double x;
	private boolean stop = false;
	private boolean holdContinue;
	private double lastTime;
	private double holdAngle;
	private double holdError;
	private double holdLastError;
	private double holdProportional;
	private double holdDerivative;
	private double holdVelocity;
	private double armAxis;
	private double wantedAngle;
	private boolean pressingRightTrigger;

	public TorBantorShooarm(Joystick player2, TalonSRX armTalon1, TalonSRX armTalon2, 
			VictorSPX shootakeTalon1, VictorSPX shootakeTalon2, 
			DigitalInput breakbeam, AnalogPotentiometer fourtwenty, 
			double scaleAngle, double switchAngle, double degreeTolerance, 
			double kF, double kP, double kD, double holdAngle, 
			Solenoid Pusher, double scaleBackwardsAngle) {	
		this.player2 = player2;
		this.armTalon1 = armTalon1;
		this.armTalon2 = armTalon2;
		this.shootakeTalon1 = shootakeTalon1;
		this.shootakeTalon2 = shootakeTalon2;
		this.Pusher = Pusher;
		this.breakbeam = breakbeam;
		this.fourtwenty = fourtwenty;
		startAngle = 47.65;//MAKE THIS WHEN THE ARM IS FLAT
		this.scaleAngle = scaleAngle;
		this.switchAngle = switchAngle;
		this.degreeTolerance = degreeTolerance;
		this.kF = kF;
		this.kP = kP;
		this.kD = kD;
		this.holdAngle = holdAngle;
		this.scaleBackwardsAngle = scaleBackwardsAngle;
		pressingRightTrigger = false;
		Pusher.set(false);
	}
	
	/*** PLAYER 2 BUTTON CONFIGURATION FOR THE ARM ***
	 
	   A (1): Intake Position (will keep intaking until you press it again or the cube is in the intake with the breakbeam)
	   X (3): Switch Position (Press again to put back to hold position)
	   Y (4): Scale Position  (Press again to put back to hold position)
	   B (2): Climb
	   RT(3): Human fire stick (shoots it)
	   LT(5): Manual override (Hold the button)
	
	********************************************/

	public void TorBantorArmAndShooterUpdate() {
		
		switchDo(); 	  // Update for the switch
		scaleDo(); 		  // Update for the scale
		shoot(); 		  // Update for the shooter
		intake(); 		  // Update for the intake
		Hold(); 		  // Update for the hold
//		manualoverride(); // Update for the manual override
		holdDownUpdate(); // Update for the hold down
//		backwardsScaleDo(); //Update for the backwards scale
		shootTake();
		vault();
		vaultHold();
		
		// Activate switch if button 'X' is pressed
		if(!stop && player2.getRawButton(3) 
				&& (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID) 
				&& scaleDo1 == scaleDo.IDLE 
				&& !player2.getRawButton(5) 
				&& intakeIt == intake.IDLE 
				&& shootIt == shoot.IDLE) {
			switchEnable = false;
			holdContinue = false;
			holdIt = holder.STOP;
			holdDown = intakeDown.IDLE;
			switchDo1 = switchDo.POS0;
		}
		
		// Activate scale if button 'Y' is pressed
		if(!stop && player2.getRawButton(4) 
				&& (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID) 
				&& (switchDo1 == switchDo.IDLE) 
				&& !player2.getRawButton(5) 
				&& intakeIt == intake.IDLE 
				&& shootIt == shoot.IDLE) {
			scaleEnable = false;
			holdContinue = false;
			holdIt = holder.STOP;
			holdDown = intakeDown.IDLE;
			scaleDo1 = 	scaleDo.POS0;
		}
		
		// Activate intake if button 'A' is pressed
		if(!stop && player2.getRawButton(1) 
				&& switchDo1 == switchDo.IDLE 
				&& scaleDo1 == scaleDo.IDLE  
				&& intakeIt == intake.IDLE 
				&& shootIt == shoot.IDLE 
				&& holdIt == holder.PD) {
			intakeIt = intake.POS0;
		}
		
		if(!stop && Math.abs(player2.getRawAxis(3)) > 0.2 
				&& (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID) 
				&& (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID)  
				&& intakeIt == intake.IDLE 
				&& shootIt == shoot.IDLE 
				&& !(holdIt == holder.START)) {
			shootIt = shoot.POS0;
		}
	}

	// Method to stop everything when climb
	public void stop(double angleToHold) {
		switchDo1 = switchDo.IDLE;
		scaleDo1 = scaleDo.IDLE;
		shootIt = shoot.IDLE;
		intakeIt = intake.IDLE;
		holdIt = holder.STOP;
		holdDown = intakeDown.IDLE;
		backScale = backwardsScale.IDLE;
		manualGo = manual.IDLE;
		armTalon1.set(ControlMode.PercentOutput, 0);
		armTalon2.set(ControlMode.PercentOutput, 0);
		shootakeTalon1.set(ControlMode.PercentOutput, 0);
		shootakeTalon2.set(ControlMode.PercentOutput, 0);
		stop = true;
		lastError = 0;
		while(true) {
			stop1(angleToHold);
		}
	}

	public void switchDo() {
		currentTime = System.currentTimeMillis();
		relativeTime = currentTime - startTime;
		x = ((relativeTime * 2 * Math.PI) / switchPos1Time) - Math.PI;
		switch(switchDo1) {
		case IDLE:
			break;
		case POS0:
			speed = 0;
			startTime = System.currentTimeMillis();
			if(((fourtwenty.get() - startAngle) * potSwitch) > (scaleAngle - degreeTolerance)) {
				switchDo1 = switchDo.IDLE;
			} else if(((fourtwenty.get() - startAngle) * potSwitch) > (switchAngle - 2 * degreeTolerance)) {
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.POS4;
			} else {
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.POS1;
			}
			break;
		case POS1:
			speed = (x + Math.sin(x) + Math.PI) * switchMaxSpeed / (2 * Math.PI);
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
				lastAngle = switchAngle - ((fourtwenty.get() - startAngle) * potSwitch);
				switchDo1 = switchDo.POS3;
			}
			break;
		case POS3:
			speed = (switchMaxSpeed * ((switchAngle - (((fourtwenty.get() - startAngle) * potSwitch))) / lastAngle)) + switchPush;
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - switchAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				lastError = 0;
				switchEnable = true;
				switchDo1 = switchDo.PID;
			}
			break;
		case PID:
			shootPower = switchShootPower;
			if(switchEnable) {
				switchPIDGO();
				switchDo1 = switchDo.PID;
			} else {
				startTime = System.currentTimeMillis();
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
				lastAngle = ((fourtwenty.get() - startAngle) * potSwitch) - holdAngle;
				switchDo1 = switchDo.POS5;
			}
			break;
		case POS5:
			switchEnable = false;
			speed = (switchMaxSpeed * ((((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) / lastAngle)) + switchCushion;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) <= degreeTolerance) {
				holdIt = holder.START;
				switchDo1 = switchDo.IDLE;
			}
			break;
		}
	}

	public void scaleDo() {
		currentTime = System.currentTimeMillis();
		relativeTime = currentTime - startTime;
		x = ((relativeTime * 2 * Math.PI) / scalePos1Time) - Math.PI;
		switch(scaleDo1) {
		case IDLE:
			break;
		case POS0:
			speed = 0;
			startTime = System.currentTimeMillis();
			if((((fourtwenty.get() - startAngle) * potSwitch)) > (scaleAngle - (degreeTolerance))) {
				endTime = currentTime + scalePos1Time;
				scaleDo1 = scaleDo.POS4;
			} else if((((fourtwenty.get() - startAngle) * potSwitch)) > (switchAngle - (2 * degreeTolerance))) {
				scaleDo1 = scaleDo.IDLE;
			} else {
				endTime = currentTime + scalePos1Time;
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
				lastAngle = scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch);
				scaleDo1 = scaleDo.POS3;
			}
			break;
		case POS3:
			speed = (scaleMaxSpeed * (scaleAngle - (((fourtwenty.get() - startAngle) * potSwitch))) / lastAngle) + scalePush;
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(Math.abs((((fourtwenty.get() - startAngle) * potSwitch)) - scaleAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				lastError = 0;
				scaleEnable = true;
				scaleDo1 = scaleDo.PID;
			}
			break;
		case PID:
			shootPower = scaleShootPower;
			if(scaleEnable) {
				scalePIDGO();
				scaleDo1 = scaleDo.PID;
			} else {
				startTime = System.currentTimeMillis();
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
				lastAngle = ((fourtwenty.get() - startAngle) * potSwitch) - holdAngle;
				scaleDo1 = scaleDo.POS5;
			}
			break;
		case POS5:
			scaleEnable = false;
			speed = (scaleMaxSpeed * ((((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) / lastAngle)) + scaleCushion;
			if(speed > scaleMaxSpeed) {
				speed = scaleMaxSpeed;
			}
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) <= degreeTolerance) {
				holdIt = holder.START;
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
			endTime = currentTime + extendTime;
			shootIt = shoot.EXTEND;
			break;
		case EXTEND:
			currentTime = System.currentTimeMillis();
			Pusher.set(true);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				shootIt = shoot.RETRACT;
			}
			break;
		case RETRACT:
			currentTime = System.currentTimeMillis();
			Pusher.set(false);
			if(currentTime >= endTime) {
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
			endTime = currentTime + 1;
			if(currentTime - lastTime >= 2000) {
				holdIt = holder.STOP;
				holdDown = intakeDown.START;
				intakeIt = intake.START;	
			} else {
				intakeIt = intake.IDLE;
			}
			break;
		case START:
			currentTime = System.currentTimeMillis();
			if(currentTime >= endTime) {
				endTime = currentTime + 1;
				intakeIt = intake.RETRACT;
			}
			break;
		case RETRACT:
			currentTime = System.currentTimeMillis();
			Pusher.set(false);
			if(currentTime >= endTime) {
				endTime = currentTime + revTime;
				intakeIt = intake.MOTORIN;
			}
			break;
		case MOTORIN:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, -intakePower * ioo);
			shootakeTalon2.set(ControlMode.PercentOutput, intakePower * ioo);
			if((currentTime >= endTime) && (((player2.getRawButton(1)) || 
			((Math.abs(shootakeTalon1.getOutputCurrent()) > threshold) ||
			(Math.abs(shootakeTalon2.getOutputCurrent()) > threshold))) || !breakbeam.get())) {
				shootakeTalon1.set(ControlMode.PercentOutput, 0);
				shootakeTalon2.set(ControlMode.PercentOutput, 0);
				lastTime = currentTime;
				holdDown = intakeDown.IDLE;
				holdIt = holder.START;
				intakeIt = intake.IDLE;
			}
			break;
		}
	}

	public void switchPIDGO() {
		currentAngle = (((fourtwenty.get() - startAngle) * potSwitch));
		error = switchAngle - currentAngle;
		proportional = kP * error;
		derivative = kD * ((error - lastError)) / kF;
		velocity = proportional + derivative;
		armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
		armTalon2.set(ControlMode.PercentOutput, velocity * uod);
		lastError = error;
	}

	public void scalePIDGO() {
		currentAngle = (((fourtwenty.get() - startAngle) * potSwitch));
		error = scaleAngle - currentAngle;
		proportional = kP * error;
		derivative = kD * ((error - lastError) / kF);
		velocity = proportional + derivative;
		armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
		armTalon2.set(ControlMode.PercentOutput, velocity * uod);
		lastError = error;
	}

	public void Hold() {
		switch(holdIt) {
		case START:
			holdLastError = 0;
			holdContinue = true;
			holdIt = holder.PD;
			break;
		case PD:
			if(holdContinue) {
				HoldPIDGO();
				holdIt = holder.PD;
			} else {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				holdIt = holder.STOP;
			}
			break;
		case STOP:
			break;
		}
	}
	
	public void HoldPIDGO() {
		holdError = (holdAngle) - ((fourtwenty.get() - startAngle) * potSwitch);
		
		holdProportional = holdError * holdkP;
		holdDerivative = (holdError - holdLastError) * holdkD / kF;
		holdVelocity = holdProportional + holdDerivative;
		if(holdVelocity > manualMax) {
			holdVelocity = manualMax;
		}
		if(holdVelocity < manualMin) {
			holdVelocity = manualMin;
		}
		armTalon1.set(ControlMode.PercentOutput, -holdVelocity * uod);
		armTalon2.set(ControlMode.PercentOutput, holdVelocity * uod);
		
		holdLastError = holdError;
	}
	
	public void manualoverride() {
		switch(manualGo) {
		case IDLE:
			if(player2.getRawButton(5) && !stop) {		
				lastError = 0;
				wantedAngle = ((fourtwenty.get() - startAngle) * potSwitch);
				manualGo = manual.GOING;
			}
			break;
		case GOING:
			switchEnable = false;
			scaleEnable = false;;
			switchDo1 = switchDo.IDLE;
			scaleDo1 = scaleDo.IDLE;
			holdIt = holder.STOP;
			if(player2.getRawButton(5) && !stop) {
				armAxis = player2.getRawAxis(5);
				armAxis *= -1;
				if(Math.abs(armAxis) < 0.15) {
					armAxis = 0;
				}
				armAxis *= armAxis * armAxis;
				wantedAngle += (kF * armAxis);
				if(wantedAngle < 0) {
					wantedAngle = 0;
				}
				if(wantedAngle > manualMaxAngle) {
					wantedAngle = manualMaxAngle;
				}
				error = wantedAngle - ((fourtwenty.get() - startAngle) * potSwitch);
				proportional = kP * error;
				derivative = (kD * (error - lastError)) / kF;
				velocity = proportional + derivative;
				if(velocity > manualMax) {
					velocity = manualMax;
				}
				if (velocity < manualMin) {
					velocity = manualMin;
				}
				
				armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
				armTalon2.set(ControlMode.PercentOutput, velocity * uod);
				lastError = error;
			} else {
				lastAngle = (((fourtwenty.get() - startAngle) * potSwitch) - holdAngle);
				manualGo = manual.GOINGDOWN;
			}
			break;
		case GOINGDOWN:
			speed = (manualMin * ((((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) / lastAngle));
			if(speed > manualMax) {
				speed = manualMax;
			}
			if(speed < manualMin) {
				speed = manualMin;
			}
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				holdIt = holder.START;
				manualGo = manual.IDLE;
			}
			break;
		}
	}
	
	public void stop1(double angleToHold) {
		currentAngle = ((fourtwenty.get() - startAngle) * potSwitch);
		wantedAngle = angleToHold;
		
		error = wantedAngle - currentAngle;
		proportional = error * kP;
		derivative = ((error - lastError) * kD) / kF;
		velocity = proportional + derivative;
		
		if(velocity > manualMax) {
			velocity = manualMax;
		}
		if(velocity < manualMin) {
			velocity = manualMin;
		}
		armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
		armTalon2.set(ControlMode.PercentOutput, velocity * uod);
		
		lastError = error;
	}
	
	public void autoFire() {
		shootIt = shoot.POS0;
	}
	
	public void holdDownUpdate() {
		switch(holdDown) {
			case IDLE:
				break;
			case START:
				holdLastError = 0;
				holdIt = holder.STOP;
				holdDown = intakeDown.PD;
				break;
			case PD:
				holdError = ((startAngle - fourtwenty.get()) * potSwitch);
				
				holdProportional = holdError * holdkP;
				holdDerivative = ((holdError - holdLastError) * holdkD) / kF; 
				holdVelocity = holdProportional + holdDerivative;
				if(holdVelocity > manualMax) {
					holdVelocity = manualMax;
				}
				if(holdVelocity < manualMin) {
					holdVelocity = manualMin;
				}
				
				armTalon1.set(ControlMode.PercentOutput, -holdVelocity * uod);
				armTalon2.set(ControlMode.PercentOutput, holdVelocity * uod);
				
				holdLastError = holdError;
				break;
		}
	}
	
	public void backwardsScaleActivate() {
		backScale = backwardsScale.POS0;
	}
	
	
	
	public void backwardsScaleDo() {
		currentTime = System.currentTimeMillis();
		relativeTime = currentTime - startTime;
		x = (relativeTime * 2 * Math.PI / scalePos1Time) - Math.PI;
		switch(backScale) {
		case IDLE:
			break;
		case POS0:
			endTime = currentTime + scalePos1Time;
			speed = 0;
			startTime = System.currentTimeMillis();
			holdContinue = false;
			holdIt = holder.STOP;
			holdDown = intakeDown.IDLE;
			backScale = backwardsScale.POS1;
			break;
		case POS1:
			speed = (x + Math.sin(x) + Math.PI) * scaleMaxSpeed / (2 * Math.PI);
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				backScale = backwardsScale.POS2;
			}
			break;
		case POS2:
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				lastAngle = scaleBackwardsAngle - ((fourtwenty.get() - startAngle) * potSwitch);
				backScale = backwardsScale.POS3;
			}
			break;
		case POS3:
			speed = (scaleMaxSpeed * (scaleBackwardsAngle -
					(((fourtwenty.get() - startAngle) * potSwitch))) / lastAngle);
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(Math.abs((((fourtwenty.get() - startAngle) * potSwitch)) - scaleBackwardsAngle) <= degreeTolerance) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				lastError = scaleBackwardsAngle - (((fourtwenty.get() - startAngle) * potSwitch));
				endTime = (currentTime + shootTime);
				shootIt = shoot.POS0;//start the shooting
				backScale = backwardsScale.PID;
			}
			break;
		case PID:
			currentAngle = (((fourtwenty.get() - startAngle) * potSwitch));
			error = scaleBackwardsAngle - currentAngle;
			proportional = kP * error;
			derivative = kD * ((error - lastError) / kF);
			velocity = proportional + derivative;
			armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
			armTalon2.set(ControlMode.PercentOutput, velocity * uod);
			lastError = error;
			if(shootIt == shoot.IDLE) {
				endTime = currentTime + scalePos1Time;
				startTime = System.currentTimeMillis();
				backScale = backwardsScale.POS4;
			}
			break;
		case POS4:
			speed = (Math.sin(x) + x + Math.PI) * scaleMaxSpeed / (2 * Math.PI);
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				lastAngle = ((fourtwenty.get() - startAngle) * potSwitch) - holdAngle;
				backScale = backwardsScale.POS5;
			}
			break;
		case POS5:
			speed = (scaleMaxSpeed * ((((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) / lastAngle)) + scaleCushion;
			if(speed > scaleMaxSpeed) {
				speed = scaleMaxSpeed;
			}
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) <= degreeTolerance) {
				holdIt = holder.START;
				backScale = backwardsScale.IDLE;
			}
			break;
		}
	}
	
	
	public void shootTake() {
		if(Math.abs(player2.getRawAxis(2)) > 0.3) {
			pressingRightTrigger = true;
			shootakeTalon1.set(ControlMode.PercentOutput, shootPower * ioo);
			shootakeTalon2.set(ControlMode.PercentOutput, -shootPower * ioo);
		} else if(pressingRightTrigger) {
			shootakeTalon1.set(ControlMode.PercentOutput, 0);
			shootakeTalon2.set(ControlMode.PercentOutput, 0);
			pressingRightTrigger = false;
		}
	}
	
	public void vaultHold() {
		switch(vaultDown1) {
		case IDLE:
			break;
		case START:
			holdLastError = 0;
			vaultContinue = true;
			vaultDown1 = vaultDown.PD;
			break;
		case PD:
			if(vaultContinue) {
				vaultPIDGO();
				vaultDown1 = vaultDown.PD;
			} else {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				vaultDown1 = vaultDown.IDLE;
			}
			break;
		}
	}
	
	public void vaultPIDGO() {
		holdError = (holdAngle * 0.5) - ((fourtwenty.get() - startAngle) * potSwitch);
		
		holdProportional = holdError * holdkP;
		holdDerivative = (holdError - holdLastError) * holdkD / kF;
		holdVelocity = holdProportional + holdDerivative;
		if(holdVelocity > manualMax) {
			holdVelocity = manualMax;
		}
		if(holdVelocity < manualMin) {
			holdVelocity = manualMin;
		}
		armTalon1.set(ControlMode.PercentOutput, -holdVelocity * uod);
		armTalon2.set(ControlMode.PercentOutput, holdVelocity * uod);
		
		holdLastError = holdError;
	}
	
	
	public void vault() {
		currentTime = System.currentTimeMillis();
		switch(vault1) {
		case IDLE:
			break;
		case START:
			if(currentTime - vaultLastTime >= 2000) {
				holdIt = holder.STOP;
				holdDown = intakeDown.IDLE;
				endTime = currentTime + revTime;
				vaultDown1 = vaultDown.START;
				vault1 = vault.GOING;
			} else {
				vault1 = vault.IDLE;
			}
			break;
		case GOING:
			if((currentTime >= endTime) && player2.getRawButton(2)) {
				vaultContinue = false;
				vaultDown1 = vaultDown.IDLE;
				vaultLastTime = currentTime;
				holdIt = holder.START;
				vault1 = vault.IDLE;
			}
			break;
		}
	}
	
	/*
	 * THESE ARE ALL THE AUTO METHODS
	 * THEY ARE AT THE BOTTOM
	 * THEY ARE VIRTUAL BUTTONS
	 * CALL IN AUTO CLASS ONCE FOR EACH OF THEM
	 */
	
	
	
	
	
	
	
	public boolean switchIsPID() {
		return (switchDo1 == switchDo.PID);
	}
	public boolean scaleIsPID() {
		return (scaleDo1 == scaleDo.PID);
	}
	
	public void pressX() {
		if(!stop 
			&& (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID) 
			&& scaleDo1 == scaleDo.IDLE 
			&& !player2.getRawButton(5) 
			&& intakeIt == intake.IDLE 
			&& shootIt == shoot.IDLE) {
		switchEnable = false;
		holdContinue = false;
		holdIt = holder.STOP;
		holdDown = intakeDown.IDLE;
		switchDo1 = switchDo.POS0;
		}
	}
	
	public void pressY() {
		if(!stop 
				&& (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID) 
				&& (switchDo1 == switchDo.IDLE) 
				&& !player2.getRawButton(5) 
				&& intakeIt == intake.IDLE 
				&& shootIt == shoot.IDLE) {
			scaleEnable = false;
			holdContinue = false;
			holdIt = holder.STOP;
			holdDown = intakeDown.IDLE;
			scaleDo1 = 	scaleDo.POS0;
		}
	}
	
	public void pressLeftTrigger() {
		shootakeTalon1.set(ControlMode.PercentOutput, shootPower * ioo);
		shootakeTalon2.set(ControlMode.PercentOutput, -shootPower * ioo);
	}
	
	public void releaseLeftTrigger() {
		shootakeTalon1.set(ControlMode.PercentOutput, 0);
		shootakeTalon2.set(ControlMode.PercentOutput, 0);
	}
	
	public boolean burning() {
		return ((Math.abs(armTalon1.getOutputCurrent()) > threshold) ||
				(Math.abs(armTalon2.getOutputCurrent()) > threshold));
	}
	
	public void ESTOP() {
		stop = true;
	}
}