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
	private Solenoid activeIntake;
	private Joystick player1;
	private Joystick player2;
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	private VictorSPX shootakeTalon1;
	private VictorSPX shootakeTalon2;
	private AnalogPotentiometer fourtwenty;
	private DigitalInput breakbeam;
	private boolean isAlreadyTriggered = false;
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
	private final double threshold = 25.0;
	private boolean alreadyHot = false;
	private long burnEndTime;
	private TorDerivative scaleDerivative;
	private TorDerivative switchDerivative;
	private boolean auto = true;

	public static enum switchDo {
		IDLE, POS0, POS1, POS2, PID, POS4, POS5;
		private switchDo() {}
	}

	public static enum scaleDo {
		IDLE, POS0, POS1, POS2, PID, POS4, POS5;
		private scaleDo() {}
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
		IDLE, START, PD, SWITCHPOS4, SWITCHPOS5, SCALEPOS4, SCALEPOS5;
		private intakeDown() {}
	}

	public switchDo switchDo1 = switchDo.IDLE;
	public scaleDo scaleDo1 = scaleDo.IDLE;
	public shoot shootIt = shoot.IDLE;
	public intake intakeIt = intake.IDLE;
	public holder holdIt = holder.START;
	public manual manualGo = manual.IDLE;
	public intakeDown holdDown = intakeDown.IDLE;
	private double speed;

	/**********************************
	 TUNABLE ARM VARIABLES
	 */ 

	private double startAngle;//HAS TO BE TUNED TO THE POTENTIOMETER READING WHEN IT IS ALREADY DOWN

	private int uod = 1; // up or down for manual override. Change it from 1 to -1 to change the control on the arm with the right player y
	private int ioo = -1; // in or out variable. Change this to switch around the outtake and intake when it is up. Change it from 1 to -1 to make it so either shoots out or (not wanted) intakes up there
	private double manualMax = 0.6; // POSITIVE. The controls on the speeds for the manual override.
	private double manualMin = -0.45; // HAS TO BE NEGATIVE!

	// Switch Variables
	private long switchPos1Time = 300; // change this to make it go up higher during the switch
	private long switchPos2Time = 100; // this is the time it is at the max speed
	private double switchMaxSpeed = 0.9; // the acceleration increment for the switch
	private double switchCushion = 0; // NEGATIVE the extra cushion from a proportional down 
	private double switchShootPower = 0.25;//0.25

	// Scale Variables
	private long scalePos1Time = 350;
	private long scalePos2Time = 25;
	private double scaleMaxSpeed = 1;
	private double scaleCushion = -0.02;
	private double scaleShootPower = 1.0;
	private final double scaleHighShootPower = 0.95;//0.95
	private final double scaleMediumShootPower = 0.75;//0.75
	private final double scaleLowShootPower = 0.6;//0.6

	// Shooter & Intake Variables
	private double shootPower = 0.2;//the power it shoots out at
	private long extendTime = 300;//the time we give to the solenoid to extend and push the cube
	private double intakePower = 0.6;
	private double downCushion = 0.3;

	// Hold PD Constants
	private double holdkP = 0.05;
	private double holdkD = 0.00001;

	private double stallPower = 0.4;

	private int potSwitch = 1;//change this from 1 to 
	//	-1 to control the pot. (pot might be mounted on backwards)

	private double vaultShootPower = 0.8;
	private double switchCurrentRatio;
	private double scaleCurrentRatio;

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
	private double scalekI;
	private double scaleIntegral = 0;

	public TorBantorShooarm(Joystick player1, Joystick player2, TalonSRX armTalon1, TalonSRX armTalon2, 
			VictorSPX shootakeTalon1, VictorSPX shootakeTalon2, 
			DigitalInput breakbeam, AnalogPotentiometer fourtwenty, 
			double scaleAngle, double switchAngle, double degreeTolerance, 
			double kF, double kP, double kD, double holdAngle, 
			Solenoid Pusher, Solenoid activeIntake, double scalekI) {	
		this.player1 = player1;
		this.player2 = player2;
		this.armTalon1 = armTalon1;
		this.armTalon2 = armTalon2;
		this.shootakeTalon1 = shootakeTalon1;
		this.shootakeTalon2 = shootakeTalon2;
		this.Pusher = Pusher;
		this.breakbeam = breakbeam;
		this.fourtwenty = fourtwenty;
		startAngle = 78.42653128410;//MAKE THIS WHEN THE ARM IS FLAT
		this.scaleAngle = scaleAngle;
		this.switchAngle = switchAngle;
		this.degreeTolerance = degreeTolerance;
		this.kF = kF;
		this.kP = kP;
		this.kD = kD;
		this.scalekI = scalekI;
		this.holdAngle = holdAngle;
		this.activeIntake = activeIntake;
		pressingRightTrigger = false;
		Pusher.set(false);
		scaleDerivative = new TorDerivative(0.005);
		switchDerivative = new TorDerivative(0.005);
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
		SmartDashboard.putBoolean("SwitchDo Idle?:", switchDo1 == switchDo.IDLE);
		SmartDashboard.putBoolean("ScaleDoIdle?:", scaleDo1 == scaleDo.IDLE);
		SmartDashboard.putBoolean("HoldDo Idle?:", holdIt == holder.STOP);
		SmartDashboard.putBoolean("HoldDown Idle?:", holdDown == intakeDown.IDLE);
		if(stop) {
			armTalon1.set(ControlMode.PercentOutput, 0);
			armTalon2.set(ControlMode.PercentOutput, 0);
			shootakeTalon1.set(ControlMode.PercentOutput, 0);
			shootakeTalon2.set(ControlMode.PercentOutput, 0);
		} else {
			switchDo(); 	  // Update for the switch
			scaleDo(); 		  // Update for the scale
			shoot(); 		  // Update for the shooter
			intake(); 		  // Update for the intake
			Hold(); 		  // Update for the hold
			manualoverride(); // Update for the manual override
			holdDownUpdate(); // Update for the hold down
			shootTake();
			burnUpdate();
			
			//for the active intake
			if((switchDo1 == switchDo.IDLE && scaleDo1 == scaleDo.IDLE && !(holdIt == holder.PD) && !breakbeam.get()) && !player2.getRawButton(2)) {
				activeIntake.set(true);//open
			} else {
				activeIntake.set(false);//close
			}

			if(intakeIt == intake.IDLE) {
				holdDown = intakeDown.IDLE;
			}

			SmartDashboard.putNumber("left output current:", armTalon1.getOutputCurrent());
			SmartDashboard.putNumber("right output current:", armTalon2.getOutputCurrent());

			// Activate switch if button 'X' is pressed
			if(!stop && player2.getRawButton(3)
					&& !player2.getRawButton(5)
					&& (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID)){
				scaleDo1 = scaleDo.IDLE;
				switchEnable = false;
				holdContinue = false;
				holdIt = holder.STOP;
				intakeIt = intake.IDLE;
				holdDown = intakeDown.IDLE;
				shootIt = shoot.IDLE;
				switchDo1 = switchDo.POS0;
			}
			
			// Activate scale if button 'Y' is pressed
			if(!stop && player2.getRawButton(4)
					&& !player2.getRawButton(5) && (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID)) {
				switchDo1 = switchDo.IDLE;
				scaleEnable = false;
				holdContinue = false;
				holdIt = holder.STOP;
				intakeIt = intake.IDLE;
				holdDown = intakeDown.IDLE;
				shootIt = shoot.IDLE;
				scaleDo1 = 	scaleDo.POS0;
			}

			// Activate intake if button 'A' is pressed
			if(!stop && player2.getRawButton(1) && intakeIt == intake.IDLE) {
				scaleEnable = false;
				switchDo1 = switchDo.IDLE;
				scaleDo1 = scaleDo.IDLE;
				intakeIt = intake.IDLE;
				shootIt = shoot.IDLE;
				intakeIt = intake.POS0;
			}

			if(!stop && 
					(Math.abs(player1.getRawAxis(3)) > 0.2) || Math.abs(player2.getRawAxis(3)) > .2
					&& (switchDo1 == switchDo.IDLE || switchDo1 == switchDo.PID) 
					&& (scaleDo1 == scaleDo.IDLE || scaleDo1 == scaleDo.PID)
					&& shootIt == shoot.IDLE) {
				shootIt = shoot.POS0;
			}
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
			shootakeTalon1.set(ControlMode.PercentOutput, 0);
			shootakeTalon2.set(ControlMode.PercentOutput, 0);
			speed = 0;
			startTime = System.currentTimeMillis();

			if(((fourtwenty.get() - startAngle) * potSwitch) > (scaleAngle - degreeTolerance)) {
				lastError = 0;
				switchEnable = true;
				shootPower = switchShootPower;
				switchDerivative.resetValue(switchAngle - ((fourtwenty.get() - startAngle) * potSwitch));
				switchDo1 = switchDo.PID;
				//wants to go down from scale to switch
			} else if(((fourtwenty.get() - startAngle) * potSwitch) > (switchAngle - 2 * degreeTolerance)) {
				//is at switch and wants to go down
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.POS4;
			} else {
				//wants to go up to switch
				switchCurrentRatio = (switchAngle - ((fourtwenty.get() - startAngle) * potSwitch)) / switchAngle;
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.POS1;
			}
			break;
		case POS1:
			shootPower = switchShootPower;
			if(((fourtwenty.get() - startAngle) * potSwitch) > switchAngle) {
				lastError = 0;
				switchEnable = true;
				shootPower = switchShootPower;
				switchDerivative.resetValue(switchAngle - ((fourtwenty.get() - startAngle) * potSwitch));
				switchDo1 = switchDo.PID;
			}
			speed = (x + Math.sin(x) + Math.PI) * switchMaxSpeed / (2 * Math.PI);
			speed *= switchCurrentRatio;
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos2Time;
				switchDo1 = switchDo.POS2;
			}
			break;
		case POS2:
			if(((fourtwenty.get() - startAngle) * potSwitch) > switchAngle) {
				lastError = 0;
				switchEnable = true;
				shootPower = switchShootPower;
				switchDerivative.resetValue(switchAngle - ((fourtwenty.get() - startAngle) * potSwitch));
				switchDo1 = switchDo.PID;
			}
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				lastError = 0;
				switchEnable = true;
				shootPower = switchShootPower;
				switchDerivative.resetValue(switchAngle - ((fourtwenty.get() - startAngle) * potSwitch));
				switchDo1 = switchDo.PID;
			}
			break;
		case PID:
			if(switchEnable) {
				shootPower = switchShootPower;
				switchPIDGO();
				switchDo1 = switchDo.PID;
			} else {
				startTime = System.currentTimeMillis();
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.IDLE;
			}
			break;
		case POS4:
			if(((fourtwenty.get() - startAngle) * potSwitch) < holdAngle) {
				switchDo1 = switchDo.IDLE;
				holdIt = holder.START;
			}
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
			if(((fourtwenty.get() - startAngle) * potSwitch) < holdAngle) {
				switchDo1 = switchDo.IDLE;
				holdIt = holder.START;
			}
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
			shootakeTalon1.set(ControlMode.PercentOutput, 0);
			shootakeTalon2.set(ControlMode.PercentOutput, 0);
			speed = 0;
			startTime = System.currentTimeMillis();
			if((((fourtwenty.get() - startAngle) * potSwitch)) > (scaleAngle - (degreeTolerance))) {
				endTime = currentTime + scalePos1Time;
				//wants to go down from scale
				scaleDo1 = scaleDo.POS4;
			} else if((((fourtwenty.get() - startAngle) * potSwitch)) > (switchAngle - (2 * degreeTolerance))) {
				//wants to go from switch to scale
				scaleCurrentRatio = (scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch)) / scaleAngle;
				endTime = currentTime + scalePos1Time;
				scaleDo1 = scaleDo.POS1;
			} else {
				//wants to go up to scale
				scaleCurrentRatio = (scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch)) / scaleAngle;
				endTime = currentTime + scalePos1Time;
				scaleDo1 = scaleDo.POS1;
			}
			break;
		case POS1:
			shootPower = scaleShootPower;
			if(((fourtwenty.get() - startAngle) * potSwitch) > scaleAngle) {
				scaleIntegral = 0;
				lastError = 0;
				scaleEnable = true;
				scaleShootPower = scaleLowShootPower;
				shootPower = scaleShootPower;
				scaleDerivative.resetValue(scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch));
				scaleDo1 = scaleDo.PID;
			}
			speed = (Math.sin(x) + x + Math.PI) * scaleMaxSpeed / (2 * Math.PI);
			speed *= scaleCurrentRatio;
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				scaleDo1 = scaleDo.POS2;
			}
			break;
		case POS2:
			if(((fourtwenty.get() - startAngle) * potSwitch) > scaleAngle) {
				scaleIntegral = 0;
				lastError = 0;
				scaleEnable = true;
				scaleShootPower = scaleLowShootPower;
				shootPower = scaleShootPower;
				scaleDerivative.resetValue(scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch));
				scaleDo1 = scaleDo.PID;
			}
			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
			armTalon2.set(ControlMode.PercentOutput, speed * uod);
			if(currentTime >= endTime) {
				scaleIntegral = 0;
				lastError = 0;
				scaleEnable = true;
				scaleShootPower = scaleLowShootPower;
				shootPower = scaleShootPower;
				scaleDerivative.resetValue(scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch));
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
			if(((fourtwenty.get() - startAngle) * potSwitch) < holdAngle) { //|| hittingDown.get()) {
				scaleDo1 = scaleDo.IDLE;
				holdIt = holder.START;
			}
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
			if(((fourtwenty.get() - startAngle) * potSwitch) < holdAngle) { //|| hittingDown.get()) {
				scaleDo1 = scaleDo.IDLE;
				holdIt = holder.START;
			}
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
			if(((currentTime - lastTime) >= 400) || auto) {
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
				endTime = currentTime + 500;
				intakeIt = intake.MOTORIN;
			}
			break;
		case MOTORIN:
			currentTime = System.currentTimeMillis();
			if(!pressingRightTrigger) {
				if(!player2.getRawButton(2)) {
					shootakeTalon1.set(ControlMode.PercentOutput, -intakePower * ioo);
					shootakeTalon2.set(ControlMode.PercentOutput, intakePower * ioo);

				} else {
					shootakeTalon1.set(ControlMode.PercentOutput, -(intakePower) * ioo);
					shootakeTalon2.set(ControlMode.PercentOutput, -(intakePower) * ioo);

				}
			}
			if(((currentTime >= endTime) && (player2.getRawButton(1))) || breakbeam.get()) {
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
		derivative = kD * switchDerivative.estimate(error);
		velocity = proportional + derivative;
		armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
		armTalon2.set(ControlMode.PercentOutput, velocity * uod);
		lastError = error;
	}

	public void scalePIDGO() {
		currentAngle = (((fourtwenty.get() - startAngle) * potSwitch));
		error = scaleAngle - currentAngle;
		proportional = kP * error;
		derivative = kD * scaleDerivative.estimate(error);
		scaleIntegral += error;
		if((scaleIntegral * scalekI) > 0.8) {
			scaleIntegral = (0.8 / scalekI);
		}
		if(Math.abs(error) < 2.5) {
			scaleIntegral = 0;
		}
		velocity = proportional + derivative + (scaleIntegral * scalekI);
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
			if(scaleDo1 == scaleDo.IDLE && switchDo1 == switchDo.IDLE) {
				shootPower = vaultShootPower;
			}
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
		if(player2.getRawButton(10)) {
			isAlreadyTriggered = true;
			switchEnable = false;
			scaleEnable = false;
			switchDo1 = switchDo.IDLE;
			scaleDo1 = scaleDo.IDLE;
			holdIt = holder.STOP;
			intakeIt = intake.IDLE;
			holdDown = intakeDown.IDLE;
			armAxis = player2.getRawAxis(5);
			if(Math.abs(armAxis) < 0.15) {
				armAxis = 0;
			}
			armAxis *= -1;
			armAxis *= armAxis * armAxis;
			velocity = armAxis * .45;
			SmartDashboard.putNumber("Velocity:", velocity);
			armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
			armTalon2.set(ControlMode.PercentOutput, velocity * uod);
		}
		else if(isAlreadyTriggered) {
			holdIt = holder.START;
			isAlreadyTriggered = false;
		}
		//		switch(manualGo) {
		//		case IDLE:
		//			if(player2.getRawButton(5)) {		
		//				lastError = 0;
		//				wantedAngle = ((fourtwenty.get() - startAngle) * potSwitch);
		//				manualGo = manual.GOING;
		//			}
		//			break;
		//		case GOING:
		//			switchEnable = false;
		//			scaleEnable = false;;
		//			switchDo1 = switchDo.IDLE;
		//			scaleDo1 = scaleDo.IDLE;
		//			holdIt = holder.STOP;
		//			intakeIt = intake.IDLE;
		//			holdDown = intakeDown.IDLE;
		//			if(player2.getRawButton(5)) {
		//				armAxis = player2.getRawAxis(5);
		//				if(Math.abs(armAxis) < 0.15) {
		//					armAxis = 0;
		//				}
		//				armAxis *= armAxis * armAxis;
		//				wantedAngle += (kF * armAxis);
		//				if(wantedAngle < 0) {
		//					wantedAngle = 0;
		//				}
		//				if(wantedAngle > manualMaxAngle) {
		//					wantedAngle = manualMaxAngle;
		//				}
		//				error = wantedAngle - ((fourtwenty.get() - startAngle) * potSwitch);
		//				proportional = kP * error;
		//				derivative = (kD * (error - lastError)) / kF;
		//				velocity = proportional + derivative;
		//				if(velocity > manualMax) {
		//					velocity = manualMax;
		//				}
		//				if (velocity < manualMin) {
		//					velocity = manualMin;
		//				}
		//				SmartDashboard.putNumber("Velocity:", velocity);
		//				armTalon1.set(ControlMode.PercentOutput, -velocity * uod);
		//				armTalon2.set(ControlMode.PercentOutput, velocity * uod);
		//				lastError = error;
		//			} else {
		//				lastAngle = (((fourtwenty.get() - startAngle) * potSwitch) - holdAngle);
		//				manualGo = manual.GOINGDOWN;
		//			}
		//			break;
		//		case GOINGDOWN:
		//			speed = (manualMin * ((((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) / lastAngle));
		//			if(speed > manualMax) {
		//				speed = manualMax;
		//			}
		//			if(speed < manualMin) {
		//				speed = manualMin;
		//			}
		//			armTalon1.set(ControlMode.PercentOutput, -speed * uod);
		//			armTalon2.set(ControlMode.PercentOutput, speed * uod);
		//			if(Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) <= degreeTolerance) {
		//				armTalon1.set(ControlMode.PercentOutput, 0);
		//				armTalon2.set(ControlMode.PercentOutput, 0);
		//				holdIt = holder.START;
		//				manualGo = manual.IDLE;
		//			}
		//			break;
		//		}
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
			startTime = System.currentTimeMillis();
			if(((fourtwenty.get() - startAngle) * potSwitch) > (scaleAngle - degreeTolerance)) {
				endTime = currentTime + scalePos1Time;
				holdDown = intakeDown.SCALEPOS4;
			} else if(((fourtwenty.get() - startAngle) * potSwitch) > (switchAngle - degreeTolerance)) {
				endTime = currentTime + switchPos1Time;
				holdDown = intakeDown.SWITCHPOS4;
			} else {
				holdDown = intakeDown.PD;
			}

			break;
		case PD:
			if(Math.abs(startAngle - fourtwenty.get()) < 1) {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				holdDown = intakeDown.IDLE;
			} else {
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
			}

			break;
		case SWITCHPOS4:
			currentTime = System.currentTimeMillis();
			relativeTime = currentTime - startTime;
			x = ((relativeTime * 2 * Math.PI) / switchPos1Time) - Math.PI;


			if(((fourtwenty.get() - startAngle) * potSwitch) < holdAngle) {
				holdDown = intakeDown.PD;
			}
			speed = (Math.sin(x) + x + Math.PI) * switchMaxSpeed / (2 * Math.PI);
			if(speed < manualMin) {
				speed = manualMin;
			}
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos2Time;
				lastAngle = ((fourtwenty.get() - startAngle) * potSwitch) - holdAngle;
				holdDown = intakeDown.SWITCHPOS5;
			}
			break;
		case SWITCHPOS5:
			currentTime = System.currentTimeMillis();

			armTalon1.set(ControlMode.PercentOutput, (speed - downCushion) * uod);
			armTalon2.set(ControlMode.PercentOutput, -(speed - downCushion) * uod);

			if(currentTime > endTime) {
				holdDown = intakeDown.PD;
			}
			break;
		case SCALEPOS4:
			currentTime = System.currentTimeMillis();
			relativeTime = currentTime - startTime;
			x = ((relativeTime * 2 * Math.PI) / scalePos1Time) - Math.PI;

			if(((fourtwenty.get() - startAngle) * potSwitch) < holdAngle) {
				holdDown = intakeDown.PD;
			}
			speed = (Math.sin(x) + x + Math.PI) * scaleMaxSpeed / (2 * Math.PI);
			if(speed < manualMin) {
				speed = manualMin;
			}
			speed -= downCushion;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				lastAngle = ((fourtwenty.get() - startAngle) * potSwitch) - holdAngle;
				holdDown = intakeDown.SCALEPOS5;
			}
			break;
		case SCALEPOS5:
			currentTime = System.currentTimeMillis();

			armTalon1.set(ControlMode.PercentOutput, (speed - downCushion) * uod);
			armTalon2.set(ControlMode.PercentOutput, -(speed - downCushion) * uod);

			if(currentTime > endTime) {
				holdDown = intakeDown.PD;
			}
			break;
		}
	}





	public void shootTake() {
		if(!(intakeIt == intake.MOTORIN)) {
			if(Math.abs(player2.getRawAxis(2)) > 0.3) {
				pressingRightTrigger = true;

				shootakeTalon1.set(ControlMode.PercentOutput, shootPower * ioo);
				shootakeTalon2.set(ControlMode.PercentOutput, -shootPower * ioo);
			} else if(player2.getRawButton(5) && scaleDo1 == scaleDo.PID) {
				pressingRightTrigger = true;

				shootakeTalon1.set(ControlMode.PercentOutput, scaleHighShootPower * ioo);
				shootakeTalon2.set(ControlMode.PercentOutput, -scaleHighShootPower * ioo);

			} else if(player2.getRawButton(6) && scaleDo1 == scaleDo.PID) {
				pressingRightTrigger = true;

				shootakeTalon1.set(ControlMode.PercentOutput, scaleMediumShootPower * ioo);
				shootakeTalon2.set(ControlMode.PercentOutput, -scaleMediumShootPower * ioo);
			} else if(pressingRightTrigger) {
				shootakeTalon1.set(ControlMode.PercentOutput, 0);
				shootakeTalon2.set(ControlMode.PercentOutput, 0);
				pressingRightTrigger = false;
			}
		}
		else {
			if(Math.abs(player2.getRawAxis(2)) > 0.3) {
				pressingRightTrigger = true;
				shootakeTalon1.set(ControlMode.PercentOutput, stallPower * ioo);
				shootakeTalon2.set(ControlMode.PercentOutput, -stallPower * ioo);
			} else if(pressingRightTrigger) {
				shootakeTalon1.set(ControlMode.PercentOutput, 0);
				shootakeTalon2.set(ControlMode.PercentOutput, 0);
				pressingRightTrigger = false;
			}
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
		System.out.println("PRESS X");
		holdContinue = false;
		holdIt = holder.STOP;
		switchEnable = true;
		scaleEnable = false;
		switchDo1 = switchDo.POS0;
	}

	public void pressY() {
		holdContinue = false;
		holdIt = holder.STOP;
		switchEnable = false;
		scaleEnable = true;
		scaleDo1 = 	scaleDo.POS0;
	}

	public void pressYStart() {
		scaleIntegral = 0;
		holdContinue = false;
		holdIt = holder.STOP;
		switchEnable = false;
		scaleEnable = true;
		scaleDerivative.resetValue(scaleAngle - ((fourtwenty.get() - startAngle) * potSwitch));
		scaleDo1 = 	scaleDo.PID;
	}

	public void pressXStart() {
		holdContinue = false;
		holdIt = holder.STOP;
		switchEnable = true;
		scaleEnable = false;
		switchDerivative.resetValue(switchAngle - ((fourtwenty.get() - startAngle) * potSwitch));
		switchDo1 = switchDo.PID;
	}

	public void pressLeftTrigger() {
		shootakeTalon1.set(ControlMode.PercentOutput, shootPower * ioo);
		shootakeTalon2.set(ControlMode.PercentOutput, -shootPower * ioo);
	}
	
	public void pressLeftTriggerControl(double power) {
		shootakeTalon1.set(ControlMode.PercentOutput, power * ioo);
		shootakeTalon2.set(ControlMode.PercentOutput, -power * ioo);
	}

	public void releaseLeftTrigger() {
		shootakeTalon1.set(ControlMode.PercentOutput, 0);
		shootakeTalon2.set(ControlMode.PercentOutput, 0);
	}

	public void burnUpdate() {
		if((Math.abs(armTalon1.getOutputCurrent()) > threshold) ||
				(Math.abs(armTalon2.getOutputCurrent()) > threshold) && !alreadyHot) {
			alreadyHot = true;
			burnEndTime = System.currentTimeMillis() + 1000;
		};
		if(alreadyHot && (Math.abs(armTalon1.getOutputCurrent()) > threshold) ||
				(Math.abs(armTalon2.getOutputCurrent()) > threshold) && (System.currentTimeMillis() > burnEndTime)) {
			STOPPROCESS();
		}
	}

	public void switchShoot() {//for auto switch shoot is going to be 0.6
		shootPower = 0.30;
	}

	public void scaleShoot() {
		shootPower = scaleHighShootPower;
	}

	public boolean inScale() {
		return (Math.abs((((fourtwenty.get() - startAngle) * potSwitch) - scaleAngle)) <= degreeTolerance * 0.2);
	}

	public boolean inSwitch() {
		return (Math.abs((((fourtwenty.get() - startAngle) * potSwitch) - switchAngle)) <= degreeTolerance);
	}

	public void STOPPROCESS() {	
		switchDo1 = switchDo.IDLE;
		scaleDo1 = scaleDo.IDLE;
		intakeIt = intake.IDLE;
		holdIt = holder.STOP;
		manualGo = manual.IDLE;
		holdDown = intakeDown.IDLE;
		armTalon1.set(ControlMode.PercentOutput, 0);
		armTalon2.set(ControlMode.PercentOutput, 0);
		shootakeTalon1.set(ControlMode.PercentOutput, 0);
		shootakeTalon2.set(ControlMode.PercentOutput, 0);
	}


	public boolean isHold() {
		return (Math.abs(((fourtwenty.get() - startAngle) * potSwitch) - holdAngle) <= (degreeTolerance * 0.4));
	}

	public void pressA() {
		shootakeTalon1.set(ControlMode.PercentOutput, -intakePower * ioo);
		shootakeTalon2.set(ControlMode.PercentOutput, intakePower * ioo);
		scaleEnable = false;
		switchDo1 = switchDo.IDLE;
		scaleDo1 = scaleDo.IDLE;
		intakeIt = intake.IDLE;
		shootIt = shoot.IDLE;
		intakeIt = intake.POS0;
	}

	public void setAutoIntake(double power) {
		intakePower = power;
	}

	public boolean isIntake() {
		return (Math.abs(fourtwenty.get() - startAngle) <= (degreeTolerance * 0.1));
	}

	public boolean inHold() {
		return (holdIt == holder.PD);
	}

	public boolean isInside() {
		return breakbeam.get();
	}
	public void shootIdle() {
		shootIt = shoot.IDLE;
	}
	public void autoSet(boolean set) {
		auto = set;
	}
}