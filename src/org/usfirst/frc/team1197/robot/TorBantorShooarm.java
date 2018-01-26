package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;

public class TorBantorShooarm {
	private Joystick player2;
	private TalonSRX armTalon1;
	private TalonSRX armTalon2;
	private TalonSRX shootakeTalon1;
	private TalonSRX shootakeTalon2;
	private Solenoid Pusher;
	
	public static enum switchDo {
		IDLE, POS0, POS1, POS2, POS3, MOTOROUT, EXTEND, RETRACT, MOTORSTOP, POS4, POS5, POS6;
		private switchDo() {}
	}
	
	public static enum scaleDo {
		IDLE, POS0, POS1, POS2, POS3, MOTOROUT, EXTEND, RETRACT, MOTORSTOP, POS4, POS5, POS6;
		private scaleDo() {}
	}
	
	public switchDo switchDo1 = switchDo.IDLE;
	public scaleDo scaleDo1 = scaleDo.IDLE;
	private double speed;
	
	/**********************************
	 TUNE THE ARM FROM HERE
	 */
	private int uod = 1;//up or down for manuel override. Change it from 1 to -1 to change the control on the arm with the right player y
	private int ioo = 1;//in or out variable. Change this to switch around the outtake and intake when it is up. Change it from 1 to -1 to make it so either shoots out or (not wanted) intakes up there
	//ioo is for pressing the BUTTON NOT MANUEL CONTROL
	
	private int ioop = -1;//in or our variable for the player under manuel control
	
	//the switch tunes
	private long switchPos1Time = 400;//change this to make it go up higher during the switch
	private long switchPos2Time = 400;//this is the time it is at the max speed
	private double switchIncrement1 = .0001;//the acceleration increment for the switch
	private double switchIncrement2 = .00005;//the deacceleration increment for the switch
	private double switchOutput = 0.5;//the power to shoot into the switch
	
	//the scale tunes
	private long scalePos1Time = 600;
	private long scalePos2Time = 600;
	private double scaleIncrement1 = .0001;
	private double scaleIncrement2 = .00005;
	private double scaleOutput = 1;//the power to shoot out into the scale
	
	//the shooting tunes
	private long extendTime = 600;//the time we give to the solenoid to extend and push the cube
	private long revTime = 500;//the time we give for the motors to go from 0 to the speed and back
	
	/**********************************
		TO HERE
	*/
	
	private long switchPos3Time = (long)Math.sqrt(((double)switchPos1Time * (double)switchPos1Time * (double)switchIncrement1 / (double)switchIncrement2));//these two lines make sure it goes down as much as it went up
	private long scalePos3Time = (long)Math.sqrt(((double)scalePos1Time * (double)switchPos1Time * (double)scaleIncrement1 / (double)scaleIncrement2));
	private long currentTime;
	private long endTime;
	
	public TorBantorShooarm(Joystick player2, TalonSRX armTalon1, TalonSRX armTalon2, TalonSRX shootakeTalon1, TalonSRX shootakeTalon2, Solenoid Pusher) {
		this.player2 = player2;
		this.armTalon1 = armTalon1;
		this.armTalon2 = armTalon2;
		this.shootakeTalon1 = shootakeTalon1;
		this.shootakeTalon2 = shootakeTalon2;
		this.Pusher = Pusher;
	}
	public void TorBantorArmAndShooterUpdate() {
		switchDo();
		scaleDo();
		if(player2.getRawButton(0) && switchDo1 == switchDo.IDLE && scaleDo1 == scaleDo.IDLE && !player2.getRawButton(5)) {
			switchDo1 = switchDo.POS0;
		}
		if(player2.getRawButton(1) && switchDo1 == switchDo.IDLE && scaleDo1 == scaleDo.IDLE && !player2.getRawButton(5)) {
			scaleDo1 = scaleDo.POS0;
		}
		manueloverride();
	}
	public void switchDo() {
		switch(switchDo1) {
		case IDLE:
			break;
		case POS0:
			currentTime = System.currentTimeMillis();
			endTime = currentTime + switchPos1Time;
			speed = 0;
			switchDo1 = switchDo.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			speed += switchIncrement1;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos2Time;
				switchDo1 = switchDo.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos3Time;
				switchDo1 = switchDo.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			speed -= switchIncrement2;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				speed = 0;
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				endTime = currentTime + revTime;
				switchDo1 = switchDo.MOTOROUT;
			}
			break;
		case MOTOROUT:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, switchOutput * ioo);
			shootakeTalon2.set(ControlMode.PercentOutput, -switchOutput * ioo);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				switchDo1 = switchDo.EXTEND;
			}
			break;
		case EXTEND:
			currentTime = System.currentTimeMillis();
			Pusher.set(true);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				switchDo1 = switchDo.RETRACT;
			}
			break;
		case RETRACT:
			currentTime = System.currentTimeMillis();
			Pusher.set(false);
			if(currentTime >= endTime) {
				endTime = currentTime + revTime;
				switchDo1 = switchDo.MOTORSTOP;
			}
			break;
		case MOTORSTOP:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, 0);
			shootakeTalon2.set(ControlMode.PercentOutput, 0);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos1Time;
				switchDo1 = switchDo.POS4;
			}
			break;
		case POS4:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			speed -= switchIncrement1;
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos2Time;
				switchDo1 = switchDo.POS5;
			}
			break;
		case POS5:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + switchPos3Time;
				switchDo1 = switchDo.POS6;
			}
		case POS6:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			speed += switchIncrement2;
			if(currentTime >= endTime) {
				switchDo1 = switchDo.IDLE;
			}
			break;
		}
	}
	public void scaleDo() {
		switch(scaleDo1) {
		case IDLE:
			break;
		case POS0:
			currentTime = System.currentTimeMillis();
			endTime = currentTime + scalePos1Time;
			speed = 0;
			scaleDo1 = scaleDo.POS1;
			break;
		case POS1:
			currentTime = System.currentTimeMillis();
			speed += scaleIncrement1;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				scaleDo1 = scaleDo.POS2;
			}
			break;
		case POS2:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos3Time;
				scaleDo1 = scaleDo.POS3;
			}
			break;
		case POS3:
			currentTime = System.currentTimeMillis();
			speed -= scaleIncrement2;
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				speed = 0;
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
				endTime = currentTime + revTime;
				scaleDo1 = scaleDo.MOTOROUT;
			}
			break;
		case MOTOROUT:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, scaleOutput * ioo);
			shootakeTalon2.set(ControlMode.PercentOutput, -scaleOutput * ioo);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				scaleDo1 = scaleDo.EXTEND;
			}
			break;
		case EXTEND:
			currentTime = System.currentTimeMillis();
			Pusher.set(true);
			if(currentTime >= endTime) {
				endTime = currentTime + extendTime;
				scaleDo1 = scaleDo.RETRACT;
			}
			break;
		case RETRACT:
			currentTime = System.currentTimeMillis();
			Pusher.set(false);
			if(currentTime >= endTime) {
				endTime = currentTime + revTime;
				scaleDo1 = scaleDo.MOTORSTOP;
			}
			break;
		case MOTORSTOP:
			currentTime = System.currentTimeMillis();
			shootakeTalon1.set(ControlMode.PercentOutput, 0);
			shootakeTalon2.set(ControlMode.PercentOutput, 0);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos1Time;
				scaleDo1 = scaleDo.POS4;
			}
			break;
		case POS4:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			speed -= scaleIncrement1;
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos2Time;
				scaleDo1 = scaleDo.POS5;
			}
			break;
		case POS5:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			if(currentTime >= endTime) {
				endTime = currentTime + scalePos3Time;
				scaleDo1 = scaleDo.POS6;
			}
		case POS6:
			currentTime = System.currentTimeMillis();
			armTalon1.set(ControlMode.PercentOutput, speed * uod);
			armTalon2.set(ControlMode.PercentOutput, -speed * uod);
			speed += scaleIncrement2;
			if(currentTime >= endTime) {
				scaleDo1 = scaleDo.IDLE;
			}
			break;
		}
		
	}
	
	public void manueloverride() {
		if(player2.getRawButton(5)) {
			switchDo1 = switchDo.IDLE;
			scaleDo1 = scaleDo.IDLE;
			if(Math.abs(player2.getRawAxis(1)) > 0.2) {
				armTalon1.set(ControlMode.PercentOutput, player2.getRawAxis(1) * uod);
				armTalon2.set(ControlMode.PercentOutput, -player2.getRawAxis(1) * uod);
			} else {
				armTalon1.set(ControlMode.PercentOutput, 0);
				armTalon2.set(ControlMode.PercentOutput, 0);
			}
			if(player2.getRawButton(6)) {
				Pusher.set(true);
			} else {
				Pusher.set(false);
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
}
