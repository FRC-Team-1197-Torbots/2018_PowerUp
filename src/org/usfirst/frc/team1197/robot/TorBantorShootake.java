package org.usfirst.frc.team1197.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;

public class TorBantorShootake {
	private Joystick player2;
	private TalonSRX shootakeTalon1;
	private TalonSRX shootakeTalon2;
	
	public TorBantorShootake(Joystick player2, TalonSRX shootakeTalon1, TalonSRX shootakeTalon2) {
		this.player2 = player2;
		this.shootakeTalon1 = shootakeTalon1;
		this.shootakeTalon2 = shootakeTalon2;
	}
	
	public void shootakeUpdate() {
		if(player2.getRawButton(6)) {
			intake();
		}
		else if(player2.getRawAxis(3) >= 0.95) {
			switchShoot();
		}
		else if(player2.getRawAxis(2) >= 0.95) {
			scaleShoot();
		}
		else {
			shootakeOff();
		}
	}
	
	public void intake() {
		shootakeTalon1.set(ControlMode.PercentOutput, 0.75);
		shootakeTalon2.set(ControlMode.PercentOutput, 0.75);
	}
	
	public void switchShoot() {
		shootakeTalon1.set(ControlMode.PercentOutput, -0.50);
		shootakeTalon2.set(ControlMode.PercentOutput, -0.50);
	}
	
	public void scaleShoot() {
		shootakeTalon1.set(ControlMode.PercentOutput, -0.75);
		shootakeTalon2.set(ControlMode.PercentOutput, -0.75);
	}
	
	public void shootakeOff() {
		shootakeTalon1.set(ControlMode.PercentOutput, 0.0);
		shootakeTalon2.set(ControlMode.PercentOutput, 0.0);
	}
}