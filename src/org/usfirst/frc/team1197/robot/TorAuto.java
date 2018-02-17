package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {
	private CenterLeftRun CenterLeftRun;
	private CenterRightRun CenterRightRun;
	private Joystick Autobox;
	private String gameData;
	private boolean alreadyStarted = false;
	private int position;
	private TorBantorShooarm shooArm;
	
	public TorAuto(TorDrive drive, Joystick Autobox, TorBantorShooarm shooArm) {
		this.Autobox = Autobox;
		this.shooArm = shooArm;
		
		CenterLeftRun = new CenterLeftRun(drive, shooArm);
		CenterRightRun = new CenterRightRun(drive, shooArm);
	}
	
	public void oneSwitchRun() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(Autobox.getRawButton(3) && Autobox.getRawButton(1)) { // left trajectories
			if(gameData.charAt(0) == 'L') { // left left
				position = 0;
			} 
			else { // left right
				position = 1;
			}
		} 
		else if(Autobox.getRawButton(2) && Autobox.getRawButton(1)) { // right trajectories
			if(gameData.charAt(0) == 'L') { // right left
				position = 2;
			} 
			else {// right right
				position = 3;
			}
		} 
		else if(Autobox.getRawButton(1)) { // center trajectories
			if(gameData.charAt(0) == 'L') { // center left
				CenterLeftRun.run();
				position = 4;
			} 
			else { // center right
				CenterRightRun.run();
				position = 5;
			}
		}
		else { // else, something is not right
			System.out.println("Something is wrong. Autobox might not be connected.");
		}
	}
	
	public void update() {
		shooArm.TorBantorArmAndShooterUpdate();
		if(position == 0) {
			
		} 
		else if(position == 1) {
			
		} 
		else if(position == 2) {
			CenterLeftRun.update();
		} 
		else if(position == 3) {
			CenterRightRun.update();
		} 
		else if(position == 4) {
			
		} 
		else if(position == 5){
			
		}
	}
	
	public void run() {
		if(!alreadyStarted) {
			oneSwitchRun();
			alreadyStarted = true;
		}
		update();
	}
}
