package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {
	private CenterLeftRun CenterLeftRun;
	private CenterRightRun CenterRightRun;
	private LeftLeftRun LeftLeftRun;
	private LeftRightRun LeftRightRun;
	private RightLeftRun RightLeftRun;
	private RightRightRun RightRightRun;
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
		LeftLeftRun = new LeftLeftRun(drive, shooArm);
		LeftRightRun = new LeftRightRun(drive, shooArm);
		RightLeftRun = new RightLeftRun(drive, shooArm);
		RightRightRun = new RightRightRun(drive, shooArm);
	}
	
	public void oneSwitchRun() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(Autobox.getRawButton(3) && Autobox.getRawButton(1)) { // left trajectories
			if(gameData.charAt(0) == 'L') { // left left
				position = 0;
				LeftLeftRun.run();
			} 
			else { // left right
				position = 1;
				LeftRightRun.run();
			}
		} 
		else if(Autobox.getRawButton(2) && Autobox.getRawButton(1)) { // right trajectories
			if(gameData.charAt(0) == 'L') { // right left
				position = 2;
				RightLeftRun.run();
			} 
			else {// right right
				position = 3;
				RightRightRun.run();
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
			LeftLeftRun.update();
		} 
		else if(position == 1) {
			LeftRightRun.update();
		} 
		else if(position == 2) {
			CenterLeftRun.update();
		} 
		else if(position == 3) {
			CenterRightRun.update();
		} 
		else if(position == 4) {
			RightLeftRun.update();
		} 
		else if(position == 5){
			RightRightRun.update();
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
