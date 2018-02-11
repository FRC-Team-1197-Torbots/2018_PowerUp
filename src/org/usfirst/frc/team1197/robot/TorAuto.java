package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {
	private TorDrive drive;
	private LeftLeftRun LeftLeftRun;
	private LeftRightRun LeftRightRun;
	private CenterLeftRun CenterLeftRun;
	private CenterRightRun CenterRightRun;
	private RightLeftRun RightLeftRun;
	private RightRightRun RightRightRun;
	private Joystick Autobox;
	private String gameData;
	public TorAuto(TorDrive drive, Joystick Autobox) {
		this.drive = drive;
		this.Autobox = Autobox;
		
		LeftLeftRun = new LeftLeftRun(drive);
		LeftRightRun = new LeftRightRun(drive);
		CenterLeftRun = new CenterLeftRun(drive);
		CenterRightRun = new CenterRightRun(drive);
		RightLeftRun = new RightLeftRun(drive);
		RightRightRun = new RightRightRun(drive);
	}
	
	public void oneSwitchRun() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(Autobox.getRawButton(3) && Autobox.getRawButton(1)) {//this is left
			if(gameData.charAt(0) == 'L') {//left left
				LeftLeftRun.run();
			} else {//left right
				LeftRightRun.run();
			}
		} else if(Autobox.getRawButton(2) && Autobox.getRawButton(1)) {//this is right
			if(gameData.charAt(0) == 'L') {//right left
				RightLeftRun.run();
			} else {//right right
				RightRightRun.run();
			}
		} else {//this is center
			if(Autobox.getRawButton(1)) {
				if(gameData.charAt(0) == 'L') {//center left
					CenterLeftRun.run();
				} else {//center right
					CenterRightRun.run();
				}
			}
		}
		
	}
}
