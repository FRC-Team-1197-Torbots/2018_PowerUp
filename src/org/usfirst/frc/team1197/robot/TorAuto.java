package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorAuto {
	private CenterLeftRun CenterLeftRun;
	private CenterRightRun CenterRightRun;
	private LeftLeftRun LeftLeftRun;
	private LeftRightRun LeftRightRun;
	private RightRightRun RightRightRun;
	private RightLeftRun RightLeftRun;
	private Joystick autobox;
	private String gameData;
	private DriveHardware drive;
	private double starttime;
	
	// DriveHardware drive, double distance, double accelerationFraction, 
	// long accelerateTime (in milliseconds)
	// long accelerationTime--this would be to get to 100% with x + sinx
	
	public TorAuto(DriveHardware drive, Joystick autobox, TorBantorShooarm shooArm) {
		this.autobox = autobox;
		this.drive = drive;
		CenterLeftRun = new CenterLeftRun(drive, shooArm);
		CenterRightRun = new CenterRightRun(drive, shooArm);
		LeftLeftRun = new LeftLeftRun(drive, shooArm);
		LeftRightRun = new LeftRightRun(drive, shooArm);
		RightRightRun = new RightRightRun(drive, shooArm);
		RightLeftRun = new RightLeftRun(drive, shooArm);
	}
	
	public void run() {
		starttime = Timer.getFPGATimestamp();
		drive.shiftToLowGear();
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // Obtaining the switch & scale colors from the FMS
		SmartDashboard.putString("Game Data", gameData);
		if(autobox.getRawButton(3)) { // Left trajectories
			if(gameData.charAt(1) == 'L') { // Left Left 
				SmartDashboard.putString("AUTO", "Left Left");
				if(gameData.charAt(0) == 'L') {
					LeftLeftRun.switchLeft();
				}
				LeftLeftRun.run(starttime);
			} 
			else { // Left Right
				SmartDashboard.putString("AUTO", "Left Right");
				if(gameData.charAt(0) == 'L') {
					LeftRightRun.switchLeft();
				}
				LeftRightRun.run(starttime);
			}
		} 
		else if(autobox.getRawButton(2)) { // Right trajectories
			if(gameData.charAt(1) == 'R') { // Right Right
				SmartDashboard.putString("AUTO", "Right Right");
				if(gameData.charAt(0) == 'R') {
					RightRightRun.switchRight();
				}
				RightRightRun.run(starttime);
			} 
			else { // Right Left
				SmartDashboard.putString("AUTO", "Right Left");
				if(gameData.charAt(0) == 'R') {
					RightLeftRun.switchRight();
				}
				RightLeftRun.run(starttime);
			}
		} 
		else { // Center trajectories
			if(gameData.charAt(0) == 'L') { // center left
				SmartDashboard.putString("AUTO", "Center Left");
				if(gameData.charAt(1) == 'L') {
					CenterLeftRun.scaleLeft();
				}
				CenterLeftRun.run(starttime);
			} 
			else { // Center Right
				SmartDashboard.putString("AUTO", "Center Right");
				if(gameData.charAt(1) == 'R') {
					CenterRightRun.scaleRight();
				}
				CenterRightRun.run(starttime);
			}
		}
	}	
}
