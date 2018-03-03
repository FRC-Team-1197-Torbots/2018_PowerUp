package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
		drive.shiftToLowGear();
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // Obtaining the switch & scale colors from the FMS
		SmartDashboard.putString("Game Data", gameData);
		if(autobox.getRawButton(3)) { // Left trajectories
			if(gameData.charAt(0) == 'L') { // Left Left 
				SmartDashboard.putString("AUTO", "Left Left");
				LeftLeftRun.run();
			} 
			else { // Left Right
				SmartDashboard.putString("AUTO", "Left Right");
				LeftRightRun.run();
			}
		} 
		else if(autobox.getRawButton(2)) { // Right trajectories
			if(gameData.charAt(0) == 'R') { // Right Right
				SmartDashboard.putString("AUTO", "Right Right");
				RightRightRun.run();
			} 
			else { // Right Left
				SmartDashboard.putString("AUTO", "Right Left");
				RightLeftRun.run();
			}
		} 
		else { // Center trajectories
			if(gameData.charAt(0) == 'L') { // center left
				SmartDashboard.putString("AUTO", "Center Left");
				CenterLeftRun.run();
			} 
			else { // Center Right
				SmartDashboard.putString("AUTO", "Center Right");
				CenterRightRun.run();
			}
		}
	}	
}
