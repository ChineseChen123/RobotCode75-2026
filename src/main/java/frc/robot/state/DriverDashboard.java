package frc.robot.state;

import static frc.robot.Constants.IOConstants.turretAlignInactiveColor;
import static frc.robot.Constants.IOConstants.turretDeadzoneColor;
import static frc.robot.Constants.IOConstants.turretHoldAlignColor;
import static frc.robot.Constants.IOConstants.turretToggleAlignColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverDashboard {

	private static boolean gameDataReceived = false;
	private static boolean inactiveFirst = false;

	public static void update() {

		if (!gameDataReceived && RobotStates.teleop.getAsBoolean()) {
			String gameData = DriverStation.getGameSpecificMessage();
			if (gameData.length() > 0) {
				gameDataReceived = true;
				inactiveFirst =
						(gameData.charAt(0) == 'R'
										&& DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
								|| (gameData.charAt(0) == 'B'
										&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
			}
		}

		SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

		double matchTime = DriverStation.getMatchTime();

		if (matchTime > 130) {
			// Transition, hub always active
			SmartDashboard.putBoolean("Active Hub", true);
			SmartDashboard.putNumber("Phase Time Remaining", matchTime - 130);
			SmartDashboard.putNumber("Match Phase", 1);
		} else if (matchTime > 105) {
			// Shift 1
			SmartDashboard.putBoolean("Active Hub", !inactiveFirst);
			SmartDashboard.putNumber("Phase Time Remaining", matchTime - 105);
			SmartDashboard.putNumber("Match Phase", 2);
		} else if (matchTime > 80) {
			// Shift 2
			SmartDashboard.putBoolean("Active Hub", inactiveFirst);
			SmartDashboard.putNumber("Phase Time Remaining", matchTime - 80);
			SmartDashboard.putNumber("Match Phase", 3);
		} else if (matchTime > 55) {
			// Shift 3
			SmartDashboard.putBoolean("Active Hub", !inactiveFirst);
			SmartDashboard.putNumber("Phase Time Remaining", matchTime - 55);
			SmartDashboard.putNumber("Match Phase", 4);
		} else if (matchTime > 30) {
			// Shift 4
			SmartDashboard.putBoolean("Active Hub", inactiveFirst);
			SmartDashboard.putNumber("Phase Time Remaining", matchTime - 30);
			SmartDashboard.putNumber("Match Phase", 5);
		} else {
			// End game, hub always active.
			SmartDashboard.putBoolean("Active Hub", true);
			SmartDashboard.putNumber("Phase Time Remaining", matchTime);
			SmartDashboard.putNumber("Match Phase", 6);
		}

		if (!RobotStates.turretIsAligning.getAsBoolean()) {
			SmartDashboard.putString("Turret Status", turretAlignInactiveColor.toHexString());
		} else if (RobotStates.turretIsInDeadzone.getAsBoolean()) {
			SmartDashboard.putString("Turret Status", turretDeadzoneColor.toHexString());
		} else if (RobotStates.actionAimTurretHold.getAsBoolean()) {
			SmartDashboard.putString("Turret Status", turretHoldAlignColor.toHexString());
		} else {
			SmartDashboard.putString("Turret Status", turretToggleAlignColor.toHexString());
		}

		SmartDashboard.putNumber("Turret Angle", RobotStates.turretAngle.getAsDouble());
		SmartDashboard.putNumber(
				"Shooter Velocity Adjustment", RobotStates.shooterVelocityAdjustment.getAsDouble());
	}
}
