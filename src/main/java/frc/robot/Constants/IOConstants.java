package frc.robot.Constants;

import edu.wpi.first.wpilibj.util.Color;

public final class IOConstants {
	public static final boolean oneDriver = false;

	public static final int leftStickPort = 0;
	public static final int rightStickPort = 1;
	public static final int controllerPort = 2;

	// left stick
	public static final int trenchDriveButton = 1;
	public static final int resetHeadingButton = 3;
	public static final int speedClampButton = 2;

	// right stick
	public static final int robotRelativeButton = 2;
	public static final int xstanceButton = 1;

	// Joystick value adjusted to 0 if within -deadband and deadband
	public static final double stickDeadband = oneDriver ? 0.05 : 0.08;
	public static final double operatorDeadband = 0.15;

	// Joystick value multiplied by...
	public static final double translationStickMapValue = oneDriver ? 1.05 : 1.5;

	// Joystick value exponentiated by...
	public static final double translationJoystickExpo = oneDriver ? 1.7 : 1.46;

	public static final Color turretHoldAlignColor = new Color(0, 255, 0);
	public static final Color turretToggleAlignColor = new Color(0, 0, 255);
	public static final Color turretDeadzoneColor = new Color(255, 255, 0);
	public static final Color turretAlignInactiveColor = new Color(255, 0, 0);
}
