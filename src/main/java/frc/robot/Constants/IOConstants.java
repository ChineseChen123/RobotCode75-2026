package frc.robot.Constants;

public final class IOConstants {
	public static final boolean oneDriver = false;

	public static final int leftStickPort = 0;
	public static final int rightStickPort = 1;
	public static final int controllerPort = 2;

	public static final int autoAlignButton = 1;
	public static final int robotRelativeButton = 2;
	public static final int algaeAlignButton = 2;
	public static final int rotateToSimilarFaceButton = 3;
	public static final int resetHeadingButton = 3;
	public static final int xstanceButton = 5;
	public static final int holdHeadingButton = 3;
	public static final int resetBranchCamButton = 10;
	public static final int hpRotateButton = 4;

	// Joystick value adjusted to 0 if within -deadband and deadband
	public static final double stickDeadband = oneDriver ? 0.05 : 0.08;
	
	// Joystick value multiplied by...
	public static final double translationStickMapValue = oneDriver ? 1.05 : 1.5;
	
	// Joystick value exponentiated by...
	public static final double translationJoystickExpo = oneDriver ? 1.7 : 1.46;
}
