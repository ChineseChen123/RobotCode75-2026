package frc.robot.subsystems.Drivetrain.controllers;

import frc.lib.util.FieldPose.FieldElement;
import frc.robot.subsystems.Drivetrain.Swerve;

public class AutoAlign {

	private static Swerve m_Swerve;
	private static ChezyController m_ChezyController;
	private static RotationController m_RotationController;
	private static FieldElement lastTarget = null;
	private static boolean isInit = false;

	/** return last part of field auto-aligned to */
	public static FieldElement lastAutoAlign() {
		return lastTarget;
	}

	/** returns whether any auto-align has been initialized */
	public static boolean isInitialized() {
		return isInit;
	}
}
