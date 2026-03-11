package frc.robot.subsystems.Drivetrain.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.util.PeddieBounds;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.state.RobotStates;
import frc.robot.subsystems.Drivetrain.Swerve;

public class AutoAlign {

	private static Swerve m_Swerve;
	private static ChezyController m_ChezyController;
	private static RotationController m_RotationController;
	private static Pose2d lastTarget = null;
	private static boolean isInit = false;

	/** return last part of field auto-aligned to */
	public static Pose2d lastAutoAlign() {
		return lastTarget;
	}

	/** returns whether any auto-align has been initialized */
	public static boolean isInitialized() {
		return isInit;
	}

	public static class TrenchAlign {
		private static Pose2d targetPose = null;
		private static ChassisSpeeds chezySpeeds = new ChassisSpeeds(0, 0, 0);

		public static void init() {
			m_Swerve = RobotContainer.getSwerve();
			m_ChezyController = m_Swerve.getChezyController();
			targetPose = PeddieBounds.getNearestTrench(RobotStates.robotPose.get());
			isInit = true;
		}

		/** returns robot-relative speeds */
		public static ChassisSpeeds execute() {
			if (!isInit) {
				System.out.println("Didn't Initialize");
				return new ChassisSpeeds(0, 0, 0);
			}

			Pose2d xLockedTargetPose =
					new Pose2d(
							RobotStates.robotPose.get().getMeasureX(),
							targetPose.getMeasureY(),
							RobotStates.robotPose.get().getRotation());

			m_Swerve.setSample(xLockedTargetPose);
			// get chezy speeds
			chezySpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							m_ChezyController.update(xLockedTargetPose), m_Swerve.getHeading());

			chezySpeeds.vxMetersPerSecond = 0;
			chezySpeeds.vyMetersPerSecond = MathUtil.clamp(chezySpeeds.vyMetersPerSecond, -2, 2);
			chezySpeeds.omegaRadiansPerSecond = 0;
			return chezySpeeds;
		}

		/** called when done or interrupted */
		public static void end() {
			System.out.println("trench align done");
		}

		/** returns whether done aligning */
		public static boolean isFinished() {
			return MathUtil.isNear(
					m_Swerve.getPose().getY(),
					targetPose.getY(),
					DrivetrainConstants.ControllerConstants.toleranceTranslation); // fake tolerance
		}
	}
}
