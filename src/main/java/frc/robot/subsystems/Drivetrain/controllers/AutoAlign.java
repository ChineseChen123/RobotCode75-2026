package frc.robot.subsystems.Drivetrain.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.PeddieBounds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

public class AutoAlign {

	private static Swerve m_Swerve;
	private static ChezyController m_ChezyController;
	private static RotationController m_RotationController;
	private static YoloController m_YoloController;
	private static ObjectDetetectorCamera m_BranchCamera;
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

	/** aligning to score coral */
	public static class ReefAlign {
		private static boolean reefAlignReady = false;
		private static Offset m_Offset;
		private static Pose2d targetPose = null;

		private static ChassisSpeeds chezySpeeds = new ChassisSpeeds(0, 0, 0);
		private static ChassisSpeeds yoloSpeeds = null;

		/** reset target before every use */
		public static void initialize(Offset offset) {
			m_Swerve = RobotContainer.getSwerve();
			m_ChezyController = m_Swerve.getChezyController();
			m_YoloController = m_Swerve.getYoloController();
			m_BranchCamera = RobotContainer.getBranchCamera();
			m_Offset = offset;

			// set our goal to the reef face closest to our current pose
			FieldElement fieldElement = PeddieBounds.getReefElement(m_Swerve.getPose());
			if (fieldElement != null) {
				targetPose =
						PeddieBounds.fieldElementToPose2d(
								new FieldPose(DriverStation.getAlliance().get(), fieldElement, m_Offset));
				m_ChezyController.reset(targetPose);
				lastTarget = fieldElement;
			}
			m_YoloController.reset(true);
			reefAlignReady = true;
			isInit = true;
		}

		/** returns robot-relative speeds */
		public static ChassisSpeeds execute() {
			if (!reefAlignReady) {
				System.out.println("Reef Align not ready!");
				return new ChassisSpeeds(0, 0, 0);
			}
			m_Swerve.setSample(targetPose);
			// get chezy speeds
			chezySpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							m_ChezyController.update(targetPose), m_Swerve.getHeading());

			// robot relative y distance from center of branch
			double yOffset = Math.abs(targetPose.relativeTo(m_Swerve.getPose()).getY());

			m_BranchCamera.updateByUnreadResults();

			// slow down when we are aligned left/right
			if (yOffset < 0.07) {
				chezySpeeds.vxMetersPerSecond *= 0.8; // 0.5 before
				chezySpeeds.vxMetersPerSecond = Math.max(chezySpeeds.vxMetersPerSecond, 0.07);
			}

			// switch left/align command to yolo align when almost aligned
			if (yOffset < 0.07
					&& m_BranchCamera.hasTargets()
					&& m_ChezyController.isRotationFinished()
					&& m_Offset != Offset.MID) {
				// System.out.println("I have switched to YOLO");
				yoloSpeeds = m_YoloController.update();
				// add yolo speeds to chezy speeds
				chezySpeeds.vyMetersPerSecond = MathUtil.applyDeadband(yoloSpeeds.vyMetersPerSecond, 0.04);
			} else if (yOffset > 0.1) {
				// when far away, prioritize y alignment with slight left velocity bias
				chezySpeeds.vyMetersPerSecond *= 1.25;
				chezySpeeds.vxMetersPerSecond *= 0.8;
				chezySpeeds.vyMetersPerSecond += 0.25;
			}
			chezySpeeds.vxMetersPerSecond = MathUtil.clamp(chezySpeeds.vxMetersPerSecond, -2, 2);
			chezySpeeds.vyMetersPerSecond = MathUtil.clamp(chezySpeeds.vyMetersPerSecond, -2, 2);
			return chezySpeeds;
		}

		/** return last aligned offset */
		public static Offset getOffset() {
			return m_Offset;
		}

		/** called when done or interrupted */
		public static void end() {
			System.out.println("reef align done");
			reefAlignReady = false;
		}

		/** returns whether done aligning */
		public static boolean isFinished() {
			return (chezySpeeds.vxMetersPerSecond <= 0.07 && m_YoloController.atGoal());
		}
	}

	/** aligning to coral station */
	public static class HPAlign {
		private static boolean hpAlignReady = false;
		private static Offset m_Offset;
		private static Pose2d targetPose = null;

		/** reset target before every use */
		public static void initialize(Offset offset) {
			m_Swerve = RobotContainer.getSwerve();
			m_ChezyController = m_Swerve.getChezyController();
			m_Offset = offset;
			Alliance alliance = DriverStation.getAlliance().get(); // default to blue if not set
			targetPose = m_Swerve.getPose();

			targetPose =
					PeddieBounds.fieldElementToPose2d(
							new FieldPose(alliance, PeddieBounds.getHPElement(m_Swerve), m_Offset));
			lastTarget = PeddieBounds.getHPElement(m_Swerve);
			m_Swerve.setSample(targetPose);

			m_ChezyController.reset(targetPose);
			hpAlignReady = true;
			isInit = true;
		}

		/** returns robot-relative speeds */
		public static ChassisSpeeds execute() {
			if (!hpAlignReady) {
				System.out.println("HP Align not ready!");
				return new ChassisSpeeds(0, 0, 0);
			}
			ChassisSpeeds speeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							m_ChezyController.update(targetPose), m_Swerve.getHeading());
			speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -1.5, 1.5);
			speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -1.5, 1.5);
			return speeds;
		}

		/** called when done or interrupted */
		public static void end() {
			m_Swerve.setFieldRelative(new ChassisSpeeds(0, 0, 0));
			m_Swerve.xStance();
			hpAlignReady = false;
		}

		/** returns whether done aligning */
		public static boolean isFinished() {
			return m_ChezyController.isFinished();
		}
	}

	/** unused */
	public static class RotateSimilarFace {
		private static boolean rotateSFReady = false;
		private static double targetHeading;

		public static void initialize() {
			m_Swerve = RobotContainer.getSwerve();
			m_RotationController = m_Swerve.getRotationController();
			targetHeading = Math.round(m_Swerve.getHeading().getDegrees() / 60.0) * 60.0;
			rotateSFReady = true;
		}

		public static ChassisSpeeds execute() {
			if (!rotateSFReady) {
				System.out.println("Similar Face Rotate not ready!");
				return new ChassisSpeeds(0, 0, 0);
			}
			m_RotationController.update(Rotation2d.fromDegrees(targetHeading));
			return new ChassisSpeeds(0, 0, m_RotationController.getOutput());
		}

		public static void end() {
			m_Swerve.setFieldRelative(new ChassisSpeeds(0, 0, 0));
			rotateSFReady = false;
		}

		public static boolean isFinished() {
			return m_RotationController.atGoal();
		}
	}

	/** unused */
	public static class RotateHPStation {
		private static boolean rotateHPReady = false;
		private static double[] angles = {54, -54, 180 - 54, 180 + 54};
		private static double targetAngle = 0;

		public static void initialize() {
			m_Swerve = RobotContainer.getSwerve();
			m_RotationController = m_Swerve.getRotationController();
			double minAngleDiff = 500;
			for (double angle : angles) {
				if (Math.abs(m_Swerve.getHeading().getDegrees() - angle) < minAngleDiff) {
					targetAngle = angle;
					minAngleDiff = Math.abs(m_Swerve.getHeading().getDegrees() - angle);
				}
			}
			rotateHPReady = true;
		}

		public static double execute() {
			if (!rotateHPReady) {
				System.out.println("HP Rotate not ready!");
				return 0;
			}

			m_RotationController.update(Rotation2d.fromDegrees(targetAngle));
			double rotationOutput = 0;
			if (m_RotationController.atGoal()) {
				rotationOutput = 0;
			} else {
				rotationOutput = m_RotationController.getOutput();
			}

			return rotationOutput;
		}

		public static void end() {
			if (Math.sqrt(
							Math.pow(m_Swerve.getChassisSpeeds().vxMetersPerSecond, 2)
									+ Math.pow(m_Swerve.getChassisSpeeds().vyMetersPerSecond, 2))
					< .05) {
				m_Swerve.setFieldRelative(new ChassisSpeeds(0, 0, 0));
				m_Swerve.xStance();
			}
			rotateHPReady = false;
		}

		public static boolean isFinished() {
			return false;
		}
	}
}
