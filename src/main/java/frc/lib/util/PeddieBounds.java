package frc.lib.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose.FieldElement;
import frc.robot.Constants.FieldConstants;
import frc.robot.state.RobotStates;
import java.util.List;

class IDVectorPair {
	public int id;
	public Translation2d vector;

	public IDVectorPair(int id, Translation2d vector) {
		this.id = id;
		this.vector = vector;
	}

	public String toString() {
		return id + ": " + vector.getNorm();
	}
}

// inspired by Peddie 5895
public class PeddieBounds {

	private static final AprilTagFields m_field = AprilTagFields.k2026RebuiltWelded;

	private static boolean onBlueAlliance() {
		return DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
	}

	/** returns field element associated with closest tag to current pose */
	public static FieldElement nearestElement(Pose2d pose) {
		List<AprilTag> tags = AprilTagFieldLayout.loadField(m_field).getTags();
		FieldElement nearestElement = null;
		double nearestDistance = Double.MAX_VALUE;

		for (AprilTag tag : tags) {
			if (tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation())
					< nearestDistance) {
				nearestDistance = tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation());
				nearestElement = tagIDToFieldElement.get(tag.ID);
			}
		}
		return nearestElement;
	}

	/** returns closest tag to current pose */
	public static int nearestTag(Pose2d pose) {
		List<AprilTag> tags = AprilTagFieldLayout.loadField(m_field).getTags();
		int nearestTag = 0;
		double nearestDistance = Double.MAX_VALUE;

		for (AprilTag tag : tags) {
			if (tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation())
					< nearestDistance) {
				nearestDistance = tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation());
				nearestTag = tag.ID;
			}
		}
		return nearestTag;
	}

	/** returns pose of nearest field element */
	public static Pose2d getNearestFieldPose2d(Pose2d currentPose, FieldPose targetPose) {
		targetPose.fieldElement = nearestElement(currentPose);
		return fieldElementToPose2d(targetPose);
	}

	/** returns pose of specified field element */
	public static Pose2d fieldElementToPose2d(FieldPose targetPose) {
		int targetTag =
				targetPose.alliance == Alliance.Blue
						? blueTags.get(targetPose.fieldElement)
						: redTags.get(targetPose.fieldElement);
		Pose2d tagPose =
				AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
						.getTagPose(targetTag)
						.get()
						.toPose2d();
		Rotation2d tagHeading = tagPose.getRotation();
		double bumperSize = 17.5;
		Pose2d poseToDrive =
				tagPose.transformBy(
						new Transform2d(Inches.of(bumperSize).in(Meters), 0, new Rotation2d(0)));

		return new Pose2d(
				poseToDrive.getX(),
				poseToDrive.getY(),
				Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees() - 180));
	}

	public static Pose2d getNearestBump(Pose2d currentPose) {
		Pose2d bumpLeft = onBlueAlliance() ? FieldConstants.blueBumpLeft : FieldConstants.redBumpLeft;
		Pose2d bumpRight =
				onBlueAlliance() ? FieldConstants.blueBumpRight : FieldConstants.redBumpRight;
		double distToLeft = bumpLeft.getTranslation().getDistance(currentPose.getTranslation());
		double distToRight = bumpRight.getTranslation().getDistance(currentPose.getTranslation());

		if (distToLeft < distToRight) return bumpLeft;
		if (distToLeft >= distToRight) return bumpRight; // readability
		return bumpRight;
	}

	public static Pose2d getNearestCorner(Pose2d currentPose) {
		Pose2d cornerDepot = onBlueAlliance() ? blueCornerDepot : redCornerDepot;
		Pose2d cornerOutpost = onBlueAlliance() ? blueCornerOutpost : redCornerOutpost;
		double distToDepot = cornerDepot.getTranslation().getDistance(currentPose.getTranslation());
		double distToOutpost = cornerOutpost.getTranslation().getDistance(currentPose.getTranslation());

		if (distToDepot < distToOutpost) return cornerDepot;
		if (distToDepot >= distToOutpost) return cornerOutpost; // readability
		return cornerOutpost;
	}

	public static Pose2d getOptimalFeedPose(Pose2d currentPose) {
		// TODO: this does NOT take into account the hub
		// im just too lazy to do this rn but basically need to find where ball might clip corner of hub
		// when feeding
		// and move those poses to the left/right to get out of corner clipping range
		// shouldn't really be an issue for further away cuz of ball height
		// best way is probably to make a line of turret to hub corner and find where that intersects
		// bump
		// ofc if the line never intersects bump then its optimal to just create a line between the
		// turret and bump and ignore corner
		// same thing for trench but that should be less of an issue
		// or we can ignore this by making the extremes more in LOL

		Pose2d bumpCenter = getNearestBump(currentPose);

		double metersAdjustment = 0.25; // idfk

		if (currentPose.getY()
						> bumpCenter.getY() - (FieldConstants.bumpWidth.in(Meters) / 2.0 + metersAdjustment)
				&& currentPose.getY()
						< bumpCenter.getY() + (FieldConstants.bumpWidth.in(Meters) / 2.0) - metersAdjustment) {
			// TODO idfk it its + or -
			return new Pose2d(
					new Translation2d(
							bumpCenter.getX() + (FieldConstants.bumpWidth.in(Meters) / 2.0), currentPose.getY()),
					Rotation2d.kZero);
		} else if (currentPose.getY()
				<= bumpCenter.getY() - (FieldConstants.bumpWidth.in(Meters) / 2.0)) {
			return new Pose2d(
					new Translation2d(
							bumpCenter.getX() + (FieldConstants.bumpWidth.in(Meters) / 2.0),
							bumpCenter.getY() - (FieldConstants.bumpWidth.in(Meters) / 2.0) + metersAdjustment),
					Rotation2d.kZero);
		} else if (currentPose.getY()
				>= bumpCenter.getY() + (FieldConstants.bumpWidth.in(Meters) / 2.0)) {
			return new Pose2d(
					new Translation2d(
							bumpCenter.getX() + (FieldConstants.bumpWidth.in(Meters) / 2.0),
							bumpCenter.getY() + (FieldConstants.bumpWidth.in(Meters) / 2.0) - metersAdjustment),
					Rotation2d.kZero);
		}
		return null;
	}

	private static final double trenchThreshold = 0.5;

	public static Pose2d getNearestTrench(Pose2d currentPose) {
		if (DriverStation.getAlliance().isEmpty()
				|| DriverStation.getAlliance().get() == Alliance.Blue) {
			double distL = currentPose.getTranslation().getDistance(blueTrenchLeft.getTranslation());
			double distR = currentPose.getTranslation().getDistance(blueTrenchRight.getTranslation());
			if (distL > distR + trenchThreshold) {
				return blueTrenchRight;
			} else if (distR > distL + trenchThreshold) {
				return blueTrenchLeft;
			} else {
				return RobotStates.fieldRelativeSpeeds.get().vyMetersPerSecond < 0
						? blueTrenchRight
						: blueTrenchLeft;
			}
		} else {
			double distL = currentPose.getTranslation().getDistance(redTrenchLeft.getTranslation());
			double distR = currentPose.getTranslation().getDistance(redTrenchRight.getTranslation());
			if (distL > distR + trenchThreshold) {
				return redTrenchRight;
			} else if (distR > distL + trenchThreshold) {
				return redTrenchLeft;
			} else {
				return RobotStates.fieldRelativeSpeeds.get().vyMetersPerSecond < 0
						? redTrenchLeft
						: redTrenchRight;
			}
		}
	}

	public static Pose3d getShootingTargetPose(Pose2d pose) {
		if (isInOwnZone(pose)) return onBlueAlliance() ? FieldConstants.blueHub : FieldConstants.redHub;
		if (DriverStation.isAutonomous()) return new Pose3d(getNearestBump(pose));

		Pose3d bump = new Pose3d(getNearestBump(pose));
		bump = new Pose3d(bump.getMeasureX(), bump.getMeasureY(), Meters.of(0), bump.getRotation());
		return bump;
	}

	public static boolean isInNeutralZone(Pose2d pose) {
		return pose.getX() > blueHub.getX() && pose.getX() < redHub.getX();
	}

	public static boolean isInOwnZone(Pose2d pose) {
		if (onBlueAlliance()) {
			return pose.getX() < blueHub.getX();
		} else {
			return pose.getX() > redHub.getX();
		}
	}

	public static boolean isInTrench(Pose2d pose) {
		return Math.abs(pose.getX() - blueHub.getX()) < 0.4
				|| Math.abs(pose.getX() - redHub.getX()) < 0.4;
	}
}
