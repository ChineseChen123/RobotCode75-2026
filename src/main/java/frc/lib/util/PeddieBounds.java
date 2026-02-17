package frc.lib.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose.FieldElement;
import frc.robot.Constants.FieldConstants;

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

	public static Pose2d getHubTarget() {
		return DriverStation.getAlliance().isPresent()
								&& DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
						? FieldConstants.blueHub
						: FieldConstants.redHub;
	}

	public static boolean isInNeutralZone(Pose2d pose) {
		return pose.getX() > blueHub.getX() && pose.getX() < redHub.getX();
	}

	public static boolean isInOwnZone(Pose2d pose) {
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
			return pose.getX() < blueHub.getX();
		} else {
			return pose.getX() > redHub.getX();
		}
	}

	public static boolean isInTrench(Pose2d pose) {
		return Math.abs(pose.getX() - blueHub.getX()) < 0.03 || Math.abs(pose.getX() - redHub.getX()) < 0.03;
	}
}
