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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.ArrayList;
import java.util.Collections;
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

	/** returns field element associated with closest tag to current pose */
	public static FieldElement nearestElement(Pose2d pose) {
		List<AprilTag> tags =
				AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();
		FieldElement nearestElement = null;
		double nearestDistance = Double.MAX_VALUE;

		for (AprilTag tag : tags) {
			if (tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation())
					< nearestDistance) {
				nearestDistance = tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation());
				//nearestElement = tagIDToFieldElement.get(tag.ID);
			}
		}
		return nearestElement;
	}

	/** returns closest tag to current pose */
	public static int nearestTag(Pose2d pose) {
		List<AprilTag> tags =
				AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();
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
	// public static Pose2d getNearestFieldPose2d(Pose2d currentPose, FieldPose targetPose) {
	// 	targetPose.fieldElement = nearestElement(currentPose);
	// 	return fieldElementToPose2d(targetPose);
	// }

	/** returns pose of specified field element */
	//public static Pose2d fieldElementToPose2d(FieldPose targetPose) {
		// int targetTag =
		// 		targetPose.alliance == Alliance.Blue
		// 				? blueTags.get(targetPose.fieldElement)
		// 				: redTags.get(targetPose.fieldElement);
		// Pose2d tagPose =
		// 		AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
		// 				.getTagPose(targetTag)
		// 				.get()
		// 				.toPose2d();
		// return tagPose;
	// 	Rotation2d tagHeading = tagPose.getRotation();
	// 	double bumperSize = 17.5;
	// 	if (FieldPose.fieldElementIsHPStation(targetPose.fieldElement)) {
	// 		bumperSize = 19;
	// 	}
	// 	Pose2d poseToDrive =
	// 			tagPose.transformBy(
	// 					new Transform2d(Inches.of(bumperSize).in(Meters), 0, new Rotation2d(0)));
	// 	// flip the pose
	// 	if (FieldPose.fieldElementIsReef(targetPose.fieldElement) && targetPose.offset == Offset.LEFT) {
	// 		tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
	// 		poseToDrive =
	// 				poseToDrive.transformBy(
	// 						new Transform2d(0, reefLeftPoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
	// 	}
	// 	if (FieldPose.fieldElementIsReef(targetPose.fieldElement)
	// 			&& targetPose.offset == Offset.RIGHT) {
	// 		tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
	// 		poseToDrive =
	// 				poseToDrive.transformBy(
	// 						new Transform2d(0, reefRightPoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
	// 	}
	// 	if (FieldPose.fieldElementIsReef(targetPose.fieldElement) && targetPose.offset == Offset.MID) {
	// 		tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
	// 		poseToDrive =
	// 				poseToDrive.transformBy(
	// 						new Transform2d(0, reefAlgaePoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
	// 	}
	// 	if (FieldPose.fieldElementIsReef(targetPose.fieldElement)) {
	// 		return new Pose2d(
	// 				poseToDrive.getX(),
	// 				poseToDrive.getY(),
	// 				Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees() - 180));
	// 	} else if (FieldPose.fieldElementIsHPStation(targetPose.fieldElement)) {
	// 		if (targetPose.offset == Offset.LEFT) {
	// 			poseToDrive =
	// 					poseToDrive.transformBy(
	// 							new Transform2d(0, hpLeftPoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
	// 		}
	// 		if (targetPose.offset == Offset.RIGHT) {
	// 			poseToDrive =
	// 					poseToDrive.transformBy(
	// 							new Transform2d(0, hpRightPoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
	// 		}
	// 		poseToDrive = poseToDrive.transformBy(new Transform2d(-0.3, 0, Rotation2d.fromDegrees(0)));
	// 		return new Pose2d(
	// 				poseToDrive.getX(),
	// 				poseToDrive.getY(),
	// 				Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees()));
	// 	}
	// 	return poseToDrive;
	 //}

	/** returns algae level of nearest reef face */
	// public static ElevatorPositions getAlgaeLevel(Pose2d currentPose) {
	// 	return algaeHeights.get(getReefElement(currentPose).toString());
	// }

	/** returns nearest reef field element */
	// public static FieldElement getReefElement(Pose2d currentPose) {
	// 	// calculate current odometry pose
	// 	Translation2d odometryPose = currentPose.getTranslation();
	// 	List<IDVectorPair> robotToTag = new ArrayList<>();
	// 	AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

	// 	if (DriverStation.getAlliance().isEmpty()
	// 			|| DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
	// 		// BLUE:
	// 		for (int i = 17; i <= 22; i++) {
	// 			Translation2d tagPose = field.getTagPose(i).get().toPose2d().getTranslation();
	// 			robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
	// 		}
	// 	} else {
	// 		// RED:
	// 		for (int i = 6; i <= 11; i++) {
	// 			Translation2d tagPose = field.getTagPose(i).get().toPose2d().getTranslation();
	// 			robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
	// 		}
	// 	}

	// 	// sort list in ascending order of vector magnitude / robot distance to tag
	// 	Collections.sort(
	// 			robotToTag, (o1, o2) -> (((Double) o1.vector.getNorm()).compareTo(o2.vector.getNorm())));

	// 	// closest tag ID
	// 	int tag0id = robotToTag.get(0).id;

	// 	return VisionConstants.tagIDToFieldElement.get(tag0id);
	// }

	private static final double hpThreshold =
			0.5; // section in the middle where bound is determined by vel

	/** return nearest coral station field element */
	/** if close to middle - depends on which way robot is moving, else based on position */
	public static FieldElement getHPElement(Swerve swerve) {
		Translation2d odometryPose = swerve.getPose().getTranslation();
		AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

		if (DriverStation.getAlliance().isEmpty()
				|| DriverStation.getAlliance().get() == Alliance.Blue) {
			// BLUE:
			double distToT =
					field.getTagPose(13).get().toPose2d().getTranslation().getDistance(odometryPose);
			double distToB =
					field.getTagPose(12).get().toPose2d().getTranslation().getDistance(odometryPose);

			if (distToT > distToB + hpThreshold) {
				return FieldElement.HB;
			} else if (distToB > distToT + hpThreshold) {
				return FieldElement.HT;
			} else {
				ChassisSpeeds fieldRelative =
						ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), swerve.getHeading());
				return fieldRelative.vyMetersPerSecond < 0 ? FieldElement.HB : FieldElement.HT;
			}
		} else {
			// RED:
			double distToT =
					field.getTagPose(2).get().toPose2d().getTranslation().getDistance(odometryPose);
			double distToB =
					field.getTagPose(1).get().toPose2d().getTranslation().getDistance(odometryPose);

			if (distToT > distToB + hpThreshold) {
				return FieldElement.HB;
			} else if (distToB > distToT + hpThreshold) {
				return FieldElement.HT;
			} else {
				ChassisSpeeds fieldRelative =
						ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), swerve.getHeading());
				return fieldRelative.vyMetersPerSecond > 0 ? FieldElement.HT : FieldElement.HB;
			}
		}
	}
}
