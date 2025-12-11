package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public final record SwerveModuleConstants(
		int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {}
