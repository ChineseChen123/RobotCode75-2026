// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/**
 * This class is meant ry.TrapezoidProflie; import edu.first.first.math.etil.Units; to house the
 * configs for specific motors All configs from CTRE motors are unit-aware, especially configs for
 * closed loop gains timeSync can only be used on a CANivore any TorqueCurrentFOC gains/control
 * modes can only be used with Phoenix pro (HIGHLY RECCOMENDED TO USE)
 */
public final class RobotConstants {
	public static final String superstructureCANBusName = "superstructure";
	public static final boolean TUNING_MODE = true; // Set to true for tunable numbers

	public static final Distance bumperThickness = Inches.of(3);
	public static final Distance bumperWidth = Inches.of(27.5 + 2 * bumperThickness.in(Inches));
}
