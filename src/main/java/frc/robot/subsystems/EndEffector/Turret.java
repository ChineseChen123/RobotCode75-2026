// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;
import static frc.robot.Constants.ShooterConstants.Turret.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class Turret extends SubsystemBase {

	private final TalonFX m_TurretMotor;

	private final DutyCycleEncoder m_TurretEncoder1;
	private final DutyCycleEncoder m_TurretEncoder2;

	/** Creates a new Turret. */
	public Turret() {
		m_TurretMotor = new TalonFX(turretMotorCanID, superstructureCANBusName);

		m_TurretEncoder1 = new DutyCycleEncoder(encoder1Port, 1, encoder1Offset.in(Rotations));
		m_TurretEncoder2 = new DutyCycleEncoder(encoder2Port, 1, encoder2Offset.in(Rotations));

		m_TurretMotor.getConfigurator().apply(MotorConfigs.getTurretMotorConfig());
	}

	// 0 = CW limit, 270 deg = CCW limit (CCW positive)
	public Optional<Angle> getTurretPosition() {
		Angle encoder1Position = Rotations.of(m_TurretEncoder1.get());
		Angle encoder2Position = Rotations.of(m_TurretEncoder2.get());

		Angle possibleMechRot =
				Degrees.of(encoder1Position.in(Degrees) * encoderPinion1Teeth / ringGearTeeth);

		// calculate minimum possible solution for encoder 1 (closest to CW limit)
		possibleMechRot =
				Rotations.of(
						MathUtil.inputModulus(
								possibleMechRot.in(Rotations), 0, encoderPinion1Teeth / ringGearTeeth));

		// iterate through possible encoder 2 solutions
		while (possibleMechRot.lte(turretRingGearRange)) {
			Angle encoder2Solution =
					Rotations.of((possibleMechRot.in(Rotations) * ringGearTeeth / encoderPinion2Teeth) % 1.0);

			if (encoder2Position.minus(encoder2Solution).abs(Degrees) < matchTolerance.in(Degrees)) {
				return Optional.of(possibleMechRot);
			}

			possibleMechRot = possibleMechRot.plus(Rotations.of(encoderPinion1Teeth / ringGearTeeth));
		}

		return Optional.empty();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
