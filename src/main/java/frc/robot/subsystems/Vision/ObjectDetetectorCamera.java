package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

public class ObjectDetetectorCamera extends SubsystemBase {
	private PhotonCamera m_Camera;
	private Optional<PhotonPipelineResult> m_Result;

	public ObjectDetetectorCamera(String name) {
		m_Camera = new PhotonCamera(name);
	}

	public void updateByUnreadResults() {
		List<PhotonPipelineResult> unreadResults = m_Camera.getAllUnreadResults();
		if (unreadResults.size() > 0) {
			m_Result = Optional.of(unreadResults.get(unreadResults.size() - 1));
		}
	}

	// Unused now
	// Using deprecated method for testing
	// public void updateByLatestResult() {
	//   m_Result = m_Camera.getLatestResult();
	// }

	public OptionalDouble getTargetPitch(int targetID) {
		if (m_Result.isPresent() && m_Result.get().hasTargets()) {
			return OptionalDouble.of(m_Result.get().getTargets().get(targetID).getPitch());
		}
		return OptionalDouble.empty();
	}

	public OptionalDouble getTargetYaw(int targetID) {
		if (m_Result.get().hasTargets()) {
			return OptionalDouble.of(m_Result.get().getTargets().get(targetID).getYaw());
		}
		return OptionalDouble.empty();
	}

	public OptionalDouble getTargetSkew(int targetID) {
		if (m_Result.get().hasTargets()) {
			return OptionalDouble.of(m_Result.get().getTargets().get(targetID).getSkew());
		}
		return OptionalDouble.empty();
	}

	public OptionalDouble getTargetArea(int targetID) {
		if (m_Result.get().hasTargets()) {
			return OptionalDouble.of(m_Result.get().getTargets().get(targetID).getArea());
		}
		return OptionalDouble.empty();
	}

	public OptionalInt getNumTargets() {
		if (m_Result.isEmpty()) {
			return OptionalInt.of(0);
		}
		if (m_Result.get().hasTargets()) {
			return OptionalInt.of(m_Result.get().getTargets().size());
		}
		return OptionalInt.empty();
	}

	public boolean hasTargets() {
		if (m_Result.isEmpty()) {
			return false;
		}
		return m_Result.get().hasTargets();
	}

	public void setDriverMode(boolean on) {
		m_Camera.setDriverMode(on);
	}

	public void reloadPipeline() {
		if (m_Camera.getPipelineIndex() == 0) {
			m_Camera.setPipelineIndex(1);
		} else {
			m_Camera.setPipelineIndex(0);
		}
	}

	public OptionalDouble getTargetXFromCenter(int targetID) {
		if (m_Result.get().hasTargets()) {
			List<TargetCorner> corners =
					m_Result.get().getTargets().get(targetID).getMinAreaRectCorners();
			double sum = 0;
			for (TargetCorner corner : corners) {
				sum += corner.x;
			}
			return OptionalDouble.of(320 - sum / 4);
		}
		return OptionalDouble.empty();
	}

	@Override
	public void periodic() {
		if (DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Blue && m_Camera.getPipelineIndex() != 1) {
				m_Camera.setPipelineIndex(1);
			} else if (DriverStation.getAlliance().get() == Alliance.Red
					&& m_Camera.getPipelineIndex() != 0) {
				m_Camera.setPipelineIndex(0);
			}
		}
	}
}
