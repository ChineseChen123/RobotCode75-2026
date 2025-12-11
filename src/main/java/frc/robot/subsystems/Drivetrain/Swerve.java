package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Degrees;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.RaiderLog.Logged;
import frc.lib.util.RaiderLog.RaiderLog.Importance;
import frc.lib.util.RaiderLog.RaiderLog;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.state.Driver;
import frc.robot.subsystems.Vision.AprilTagCamera;

public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

	// ── Sim loop ──────────────────────────────────────────────────────────────────
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier simNotifier = null;
	private double lastSimTimeSec;

	// ── Alliance perspective ──────────────────────────────────────────────────────
	private static final Rotation2d kBluePerspective = Rotation2d.kZero;
	private static final Rotation2d kRedPerspective = Rotation2d.k180deg;
	private boolean appliedOperatorPerspective = false;

	// ── HW ────────────────────────────────────────────────────────────────────────
	private final Pigeon2 gyro;

	// ── Control state ─────────────────────────────────────────────────────────────
	private boolean fieldRelative = true;
	private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();
	private Pose2d samplePose = new Pose2d();

	private final SwerveRequest.ApplyFieldSpeeds fieldRequest =
			new SwerveRequest.ApplyFieldSpeeds()
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private final SwerveRequest.ApplyRobotSpeeds robotRequest =
			new SwerveRequest.ApplyRobotSpeeds()
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private final SwerveRequest.SwerveDriveBrake xStanceRequest =
			new SwerveRequest.SwerveDriveBrake()
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	// ── Auto controllers ─────────────────────────────────────────────────────────
	private final PIDController xController = new PIDController(2.65, 0, 0);
	private final PIDController yController = new PIDController(3.9, 0, 0);
	private final PIDController rController = new PIDController(3.05, 0, 0);

	// ── Cameras ───────────────────────────────────────────────────────────────────

	private Pose2d[] estimatedPosesFromCameras;

	/**
	 * @param drivetrainConstants Drivetrain-wide constants
	 * @param odometryUpdateFrequency Odometry loop Hz (0 → default: 250Hz CAN FD / 100Hz CAN 2.0)
	 * @param odometryStdDev Odometry std dev [x, y, theta]ᵀ (m, m, rad)
	 * @param visionStdDev Vision std dev [x, y, theta]ᵀ (m, m, rad)
	 * @param modules Module constants
	 */
	public Swerve(
			SwerveDrivetrainConstants drivetrainConstants,
			double odometryUpdateFrequency,
			Matrix<N3, N1> odometryStdDev,
			Matrix<N3, N1> visionStdDev,
			SwerveModuleConstants<?, ?, ?>... modules) {

		super(
				TalonFX::new,
				TalonFX::new,
				CANcoder::new,
				drivetrainConstants,
				odometryUpdateFrequency,
				odometryStdDev,
				visionStdDev,
				modules);

		gyro = this.getPigeon2();
		zeroGyro();

		estimatedPosesFromCameras = null;

		if (Utils.isSimulation()) startSimThread();
	}

	// ── Driving API ───────────────────────────────────────────────────────────────

	/** Drive with XY translation (m/s) and CCW rotation (rad/s). */
	public void drive(Translation2d translation, double omega) {
		final ChassisSpeeds speeds =
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(
								translation.getX(), translation.getY(), omega, getHeading())
						: new ChassisSpeeds(translation.getX(), translation.getY(), omega);

		setRobotRelative(speeds); // Apply robot request for consistent open-loop behavior
	}

	public void setFieldRelative(ChassisSpeeds speeds) {
		setpointSpeeds = speeds;
		setControl(fieldRequest.withSpeeds(speeds));
	}

	public void setRobotRelative(ChassisSpeeds speeds) {
		setpointSpeeds = speeds;
		setControl(robotRequest.withSpeeds(speeds));
	}

	/** Convenience for toggling frame. */
	public void toggleRobotRelative() {
		fieldRelative = false;
	}

	/** Convenience for toggling frame. */
	public void toggleFieldRelative() {
		fieldRelative = true;
	}

	public boolean getFieldRelative() {
		return fieldRelative;
	}

	/** Brake into an X stance. */
	public void xStance() {
		setControl(xStanceRequest);
	}

	/** Stop all motion (field frame). */
	public void stopModules() {
		setControl(fieldRequest.withSpeeds(new ChassisSpeeds()));
	}

	// ── Auto helpers ─────────────────────────────────────────────────────────────

	/** Follow a Choreo swerve sample using PIDs on X/Y/Heading. */
	public void followSwerveSample(SwerveSample sample) {
		this.samplePose = sample.getPose();
		final Pose2d pose = getPose();

		final double vx = sample.vx + xController.calculate(pose.getX(), sample.x);
		final double vy = sample.vy + yController.calculate(pose.getY(), sample.y);
		final double omega =
				sample.omega + rController.calculate(pose.getRotation().getRadians(), sample.heading);

		setFieldRelative(new ChassisSpeeds(vx, vy, omega));
	}

	// ── Pose / state ─────────────────────────────────────────────────────────────

	/** Current odometry pose. */
	@Logged(key = "Pose", importance = Importance.CRITICAL)
	public Pose2d getPose() {
		return this.getState().Pose;
	}

	/** Reset odometry pose. */
	public void setPose(Pose2d pose) {
		this.resetPose(pose);
	}

	/** Processed module states (velocity + angle). */
	@Logged(key = "Module States", importance = Importance.INFO)
	public SwerveModuleState[] getModuleStates() {
		return this.getState().ModuleStates;
	}

	/** Module setpoints. */
	@Logged(key = "Module Setpoints", importance = Importance.DEBUG)
	public SwerveModuleState[] getModuleSetpoints() {
		return this.getState().ModuleTargets;
	}

	/** Module positions (distance + angle). */
	public SwerveModulePosition[] getModulePositions() {
		return this.getState().ModulePositions;
	}

	/** Robot-relative chassis speeds (from kinematics). */
	@Logged(key = "Chassis Speeds", importance = Importance.INFO)
	public ChassisSpeeds getChassisSpeeds() {
		return this.getState().Speeds;
	}

	/** Cached setpoint speeds we most recently commanded. */
	@Logged(key = "Setpoint Speeds", importance = Importance.DEBUG)
	public ChassisSpeeds getSetpointSpeeds() {
		return setpointSpeeds;
	}

	/** Arbitrary "sample" pose holder for your UI/logging. */
	public void setSample(Pose2d pose) {
		this.samplePose = pose;
	}

	// @Logged(key = "Sample Pose", importance = Importance.DEBUG)
	public Pose2d getSample() {
		return samplePose;
	}

	// @Logged(key = "Pose Estimates", importance = Importance.DEBUG)
	public Pose2d[] getEstimatedPosesFromCameras() {
		return estimatedPosesFromCameras;
	}

	// ── Gyro ─────────────────────────────────────────────────────────────────────

	/** Zero yaw to alliance-forward (Blue=0°, Red=180°). */
	public void zeroGyro() {
		final double yawDeg =
				DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 180;
		gyro.setYaw(yawDeg);
	}

	/** Zero yaw to custom angle. */
	public void zeroGyro(Rotation2d start) {
		gyro.setYaw(start.getDegrees());
	}

	/** Field heading from gyro (deg → Rotation2d). */
	@Logged(key = "Heading", importance = Importance.CRITICAL)
	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(gyro.getYaw(true).getValue().in(Degrees));
	}

	// ── Vision ───────────────────────────────────────────────────────────────────

	/** Timestamp is FPGA time in seconds. */
	public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
		if (visionPose == null) return;
		super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestampSeconds));
	}

	// ── Command triggers ─────────────────────────────────────────────────────────

	public void bindCommands() {
		Driver driver = RobotContainer.getDriver();
		this.setDefaultCommand(new TeleopSwerve(this, driver.leftY(), driver.leftX(), driver.rightX()));
	}

	// ── WPILib lifecycle ─────────────────────────────────────────────────────────

	@Override
	public void periodic() {
		// Apply operator perspective each disable/enable cycle so restarts keep alignment sane.
		if (!appliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance()
					.ifPresent(
							alliance -> {
								setOperatorPerspectiveForward(
										alliance == Alliance.Red ? kRedPerspective : kBluePerspective);
								appliedOperatorPerspective = true;
							});
		}

		AprilTagCamera[] cameras = RobotContainer.getAprilTagCameras();
		if (estimatedPosesFromCameras == null || cameras.length != estimatedPosesFromCameras.length) {
			estimatedPosesFromCameras = new Pose2d[cameras.length];
		}
		for (int i = 0; i < cameras.length; i++) {
			cameras[i].updateHeading(getHeading());
			cameras[i].updatePoseEstimator(getPose());
			if (cameras[i].getEstimatedPose() != null) {
				addVisionMeasurement(
						cameras[i].getEstimatedPose().estimatedPose.toPose2d(),
						cameras[i].getEstimatedPose().timestampSeconds);
				estimatedPosesFromCameras[i] = cameras[i].getEstimatedPose().estimatedPose.toPose2d();
			}
		}
	}

	// ── Internals ────────────────────────────────────────────────────────────────

	private void startSimThread() {
		lastSimTimeSec = Utils.getCurrentTimeSeconds();

		simNotifier =
				new Notifier(
						() -> {
							final double now = Utils.getCurrentTimeSeconds();
							final double dt = now - lastSimTimeSec;
							lastSimTimeSec = now;
							updateSimState(dt, RobotController.getBatteryVoltage());
						});
		simNotifier.startPeriodic(kSimLoopPeriod);
	}
}
