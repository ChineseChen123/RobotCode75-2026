package frc.lib.dashboard;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.Auto.ActionFactory;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.EnumSet;
import java.util.Map;

/*
 * Takes in a string to parse into an auto
 * Enter Command - place to type/paste in the string
 * Feedback - displays errors, shows completed path
 * Generate - checks paths for errors, displays trajectories on field, sets auto command
 * Reset - reset trajectories, string, and auto command
 * Presets - sendable chooser of preset autos (must click generate after selecting)
 *
 * FORMAT FOR AUTO STRING
 * Separated into words by spaces - commands in the same word are executed simultaneously
 * Each word can contain one point and one action max
 *  - All points are strings of letters (case insensitive)
 *  - All actions are numbers
 * !!! The first word MUST be only one starting point (SD/SM/SO) without an action
 * Points are labeled according to whether they are closer to depot/outpost
 */

/* ACTIONS
 * 1 - start shooter/turret aligning
 * 2 - stop shooter/turret aligning
 * 3 - start indexer
 * 4 - stop indexer
 * 5 - run intake
 * 6 - stop intake
 * 7 - shoot 8 preload
 */

/* POINTS
 * SD - start depot (3.6044533252716064, 7.642, 180)
 * SM - start middle ()
 * SO - start outpost (3.6044533252716064, 0.42545, 180)
 *
 * D - depot ()
 * O - outpost ()
 * ND - depot side neutral zone ()
 * NO - outpost side neutral zone ()
 * TO - outpost side trench ()
 * TD - depot side trench ()
 *
 * TWD - depot side tower ()
 * TWO - outpost side tower ()
 */

public class AutoSelector {
	private Command m_autoCommand = Commands.runOnce(() -> {});
	private Pose2d m_startPose;
	private final ActionFactory m_actionFactory;
	private final Swerve m_swerve;
	private Map<String, Pose2d> m_startPositions;

	private GenericEntry autoStringEntry;
	private GenericEntry feedbackEntry;

	private final SendableChooser<String> presetChooser;

	public final AutoFactory choreoFactory;

	public AutoSelector() {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		NetworkTable table = nt.getTable("Shuffleboard").getSubTable("Auto");
		autoStringEntry = table.getTopic("Enter Command").getGenericEntry();
		feedbackEntry = table.getTopic("Feedback").getGenericEntry();
		feedbackEntry.setString("Enter a command!");

		m_actionFactory = new ActionFactory();
		m_swerve = RobotContainer.getSwerve();

		// initialize presets
		presetChooser = new SendableChooser<>();
		presetChooser.setDefaultOption("Custom", "");
		presetChooser.addOption("Outpost Side NZ + Outpost", Presets.outpostSideNZOutpost);
		presetChooser.addOption("Depot Side NZ + Depot", Presets.depotSideNZDepot);
		presetChooser.addOption("Mid Simple", Presets.midSimple);
		presetChooser.addOption("Depot Side NZ + Feed", Presets.depotSideNZFeed);
		presetChooser.addOption("Outpost Side NZ + Feed", Presets.outpostSideNZFeed);

		// define auto factory for autos
		choreoFactory =
				new AutoFactory(
						m_swerve::getPose, m_swerve::setPose, m_swerve::followSwerveSample, true, m_swerve);
	}

	/** complete reset */
	public void reset() {
		autoStringEntry.setString("");
		feedbackEntry.setString("Enter a command!");
		m_autoCommand = Commands.runOnce(() -> {});
		m_startPose = null;
	}

	/** set text in text entry on dashboard */
	public void setFeedback(String feedback) {
		feedbackEntry.setString(feedback);
	}

	public String getFeedback() {
		return feedbackEntry.getString("");
	}

	/** set up widgets and listeners on dashboard */
	public void setupAutoTab() {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		NetworkTable ntTable = nt.getTable("Shuffleboard").getSubTable("Auto");

		ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

		autoTab.add("Enter Command", "").withSize(4, 1).withPosition(0, 0);

		autoTab.add(presetChooser).withSize(2, 1).withPosition(2, 2);

		autoTab.addString("Feedback", () -> getFeedback()).withSize(8, 1).withPosition(0, 1);

		autoTab
				.add("Generate", true)
				.withWidget(BuiltInWidgets.kToggleButton)
				.withSize(1, 1)
				.withPosition(0, 2);
		autoTab
				.add("Reset", true)
				.withWidget(BuiltInWidgets.kToggleButton)
				.withSize(1, 1)
				.withPosition(1, 2);

		// bind button clicks to run methods
		ntTable.addListener(
				"Generate",
				EnumSet.of(Kind.kValueAll),
				(table, key, event) -> {
					generatePaths();
				});
		ntTable.addListener(
				"Reset",
				EnumSet.of(Kind.kValueAll),
				(table, key, event) -> {
					reset();
				});

		if (DriverStation.getAlliance().isEmpty()
				|| DriverStation.getAlliance().get() == Alliance.Blue) {
			m_startPositions = AutoConstants.blueStartPositions;
		} else {
			m_startPositions = AutoConstants.redStartPositions;
		}
	}

	/** parse specified string and convert into command */
	/** takes a couple seconds - has to pull Choreo paths from Rio */
	public void generatePaths() {
		setFeedback("Generating paths...");

		// if there is no preset, just use the string
		if (presetChooser.getSelected() != "") {
			autoStringEntry.setString(presetChooser.getSelected());
		}

		// note that this gets set by preset if a preset is chosen
		String autoString = autoStringEntry.getString("");
		String[] words = autoString.split(" ");

		// if (!m_startPositions.containsKey(words[0].toLowerCase())) {
		// 	setFeedback("Invalid start position");
		// 	return;
		// }
		// m_startPose = m_startPositions.get(words[0].toLowerCase());

		SequentialCommandGroup sequential = new SequentialCommandGroup();
		// feedback string with parsed commands
		StringBuilder s = new StringBuilder();
		boolean isOdometryReset = false;

		if (autoString.length() == 0) {
			m_autoCommand = sequential;
			setFeedback("Empty path. Is this intentional?");
			return;
		}

		String lastPose = "";
		for (int i = 0; i < words.length; i++) {
			Command parallelGroup = null;
			// parse movement and actions separately in each word
			StringBuilder pointString = new StringBuilder();
			StringBuilder actionString = new StringBuilder();
			for (int j = 0; j < words[i].length(); j++) {
				char c = words[i].charAt(j);
				if (Character.isLetter(c)) {
					pointString.append(c);
				} else {
					actionString.append(c);
				}
			}

			String point = pointString.toString().toLowerCase();
			int action = actionString.length() > 0 ? Integer.parseInt(actionString.toString()) : -1;

			if (lastPose == "" && point != "") {
				lastPose = point;
				continue;
			}
			if (point != "" && lastPose != "") {
				try {
					// m_trajectories.add(
					//     new ChoreoTrajectory(Choreo.loadTrajectory("" + lastPose + "_" + point).get()));
					// so sdo
					// reset pose and gyro if not done yet
					if (!isOdometryReset) {
						var trajectory = Choreo.loadTrajectory("" + lastPose + "_" + point);
						sequential.addCommands(
								Commands.runOnce(
										() ->
												m_swerve.zeroGyro(
														trajectory
																.get()
																.getInitialPose(DriverStation.getAlliance().get() == Alliance.Red)
																.get()
																.getRotation())),
								choreoFactory.resetOdometry("" + lastPose + "_" + point));
						isOdometryReset = true;
					}

					// generate movement command and add to group
					parallelGroup = choreoFactory.trajectoryCmd("" + lastPose + "_" + point);
					// if (DriverStation.getAlliance().get() == Alliance.Red) {
					//   m_trajectories.set(
					//       m_trajectories.size() - 1,
					//       new ChoreoTrajectory(m_trajectories.get(m_trajectories.size() -
					// 1).traj.flipped()));
					// }
					s.append("" + lastPose + "-" + point + " ");
					lastPose = point;
				} catch (Exception e) {
					setFeedback(e.getMessage());
					m_autoCommand = Commands.runOnce(() -> {});
					return;
				}
			}
			if (action != -1 && m_actionFactory.getCommand(action) != null) {
				// convert action number into command and add to group
				if (parallelGroup != null) {
					// with movement, action ends when path ends
					parallelGroup = parallelGroup.deadlineFor(m_actionFactory.getCommand(action));
				} else {
					// no movement - run action until done
					parallelGroup = m_actionFactory.getCommand(action);
				}
				s.append(m_actionFactory.getName(action) + " ");
			} else if (action != -1) {
				setFeedback("Action Not Found: " + action);
				m_autoCommand = Commands.runOnce(() -> {});
				return;
			}
			sequential.addCommands(parallelGroup);
			sequential.addCommands(new InstantCommand(() -> m_swerve.stopModules(), m_swerve));
		}

		setFeedback(s.toString());
		m_autoCommand = sequential;
	}

	/** return generated command */
	public Command getAutoCommand() {
		return m_autoCommand;
	}

	public Pose2d getStartPose() {
		return m_startPose;
	}
}
