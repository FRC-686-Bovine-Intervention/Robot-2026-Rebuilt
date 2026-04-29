package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants.IntakeLocation;
import frc.robot.auto.AutoConstants.ScoringLocation;
import frc.robot.auto.AutoConstants.StartingPosition;
import frc.robot.auto.AutoRoutine;
import frc.robot.subsystems.drive.commands.FollowTrajectoryCommand;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.flipping.AllianceFlipped;

public class CompatSwipe extends AutoRoutine {

	private static final AutoQuestion<StartingPosition> startPosition = new AutoQuestion<>("Starting Position") {
		private static final Map.Entry<String, StartingPosition> startOutsideLeftTrench =   Settings.option("L Trench", StartingPosition.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, StartingPosition> startLeftBump =           Settings.option("L Bump",  StartingPosition.LEFT_BUMP);
		private static final Map.Entry<String, StartingPosition> startCenter =             Settings.option("Center",   StartingPosition.CENTER);
		private static final Map.Entry<String, StartingPosition> startRightBump =          Settings.option("R Bump",  StartingPosition.RIGHT_BUMP);
		private static final Map.Entry<String, StartingPosition> startOutsideRightTrench =  Settings.option("R Trench", StartingPosition.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<StartingPosition> generateSettings() {
			return Settings.from(
				startOutsideLeftTrench,
				startOutsideLeftTrench,
				startLeftBump,
				startCenter,
				startRightBump,
				startOutsideRightTrench
			);
		}
	};

	private static enum IntakePath {
		TrenchBumperObliterator,
		TrenchInnerSwipe,
		Bump,
		Depot,
	}

	private static final AutoQuestion<IntakePath> firstIntakePath = new AutoQuestion<>("First Intake Path") {
		private static final Map.Entry<String, IntakePath> trenchBumperObliterator = Settings.option("Trench Bumper Obliterator", IntakePath.TrenchBumperObliterator);
		private static final Map.Entry<String, IntakePath> trenchInnerSwipe = Settings.option("Trench Inner Swipe", IntakePath.TrenchBumperObliterator);
		private static final Map.Entry<String, IntakePath> bump = Settings.option("Bump", IntakePath.Bump);
		private static final Map.Entry<String, IntakePath> depot = Settings.option("Depot", IntakePath.Depot);

		@Override
		protected Settings<IntakePath> generateSettings() {
			final var startPosition = CompatSwipe.startPosition.getResponse();
			if (
				startPosition == StartingPosition.OUTSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.INSIDE_LEFT_TRENCH
			) {
				return Settings.from(
					trenchBumperObliterator,
					trenchBumperObliterator,
					trenchInnerSwipe,
					bump,
					depot
				);
			} else if (
				startPosition == StartingPosition.LEFT_BUMP
			) {
				return Settings.from(
					bump,
					trenchBumperObliterator,
					trenchInnerSwipe,
					bump,
					depot
				);
			} else if (
				startPosition == StartingPosition.CENTER
			) {
				return Settings.from(
					depot,
					depot
				);
			} else if (
				startPosition == StartingPosition.RIGHT_BUMP
			) {
				return Settings.from(
					bump,
					bump,
					trenchBumperObliterator,
					trenchInnerSwipe
				);
			} else {
				return Settings.from(
					trenchBumperObliterator,
					bump,
					trenchBumperObliterator,
					trenchInnerSwipe
				);
			}
		}
	};

	private static enum ReturnPath {
		TrenchFast,
		TrenchAway,
		BumpFast,
		BumpAway,

	}

	private static final AutoQuestion<ReturnPath> firstReturnPath = new AutoQuestion<>("First Return Path") {
		private static final Map.Entry<String, ReturnPath> trenchFast = Settings.option("Trench Fast", ReturnPath.TrenchFast);
		private static final Map.Entry<String, ReturnPath> trenchAway = Settings.option("Trench Away", ReturnPath.TrenchAway);
		private static final Map.Entry<String, ReturnPath> bumpFast = Settings.option("Bump Fast", ReturnPath.BumpFast);
		private static final Map.Entry<String, ReturnPath> bumpAway = Settings.option("Bump Away", ReturnPath.BumpAway);

		@Override
		protected Settings<ReturnPath> generateSettings() {
			final var startPosition = CompatSwipe.startPosition.getResponse();
			final var firstIntakePath = CompatSwipe.firstIntakePath.getResponse();
			if (
				firstIntakePath == IntakePath.Depot
			) {
				return Settings.from(
					bumpAway,
					bumpAway
				);
			} else if (
				startPosition == StartingPosition.OUTSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.INSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.LEFT_BUMP
			) {
				if (
					firstIntakePath == IntakePath.TrenchBumperObliterator
					|| firstIntakePath == IntakePath.TrenchInnerSwipe
				) {
					return Settings.from(
						trenchFast,
						trenchFast,
						trenchAway,
						bumpFast,
						bumpAway
					);
				} else {
					return Settings.from(
						bumpFast,
						trenchFast,
						trenchAway,
						bumpFast,
						bumpAway
					);
				}
			} else {
				if (
					firstIntakePath == IntakePath.TrenchBumperObliterator
					|| firstIntakePath == IntakePath.TrenchInnerSwipe
				) {
					return Settings.from(
						trenchFast,
						bumpFast,
						bumpAway,
						trenchFast,
						trenchAway
					);
				} else {
					return Settings.from(
						bumpFast,
						bumpFast,
						bumpAway,
						trenchFast,
						trenchAway
					);
				}
			}
		}
	};

	private static final AutoQuestion<Optional<IntakePath>> secondIntakePath = new AutoQuestion<>("Second Intake Path") {
		private static final Map.Entry<String, Optional<IntakePath>> trenchBumperObliterator = Settings.option("Trench Bumper Obliterator", Optional.of(IntakePath.TrenchBumperObliterator));
		private static final Map.Entry<String, Optional<IntakePath>> trenchInnerSwipe = Settings.option("Trench Inner Swipe", Optional.of(IntakePath.TrenchBumperObliterator));
		private static final Map.Entry<String, Optional<IntakePath>> bump = Settings.option("Bump", Optional.of(IntakePath.Bump));
		private static final Map.Entry<String, Optional<IntakePath>> depot = Settings.option("Depot", Optional.of(IntakePath.Depot));
		private static final Map.Entry<String, Optional<IntakePath>> noIntake = Settings.option("None", Optional.empty());

		@Override
		protected Settings<Optional<IntakePath>> generateSettings() {
			final var startPosition = CompatSwipe.startPosition.getResponse();
			final var firstIntakePath = CompatSwipe.firstIntakePath.getResponse();
			final var firstReturnPath = CompatSwipe.firstReturnPath.getResponse();

			if (
				startPosition == StartingPosition.OUTSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.INSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.LEFT_BUMP
			) {
				if (
					firstReturnPath == ReturnPath.TrenchFast
					|| firstReturnPath == ReturnPath.TrenchAway
				) {
					if (firstIntakePath == IntakePath.Depot) {
						return Settings.from(
							trenchInnerSwipe,
							trenchBumperObliterator,
							trenchInnerSwipe,
							bump,
							noIntake
						);
					} else {
						return Settings.from(
							trenchInnerSwipe,
							trenchBumperObliterator,
							trenchInnerSwipe,
							bump,
							depot,
							noIntake
						);
					}
				} else {
					if (firstIntakePath == IntakePath.Depot) {
						return Settings.from(
							bump,
							trenchBumperObliterator,
							trenchInnerSwipe,
							bump,
							noIntake
						);
					} else {
						return Settings.from(
							bump,
							trenchBumperObliterator,
							trenchInnerSwipe,
							bump,
							depot,
							noIntake
						);
					}
				}
			} else {
				if (
					firstReturnPath == ReturnPath.TrenchFast
					|| firstReturnPath == ReturnPath.TrenchAway
				) {
					return Settings.from(
						trenchInnerSwipe,
						bump,
						trenchBumperObliterator,
						trenchInnerSwipe,
						noIntake
					);
				} else {
					return Settings.from(
						bump,
						bump,
						trenchBumperObliterator,
						trenchInnerSwipe,
						noIntake
					);
				}
			}
		}
	};

	private static final AutoQuestion<Optional<ReturnPath>> secondReturnPath = new AutoQuestion<>("Second Return Path") {
		private static final Map.Entry<String, Optional<ReturnPath>> trenchFast = Settings.option("Trench Fast", Optional.of(ReturnPath.TrenchFast));
		private static final Map.Entry<String, Optional<ReturnPath>> trenchAway = Settings.option("Trench Away", Optional.of(ReturnPath.TrenchAway));
		private static final Map.Entry<String, Optional<ReturnPath>> bumpFast = Settings.option("Bump Fast", Optional.of(ReturnPath.BumpFast));
		private static final Map.Entry<String, Optional<ReturnPath>> bumpAway = Settings.option("Bump Away", Optional.of(ReturnPath.BumpAway));
		private static final Map.Entry<String, Optional<ReturnPath>> noReturn = Settings.option("None", Optional.empty());

		@Override
		protected Settings<Optional<ReturnPath>> generateSettings() {
			final var startPosition = CompatSwipe.startPosition.getResponse();
			final var secondIntakePath = CompatSwipe.secondIntakePath.getResponse();

			if (secondIntakePath.isEmpty()) {
				return Settings.from(
					noReturn,
					noReturn
				);
			}

			if (
				secondIntakePath.get() == IntakePath.Depot
			) {
				return Settings.from(
					bumpAway,
					bumpAway
				);
			} else if (
				startPosition == StartingPosition.OUTSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.INSIDE_LEFT_TRENCH
				|| startPosition == StartingPosition.LEFT_BUMP
			) {
				if (
					secondIntakePath.get() == IntakePath.TrenchBumperObliterator
					|| secondIntakePath.get() == IntakePath.TrenchInnerSwipe
				) {
					return Settings.from(
						trenchFast,
						trenchFast,
						trenchAway,
						bumpFast,
						bumpAway
					);
				} else {
					return Settings.from(
						bumpFast,
						trenchFast,
						trenchAway,
						bumpFast,
						bumpAway
					);
				}
			} else {
				if (
					secondIntakePath.get() == IntakePath.TrenchBumperObliterator
					|| secondIntakePath.get() == IntakePath.TrenchInnerSwipe
				) {
					return Settings.from(
						trenchFast,
						bumpFast,
						bumpAway,
						trenchFast,
						trenchAway
					);
				} else {
					return Settings.from(
						bumpFast,
						bumpFast,
						bumpAway,
						trenchFast,
						trenchAway
					);
				}
			}
		}
	};

	// private static final AutoQuestion<Boolean> enableTransitionCutoff = new AutoQuestion<>("Transition Cutoff") {
	// 	private static final Map.Entry<String, Boolean> yes = Settings.option("Yes", true);
	// 	private static final Map.Entry<String, Boolean> no = Settings.option("No", false);

	// 	@Override
	// 	protected Settings<Boolean> generateSettings() {
	// 		return Settings.from(no, yes, no);
	// 	}
	// };

	private final RobotContainer robot;

	public CompatSwipe(RobotContainer robot) {
		super("Compat Swipe", List.of(
			startPosition,
			firstIntakePath,
			firstReturnPath,
			secondIntakePath,
			secondReturnPath
			// enableTransitionCutoff
		));
		this.robot = robot;
	}

	@Override
	public Command generateCommand(DoubleSupplier autoTimer) {
		final var startPosition = CompatSwipe.startPosition.getResponse();
		final var firstIntakePath = CompatSwipe.firstIntakePath.getResponse();
		final var firstReturnPath = CompatSwipe.firstReturnPath.getResponse();
		final var secondIntakePath = CompatSwipe.secondIntakePath.getResponse();
		final var secondReturnPath = CompatSwipe.secondReturnPath.getResponse();
		// final var enableTransitionCutoff = CompatSwipe.enableTransitionCutoff.getResponse();

		final var commands = new ArrayList<Command>();
		final var firstTraj = generatePath(startPosition, firstReturnPath, firstIntakePath, false).getOurs();

		final var firstCommand = AutoCommons.swipe(
			this.robot,
			firstTraj,
			2.0,
			0.5,
			0.5,
			2.5,
			18.5,
			15.5,
			autoTimer,
			false,
			true
		);
		commands.add(firstCommand);

		final var secondTraj = generatePath(firstReturnPath, secondReturnPath, secondIntakePath).getOurs();
		final var secondCommand = AutoCommons.swipe(
			this.robot,
			secondTraj,
			0.0,
			0.0,
			0.5,
			2.5,
			18.5,
			15.5,
			autoTimer,
			false,
			true
		);
		commands.add(secondCommand);

		// final var thirdTraj = generatePath(secondReturnPath, ScoringLocation.STOP, IntakeLocation.HALF_INNER_SWIPE).getOurs();
		// final var thirdCommand = Commands.parallel(
		// 	Commands.deadline(
		// 		new FollowTrajectoryCommand(this.robot.drive, thirdTraj, true).withName("Grab Ball").asProxy(),
		// 		this.robot.intake.rollers.intake().asProxy()
		// 	),
		// 	this.robot.intake.slam.deploy(robot.extensionSystem).asProxy()
		// );
		// commands.add(thirdCommand);

		return Commands.parallel(
			AutoCommons.setOdometryFlipped(startPosition.pose),
			Commands.sequence(commands.toArray(Command[]::new))
		);
	}


	private static AllianceFlipped<Trajectory<SwerveSample>> generatePath(
		StartingPosition startingPosition,
		IntakePath intakePath,
		Optional<ReturnPath> returnPath
	) {
		if (
			(
				startingPosition == StartingPosition.OUTSIDE_LEFT_TRENCH
				|| startingPosition == StartingPosition.INSIDE_LEFT_TRENCH
				|| startingPosition == StartingPosition.LEFT_BUMP
			)
			&& intakePath != IntakePath.Depot
		) {

		}
	}

	private static AllianceFlipped<Trajectory<SwerveSample>> generatePath(ScoringLocation startingPosition, ScoringLocation scoringLocation, IntakeLocation intakeLocation) {
		if ((startingPosition == ScoringLocation.LEFT || startingPosition == ScoringLocation.OUTSIDE_LEFT_TRENCH) && scoringLocation.canFlip * startingPosition.canFlip * intakeLocation.canFlip > 0) {
			//Needs to flip across Xenterline
			var blueTraj = AutoCommons.loadBlueChoreoTrajectory(startingPosition.rightAlias + "To" + scoringLocation.rightAlias + "Intake" + intakeLocation.alias);
			var newBlueTraj = AllianceFlipUtil.flip(blueTraj.getBlue(), FieldFlipType.XenterLineMirror);
			return AllianceFlipped.fromBlue(newBlueTraj);
		} else {
			return AutoCommons.loadBlueChoreoTrajectory(startingPosition.alias + "To" + scoringLocation.alias + "Intake" + intakeLocation.alias);
		}
	}
}
