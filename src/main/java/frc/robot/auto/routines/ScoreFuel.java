package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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

public class ScoreFuel extends AutoRoutine {

	private static final AutoQuestion<StartingPosition> startPosition = new AutoQuestion<StartingPosition>("Starting Position") {
		// private static final Map.Entry<String, StartingPosition> startLeftBump =           Settings.option("LB",  StartingPosition.LEFT_BUMP);
		// private static final Map.Entry<String, StartingPosition> startRightBump =          Settings.option("RB",  StartingPosition.RIGHT_BUMP);
		private static final Map.Entry<String, StartingPosition> startCenter =             Settings.option("Center",   StartingPosition.CENTER);
		private static final Map.Entry<String, StartingPosition> startInsideLeftTrench =   Settings.option("L Trench", StartingPosition.INSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, StartingPosition> startInsideRightTrench =  Settings.option("R Trench", StartingPosition.INSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<StartingPosition> generateSettings() {
			return Settings.from(startInsideLeftTrench, startInsideLeftTrench, startInsideRightTrench);
		}
	};

	private static final AutoQuestion<ScoringLocation> firstScoringLocation = new AutoQuestion<ScoringLocation>("First Score Location") {
		// private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		// private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> center = Settings.option("Center", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("L Trench", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("R Trench", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			var startPosition = ScoreFuel.startPosition.getResponse();
			if (
				startPosition == StartingPosition.CENTER
			) {
				return Settings.from(
					center,
					center
				);
			} else if (
				startPosition == StartingPosition.INSIDE_LEFT_TRENCH
			) {
				return Settings.from(
					outsideLeftTrench,
					outsideLeftTrench,
					outsideRightTrench
				);
			} else if (
				startPosition == StartingPosition.INSIDE_RIGHT_TRENCH
			) {
				return Settings.from(
					outsideRightTrench,
					outsideLeftTrench,
					outsideRightTrench
				);
			} else {
				return null;
			}
		}
	};

	private static final AutoQuestion<IntakeLocation> firstIntakeLocation = new AutoQuestion<IntakeLocation>("First Intake Location") {
		private static final Map.Entry<String, IntakeLocation> fullOuterSwipe = Settings.option("Full Outer Swipe", IntakeLocation.FULL_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> fullInnerSwipe = Settings.option("Full Inner Swipe", IntakeLocation.FULL_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOuterSwipe = Settings.option("Half Outer Swipe", IntakeLocation.HALF_OUTER_SWIPE);
		// private static final Map.Entry<String, IntakeLocation> halfInnerSwipe = Settings.option("HIS", IntakeLocation.HALF_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("Bumper Obliterator 10000", IntakeLocation.OPPONENT_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOpponentSwipe = Settings.option("Intake Obliterator 5000", IntakeLocation.HALF_OPPONENT_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfSweep = Settings.option("Bumper Obliterator 5000", IntakeLocation.HALF_SWEEP);
		private static final Map.Entry<String, IntakeLocation> depot = Settings.option("Depot", IntakeLocation.DEPOT);
		// private static final Map.Entry<String, IntakeLocation> outpost = Settings.option("O", IntakeLocation.OUTPOST);
		private static final Map.Entry<String, IntakeLocation> noIntake = Settings.option("No Intake", IntakeLocation.FALSE);

		@Override
		protected Settings<IntakeLocation> generateSettings() {
			var startPosition = ScoreFuel.startPosition.getResponse();
			var scoreLocation = ScoreFuel.firstScoringLocation.getResponse();
			if (
				(startPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_LEFT_TRENCH) ||
				(startPosition == StartingPosition.INSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_RIGHT_TRENCH)
			) {
				return Settings.from(
					halfSweep,
					halfOuterSwipe,
					halfOpponentSwipe,
					halfSweep
				);
			} else if (
				(startPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_RIGHT_TRENCH) ||
				(startPosition == StartingPosition.INSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_LEFT_TRENCH)
			) {
				return Settings.from(
					fullOuterSwipe,
					fullOuterSwipe,
					fullInnerSwipe,
					opponentSwipe
				);
			} else if (
				startPosition == StartingPosition.CENTER
			) {
				return Settings.from(
					noIntake,
					noIntake,
					depot
				);
			} else {
				return null;
			}
		}
	};

	private static final AutoQuestion<ScoringLocation> secondScoringLocation = new AutoQuestion<ScoringLocation>("Second Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L Bump", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R Bump", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> none = Settings.option("NONE", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("L Trench", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("R Trench", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			var startPosition = ScoreFuel.firstScoringLocation.getResponse();
			var firstIntakeLocation = ScoreFuel.firstIntakeLocation.getResponse();
			if (
				firstIntakeLocation == IntakeLocation.FALSE || firstIntakeLocation == IntakeLocation.DEPOT
			) {
				return Settings.from(none, none);
			} else if (
				startPosition == ScoringLocation.OUTSIDE_LEFT_TRENCH
			) {
				return Settings.from(
					left,
					outsideLeftTrench,
					outsideRightTrench,
					left
				);
			} else if (
				startPosition == ScoringLocation.OUTSIDE_RIGHT_TRENCH
			) {
				return Settings.from(
					right,
					outsideLeftTrench,
					outsideRightTrench,
					right
				);
			} else {
				return null;
			}
		}
	};

	private static final AutoQuestion<IntakeLocation> secondIntakeLocation = new AutoQuestion<IntakeLocation>("Second Intake Location") {
		// private static final Map.Entry<String, IntakeLocation> fullOuterSwipe = Settings.option("FOS", IntakeLocation.FULL_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> fullInnerSwipe = Settings.option("Full Inner Swipe", IntakeLocation.FULL_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOuterSwipe = Settings.option("Half Outer Swipe", IntakeLocation.HALF_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfInnerSwipe = Settings.option("Half Inner Swipe", IntakeLocation.HALF_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfInnerLoopSwipe = Settings.option("Half Inner Loop Swipe", IntakeLocation.HALF_INNER_LOOP_SWIPE);
		// private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("OpS", IntakeLocation.OPPONENT_SWIPE);
		// private static final Map.Entry<String, IntakeLocation> halfOpponentSwipe = Settings.option("HOpS", IntakeLocation.HALF_OPPONENT_SWIPE);
		// private static final Map.Entry<String, IntakeLocation> halfSweep = Settings.option("HSw", IntakeLocation.HALF_SWEEP);
		// private static final Map.Entry<String, IntakeLocation> depot = Settings.option("D", IntakeLocation.DEPOT);
		// private static final Map.Entry<String, IntakeLocation> outpost = Settings.option("O", IntakeLocation.OUTPOST)
		private static final Map.Entry<String, IntakeLocation> noIntake = Settings.option("No Intake", IntakeLocation.FALSE);

		@Override
		protected Settings<IntakeLocation> generateSettings() {
			var startPosition = ScoreFuel.firstScoringLocation.getResponse();
			var scoreLocation = ScoreFuel.secondScoringLocation.getResponse();
			if (
				(startPosition == ScoringLocation.OUTSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_LEFT_TRENCH) ||
				(startPosition == ScoringLocation.OUTSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_RIGHT_TRENCH)
			) {
				return Settings.from(
					halfInnerLoopSwipe,
					halfInnerSwipe,
					halfOuterSwipe,
					halfInnerLoopSwipe
				);
			} else if (
				(startPosition == ScoringLocation.OUTSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_LEFT_TRENCH) ||
				(startPosition == ScoringLocation.OUTSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_RIGHT_TRENCH)
			) {
				return Settings.from(
					fullInnerSwipe,
					fullInnerSwipe
				);
			} else if (
				(startPosition == ScoringLocation.OUTSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.LEFT) ||
				(startPosition == ScoringLocation.OUTSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.RIGHT)
			) {
				return Settings.from(
					halfInnerLoopSwipe,
					halfInnerLoopSwipe,
					halfInnerSwipe,
					halfOuterSwipe
				);
			} else if (
				startPosition == null
			) {
				return Settings.from(
					noIntake,
					noIntake
				);
			} else {
				return null;
			}
		}
	};

	private static final AutoQuestion<Boolean> enableTransitionCutoff = new AutoQuestion<Boolean>("Transition Cutoff") {
		private static final Map.Entry<String, Boolean> yes = Settings.option("Yes", true);
		private static final Map.Entry<String, Boolean> no = Settings.option("No", false);

		@Override
		protected Settings<Boolean> generateSettings() {
			return Settings.from(no, yes, no);
		}
	};

	private final RobotContainer robot;

	public ScoreFuel(RobotContainer robot) {
		super("Score Fuel", List.of(
			startPosition,
			firstScoringLocation,
			firstIntakeLocation,
			secondScoringLocation,
			secondIntakeLocation,
			enableTransitionCutoff
		));
		this.robot = robot;
	}

	@Override
	public Command generateCommand(DoubleSupplier autoTimer) {
		final var startPosition = ScoreFuel.startPosition.getResponse();
		final var firstIntakeLocation = ScoreFuel.firstIntakeLocation.getResponse();
		final var firstScoringLocation = ScoreFuel.firstScoringLocation.getResponse();
		final var secondIntakeLocation = ScoreFuel.secondIntakeLocation.getResponse();
		final var secondScoringLocation = ScoreFuel.secondScoringLocation.getResponse();
		final var enableTransitionCutoff = ScoreFuel.enableTransitionCutoff.getResponse();

		var commands = new ArrayList<Command>();
		var firstTraj = generatePath(startPosition, firstScoringLocation, firstIntakeLocation, false).getOurs();
		var firstCommand = AutoCommons.swipe(
			this.robot,
			firstTraj,
			2.0,
			0.5,
			0.5,
			2.5,
			18.5,
			15.5,
			autoTimer,
			enableTransitionCutoff
		);
		commands.add(firstCommand);

		var secondTraj = generatePath(firstScoringLocation, secondScoringLocation, secondIntakeLocation).getOurs();
		var secondCommand = AutoCommons.swipe(
			this.robot,
			secondTraj,
			0.0,
			0.0,
			0.5,
			2.5,
			18.5,
			15.5,
			autoTimer,
			enableTransitionCutoff
		);
		commands.add(secondCommand);

		var thirdTraj = generatePath(secondScoringLocation, ScoringLocation.STOP, IntakeLocation.HALF_INNER_SWIPE).getOurs();
		var thirdCommand = Commands.parallel(
			Commands.deadline(
				new FollowTrajectoryCommand(this.robot.drive, thirdTraj, true).withName("Grab Ball").asProxy(),
				this.robot.intake.rollers.intake().asProxy()
			),
			this.robot.intake.slam.deploy(robot.extensionSystem).asProxy()
		);
		commands.add(thirdCommand);

		return Commands.parallel(
			AutoCommons.setOdometryFlipped(startPosition.pose),
			Commands.sequence(commands.toArray(Command[]::new))
		);
	}


	private static AllianceFlipped<Trajectory<SwerveSample>> generatePath(StartingPosition startingPosition, ScoringLocation scoringLocation, IntakeLocation intakeLocation, boolean comeBackThroughTrenchSideways) {
		if (startingPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoringLocation.canFlip * intakeLocation.canFlip > 0) {
			//Needs to flip across Xenterline
			var blueTraj = AutoCommons.loadBlueChoreoTrajectory(startingPosition.rightAlias + "To" + scoringLocation.rightAlias + "Intake" + intakeLocation.alias + (comeBackThroughTrenchSideways ? "_Sideswipe" : ""));
			var newBlueTraj = AllianceFlipUtil.flip(blueTraj.getBlue(), FieldFlipType.XenterLineMirror);
			return AllianceFlipped.fromBlue(newBlueTraj);
		} else {
			return AutoCommons.loadBlueChoreoTrajectory(startingPosition.alias + "To" + scoringLocation.alias + "Intake" + intakeLocation.alias + (comeBackThroughTrenchSideways ? "_Sideswipe" : ""));
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
