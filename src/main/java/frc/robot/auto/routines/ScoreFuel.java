package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants.IntakeLocation;
import frc.robot.auto.AutoConstants.ScoringLocation;
import frc.robot.auto.AutoConstants.StartingPosition;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.intake.Intake;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.FunctionalUtil;

public class ScoreFuel extends AutoRoutine {

	private static final AutoQuestion<StartingPosition> startPosition = new AutoQuestion<StartingPosition>("Starting Position") {
		// private static final Map.Entry<String, StartingPosition> startLeftBump =           Settings.option("LB",  StartingPosition.LEFT_BUMP);
		// private static final Map.Entry<String, StartingPosition> startRightBump =          Settings.option("RB",  StartingPosition.RIGHT_BUMP);
		private static final Map.Entry<String, StartingPosition> startCenter =             Settings.option("Center",   StartingPosition.CENTER);
		private static final Map.Entry<String, StartingPosition> startInsideLeftTrench =   Settings.option("L Trench", StartingPosition.INSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, StartingPosition> startInsideRightTrench =  Settings.option("R Trench", StartingPosition.INSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<StartingPosition> generateSettings() {
			return Settings.from(startInsideRightTrench, startInsideLeftTrench, startCenter, startInsideRightTrench);
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
		private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("Opponent Swipe", IntakeLocation.OPPONENT_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOpponentSwipe = Settings.option("Half Opponent Swipe", IntakeLocation.HALF_OPPONENT_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfSweep = Settings.option("Half Sweep", IntakeLocation.HALF_SWEEP);
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
					halfOuterSwipe,
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
					outsideLeftTrench,
					outsideLeftTrench,
					outsideRightTrench,
					left
				);
			} else if (
				startPosition == ScoringLocation.OUTSIDE_RIGHT_TRENCH
			) {
				return Settings.from(
					outsideRightTrench,
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
					halfOuterSwipe,
					halfOuterSwipe,
					halfInnerSwipe
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
					halfInnerSwipe,
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

	private final RobotContainer robot;
	private final Drive drive;
	private final Intake intake;

	public ScoreFuel(RobotContainer robot) {
		super("Score Fuel", List.of(
			startPosition,
			firstScoringLocation,
			firstIntakeLocation,
			secondScoringLocation,
			secondIntakeLocation
		));
		this.robot = robot;
		this.drive = robot.drive;
		this.intake = robot.intake;
	}

	@Override
	public Command generateCommand() {
		var startPosition = ScoreFuel.startPosition.getResponse();
		var firstIntakeLocation = ScoreFuel.firstIntakeLocation.getResponse();
		var firstScoringLocation = ScoreFuel.firstScoringLocation.getResponse();
		var secondIntakeLocation = ScoreFuel.secondIntakeLocation.getResponse();
		var secondScoringLocation = ScoreFuel.secondScoringLocation.getResponse();

		var commands = new ArrayList<Command>();
		var firstTraj = generatePath(startPosition, firstScoringLocation, firstIntakeLocation).getOurs();
		var firstPathCommand = new FollowTrajectoryCommand(this.drive, firstTraj, true);
		var firstCommand = Commands.deadline(
			Commands.sequence(
				Commands.deadline(
					firstPathCommand,
					this.robot.intake.rollers.intake().asProxy()
				),
				Commands.deadline(
					Commands.waitSeconds(4.0),
					this.robot.shooter.aimHoodAtHub().asProxy(),
					this.robot.shooter.aimDriveAtHub(this.robot.drive.rotationalSubsystem).asProxy(),
					this.robot.rollers.feed().onlyWhile(() -> this.robot.shooter.withinTolerance()).repeatedly().withName("Feed when ready").asProxy()
				)
			),
			this.robot.shooter.aimingSystem.aimAtHub(
				FunctionalUtil.evalNow(firstTraj.getFinalPose(false).get()),
				FunctionalUtil.evalNow(new ChassisSpeeds()),
				FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
			).asProxy(),
			this.robot.shooter.aimFlywheelAtHub().asProxy()
		);
		commands.add(firstCommand);

		var secondTraj = generatePath(firstScoringLocation, secondScoringLocation, secondIntakeLocation).getOurs();
		var secondPathCommand = new FollowTrajectoryCommand(this.drive, firstTraj, true);
		var secondCommand = Commands.deadline(
			Commands.sequence(
				Commands.deadline(
					secondPathCommand,
					this.robot.intake.rollers.intake().asProxy()
				),
				Commands.deadline(
					Commands.waitSeconds(4.0),
					this.robot.shooter.aimHoodAtHub().asProxy(),
					this.robot.shooter.aimDriveAtHub(this.robot.drive.rotationalSubsystem).asProxy(),
					this.robot.rollers.feed().onlyWhile(() -> this.robot.shooter.withinTolerance()).repeatedly().withName("Feed when ready").asProxy()
				)
			),
			this.robot.shooter.aimingSystem.aimAtHub(
				FunctionalUtil.evalNow(secondTraj.getFinalPose(false).get()),
				FunctionalUtil.evalNow(new ChassisSpeeds()),
				FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
			).asProxy(),
			this.robot.shooter.aimFlywheelAtHub().asProxy()
		);
		commands.add(secondCommand);

		return Commands.parallel(
			AutoCommons.setOdometryFlipped(startPosition.pose),
			Commands.parallel(
				this.intake.slam.deploy(this.robot.extensionSystem),
				Commands.sequence(commands.toArray(Command[]::new))
			)
		);
	}


	private static AllianceFlipped<Trajectory<SwerveSample>> generatePath(StartingPosition startingPosition, ScoringLocation scoringLocation, IntakeLocation intakeLocation) {
		if (startingPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoringLocation.canFlip * intakeLocation.canFlip > 0) {
			//Needs to flip across Xenterline
			var blueTraj = AutoCommons.loadBlueChoreoTrajectory(startingPosition.rightAlias + "To" + scoringLocation.rightAlias + "Intake" + intakeLocation.alias);
			var newBlueTraj = AllianceFlipUtil.flip(blueTraj.getBlue(), FieldFlipType.XenterLineMirror);
			return AllianceFlipped.fromBlue(newBlueTraj);
		} else {
			return AutoCommons.loadBlueChoreoTrajectory(startingPosition.alias + "To" + scoringLocation.alias + "Intake" + intakeLocation.alias);
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
