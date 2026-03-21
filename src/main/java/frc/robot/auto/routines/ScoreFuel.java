package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipped;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.misc.FunctionalUtil;

public class ScoreFuel extends AutoRoutine {
	// scoring preload (reef pipes)
	// starting position (closest (center pillar), far?)
	// scoring coral 1 (1/2 reef pipes - 1)
	// scoring coral 2 (1/2 reef pipes - 2)
	// which part of the coral station (close, mid, far)

	private static final AutoQuestion<StartingPosition> startPosition = new AutoQuestion<StartingPosition>("Starting Position") {
		private static final Map.Entry<String, StartingPosition> startLeftBump =           Settings.option("LB",  StartingPosition.LEFT_BUMP);
		private static final Map.Entry<String, StartingPosition> startRightBump =          Settings.option("RB",  StartingPosition.RIGHT_BUMP);
		private static final Map.Entry<String, StartingPosition> startCenter =             Settings.option("C",   StartingPosition.CENTER);
		private static final Map.Entry<String, StartingPosition> startInsideLeftTrench =   Settings.option("ILT", StartingPosition.INSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, StartingPosition> startInsideRightTrench =  Settings.option("IRT", StartingPosition.INSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<StartingPosition> generateSettings() {
			return Settings.from(startInsideRightTrench, startInsideLeftTrench, startLeftBump, startCenter, startRightBump, startInsideRightTrench);
		}
	};

	private static final AutoQuestion<ScoringLocation> firstScoringLocation = new AutoQuestion<ScoringLocation>("First Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> center = Settings.option("C", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

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
				startPosition == StartingPosition.LEFT_BUMP
			) {
				return Settings.from(
					left,
					left,
					right
				);
			} else if (
				startPosition == StartingPosition.RIGHT_BUMP
			) {
				return Settings.from(
					right,
					left,
					right
				);
			} else if (
				startPosition == StartingPosition.INSIDE_LEFT_TRENCH
			) {
				return Settings.from(
					outsideLeftTrench,
					outsideLeftTrench,
					left,
					outsideRightTrench
				);
			} else if (
				startPosition == StartingPosition.INSIDE_RIGHT_TRENCH
			) {
				return Settings.from(
					outsideRightTrench,
					outsideLeftTrench,
					right,
					outsideRightTrench
				);
			} else {
				return null;
			}
		}
	};

	private static final AutoQuestion<IntakeLocation> firstIntakeLocation = new AutoQuestion<IntakeLocation>("First Intake Location") {
		private static final Map.Entry<String, IntakeLocation> fullOuterSwipe = Settings.option("FOS", IntakeLocation.FULL_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> fullInnerSwipe = Settings.option("FIS", IntakeLocation.FULL_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOuterSwipe = Settings.option("HOS", IntakeLocation.HALF_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfInnerSwipe = Settings.option("HIS", IntakeLocation.HALF_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("OS", IntakeLocation.OPPONENT_SWIPE);
		// private static final Map.Entry<String, IntakeLocation> depot = Settings.option("D", IntakeLocation.DEPOT);
		// private static final Map.Entry<String, IntakeLocation> outpost = Settings.option("O", IntakeLocation.OUTPOST);
		private static final Map.Entry<String, IntakeLocation> noIntake = Settings.option("F", IntakeLocation.FALSE);

		@Override
		protected Settings<IntakeLocation> generateSettings() {
			var startPosition = ScoreFuel.startPosition.getResponse();
			var scoreLocation = ScoreFuel.firstScoringLocation.getResponse();
			if (
				(startPosition == StartingPosition.LEFT_BUMP && scoreLocation == ScoringLocation.LEFT) ||
				(startPosition == StartingPosition.RIGHT_BUMP && scoreLocation == ScoringLocation.RIGHT)
			) {
				return Settings.from(
					halfOuterSwipe,
					halfOuterSwipe
				);
			} else if (
				(startPosition == StartingPosition.LEFT_BUMP && scoreLocation == ScoringLocation.RIGHT) ||
				(startPosition == StartingPosition.RIGHT_BUMP && scoreLocation == ScoringLocation.LEFT)
			) {
				return Settings.from(
					fullOuterSwipe,
					fullOuterSwipe
				);
			} else if (
				(startPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_LEFT_TRENCH) ||
				(startPosition == StartingPosition.INSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_RIGHT_TRENCH)
			) {
				return Settings.from(
					halfOuterSwipe,
					halfOuterSwipe
				);
			} else if (
				(startPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_RIGHT_TRENCH) ||
				(startPosition == StartingPosition.INSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.OUTSIDE_LEFT_TRENCH)
			) {
				return Settings.from(
					fullOuterSwipe,
					fullOuterSwipe,
					opponentSwipe
				);
			} else if (
				(startPosition == StartingPosition.INSIDE_LEFT_TRENCH && scoreLocation == ScoringLocation.LEFT) ||
				(startPosition == StartingPosition.INSIDE_RIGHT_TRENCH && scoreLocation == ScoringLocation.RIGHT)
			) {
				return Settings.from(
					halfOuterSwipe,
					halfOuterSwipe
				);
			} else if (
				startPosition == StartingPosition.CENTER
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

	private static final AutoQuestion<ScoringLocation> secondScoringLocation = new AutoQuestion<ScoringLocation>("Second Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		// private static final Map.Entry<String, ScoringLocation> center = Settings.option("C", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			var startPosition = ScoreFuel.firstScoringLocation.getResponse();
			var firstIntakeLocation = ScoreFuel.firstIntakeLocation.getResponse();
			if (
				firstIntakeLocation == IntakeLocation.FALSE
			) {
				return null;
			} else if (
				startPosition == ScoringLocation.LEFT
			) {
				return Settings.from(
					left,
					left
				);
			} else if (
				startPosition == ScoringLocation.RIGHT
			) {
				return Settings.from(
					right,
					right
				);
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
		private static final Map.Entry<String, IntakeLocation> fullOuterSwipe = Settings.option("FOS", IntakeLocation.FULL_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> fullInnerSwipe = Settings.option("FIS", IntakeLocation.FULL_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOuterSwipe = Settings.option("HOS", IntakeLocation.HALF_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfInnerSwipe = Settings.option("HIS", IntakeLocation.HALF_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("OS", IntakeLocation.OPPONENT_SWIPE);
		// private static final Map.Entry<String, IntakeLocation> depot = Settings.option("D", IntakeLocation.DEPOT);
		// private static final Map.Entry<String, IntakeLocation> outpost = Settings.option("O", IntakeLocation.OUTPOST)
		private static final Map.Entry<String, IntakeLocation> noIntake = Settings.option("F", IntakeLocation.FALSE);

		@Override
		protected Settings<IntakeLocation> generateSettings() {
			var startPosition = ScoreFuel.firstScoringLocation.getResponse();
			var scoreLocation = ScoreFuel.secondScoringLocation.getResponse();
			if (
				(startPosition == ScoringLocation.LEFT && scoreLocation == ScoringLocation.LEFT) ||
				(startPosition == ScoringLocation.RIGHT && scoreLocation == ScoringLocation.RIGHT)
			) {
				return Settings.from(
					halfOuterSwipe,
					halfOuterSwipe,
					halfInnerSwipe
				);
			} else if (
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

	private static final AutoQuestion<ScoringLocation> thirdScoringLocation = new AutoQuestion<ScoringLocation>("Third Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> center = Settings.option("C", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			var startPosition = ScoreFuel.secondScoringLocation.getResponse();
			var firstIntakeLocation = ScoreFuel.secondIntakeLocation.getResponse();
			if (
				firstIntakeLocation == IntakeLocation.FALSE
			) {
				return null;
			} else if (
				startPosition == ScoringLocation.LEFT
			) {
				return Settings.from(
					left,
					left
				);
			} else if (
				startPosition == ScoringLocation.RIGHT
			) {
				return Settings.from(
					right,
					right
				);
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

	private static final AutoQuestion<IntakeLocation> thirdIntakeLocation = new AutoQuestion<IntakeLocation>("Third Intake Location") {
		private static final Map.Entry<String, IntakeLocation> fullOuterSwipe = Settings.option("FOS", IntakeLocation.FULL_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> fullInnerSwipe = Settings.option("FIS", IntakeLocation.FULL_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfOuterSwipe = Settings.option("HOS", IntakeLocation.HALF_OUTER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> halfInnerSwipe = Settings.option("HIS", IntakeLocation.HALF_INNER_SWIPE);
		private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("OS", IntakeLocation.OPPONENT_SWIPE);
		private static final Map.Entry<String, IntakeLocation> depot = Settings.option("D", IntakeLocation.DEPOT);
		private static final Map.Entry<String, IntakeLocation> outpost = Settings.option("O", IntakeLocation.OUTPOST);
		private static final Map.Entry<String, IntakeLocation> noIntake = Settings.option("F", IntakeLocation.FALSE);

		@Override
		protected Settings<IntakeLocation> generateSettings() {
			var startPosition = ScoreFuel.secondScoringLocation.getResponse();
			var scoreLocation = ScoreFuel.thirdScoringLocation.getResponse();
			if (
				(startPosition == ScoringLocation.LEFT && scoreLocation == ScoringLocation.LEFT) ||
				(startPosition == ScoringLocation.RIGHT && scoreLocation == ScoringLocation.RIGHT)
			) {
				return Settings.from(
					halfOuterSwipe,
					halfOuterSwipe,
					halfInnerSwipe
				);
			} else if (
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
	private final Shooter shooter;
	private final Rollers rollers;
	private final Intake intake;

	public ScoreFuel(RobotContainer robot) {
		super("Score Fuel", List.of(
			startPosition,
			firstScoringLocation,
			firstIntakeLocation,
			secondScoringLocation,
			secondIntakeLocation,
			thirdScoringLocation,
			thirdIntakeLocation
		));
		this.robot = robot;
		this.drive = robot.drive;
		this.shooter = robot.shooter;
		this.rollers = robot.rollers;
		this.intake = robot.intake;
	}

	@Override
	public Command generateCommand() {
		var startPosition = ScoreFuel.startPosition.getResponse();
		var firstIntakeLocation = ScoreFuel.firstIntakeLocation.getResponse();
		var firstScoringLocation = ScoreFuel.firstScoringLocation.getResponse();
		var secondIntakeLocation = ScoreFuel.secondIntakeLocation.getResponse();
		var secondScoringLocation = ScoreFuel.secondScoringLocation.getResponse();
		var thirdIntakeLocation = ScoreFuel.thirdIntakeLocation.getResponse();
		var thirdScoringLocation = ScoreFuel.thirdScoringLocation.getResponse();

		var commands = new ArrayList<Command>();
		var firstTraj = generatePath(startPosition, firstScoringLocation, firstIntakeLocation).getOurs();
		Logger.recordOutput("DEBUG/FirstTraj", startPosition.alias + "To" + firstScoringLocation.alias + "Intake" + firstIntakeLocation.alias);
		var firstPathCommand = AutoCommons.followPathCommand(firstTraj, drive);
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
			this.robot.shooter.aimLeftFlywheelAtHub().asProxy(),
			this.robot.shooter.aimRightFlywheelAtHub().asProxy()
		);
		commands.add(firstCommand);

		var secondTraj = generatePath(firstScoringLocation, secondScoringLocation, secondIntakeLocation).getOurs();
		var secondPathCommand = AutoCommons.followPathCommand(secondTraj, drive);
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
			this.robot.shooter.aimLeftFlywheelAtHub().asProxy(),
			this.robot.shooter.aimRightFlywheelAtHub().asProxy()
		);
		commands.add(secondCommand);

		var thirdTraj = generatePath(secondScoringLocation, thirdScoringLocation, thirdIntakeLocation).getOurs();
		var thirdPathCommand = AutoCommons.followPathCommand(thirdTraj, drive);
		var thirdCommand = Commands.deadline(
			Commands.sequence(
				Commands.deadline(
					thirdPathCommand,
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
				FunctionalUtil.evalNow(thirdTraj.getFinalPose(false).get()),
				FunctionalUtil.evalNow(new ChassisSpeeds()),
				FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
			).asProxy(),
			this.robot.shooter.aimLeftFlywheelAtHub().asProxy(),
			this.robot.shooter.aimRightFlywheelAtHub().asProxy()
		);
		commands.add(thirdCommand);

		return Commands.parallel(
			AutoCommons.setOdometryFlipped(startPosition.pose),
			Commands.parallel(
				this.intake.slam.pushdown(this.robot.extensionSystem),
				Commands.sequence(commands.toArray(Command[]::new))
			)
		);
	}


	private static AllianceFlipped<Trajectory<SwerveSample>> generatePath(StartingPosition startingPosition, ScoringLocation scoringLocation, IntakeLocation intakeLocation) {
		if ((startingPosition == StartingPosition.LEFT_BUMP || startingPosition == StartingPosition.INSIDE_LEFT_TRENCH) && scoringLocation.canFlip * intakeLocation.canFlip > 0) {
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
