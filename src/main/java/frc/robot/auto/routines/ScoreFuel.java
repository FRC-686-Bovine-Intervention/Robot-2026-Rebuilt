package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.Predicate;
import java.util.stream.Stream;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants.IntakeLocation;
import frc.robot.auto.AutoConstants.ScoringLocation;
import frc.robot.auto.AutoConstants.StartingPosition;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.feeder.Feeder;
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
        private static final Map.Entry<String, StartingPosition> startOutsideLeftTrench =  Settings.option("OLT", StartingPosition.OUTSIDE_LEFT_TRENCH);
        private static final Map.Entry<String, StartingPosition> startOutsideRightTrench = Settings.option("ORT", StartingPosition.OUTSIDE_RIGHT_TRENCH);
        private static final Map.Entry<String, StartingPosition> startLeftBump =           Settings.option("LB",  StartingPosition.LEFT_BUMP);
        private static final Map.Entry<String, StartingPosition> startRightBump =          Settings.option("RB",  StartingPosition.RIGHT_BUMP);
        private static final Map.Entry<String, StartingPosition> startCenter =             Settings.option("C",   StartingPosition.CENTER);
        private static final Map.Entry<String, StartingPosition> startInsideLeftTrench =   Settings.option("ILT", StartingPosition.INSIDE_LEFT_TRENCH);
        private static final Map.Entry<String, StartingPosition> startInsideRightTrench =  Settings.option("IRT", StartingPosition.INSIDE_RIGHT_TRENCH);

        @Override
        protected Settings<StartingPosition> generateSettings() {
            return Settings.from(startInsideRightTrench, startInsideLeftTrench, startOutsideLeftTrench, startLeftBump, startCenter, startRightBump, startOutsideRightTrench, startInsideRightTrench);
        }
    };

    private static final AutoQuestion<IntakeLocation> firstIntakeLocation = new AutoQuestion<IntakeLocation>("First Intake Location") {
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
            return Settings.from(halfOuterSwipe, halfOuterSwipe, fullOuterSwipe, opponentSwipe, outpost, depot, halfInnerSwipe, fullInnerSwipe, noIntake);
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
			return Settings.from(outsideRightTrench, outsideLeftTrench, left, center, right, outsideRightTrench);
		}
    };

	private static final AutoQuestion<IntakeLocation> secondIntakeLocation = new AutoQuestion<IntakeLocation>("Second Intake Location") {
        private static final Map.Entry<String, IntakeLocation> fullOuterSwipe = Settings.option("FOS", IntakeLocation.FULL_OUTER_SWIPE);
        private static final Map.Entry<String, IntakeLocation> fullInnerSwipe = Settings.option("FIS", IntakeLocation.FULL_INNER_SWIPE);
        private static final Map.Entry<String, IntakeLocation> halfOuterSwipe = Settings.option("HOS", IntakeLocation.HALF_OUTER_SWIPE);
        private static final Map.Entry<String, IntakeLocation> halfInnerSwipe = Settings.option("HIS", IntakeLocation.HALF_INNER_SWIPE);
        private static final Map.Entry<String, IntakeLocation> opponentSwipe = Settings.option("OS", IntakeLocation.OPPONENT_SWIPE);
        private static final Map.Entry<String, IntakeLocation> depot = Settings.option("D", IntakeLocation.DEPOT);
        private static final Map.Entry<String, IntakeLocation> outpost = Settings.option("O", IntakeLocation.OUTPOST);
        
        @Override
        protected Settings<IntakeLocation> generateSettings() {
            return Settings.from(halfOuterSwipe, halfOuterSwipe, fullOuterSwipe, opponentSwipe, outpost, depot, halfInnerSwipe, fullInnerSwipe);
        }
    };

    private static final AutoQuestion<ScoringLocation> secondScoringLocation = new AutoQuestion<ScoringLocation>("Second Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> center = Settings.option("C", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			return Settings.from(outsideRightTrench, outsideLeftTrench, left, center, right, outsideRightTrench);
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
        
        @Override
        protected Settings<IntakeLocation> generateSettings() {
            return Settings.from(halfOuterSwipe, halfOuterSwipe, fullOuterSwipe, opponentSwipe, outpost, depot, halfInnerSwipe, fullInnerSwipe);
        }
    };

    private static final AutoQuestion<ScoringLocation> thirdScoringLocation = new AutoQuestion<ScoringLocation>("Third Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> center = Settings.option("C", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> insideLeftTrench = Settings.option("ILT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> insideRightTrench = Settings.option("IRT", ScoringLocation.INSIDE_RIGHT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			return Settings.from(outsideRightTrench, insideLeftTrench, outsideLeftTrench, left, center, right, outsideRightTrench, insideRightTrench);
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
			firstIntakeLocation,
			firstScoringLocation,
			secondIntakeLocation,
			secondScoringLocation,
			thirdIntakeLocation,
			thirdScoringLocation
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
		var startToIntakeTraj = AutoCommons.loadBlueChoreoTrajectory(startPosition.alias + "To" + firstIntakeLocation.alias).getOurs();
        var startToIntake = AutoCommons.followPathCommand(startToIntakeTraj, drive);
		var command = Commands.deadline(
			Commands.sequence(
				Commands.deadline(
					startToScorePreload,
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
				FunctionalUtil.evalNow(firstTrajBallGrab.getFinalPose(false).get()),
				FunctionalUtil.evalNow(new ChassisSpeeds()),
				FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
			).asProxy(),
			this.robot.shooter.aimLeftFlywheelAtHub().asProxy(),
			this.robot.shooter.aimRightFlywheelAtHub().asProxy()
		);

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