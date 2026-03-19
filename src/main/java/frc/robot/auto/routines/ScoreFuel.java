package frc.robot.auto.routines;

import static frc.robot.auto.AutoCommons.getStartingPositionAsString;
import static frc.robot.auto.AutoCommons.pipeOptionalOptions;
import static frc.robot.auto.AutoCommons.pipeOptions;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.Predicate;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoConstants.IntakeLocation;
import frc.robot.auto.AutoConstants.ScoringLocation;
import frc.robot.auto.AutoConstants.StartingPosition;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.PipeConcept;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.FunctionalUtil;
import frc.util.misc.MathExtraUtil;

public class ScoreFuel extends AutoRoutine {
    // scoring preload (reef pipes)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef pipes - 1)
    // scoring coral 2 (1/2 reef pipes - 2)
    // which part of the coral station (close, mid, far)

	private static final AutoQuestion<Boolean> scorePreloadFirst = new AutoQuestion<Boolean>("Score Preload") {
		private static final Map.Entry<String, Boolean> yes = Settings.option("Y", true);
		private static final Map.Entry<String, Boolean> no = Settings.option("N", false);

		@Override
		protected Settings<Boolean> generateSettings() {
			return Settings.from(no, yes, no);
		}
	};

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
        
        @Override
        protected Settings<IntakeLocation> generateSettings() {
            return Settings.from(halfOuterSwipe, halfOuterSwipe, fullOuterSwipe, opponentSwipe, outpost, depot, halfInnerSwipe, fullInnerSwipe);
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
		private static final Map.Entry<String, ScoringLocation> insideLeftTrench = Settings.option("ILT", ScoringLocation.INSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> insideRightTrench = Settings.option("IRT", ScoringLocation.INSIDE_RIGHT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			return Settings.from(outsideRightTrench, insideLeftTrench, outsideLeftTrench, left, center, right, outsideRightTrench, insideRightTrench);
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
		private static final Map.Entry<String, ScoringLocation> insideLeftTrench = Settings.option("ILT", ScoringLocation.INSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> insideRightTrench = Settings.option("IRT", ScoringLocation.INSIDE_RIGHT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			return Settings.from(outsideRightTrench, insideLeftTrench, outsideLeftTrench, left, center, right, outsideRightTrench, insideRightTrench);
		}
    };

	private static final AutoQuestion<ScoringLocation> fourthScoringLocation = new AutoQuestion<ScoringLocation>("Optional Fourth Score Location") {
		private static final Map.Entry<String, ScoringLocation> left = Settings.option("L", ScoringLocation.LEFT);
		private static final Map.Entry<String, ScoringLocation> right = Settings.option("R", ScoringLocation.RIGHT);
		private static final Map.Entry<String, ScoringLocation> center = Settings.option("C", ScoringLocation.CENTER);
		private static final Map.Entry<String, ScoringLocation> insideLeftTrench = Settings.option("ILT", ScoringLocation.INSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> insideRightTrench = Settings.option("IRT", ScoringLocation.INSIDE_RIGHT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideLeftTrench = Settings.option("OLT", ScoringLocation.OUTSIDE_LEFT_TRENCH);
		private static final Map.Entry<String, ScoringLocation> outsideRightTrench = Settings.option("ORT", ScoringLocation.OUTSIDE_RIGHT_TRENCH);

		@Override
		protected Settings<ScoringLocation> generateSettings() {
			return Settings.from(outsideRightTrench, insideLeftTrench, outsideLeftTrench, left, center, right, outsideRightTrench, insideRightTrench);
		}
    };

    private final Drive drive;
    private final Shooter shooter;
	private final Rollers rollers;
    private final Intake intake;

    public ScoreFuel(RobotContainer robot) {
        super("Score Fuel", List.of(
            scorePreloadFirst,
			startPosition,
			firstIntakeLocation,
			firstScoringLocation,
			secondIntakeLocation,
			secondScoringLocation,
			thirdIntakeLocation,
			thirdScoringLocation,
			fourthScoringLocation
        ));
        this.drive = robot.drive;
		this.shooter = robot.shooter;
		this.rollers = robot.rollers;
        this.intake = robot.intake;
    }
    
    @Override
    public Command generateCommand() {
        var startPosition = ScoreFuel.startPosition.getResponse();
        var scorePreloadFirst = ScoreFuel.scorePreloadFirst.getResponse();
        var firstIntakeLocation = ScoreFuel.firstIntakeLocation.getResponse();
        var firstScoringLocation = ScoreFuel.firstScoringLocation.getResponse();
        var secondIntakeLocation = ScoreFuel.secondIntakeLocation.getResponse();
        var secondScoringLocation = ScoreFuel.secondScoringLocation.getResponse();
        var thirdIntakeLocation = ScoreFuel.thirdIntakeLocation.getResponse();
        var thirdScoringLocation = ScoreFuel.thirdScoringLocation.getResponse();
        var fourthScoringLocation = ScoreFuel.fourthScoringLocation.getResponse();
        
		var commands = new ArrayList<Command>();

		if (scorePreloadFirst.booleanValue()) {
			String startToScorePath;
        	startToScorePath = startPosition.alias + "Score" + firstScoringLocation.alias;
			var startToScorePreloadTraj = AutoCommons.loadBlueChoreoTrajectory(startToScorePath).getOurs();
        	var startToScorePreload = AutoCommons.followPathCommand(startToScorePreloadTraj, drive);
        	commands.add(
				Commands.deadline(
					Commands.sequence(
						startToScorePreload,
						Commands.deadline(
							Commands.waitSeconds(4.0),
							this.shooter.aimHoodAtHub().asProxy(),
							this.shooter.aimDriveAtHub(this.drive.rotationalSubsystem).asProxy(),
							this.rollers.feed().onlyWhile(() -> this.shooter.withinTolerance()).repeatedly().withName("Feed when ready").asProxy()
						)
					),
					this.shooter.aimingSystem.aimAtHub(
						FunctionalUtil.evalNow(startToScorePreloadTraj.getFinalPose(false).get()),
						FunctionalUtil.evalNow(new ChassisSpeeds()),
						FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
					).asProxy(),
					this.shooter.aimLeftFlywheelAtHub().asProxy(),
					this.shooter.aimRightFlywheelAtHub().asProxy()
				)
			);
		}


        if (secondCoralPipe.isPresent()) {
            var firstPipeToStation = AutoPaths.loadChoreoTrajectory(
                firstCoralPipe.getLetter() +
                " To Station " +
                getStationPositionAsString(stationPosition) +
                (shouldUseForwardCoralStation ? " Forward" : "")
            );
            commands.add(AutoCommons.pickupCoralFromStation(firstPipeToStation, Direction.Backward, drive, superstructure, intake));
    
            var pipe2 = secondCoralPipe.get();

            var stationToSecondPipe = AutoPaths.loadChoreoTrajectory(
                "Station "
                + getStationPositionAsString(stationPosition)
                + (shouldUseForwardCoralStation ? " Forward" : "")
                + " To "
                + pipe2.getLetter()
            );
            commands.add(AutoCommons.scoreOnReef(stationToSecondPipe, pipe2.getBranch(BranchLevel.Level4), Direction.Forward, drive, superstructure, intake));

            if (thirdCoralPipe.isPresent()) {
                var secondPipeToStation = AutoPaths.loadChoreoTrajectory(
                    pipe2.getLetter() +
                    " To Station "+
                    getStationPositionAsString(stationPosition)
                    + (shouldUseForwardCoralStation ? " Forward" : "")
                );
                commands.add(AutoCommons.pickupCoralFromStation(secondPipeToStation, Direction.Backward, drive, superstructure, intake));
                
                var pipe3 = thirdCoralPipe.get();
        
                var stationToThirdPipe = AutoPaths.loadChoreoTrajectory(
                    "Station "
                    + getStationPositionAsString(stationPosition)
                    + (shouldUseForwardCoralStation ? " Forward" : "")
                    + " To "
                    + pipe3.getLetter()
                );
                commands.add(AutoCommons.scoreOnReef(stationToThirdPipe, pipe3.getBranch(BranchLevel.Level4), Direction.Forward, drive, superstructure, intake));

                if (fourthCoralPipe.isPresent()) {
                    var thirdPipeToStation = AutoPaths.loadChoreoTrajectory(
                        pipe3.getLetter() +
                        " To Station "+
                        getStationPositionAsString(stationPosition)
                        + (shouldUseForwardCoralStation ? " Forward" : "")
                    );
                    commands.add(AutoCommons.pickupCoralFromStation(thirdPipeToStation, Direction.Backward, drive, superstructure, intake));
                    
                    var pipe4 = fourthCoralPipe.get();
            
                    var stationToFourthPipe = AutoPaths.loadChoreoTrajectory(
                        "Station "
                        + getStationPositionAsString(stationPosition)
                        + (shouldUseForwardCoralStation ? " Forward" : "")
                        + " To "
                        + pipe4.getLetter()
                    );
                    commands.add(AutoCommons.scoreOnReef(stationToFourthPipe, pipe4.getBranch(BranchLevel.Level4), Direction.Forward, drive, superstructure, intake));
                }
            }
        }

        return Commands.parallel(
            AutoCommons.setOdometryFlipped(startPosition, drive),
            Commands.sequence(commands.toArray(Command[]::new))
        );
    }

    private String getStationPositionAsString(CoralStationPosition _stationPosition){
        return switch (_stationPosition) {
            case CLOSE -> "Close";
            case MID -> "Mid";
            case FAR -> "Far";
            default -> null;
        };
    }
}