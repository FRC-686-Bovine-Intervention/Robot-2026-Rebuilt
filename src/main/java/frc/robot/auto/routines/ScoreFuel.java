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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.CoralStationPosition;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.PipeConcept;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.MathExtraUtil;

public class ScoreFuel extends AutoRoutine {
    // scoring preload (reef pipes)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef pipes - 1)
    // scoring coral 2 (1/2 reef pipes - 2)
    // which part of the coral station (close, mid, far)

    private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<AllianceFlipped<Pose2d>>("Starting Position") {
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftLeftCage = Settings.option("LL", AutoConstants.startLeftLeftCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftMiddleCage = Settings.option("LM", AutoConstants.startLeftMiddleCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftRightCage = Settings.option("LR", AutoConstants.startLeftRightCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startDeadCenter = Settings.option("C", AutoConstants.startDeadCenter);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightLeftCage = Settings.option("RL", AutoConstants.startRightLeftCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightMiddleCage = Settings.option("RM", AutoConstants.startRightMiddleCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightRightCage = Settings.option("RR", AutoConstants.startRightRightCage);

        @Override
        protected Settings<AllianceFlipped<Pose2d>> generateSettings() {
            return Settings.from(startDeadCenter, startLeftLeftCage, startLeftMiddleCage, startLeftRightCage, startDeadCenter, startRightLeftCage, startRightMiddleCage, startRightRightCage);
        }
    };

    private static final AutoQuestion<CoralStationPosition> stationPosition = new AutoQuestion<CoralStationPosition>("Coral Station Position") {
        private static final Map.Entry<String, CoralStationPosition> stationFar = Settings.option("Far", CoralStationPosition.FAR);
        private static final Map.Entry<String, CoralStationPosition> stationMid = Settings.option("Mid", CoralStationPosition.MID);
        private static final Map.Entry<String, CoralStationPosition> stationClose = Settings.option("Close", CoralStationPosition.CLOSE);
        
        @Override
        protected Settings<CoralStationPosition> generateSettings() {
            return Settings.from(stationFar, stationClose, stationMid, stationFar);
        }
    };
    
    private static boolean isRightCoralStation(PipeConcept pipe) {
        return isRightCoralStation(pipe.id);
    }
    private static boolean isRightCoralStation(int pipeID) {
        return MathExtraUtil.isWithin(pipeID, 1, 6);
    }

    private static final AutoQuestion<PipeConcept> firstCoralPipe = new AutoQuestion<PipeConcept>("Score First Pipe") {
        @Override
        protected Settings<PipeConcept> generateSettings() {
            var startPosition = ScoreCoral.startPosition.getResponse();
            if (
                startPosition == AutoConstants.startLeftLeftCage ||
                startPosition == AutoConstants.startLeftMiddleCage
            ) {
                return Settings.from(pipeOptions[9],
                    pipeOptions[0],
                    pipeOptions[8],
                    pipeOptions[9],
                    pipeOptions[10],
                    pipeOptions[11]
                );
            } else if (startPosition == AutoConstants.startLeftRightCage) {
                return Settings.from(pipeOptions[9],
                    pipeOptions[6],
                    pipeOptions[7],
                    pipeOptions[8],
                    pipeOptions[9]
                );
            } else if (startPosition == AutoConstants.startDeadCenter) {
                return Settings.from(pipeOptions[6],
                    pipeOptions[4],
                    pipeOptions[5],
                    pipeOptions[6],
                    pipeOptions[7],
                    pipeOptions[8],
                    pipeOptions[9]
                );
            } else if (startPosition == AutoConstants.startRightLeftCage) {
                return Settings.from(pipeOptions[4],
                    pipeOptions[4],
                    pipeOptions[5],
                    pipeOptions[6],
                    pipeOptions[7]
                );
            } else if (
                startPosition == AutoConstants.startRightMiddleCage ||
                startPosition == AutoConstants.startRightRightCage
            ) {
                return Settings.from(pipeOptions[4],
                    pipeOptions[1],
                    pipeOptions[2],
                    pipeOptions[3],
                    pipeOptions[4],
                    pipeOptions[5]
                );
            } else {
                return null;
            }
        }

    };

    private static final AutoQuestion<Optional<PipeConcept>> secondCoralPipe = new AutoQuestion<Optional<PipeConcept>>("Score Second Pipe") {
        @Override
        protected Settings<Optional<PipeConcept>> generateSettings() {
            Predicate<Map.Entry<String, Optional<PipeConcept>>> notInPreviousResponses = (option) -> option.getValue().isEmpty() || !option.getValue().equals(Optional.of(firstCoralPipe.getResponse()));
            if (isRightCoralStation(firstCoralPipe.getResponse())) {
                var options = Stream.of(
                    pipeOptionalOptions[12],
                    pipeOptionalOptions[1],
                    pipeOptionalOptions[2],
                    pipeOptionalOptions[3],
                    pipeOptionalOptions[4],
                    pipeOptionalOptions[5],
                    pipeOptionalOptions[6]
                )
                    .filter(notInPreviousResponses)
                    .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = Stream.of(
                    pipeOptionalOptions[2],
                    pipeOptionalOptions[3],
                    pipeOptionalOptions[1]
                )
                    .filter(notInPreviousResponses)
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            } else {
                var options = Stream.of(
                    pipeOptionalOptions[12],
                    pipeOptionalOptions[0],
                    pipeOptionalOptions[11],
                    pipeOptionalOptions[10],
                    pipeOptionalOptions[9],
                    pipeOptionalOptions[8],
                    pipeOptionalOptions[7]
                )
                    .filter(notInPreviousResponses)
                    .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = Stream.of(
                    pipeOptionalOptions[11],
                    pipeOptionalOptions[10],
                    pipeOptionalOptions[0]
                )
                    .filter(notInPreviousResponses)
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            }
        }
    };

    private static final AutoQuestion<Optional<PipeConcept>> thirdCoralPipe = new AutoQuestion<Optional<PipeConcept>>("Score Third Pipe") {
        @Override
        protected Settings<Optional<PipeConcept>> generateSettings() {
            if (secondCoralPipe.getResponse().isEmpty()) {
                return Settings.from(pipeOptionalOptions[12], pipeOptionalOptions[12]);
            }
            Predicate<Map.Entry<String, Optional<PipeConcept>>> notInPreviousResponses = (option) -> option.getValue().isEmpty() || (!option.getValue().equals(Optional.of(firstCoralPipe.getResponse())) && !option.getValue().equals(secondCoralPipe.getResponse()));
            if (isRightCoralStation(secondCoralPipe.getResponse().get())) {
                var options = Stream.of(
                    pipeOptionalOptions[12],
                    pipeOptionalOptions[1],
                    pipeOptionalOptions[2],
                    pipeOptionalOptions[3],
                    pipeOptionalOptions[4],
                    pipeOptionalOptions[5],
                    pipeOptionalOptions[6]
                )
                    .filter(notInPreviousResponses)
                    .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = Stream.of(
                    pipeOptionalOptions[2],
                    pipeOptionalOptions[3],
                    pipeOptionalOptions[1]
                )
                    .filter(notInPreviousResponses)
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            } else {
                var options = Stream.of(
                    pipeOptionalOptions[12],
                    pipeOptionalOptions[0],
                    pipeOptionalOptions[11],
                    pipeOptionalOptions[10],
                    pipeOptionalOptions[9],
                    pipeOptionalOptions[8],
                    pipeOptionalOptions[7]
                )
                    .filter(notInPreviousResponses)
                    .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = Stream.of(
                    pipeOptionalOptions[11],
                    pipeOptionalOptions[10],
                    pipeOptionalOptions[0]
                )
                    .filter(notInPreviousResponses)
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            }
        }
    };

    private static final AutoQuestion<Optional<PipeConcept>> fourthCoralPipe = new AutoQuestion<Optional<PipeConcept>>("Score Fourth Pipe") {
        @Override
        protected Settings<Optional<PipeConcept>> generateSettings() {
            if (thirdCoralPipe.getResponse().isEmpty()) {
                return Settings.from(pipeOptionalOptions[12], pipeOptionalOptions[12]);
            }
            Predicate<Map.Entry<String, Optional<PipeConcept>>> notInPreviousResponses = (option) -> option.getValue().isEmpty() || (!option.getValue().equals(Optional.of(firstCoralPipe.getResponse())) && !option.getValue().equals(secondCoralPipe.getResponse()) && !option.getValue().equals(thirdCoralPipe.getResponse()));
            if (isRightCoralStation(thirdCoralPipe.getResponse().get())) {
                var options = Stream.of(
                    pipeOptionalOptions[12],
                    pipeOptionalOptions[1],
                    pipeOptionalOptions[2],
                    pipeOptionalOptions[3],
                    pipeOptionalOptions[4],
                    pipeOptionalOptions[5],
                    pipeOptionalOptions[6]
                )
                    .filter(notInPreviousResponses)
                    .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = Stream.of(
                    pipeOptionalOptions[2],
                    pipeOptionalOptions[3],
                    pipeOptionalOptions[1]
                )
                    .filter(notInPreviousResponses)
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            } else {
                var options = Stream.of(
                    pipeOptionalOptions[12],
                    pipeOptionalOptions[0],
                    pipeOptionalOptions[11],
                    pipeOptionalOptions[10],
                    pipeOptionalOptions[9],
                    pipeOptionalOptions[8],
                    pipeOptionalOptions[7]
                )
                    .filter(notInPreviousResponses)
                    .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = Stream.of(
                    pipeOptionalOptions[11],
                    pipeOptionalOptions[10],
                    pipeOptionalOptions[0]
                )
                    .filter(notInPreviousResponses)
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            }
        }
    };

    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;

    public ScoreCoral(RobotContainer robot) {
        super("Score Coral", List.of(
            startPosition,
            stationPosition,
            firstCoralPipe,
            secondCoralPipe,
            thirdCoralPipe,
            fourthCoralPipe
        ));
        this.drive = robot.drive;
        this.superstructure = robot.superstructure;
        this.intake = robot.intake;
    }
    
    @Override
    public Command generateCommand() {
        var startPosition = ScoreCoral.startPosition.getResponse();
        var stationPosition = ScoreCoral.stationPosition.getResponse();
        var firstCoralPipe = ScoreCoral.firstCoralPipe.getResponse();
        var secondCoralPipe = ScoreCoral.secondCoralPipe.getResponse();
        var thirdCoralPipe = ScoreCoral.thirdCoralPipe.getResponse();
        var fourthCoralPipe = ScoreCoral.fourthCoralPipe.getResponse();
        var commands = new ArrayList<Command>();
        boolean shouldUseForwardCoralStation = false;

        String startToScorePath;
        startToScorePath = getStartingPositionAsString(startPosition) + " To " + firstCoralPipe.getLetter();
        var startToScorePreload = AutoPaths.loadChoreoTrajectory(startToScorePath);
        commands.add(AutoCommons.scoreOnReef(startToScorePreload, firstCoralPipe.getBranch(BranchLevel.Level4), Direction.Forward, drive, superstructure, intake));


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