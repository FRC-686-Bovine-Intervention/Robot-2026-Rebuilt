package frc.robot.auto.routines;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.commands.FollowTrajectoryCommand;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.FunctionalUtil;

public class DoubleSwipe extends AutoRoutine {
	private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<>("Start Position") {
		private final Map.Entry<String, AllianceFlipped<Pose2d>> startInLeftTrench = AutoQuestion.Settings.option("In L Trench", AutoConstants.startInLeftTrench);
		private final Map.Entry<String, AllianceFlipped<Pose2d>> startInRightTrench = AutoQuestion.Settings.option("In R Trench", AutoConstants.startInRightTrench);

		@Override
		protected AutoQuestion.Settings<AllianceFlipped<Pose2d>> generateSettings() {
			return AutoQuestion.Settings.from(this.startInLeftTrench, this.startInLeftTrench, this.startInRightTrench);
		}
	};
	private static final AutoQuestion<Boolean> bump1 = new AutoQuestion<>("Bump") {
		private final Map.Entry<String, Boolean> yes = AutoQuestion.Settings.option("Yes", true);
		private final Map.Entry<String, Boolean> no = AutoQuestion.Settings.option("No", false);

		@Override
		protected AutoQuestion.Settings<Boolean> generateSettings() {
			return AutoQuestion.Settings.from(this.yes, this.yes, this.no);
		}
	};

	private final RobotContainer robot;

	public DoubleSwipe(RobotContainer robot) {
		super(
			"Double Swipe",
			List.of(
				DoubleSwipe.startPosition,
				DoubleSwipe.bump1
			)
		);

		this.robot = robot;
	}

	@Override
	public Command generateCommand() {
		final var startPosition = DoubleSwipe.startPosition.getResponse();
		final var bump1 = DoubleSwipe.bump1.getResponse();

		final String firstTrajBallGrabName;
		final String secondTrajBallGrabName;

		if (bump1) {
			if (startPosition == AutoConstants.startInLeftTrench) {
				firstTrajBallGrabName = "LeftTrenchGrab1Bump";
				secondTrajBallGrabName = "LeftTrenchGrab2Bump";
			} else {
				firstTrajBallGrabName = "RightTrenchGrab1Bump";
				secondTrajBallGrabName = "RightTrenchGrab2Bump";
			}
		} else {
			if (startPosition == AutoConstants.startInLeftTrench) {
				firstTrajBallGrabName = "LeftTrenchGrab1";
				secondTrajBallGrabName = "LeftTrenchGrab2";
			} else {
				firstTrajBallGrabName = "RightTrenchGrab1";
				secondTrajBallGrabName = "RightTrenchGrab2";
			}
		}

		final var firstTrajBallGrab = AutoCommons.loadBlueChoreoTrajectory(firstTrajBallGrabName).getOurs();
		final var secondTrajBallGrab = AutoCommons.loadBlueChoreoTrajectory(secondTrajBallGrabName).getOurs();

		return Commands.parallel(
			AutoCommons.setOdometryFlipped(startPosition),
			this.robot.intake.slam.pushdown(this.robot.extensionSystem).asProxy(),
			Commands.sequence(
				Commands.deadline(
					Commands.sequence(
						Commands.deadline(
							new FollowTrajectoryCommand(this.robot.drive, firstTrajBallGrab, true).withName("First Grab Ball").asProxy(),
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
				),
				Commands.deadline(
					Commands.sequence(
						Commands.deadline(
							new FollowTrajectoryCommand(this.robot.drive, secondTrajBallGrab, true).withName("Second Grab Ball").asProxy(),
							this.robot.intake.rollers.intake().asProxy()
						),
						Commands.parallel(
							// Commands.waitSeconds(5.0),
							this.robot.shooter.aimHoodAtHub().asProxy(),
							this.robot.shooter.aimDriveAtHub(this.robot.drive.rotationalSubsystem).asProxy(),
							this.robot.rollers.feed().onlyWhile(() -> this.robot.shooter.withinTolerance()).repeatedly().withName("Feed when ready").asProxy()
						)
					),
					this.robot.shooter.aimingSystem.aimAtHub(
						FunctionalUtil.evalNow(secondTrajBallGrab.getFinalPose(false).get()),
						FunctionalUtil.evalNow(new ChassisSpeeds()),
						FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
					).asProxy(),
					this.robot.shooter.aimLeftFlywheelAtHub().asProxy(),
					this.robot.shooter.aimRightFlywheelAtHub().asProxy()
				)
			)
		);
	}
}
