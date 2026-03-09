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

	private final RobotContainer robot;

	public DoubleSwipe(RobotContainer robot) {
		super(
			"Double Swipe",
			List.of(
				DoubleSwipe.startPosition
			)
		);

		this.robot = robot;
	}

	@Override
	public Command generateCommand() {
		final var startPosition = DoubleSwipe.startPosition.getResponse();

		final var firstTrajToCenterline = AutoCommons.loadChoreoTrajectory("");
		final var firstTrajToTrench = AutoCommons.loadChoreoTrajectory("");

		return Commands.parallel(
			AutoCommons.setOdometryFlipped(startPosition),
			this.robot.intake.slam.deploy(this.robot.extensionSystem).asProxy(),
			Commands.sequence(
				Commands.deadline(
					Commands.sequence(
						new FollowTrajectoryCommand(this.robot.drive, firstTrajToCenterline, true).withName("To Centerline").asProxy(),
						Commands.deadline(
							new FollowTrajectoryCommand(this.robot.drive, firstTrajToTrench, true).withName("Grab Balls").asProxy(),
							this.robot.intake.rollers.intake().asProxy()
						),
						Commands.deadline(
							Commands.waitSeconds(5.0),
							this.robot.shooter.aimHoodAtHub().asProxy(),
							this.robot.shooter.aimDriveAtHub(this.robot.drive.rotationalSubsystem).asProxy(),
							this.robot.rollers.feed().asProxy()
						)
					),
					this.robot.shooter.aimingSystem.aimAtHub(
						FunctionalUtil.evalNow(firstTrajToTrench.getFinalPose(false).get()),
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
