package frc.robot.auto;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.commands.FollowTrajectoryCommand;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.FunctionalUtil;

public class AutoCommons {
	public static Command setOdometryFlipped(AllianceFlipped<Pose2d> pose) {
		return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose.getOurs()));
	}

	public static AllianceFlipped<Trajectory<SwerveSample>> loadBlueChoreoTrajectory(String name) {
		Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
		if (traj.isEmpty()) {
			throw new NullPointerException("No such Choreo trajectory: " + name);
		}
		return AllianceFlipped.fromBlue(traj.get());
	}

	public static Command swipe(
		RobotContainer robot,
		Trajectory<SwerveSample> traj,
		double intakeDeployDelay,
		double noBallTimeout,
		double minShotTime,
		double autoTimeCutoff,
		double autoTimeDisableCutoff,
		DoubleSupplier autoTimer,
		boolean enableAutoCutoff
	) {
		final Command intakeDeployCommand;
		if (intakeDeployDelay == 0.0) {
			intakeDeployCommand = robot.intake.slam.deploy(robot.extensionSystem).asProxy();
		} else {
			intakeDeployCommand = Commands.sequence(
				Commands.waitSeconds(intakeDeployDelay),
				robot.intake.slam.deploy(robot.extensionSystem).asProxy()
			);
		}
		final Command endingCommand;
		if (enableAutoCutoff) {
			endingCommand = Commands.race(
				Commands.parallel(
					robot.rollers.untilNoBalls(noBallTimeout),
					Commands.waitSeconds(minShotTime)
				),
				Commands.parallel(
					Commands.waitUntil(() -> autoTimer.getAsDouble() >= autoTimeCutoff),
					Commands.waitUntil(() -> robot.shooter.withinShootingTolerance() && autoTimer.getAsDouble() <= autoTimeDisableCutoff)
				)
			);
		} else {
			endingCommand = Commands.parallel(
				robot.rollers.untilNoBalls(noBallTimeout),
				Commands.waitSeconds(minShotTime)
			);
		}
		return Commands.deadline(
			Commands.sequence(
				Commands.deadline(
					new FollowTrajectoryCommand(robot.drive, traj, true).withName("Grab Ball").asProxy(),
					intakeDeployCommand,
					robot.intake.rollers.intake().asProxy()
				),
				Commands.deadline(
					endingCommand,
					robot.rollers.feed().onlyWhile(() -> robot.shooter.withinShootingTolerance()).repeatedly().withName("Feed when ready").asProxy(),
					robot.intake.slam.hopperAgitate(robot.extensionSystem).asProxy(),
					robot.shooter.aimHoodAtHub().asProxy(),
					robot.shooter.aimDriveAtHub(robot.drive.rotationalSubsystem).asProxy(),
					robot.drive.translationSubsystem.simplePIDTo(FunctionalUtil.evalNow(traj.getFinalPose(false).get().getTranslation())).asProxy()
				)
			),
			robot.shooter.aimingSystem.aimAtHub(
				FunctionalUtil.evalNow(traj.getFinalPose(false).get()),
				FunctionalUtil.evalNow(new ChassisSpeeds()),
				FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs()),
				false
			).asProxy(),
			robot.shooter.aimFlywheelAtHub().asProxy()
		);
	}
}
