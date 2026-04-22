package frc.robot.auto;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import frc.util.commands.ContinuouslySwappingCommand;
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

	public static Command feedWhenReadyOrPrestage(RobotContainer robot, boolean enablePrestage) {
		return new ContinuouslySwappingCommand(
			new Supplier<>() {
				private final Command feedCommand = robot.rollers.feed().withName("Auto Feed").asProxy();
				private final Command prestageCommand = AutoCommons.passivePrestage(robot, enablePrestage);

				@Override
				public Command get() {
					if (robot.shooter.withinShootingTolerance()) {
						return this.feedCommand;
					}
					return this.prestageCommand;
				}
			},
			Set.of()
		);
	}

	public static Command passivePrestage(RobotContainer robot, boolean enablePrestage) {
		return new ContinuouslySwappingCommand(
			new Supplier<>() {
				private final Command prestageCommand = robot.rollers.passivePrestage().withName("Auto Prestage").asProxy();
				private final Command idleCommand = robot.rollers.idle().withName("Auto Idle").asProxy();

				@Override
				public Command get() {
					if (!robot.rollers.isFeederSensorTripped() && enablePrestage) {
						return this.prestageCommand;
					}
					return this.idleCommand;
				}
			},
			Set.of()
		);
	}

	public static Command swipe(
		RobotContainer robot,
		Trajectory<SwerveSample> traj,
		double flywheelSpinupDelay,
		double intakeDeployDelay,
		double noBallTimeout,
		double minShotTime,
		double autoTimeCutoff,
		double autoTimeDisableCutoff,
		DoubleSupplier autoTimer,
		boolean enableAutoCutoff,
		boolean enablePrestage
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

		final Command flywheelSpinupCommand;
		if (flywheelSpinupDelay == 0.0) {
			flywheelSpinupCommand = robot.shooter.aimFlywheelAtHub().asProxy();
		} else {
			flywheelSpinupCommand = Commands.sequence(
				Commands.waitSeconds(flywheelSpinupDelay),
				robot.shooter.aimFlywheelAtHub().asProxy()
			);
		}

		final Command endingConditionCommand;
		if (enableAutoCutoff) {
			endingConditionCommand = Commands.race(
				Commands.sequence(
					Commands.waitUntil(() -> robot.shooter.withinShootingTolerance()),
					Commands.waitSeconds(minShotTime),
					robot.rollers.untilNoBalls(noBallTimeout)
				),
				Commands.parallel(
					Commands.waitUntil(() -> autoTimer.getAsDouble() >= autoTimeCutoff),
					Commands.waitUntil(() -> robot.shooter.withinShootingTolerance() && autoTimer.getAsDouble() <= autoTimeDisableCutoff)
				)
			);
		} else {
			endingConditionCommand = Commands.sequence(
				Commands.waitUntil(() -> robot.shooter.withinShootingTolerance()),
				Commands.waitSeconds(minShotTime),
				robot.rollers.untilNoBalls(noBallTimeout)
			);
		}

		return Commands.deadline(
			Commands.sequence(
				Commands.deadline(
					new FollowTrajectoryCommand(robot.drive, traj, true).withName("Grab Ball").asProxy(),
					intakeDeployCommand,
					robot.intake.rollers.intake().asProxy(),
					AutoCommons.passivePrestage(robot, enablePrestage)
				),
				Commands.deadline(
					endingConditionCommand,
					AutoCommons.feedWhenReadyOrPrestage(robot, enablePrestage),
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
			flywheelSpinupCommand
		);
	}
}
