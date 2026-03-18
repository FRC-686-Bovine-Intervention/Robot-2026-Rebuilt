package frc.robot.auto.routines;

import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.util.misc.FunctionalUtil;

public class Preloads extends AutoRoutine {
	private final RobotContainer robot;

	public Preloads(RobotContainer robot) {
		super(
			"Preloads",
			List.of()
		);
		this.robot = robot;
	}

	@Override
	public Command generateCommand() {
		return Commands.parallel(
			AutoCommons.setOdometryFlipped(AutoConstants.startHubFront),
			this.robot.shooter.aimingSystem.aimAtHub(
				FunctionalUtil.evalNow(AutoConstants.startHubFront.getOurs()),
				FunctionalUtil.evalNow(new ChassisSpeeds()),
				FunctionalUtil.evalNow(FieldConstants.hubAimPoint.getOurs())
			).asProxy(),
			this.robot.shooter.aimHoodAtHub().asProxy(),
			this.robot.shooter.aimLeftFlywheelAtHub().asProxy(),
			this.robot.shooter.aimRightFlywheelAtHub().asProxy(),
			this.robot.shooter.aimRightFlywheelAtHub().asProxy(),
			this.robot.shooter.aimDriveAtHub(this.robot.drive.rotationalSubsystem).asProxy(),
			this.robot.rollers.feed().onlyWhile(() -> this.robot.shooter.withinTolerance()).repeatedly().withName("Feed when ready").asProxy()
		);
	}
}
