package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.util.flipping.AllianceFlipped;

public class AutoCommons {
	public static Command setOdometryFlipped(AllianceFlipped<Pose2d> pose, Drive drive) {
		return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose.getOurs()));
	}
}
