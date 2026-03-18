package frc.robot.auto;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.util.flipping.AllianceFlipped;

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
}
