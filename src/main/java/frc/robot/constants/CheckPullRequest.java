package frc.robot.constants;

public class CheckPullRequest {
	public static void main(String[] args) {
		if (RobotConstants.tuningMode) {
			System.err.println("Do not merge, non-default constants are configured.");
			System.exit(1);
		}
	}
}
