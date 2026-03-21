package frc.robot;

public enum RobotType {
	ROBOT_2025_COMP,
	ROBOT_2025_PRAC,
	;
	private static final boolean isReplay = false;
	public static Mode getMode() {
		return Robot.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);
	}
	public static RobotType getRobot() {
		return getMode().defaultRobotType;
	}

	public static enum Mode {
		REAL    (RobotType.ROBOT_2025_PRAC),
		SIM     (RobotType.ROBOT_2025_COMP),
		REPLAY  (RobotType.ROBOT_2025_PRAC),
		;
		private final RobotType defaultRobotType;
		Mode(RobotType defaultType) {
			this.defaultRobotType = defaultType;
		}
	}
}
