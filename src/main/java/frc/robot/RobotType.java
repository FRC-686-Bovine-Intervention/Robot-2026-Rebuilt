package frc.robot;

public enum RobotType {
	ROBOT_2026_COMP,
	ROBOT_2026_PRAC,
	;
	private static final boolean isReplay = true;
	public static Mode getMode() {
		return Robot.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);
	}
	public static RobotType getRobot() {
		return getMode().defaultRobotType;
	}

	public static enum Mode {
		REAL    (RobotType.ROBOT_2026_COMP),
		SIM     (RobotType.ROBOT_2026_COMP),
		REPLAY  (RobotType.ROBOT_2026_COMP),
		;
		private final RobotType defaultRobotType;
		Mode(RobotType defaultType) {
			this.defaultRobotType = defaultType;
		}
	}
}
