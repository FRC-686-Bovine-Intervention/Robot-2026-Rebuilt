package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
	private final RollersIO io;
	private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

	public Rollers(RollersIO io) {
		super("Intake/Rollers");
		this.io = io;
	}
}
