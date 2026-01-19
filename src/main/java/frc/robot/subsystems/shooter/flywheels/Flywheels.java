package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase {
	private final FlywheelsIO io;
	private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

	public Flywheels(FlywheelsIO io) {
		super("Shooter/Flywheels");
		this.io = io;
	}
}
