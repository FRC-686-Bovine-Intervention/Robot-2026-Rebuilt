package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.aiming.Aiming;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheels leftFlywheels;
	public final Flywheels centerFlywheels;
	public final Flywheels rightFlywheels;
	public final Hood hood;
	public final Aiming aiming = new Aiming();
}
