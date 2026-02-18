package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.aiming.Aiming;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheel leftFlywheel;
	public final Flywheel rightFlywheel;
	public final Hood hood;
	public final Aiming aiming = new Aiming();
}
