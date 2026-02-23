package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExtensionSystem;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Intake {
	public final IntakeRollers rollers;
	public final IntakeSlam slam;

	public Command intake(ExtensionSystem extension) {
		return Commands.parallel(
			this.rollers.intake(),
			this.slam.deploy(extension)
		).withName("Intake");
	}

	public Command eject(ExtensionSystem extension) {
		return Commands.parallel(
			this.slam.deploy(extension),
			this.rollers.eject()
		).withName("Eject");
	}

	public Command retract() {
		return Commands.parallel(
			this.slam.stow(),
			this.rollers.idle()
		).withName("Retract");
	}
}
