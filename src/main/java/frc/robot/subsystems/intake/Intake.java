package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Intake {
	public final IntakeRollers rollers;
	public final IntakeSlam slam;

	public Command intake() {
		return Commands.parallel(
			this.rollers.intake(),
			this.slam.deploy()
		);
	}
	public Command eject() {
		return Commands.parallel(
			this.slam.deploy(),
			this.rollers.eject()
		);
	}
	public Command retract() {
		return Commands.parallel(
			this.slam.retract(),
			this.rollers.idle()
		);
	}
}
