package frc.robot.automations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.EdgeDetector;

public class ShootingDriveLock implements Runnable {
	private final Drive drive;
	private final Shooter shooter;
	private final Command command;

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public ShootingDriveLock(Drive drive, Shooter shooter) {
		this.drive = drive;
		this.shooter = shooter;

		var state = new SwerveModuleState(0, new Rotation2d(3* Math.PI / 4));
		this.command = new Command() {
			{
				setName("Drive Lock");
				addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);
			}

			@Override
			public void execute() {
				for (var module: drive.modules) {
					module.runSetpoint(state);
				}
			}
		};
	}

	@Override
	public void run() {
		this.edgeDetector.update(
			this.shooter.aimingSystem.getCurrentCommand() != null
			//&& this.shooter.withinTolerance()
			&& this.drive.translationSubsystem.getCurrentCommand() == null
			&& (this.drive.rotationalSubsystem.getCurrentCommand().getName() == "Aim at Hub" || this.drive.rotationalSubsystem.getCurrentCommand() == null)
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
