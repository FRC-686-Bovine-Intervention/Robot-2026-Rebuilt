package frc.robot.automations;

import org.littletonrobotics.junction.Logger;

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

		SwerveModuleState[] lockStates = new SwerveModuleState[] {
			new SwerveModuleState(0, new Rotation2d(Math.PI/4)),
			new SwerveModuleState(0, new Rotation2d(-Math.PI/4)),
			new SwerveModuleState(0, new Rotation2d(-Math.PI/4)),
			new SwerveModuleState(0, new Rotation2d(Math.PI/4))
		};
		command = new Command() {
			{
				setName("Drive Lock");
				// addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);
			}

			@Override
			public void execute() {
				for (int i = 0; i < 4; i++) {
					// var state = new SwerveModuleState(0, new Rotation2d(i < 2 ? i * Math.PI/2 + Math.PI/4 : i * Math.PI/2 + 3 * Math.PI/4));
					drive.modules[i].runSetpoint(lockStates[i]);
				}
			}
		}.repeatedly();
	}

	@Override
	public void run() {
		this.edgeDetector.update(
			this.shooter.aimingSystem.getCurrentCommand() != null
			// && this.shooter.withinTolerance()
			&& this.drive.translationSubsystem.getCurrentCommand() == null
			&& (this.drive.rotationalSubsystem.getCurrentCommand().getName() == "Aim at Hub" || this.drive.rotationalSubsystem.getCurrentCommand() == null)
		);

		Logger.recordOutput("DEBUG/DriveLock/AimingSystemActive", this.shooter.aimingSystem.getCurrentCommand() != null);
		Logger.recordOutput("DEBUG/DriveLock/WithinTolerance", this.shooter.withinTolerance());
		Logger.recordOutput("DEBUG/DriveLock/TranslationalSystemInactive", this.drive.translationSubsystem.getCurrentCommand() == null);
		Logger.recordOutput("DEBUG/DriveLock/IsActive", edgeDetector.getValue());

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
