package frc.robot.automations;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.util.EdgeDetector;

public class AutoIntakeOrient implements Runnable {
	private final Drive drive;
	private final IntakeRollers intakeRollers;
	private final Command command;

	private static final LoggedNetworkBoolean enabled = new LoggedNetworkBoolean("Automations/Auto Intake Orient/Enabled", true);

	private final EdgeDetector edgeDetector = new EdgeDetector(false);

	public AutoIntakeOrient(Drive drive, IntakeRollers intakeRollers) {
		this.drive = drive;
		this.intakeRollers = intakeRollers;

		this.command = this.drive.rotationalSubsystem.pidControlledHeading(() -> {
			var speeds = drive.getFieldMeasuredSpeeds();
			double targetAngleRads = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

			return new Rotation2d(targetAngleRads);
		});
	}

	@Override
	public void run() {
		boolean isIntaking = this.intakeRollers.getCurrentCommand() != null ? this.intakeRollers.getCurrentCommand().getName() == "Intake" : false;
		this.edgeDetector.update(
			isIntaking
			&& (this.drive.rotationalSubsystem.getCurrentCommand() == null || this.drive.rotationalSubsystem.getCurrentCommand() == this.command)
			&& this.drive.translationSubsystem.getCurrentCommand() != null
			&& enabled.getAsBoolean()
		);

		if (this.edgeDetector.risingEdge() && !this.command.isScheduled()) {
			CommandScheduler.getInstance().schedule(this.command);
		} else if (this.edgeDetector.fallingEdge() && this.command.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.command);
		}
	}
}
