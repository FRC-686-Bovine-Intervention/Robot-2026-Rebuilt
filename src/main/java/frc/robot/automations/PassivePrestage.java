package frc.robot.automations;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.rollers.Rollers;

public class PassivePrestage implements Runnable {
	private final Rollers rollers;
	private final Command feederIdleCommand;
	private final Command passivePrestageCommand;

	private final LoggedNetworkBoolean enableSwitch = new LoggedNetworkBoolean("SmartDashboard/Automations/Passive Prestage/Enabled", true);

	public PassivePrestage(Rollers rollers, Command feederIdleCommand, Command passivePrestageCommand) {
		this.rollers = rollers;
		this.feederIdleCommand = feederIdleCommand;
		this.passivePrestageCommand = passivePrestageCommand;
	}

	@Override
	public void run() {
		if (
			this.enableSwitch.getAsBoolean()
			&& DriverStation.isTeleopEnabled()
			&& feederIdleCommand.isScheduled()
			&& !rollers.isFeederSensorTripped()
		) {
			CommandScheduler.getInstance().schedule(this.passivePrestageCommand);
		}
		if (!this.enableSwitch.getAsBoolean() && this.passivePrestageCommand.isScheduled()) {
			CommandScheduler.getInstance().cancel(this.passivePrestageCommand);
		}
	}
}
