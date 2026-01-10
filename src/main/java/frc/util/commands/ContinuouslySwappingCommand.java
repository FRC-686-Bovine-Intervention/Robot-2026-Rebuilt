package frc.util.commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ContinuouslySwappingCommand extends Command {
	private final Supplier<Command> supplier;
	private Command currentlyRunningCommand;

	public ContinuouslySwappingCommand(Supplier<Command> supplier, Set<Subsystem> requirements) {
		this.supplier = supplier;
		addRequirements(requirements);
	}

	@Override
	public void initialize() {
		currentlyRunningCommand = null;
		execute();
	}

	@Override
	public void execute() {
		if (currentlyRunningCommand != null) {
			currentlyRunningCommand.execute();
		}
		var newCommand = supplier.get();
		if (currentlyRunningCommand != newCommand) {
			if (currentlyRunningCommand != null) {
				currentlyRunningCommand.end(true);
			}
			currentlyRunningCommand = newCommand;
			currentlyRunningCommand.initialize();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (currentlyRunningCommand != null) {
			currentlyRunningCommand.end(interrupted);
		}
	}

	@Override
	public boolean isFinished() {
		if (currentlyRunningCommand == null) {
			return false;
		}
		return currentlyRunningCommand.isFinished();
	}
}
