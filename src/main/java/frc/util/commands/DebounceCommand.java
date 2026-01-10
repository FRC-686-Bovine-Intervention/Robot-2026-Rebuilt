package frc.util.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

public class DebounceCommand extends Command {
	private final BooleanSupplier source;
	private final Debouncer debouncer;

	public DebounceCommand(BooleanSupplier source, Debouncer debouncer) {
		this.source = source;
		this.debouncer = debouncer;
	}

	@Override
	public boolean isFinished() {
		return this.debouncer.calculate(this.source.getAsBoolean());
	}
}
