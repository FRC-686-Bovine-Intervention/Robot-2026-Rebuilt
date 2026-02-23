package frc.robot.subsystems.rollers.indexer;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Indexer extends SubsystemBase {
	private final IndexerIO io;
	private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

	private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Subsystems/Rollers/Indexer/Commands/Idle/Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> indexVoltage = LoggedTunable.from("Subsystems/Rollers/Indexer/Commands/Index/Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Subsystems/Rollers/Indexer/Commands/Eject/Voltage", Volts::of, 0.0);

	public Indexer(IndexerIO io) {
		super("Rollers/Indexer");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Rollers/Indexer", this.inputs);
	}

	private Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
		final var indexer = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(indexer);
			}

			@Override
			public void execute() {
				indexer.io.setVolts(voltsSupplier.getAsDouble());
			}

			@Override
			public void end(boolean interrupted) {
				indexer.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command idle() {
		return this.genVoltageCommand(
			"Idle",
			() -> Indexer.idleVoltage.get().in(Volts)
		);
	}

	public Command index() {
		return this.genVoltageCommand(
			"Index",
			() -> Indexer.indexVoltage.get().in(Volts)
		);
	}

	public Command eject() {
		return this.genVoltageCommand(
			"Eject",
			() -> Indexer.ejectVoltage.get().in(Volts)
		);
	}
}
