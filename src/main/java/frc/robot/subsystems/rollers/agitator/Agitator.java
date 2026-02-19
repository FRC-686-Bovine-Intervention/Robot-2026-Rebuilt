package frc.robot.subsystems.rollers.agitator;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Agitator extends SubsystemBase {
	private final AgitatorIO io;
	private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

	private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Rollers/Agitiator/Idle Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> indexVoltage = LoggedTunable.from("Rollers/Agitiator/Index Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Rollers/Agitiator/Eject Voltage", Volts::of, 0.0);

	public Agitator(AgitatorIO io) {
		super("Rollers/Agitator");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Rollers/Agitator", this.inputs);
	}

	private Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
		final var indexer = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(indexer);
			}

			@Override
			public void initialize() {

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
			() -> idleVoltage.get().in(Volts)
		);
	}

	public Command index() {
		return this.genVoltageCommand(
			"Index",
			() -> indexVoltage.get().in(Volts)
		);
	}

	public Command eject() {
		return this.genVoltageCommand(
			"Eject",
			() -> ejectVoltage.get().in(Volts)
		);
	}
}
