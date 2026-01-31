package frc.robot.subsystems.rollers.feeder;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Feeder extends SubsystemBase {
	private final FeederIO io;
	private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

	private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Rollers/Feeder/Idle Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> feederVoltage = LoggedTunable.from("Rollers/Feeder/Index Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Rollers/Feeder/Eject Voltage", Volts::of, 0.0);

	public Feeder(FeederIO io) {
		super("Rollers/Feeder");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Rollers/Feeder", this.inputs);
	}

	private Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
		final var feeder = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(feeder);
			}

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				feeder.io.setVolts(voltsSupplier.getAsDouble());
			}

			@Override
			public void end(boolean interrupted) {
				feeder.io.stop(NeutralMode.DEFAULT);
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
			"Feed",
			() -> feederVoltage.get().in(Volts)
		);
	}

	public Command eject() {
		return this.genVoltageCommand(
			"Eject",
			() -> ejectVoltage.get().in(Volts)
		);
	}
}
