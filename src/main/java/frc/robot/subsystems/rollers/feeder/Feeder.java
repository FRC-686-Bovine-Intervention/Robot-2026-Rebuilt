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
	private static final LoggedTunable<Voltage> feedVoltage = LoggedTunable.from("Rollers/Feeder/Feed Voltage", Volts::of, 6.0);

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

	public Command feed() {
		return this.genVoltageCommand(
			"Feed",
			() -> feedVoltage.get().in(Volts)
		);
	}
}
