package frc.robot.subsystems.rollers.agitator;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Agitator extends SubsystemBase {
	private final AgitatorIO io;
	private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

	private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Subsystems/Rollers/Agitiator/Commands/Idle/Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> agitateVoltage = LoggedTunable.from("Subsystems/Rollers/Agitiator/Commands/Agitate/Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Subsystems/Rollers/Agitiator/Commands/Eject/Voltage", Volts::of, 0.0);

	private final Alert motorDisconnectedAlert = new Alert("Subsystems/Rollers/Agitator/Alerts", "Motor Disconnected", AlertType.kError);
	private final Alert motorDisconnectedGlobalAlert = new Alert("Agitator Motor Disconnected!", AlertType.kError);

	public Agitator(AgitatorIO io) {
		super("Rollers/Agitator");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Rollers/Agitator", this.inputs);

		this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
		this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);
	}

	private Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
		final var agitator = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(agitator);
			}

			@Override
			public void execute() {
				agitator.io.setVolts(voltsSupplier.getAsDouble());
			}

			@Override
			public void end(boolean interrupted) {
				agitator.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command idle() {
		return this.genVoltageCommand(
			"Idle",
			() -> Agitator.idleVoltage.get().in(Volts)
		);
	}

	public Command index() {
		return this.genVoltageCommand(
			"Index",
			() -> Agitator.agitateVoltage.get().in(Volts)
		);
	}

	public Command eject() {
		return this.genVoltageCommand(
			"Eject",
			() -> Agitator.ejectVoltage.get().in(Volts)
		);
	}
}
