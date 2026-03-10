package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class IntakeRollers extends SubsystemBase {
	private final IntakeRollersIO io;
	private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

	private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Subsystems/Intake/Rollers/Commands/Idle/Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> intakeVoltage = LoggedTunable.from("Subsystems/Intake/Rollers/Commands/Intake/Voltage", Volts::of, 12.0);
	private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Subsystems/Intake/Rollers/Commands/Eject/Voltage", Volts::of, 0.0);

	public IntakeRollers(IntakeRollersIO io) {
		super("Intake/Rollers");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Intake/Rollers", this.inputs);
	}

	private Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
		final var rollers = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(rollers);
			}

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				rollers.io.setVolts(voltsSupplier.getAsDouble());
			}

			@Override
			public void end(boolean interrupted) {
				rollers.io.stop(NeutralMode.DEFAULT);
			}
		};
	}

	public Command idle() {
		return this.genVoltageCommand(
			"Idle",
			() -> IntakeRollers.idleVoltage.get().in(Volts)
		);
	}

	public Command intake() {
		return this.genVoltageCommand(
			"Intake",
			() -> IntakeRollers.intakeVoltage.get().in(Volts)
		);
	}

	public Command eject() {
		return this.genVoltageCommand(
			"Eject",
			() -> IntakeRollers.ejectVoltage.get().in(Volts)
		);
	}
}
