package frc.robot.subsystems.rollers.feeder;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BatteryLogger;
import frc.robot.RobotType;
import frc.robot.RobotType.Mode;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Feeder extends SubsystemBase {
	private final FeederIO io;
	private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

	private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Subsystems/Rollers/Feeder/Commands/Idle/Voltage", Volts::of, 0.0);
	private static final LoggedTunable<Voltage> feedVoltage = LoggedTunable.from("Subsystems/Rollers/Feeder/Commands/Feed/Voltage", Volts::of, 12.0);
	private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Subsystems/Rollers/Feeder/Commands/Eject/Voltage", Volts::of, -4.0);
	private static final LoggedTunable<Voltage> passivePrestageVoltage = LoggedTunable.from("Subsystems/Rollers/Feeder/Commands/Passive Prestage/Voltage", Volts::of, 1.0);

	private final Alert leftMotorDisconnectedAlert = new Alert("Subsystems/Rollers/Feeder/Alerts", "Left Motor Disconnected", AlertType.kError);
	private final Alert leftMotorDisconnectedGlobalAlert = new Alert("Feeder Left Motor Disconnected!", AlertType.kError);
	private final Alert rightMotorDisconnectedAlert = new Alert("Subsystems/Rollers/Feeder/Alerts", "Right Motor Disconnected", AlertType.kError);
	private final Alert rightMotorDisconnectedGlobalAlert = new Alert("Feeder Right Motor Disconnected!", AlertType.kError);

	public Feeder(FeederIO io) {
		super("Rollers/Feeder");
		this.io = io;
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Inputs/Rollers/Feeder", this.inputs);

		this.leftMotorDisconnectedAlert.set(!this.inputs.leftMotorConnected);
		this.leftMotorDisconnectedGlobalAlert.set(!this.inputs.leftMotorConnected);
		this.rightMotorDisconnectedAlert.set(!this.inputs.rightMotorConnected);
		this.rightMotorDisconnectedGlobalAlert.set(!this.inputs.rightMotorConnected);

		if (RobotType.getMode() == Mode.REPLAY) {
			BatteryLogger.getInstance().logMechanism(
				"Rollers/Feeder",
				this.inputs.leftMotor.getSupplyCurrentAmps()
				+ this.inputs.rightMotor.getSupplyCurrentAmps()
			);
		}
	}

	private Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
		final var feeder = this;
		return new Command() {
			{
				this.setName(name);
				this.addRequirements(feeder);
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
			() -> Feeder.idleVoltage.get().in(Volts)
		);
	}

	public Command feed() {
		return this.genVoltageCommand(
			"Feed",
			() -> Feeder.feedVoltage.get().in(Volts)
		);
	}

	public Command eject() {
		return this.genVoltageCommand(
			"Eject",
			() -> Feeder.ejectVoltage.get().in(Volts)
		);
	}

	public Command passivePrestage() {
		return this.genVoltageCommand(
			"Passive Prestage",
			() -> Feeder.passivePrestageVoltage.get().in(Volts)
		);
	}
}
