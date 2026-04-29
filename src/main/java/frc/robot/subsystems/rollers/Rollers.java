package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.util.VirtualSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Rollers extends VirtualSubsystem {
	public final Indexer indexer;
	public final Feeder feeder;
	private final RollerSensorsIO sensorsIO;
	private final RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

	@Getter
	private boolean feederSensorTripped = false;

	@Override
	public void periodic() {
		this.sensorsIO.updateInputs(this.sensorsInputs);
		Logger.processInputs("Inputs/Rollers/Sensors", this.sensorsInputs);

		this.feederSensorTripped = this.sensorsInputs.feederSensor;

		Leds.getInstance().fuelStagedAnimation.setFlag(this.isFeederSensorTripped());
	}

	public Command idle() {
		return
			Commands.parallel(
				this.indexer.idle(),
				this.feeder.idle()
			)
			.withName("Idle")
		;
	}

	public Command feed() {
		return
			Commands.parallel(
				this.indexer.index(),
				this.feeder.feed()
			)
			.withName("Feed")
		;
	}

	public Command eject() {
		return
			Commands.parallel(
				this.indexer.eject(),
				this.feeder.eject()
			)
			.withName("Eject")
		;
	}

	public Command passivePrestage() {
		return
			Commands.parallel(
				this.indexer.passivePrestage(),
				this.feeder.passivePrestage()
			)
			.withName("Passive Prestage")
		;
	}

	public Command untilNoBalls(double debounceSeconds) {
		final var rollers = this;
		return new Command() {
			private final Debouncer debouncer = new Debouncer(debounceSeconds, DebounceType.kFalling);

			{
				this.setName("Until no balls");
			}

			@Override
			public void initialize() {
				this.debouncer.calculate(true);
			}

			@Override
			public boolean isFinished() {
				return !this.debouncer.calculate(rollers.isFeederSensorTripped());
			}
		};
	}
}
