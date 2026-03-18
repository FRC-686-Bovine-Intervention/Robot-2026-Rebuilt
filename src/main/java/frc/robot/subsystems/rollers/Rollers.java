package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.rollers.agitator.Agitator;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.util.VirtualSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Rollers extends VirtualSubsystem {
	public final Indexer indexer;
	public final Agitator agitator;
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
	}

	public Command feed() {
		return
			Commands.parallel(
				this.indexer.index(),
				this.agitator.index(),
				this.feeder.feed()
			)
			.withName("Feed")
		;
	}
}
