package frc.robot.subsystems.rollers;

import frc.robot.subsystems.rollers.agitator.Agitator;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.indexer.Indexer;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Rollers {
	public final Indexer indexer;
	public final Agitator agitiator;
	public final Feeder feeder;
}
