package frc.robot.automations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.Rollers;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class PassiveRollerPreStage implements Runnable {
	private final Rollers rollers;
	private final Command prestageCommand;

	@Override
	public void run() {

	}
}
