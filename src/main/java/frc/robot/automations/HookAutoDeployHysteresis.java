package frc.robot.automations;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.hook.Hook;
import frc.util.EdgeDetector;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class HookAutoDeployHysteresis implements Runnable {
	private final Hook hook;
	private final Command hookAutoDeployCommand;

	private static final LoggedNetworkBoolean enabled = new LoggedNetworkBoolean("Automations/Climber Hook Auto Deploy Hysteresis/Enabled", true);

	private static final LoggedTunable<Distance> hysteresisThreshold = LoggedTunable.from("Automations/Climber Hook Auto Deploy Hysterisis/Threshold", Inches::of, 4.0);

	private final EdgeDetector teleopEnableEdgeDetector = new EdgeDetector(false);

	public HookAutoDeployHysteresis(Hook hook, Command hookAutoDeployCommand) {
		this.hook = hook;
		this.hookAutoDeployCommand = hookAutoDeployCommand;
	}

	@Override
	public void run() {
		this.teleopEnableEdgeDetector.update(DriverStation.isTeleopEnabled() && enabled.getAsBoolean());
		if (this.teleopEnableEdgeDetector.risingEdge() && this.hook.getMeasuredLengthMeters() <= HookAutoDeployHysteresis.hysteresisThreshold.get().in(Meters)) {
			CommandScheduler.getInstance().schedule(this.hookAutoDeployCommand);
		}
	}
}
