package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.leds.Leds;
import frc.util.VirtualSubsystem;

public class AutoManager extends VirtualSubsystem {
	private final AutoSelector selector;

	private Command autonomousCommand;
	// private final SuppliedEdgeDetector autoEnabled = new SuppliedEdgeDetector(DriverStation::isAutonomousEnabled);

	public AutoManager(AutoSelector selector) {
		this.selector = selector;
	}

	@Override
	public void periodic() {
		// autoEnabled.update();
		// if(autoEnabled.risingEdge()) {
		//     autonomousCommand = selector.getSelectedAutoCommand();
		//     if(autonomousCommand != null) {
		//         autonomousCommand.schedule();
		//     }
		// }
		// if(autoEnabled.fallingEdge() && autonomousCommand != null) {
		//     autonomousCommand.cancel();
		// }
	}

	public void startAuto() {
		autonomousCommand = selector.getSelectedAutoCommand();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	public void endAuto() {
		if(autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	public static Command generateAutoCommand(AutoRoutine routine, double initialDelaySeconds) {
		return new Command() {
			private final Command autoCommand = routine.generateCommand();
			private final Timer autoTimer = new Timer();
			private double autoFinishTime = 0.0;
			private boolean autoCommandRunning = false;
			private boolean autoCommandFinished = false;

			{
				this.setName("AUTO " + routine.name);
				this.addRequirements(this.autoCommand.getRequirements());
				CommandScheduler.getInstance().registerComposedCommands(this.autoCommand);
			}

			@Override
			public void initialize() {
				this.autoCommandRunning = false;
				this.autoCommandFinished = false;
				this.autoTimer.restart();
				this.runCommand();
				Leds.getInstance().autonomousBackgroundAnimation.setFlag(true);
			}

			@Override
			public void execute() {
				this.runCommand();
			}

			private void runCommand() {
				if (!this.autoCommandRunning && !this.autoCommandFinished && this.autoTimer.hasElapsed(initialDelaySeconds)) {
					this.autoCommand.initialize();
					this.autoCommandRunning = true;
				} else if (this.autoCommandRunning) {
					this.autoCommand.execute();
					if (this.autoCommand.isFinished()) {
						this.endCommand(false);
					} else if (DriverStation.isDisabled() && !this.autoCommand.runsWhenDisabled()) {
						this.endCommand(true);
					}
				}

				Leds.getInstance().autonomousDelayingAnimation.setFlag(!this.autoCommandRunning && !this.autoCommandFinished);
				Leds.getInstance().autonomousDelayingAnimation.setPos(this.autoTimer.get() / initialDelaySeconds);
				Leds.getInstance().autonomousRunningAnimation.setFlag(this.autoCommandRunning);
				if (this.autoCommandFinished) {
					if (this.autoTimer.get() <= AutoConstants.allottedAutoTime.in(Seconds)) {
						Leds.getInstance().autonomousFinishedAnimation.setPos(1.0 - ((this.autoTimer.get() - this.autoFinishTime) / (AutoConstants.allottedAutoTime.in(Seconds) - this.autoFinishTime)));
					} else {
						Leds.getInstance().autonomousFinishedAnimation.setPos((this.autoTimer.get() - AutoConstants.allottedAutoTime.in(Seconds)) / AutoConstants.disabledTime.in(Seconds));
					}
				}
			}

			private void endCommand(boolean interrupted) {
				this.autoCommand.end(interrupted);
				this.autoCommandRunning = false;
				this.autoCommandFinished = true;
				this.autoFinishTime = this.autoTimer.get();

				var overrun = this.autoTimer.get() > AutoConstants.allottedAutoTime.in(Seconds);

				if (overrun) {
					System.out.println(String.format("[AutoManager] Autonomous overran the allotted %.1f seconds!", AutoConstants.allottedAutoTime.in(Seconds)));
					Leds.getInstance().autonomousOverrunAnimation.setFlag(true);
				} else {
					Leds.getInstance().autonomousFinishedAnimation.setFlag(true);
				}

				if (interrupted) {
					System.out.println(String.format("[AutoManager] Autonomous interrupted after %.2f seconds", this.autoFinishTime));
				} else {
					System.out.println(String.format("[AutoManager] Autonomous finished in %.2f seconds", this.autoFinishTime));
				}
			}

			@Override
			public void end(boolean interrupted) {
				if (this.autoCommandRunning) {
					this.endCommand(interrupted);
				}
				this.autoTimer.stop();

				Leds.getInstance().autonomousBackgroundAnimation.setFlag(false);
				Leds.getInstance().autonomousDelayingAnimation.setFlag(false);
				Leds.getInstance().autonomousRunningAnimation.setFlag(false);
				Leds.getInstance().autonomousFinishedAnimation.setFlag(false);
				Leds.getInstance().autonomousOverrunAnimation.setFlag(false);
			}

			@Override
			public boolean isFinished() {
				return this.autoCommandFinished && this.autoTimer.get() - Math.max(this.autoFinishTime, AutoConstants.allottedAutoTime.in(Seconds)) > AutoConstants.disabledTime.in(Seconds);
			}

			@Override
			public boolean runsWhenDisabled() {
				return true;
			}
		};
	}
}
