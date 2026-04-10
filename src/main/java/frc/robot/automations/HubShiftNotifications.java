package frc.robot.automations;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HubShifts;
import frc.robot.HubShifts.Shift;
import frc.robot.subsystems.leds.Leds;
import frc.util.EdgeDetector;
import frc.util.controllers.XboxController;

public class HubShiftNotifications implements Runnable {
	private final Command rumbleCountdownCommand;
	private final Command rumbleActiveWarningCommand;
	private final Command ledsActiveWarningCommand;
	private final Command rumbleActiveAlertCommand;
	private final Command ledsActiveAlertCommand;
	private final Command rumbleInactiveAlertCommand;

	public HubShiftNotifications(XboxController controller) {
		final var leftRumble = controller.leftRumble;
		final var rightRumble = controller.rightRumble;
		this.rumbleCountdownCommand = new Command() {
			{
				this.setName("Hub Shift Countdown");
				this.addRequirements(leftRumble, rightRumble);
			}

			private Shift initShift;

			@Override
			public void initialize() {
				this.initShift = HubShifts.getCurrentShift();
			}

			@Override
			public void execute() {
				var wrappedTime = this.initShift.getSecsSinceShiftStarted() % 1.0;
				if (initShift.isHubActive().getOurs()) {
					if (initShift.next().isHubActive().getOurs()) {
						leftRumble.setRumble((wrappedTime < 0.3) ? (0.8) : (0.0));
					} else {
						if (this.initShift.getSecsLeftInShift() > 0.0) {
							rightRumble.setRumble((wrappedTime < 0.5) ? (0.8) : (0.0));
						} else {
							leftRumble.setRumble((wrappedTime < 0.3) ? (0.8) : (0.0));
						}
					}
				} else {
					if (initShift.next().isHubActive().getOurs()) {
						if (this.initShift.getSecsLeftInShift() > 0.0) {
							leftRumble.setRumble((wrappedTime < 0.3) ? (0.8) : (0.0));
						} else {
							rightRumble.setRumble((wrappedTime < 0.5) ? (0.8) : (0.0));
						}
					} else {
						rightRumble.setRumble((wrappedTime < 0.5) ? (0.8) : (0.0));
					}
				}
			}

			@Override
			public void end(boolean interrupted) {
				leftRumble.setRumble(0.0);
				rightRumble.setRumble(0.0);
			}

			@Override
			public boolean isFinished() {
				return this.initShift.getSecsLeftInShift() < -1.0;
			}
		};
		this.rumbleActiveWarningCommand = new Command() {
			{
				this.setName("Hub Shift Active Warning");
				this.addRequirements(leftRumble, rightRumble);
			}

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				var wrappedTime = HubShifts.getCurrentShift().getSecsSinceShiftStarted() % 0.25;
				if (wrappedTime < 0.125) {
					leftRumble.setRumble(0.8);
					rightRumble.setRumble(0.8);
				} else {
					leftRumble.setRumble(0.0);
					rightRumble.setRumble(0.0);
				}
			}

			@Override
			public void end(boolean interrupted) {
				leftRumble.setRumble(0.0);
				rightRumble.setRumble(0.0);
			}

			@Override
			public boolean isFinished() {
				return HubShifts.getCurrentShift().getSecsLeftInShift() < 9.0;
			}
		};
		this.rumbleActiveAlertCommand = new Command() {
			{
				this.setName("Hub Shift Active Alert");
				this.addRequirements(leftRumble, rightRumble);
			}

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				leftRumble.setRumble(0.8);
				rightRumble.setRumble(0.8);
			}

			@Override
			public void end(boolean interrupted) {
				leftRumble.setRumble(0.0);
				rightRumble.setRumble(0.0);
			}

			@Override
			public boolean isFinished() {
				return HubShifts.getCurrentShift().getSecsLeftInShift() < 4.0;
			}
		};
		this.rumbleInactiveAlertCommand = new Command() {
			{
				this.setName("Hub Shift Inactive Alert");
				this.addRequirements(leftRumble, rightRumble);
			}

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {
				leftRumble.setRumble(0.8);
				rightRumble.setRumble(0.8);
			}

			@Override
			public void end(boolean interrupted) {
				leftRumble.setRumble(0.0);
				rightRumble.setRumble(0.0);
			}

			@Override
			public boolean isFinished() {
				return HubShifts.getCurrentShift().getSecsLeftInShift() < 5.0;
			}
		};

		this.ledsActiveWarningCommand = Leds.getInstance().hubShiftDynamicWarningAnimation.setFlagCommand().withTimeout(1.0);
		this.ledsActiveAlertCommand = Leds.getInstance().hubShiftDynamicAlertAnimation.setFlagCommand().withTimeout(1.0);
	}

	private final EdgeDetector activeWarningDetector = new EdgeDetector(false);
	private final EdgeDetector activeAlertDetector = new EdgeDetector(false);
	private final EdgeDetector inactiveAlertDetector = new EdgeDetector(false);
	private final EdgeDetector countdownDetector = new EdgeDetector(false);

	@Override
	public void run() {
		this.countdownDetector.update(HubShifts.getCurrentShift() != Shift.Auto && HubShifts.getCurrentShift().getSecsLeftInShift() < 3.0);

		this.activeWarningDetector.update(HubShifts.getCurrentShift() != Shift.Auto && HubShifts.getCurrentShift().getSecsLeftInShift() < 10.0 && !HubShifts.getCurrentShift().isHubActive().getOurs() && HubShifts.getCurrentShift().next().isHubActive().getOurs());
		this.activeAlertDetector.update(HubShifts.getCurrentShift() != Shift.Auto && HubShifts.getCurrentShift().getSecsLeftInShift() < 5.0 && !HubShifts.getCurrentShift().isHubActive().getOurs() && HubShifts.getCurrentShift().next().isHubActive().getOurs());
		this.inactiveAlertDetector.update(HubShifts.getCurrentShift() != Shift.Auto && HubShifts.getCurrentShift().getSecsLeftInShift() < 6.0 && HubShifts.getCurrentShift().isHubActive().getOurs() && !HubShifts.getCurrentShift().next().isHubActive().getOurs());

		if (this.countdownDetector.risingEdge()) {
			CommandScheduler.getInstance().schedule(this.rumbleCountdownCommand);
		}
		Leds.getInstance().hubShiftStaticGoodAnimation.setFlag(DriverStation.isTeleopEnabled() && !HubShifts.getCurrentShift().isHubActive().getOurs());
		if (this.activeWarningDetector.risingEdge()) {
			CommandScheduler.getInstance().schedule(this.rumbleActiveWarningCommand);
			CommandScheduler.getInstance().schedule(this.ledsActiveWarningCommand);
		}
		Leds.getInstance().hubShiftStaticWarningAnimation.setFlag(this.activeWarningDetector.getValue());
		if (this.activeAlertDetector.risingEdge()) {
			CommandScheduler.getInstance().schedule(this.rumbleActiveAlertCommand);
			CommandScheduler.getInstance().schedule(this.ledsActiveAlertCommand);
		}
		Leds.getInstance().hubShiftStaticAlertAnimation.setFlag(this.activeAlertDetector.getValue());
		if (this.inactiveAlertDetector.risingEdge()) {
			CommandScheduler.getInstance().schedule(this.rumbleInactiveAlertCommand);
		}
	}
}
