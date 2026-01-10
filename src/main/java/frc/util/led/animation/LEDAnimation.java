package frc.util.led.animation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class LEDAnimation {
	public boolean flag;

	public void setFlag(boolean flag) {
		this.flag = flag;
	}
	public Command setFlagCommand() {
		return Commands.startEnd(
			() -> setFlag(true),
			() -> setFlag(false)
		)
		.ignoringDisable(true)
		.until(() -> !flag);
	}

	public void applyIfFlagged() {
		if (flag) {
			apply();
		}
	}
	public abstract void apply();
}
