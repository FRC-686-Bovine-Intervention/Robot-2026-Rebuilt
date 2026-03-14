package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.HubShifts;
import frc.util.led.animation.LEDAnimation;
import frc.util.led.strips.LEDStrip;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class HubShiftAnimation extends LEDAnimation {
	private final LEDStrip strip;
	private final Color activeNowColor;
	private final Color inactiveNowColor;
	private final Color activeFutureColor;
	private final Color inactiveFutureColor;

	@Override
	public void apply() {
		this.strip.apply((pos) -> {
			var curShift = HubShifts.getCurrentShift();
			var activeBarPos = MathUtil.inverseInterpolate(0.0, curShift.getShiftLength(), curShift.getSecsLeftInShift());
			if (pos <= activeBarPos) {
				if (curShift.isHubActive().getOurs()) {
					return this.activeNowColor;
				} else {
					return this.inactiveNowColor;
				}
			} else {
				if (curShift.next().isHubActive().getOurs()) {
					return this.activeFutureColor;
				} else {
					return this.inactiveFutureColor;
				}
			}
		});
	}
}
