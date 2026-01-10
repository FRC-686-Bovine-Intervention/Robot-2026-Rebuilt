package frc.util.led.animation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.led.functions.Gradient;
import frc.util.led.functions.InterpolationFunction;
import frc.util.led.functions.WaveFunction;
import frc.util.led.strips.LEDStrip;

public class AllianceColorAnimation {
	private final LEDStrip strip;
	// private static final Color blueAllianceColor = Color.kFirstBlue;
	// private static final Color redAllianceColor = Color.kFirstRed;
	private final Gradient unknownGradient;
	private final Gradient blueGradient;
	private final Gradient redGradient;

	public AllianceColorAnimation(LEDStrip strip, Color blueAllianceColor, Color redAllianceColor) {
		this.strip = strip;
		this.unknownGradient = InterpolationFunction.linear.gradient(blueAllianceColor, redAllianceColor);
		this.blueGradient = InterpolationFunction.linear.gradient(blueAllianceColor, Color.kBlack);
		this.redGradient = InterpolationFunction.linear.gradient(redAllianceColor, Color.kBlack);
	}

	public void apply() {
		strip.apply((pos) -> {
			var alliance = DriverStation.getAlliance();
			Gradient gradient;
			if (alliance.isEmpty()) {
				gradient = unknownGradient;
			} else {
				if (alliance.get() == Alliance.Blue) {
					gradient = blueGradient;
				} else {
					gradient = redGradient;
				}
			}
			return gradient.apply(
				WaveFunction.Sinusoidal.applyAsDouble(
					pos * 4 - Timer.getTimestamp()
				)
			);
		});
	}
}
