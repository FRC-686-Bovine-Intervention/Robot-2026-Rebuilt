package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.util.mechanismUtil.GearRatio;

public class IntakeSlamConstants {
	public static final Angle minAngle = Degrees.of(0);
	public static final Angle maxAngle = Degrees.of(67);

	public static final Angle cancoderZeroOffset = Degrees.of(67);

	public static final GearRatio motorToMechanism = new GearRatio()
		.planetary(5)
		.planetary(5)
		.sprocket(20).sprocket(20)
		.sprocket(20).sprocket(20)
	;
	public static final GearRatio sensorToMechanism = new GearRatio()

	;

	public static final boolean calibrationSensorInverted = false;
}
