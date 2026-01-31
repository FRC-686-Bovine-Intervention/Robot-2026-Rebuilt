package frc.robot.subsystems.climber.hook;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;


//REVIEW IF WE DON'T NEED ANYTHING
public class HookConstants {
	public static final Distance pivotOffset = Inches.zero();
	public static final Transform3d stage2Base = new Transform3d(
		new Translation3d(
			Inches.of(-0.5),
			Inches.zero(),
			pivotOffset
		),
		Rotation3d.kZero
	);
	//public static final Transform3d stage3Base = new Transform3d(
		//new Translation3d(
			//Inches.of(0.5),
			//Inches.zero(),
			//Inches.zero()
		//),
		//Rotation3d.kZero
	//);
	//public static final Transform3d stage4Base = new Transform3d(
		//new Translation3d(
			//Inches.of(0.5),
			//Inches.zero(),
			//Inches.zero()
		//),
		//Rotation3d.kZero
	//);

public static final int movingStageCount = 1;

	// public static final Distance sprocketRadius = Inches.of(1.273).div(2);
	// public static final Distance sprocketRadius = Inches.of(0.25).times(16).div(Math.PI*2);

	public static final Distance stageExtension = Inches.of(17);
	public static final Distance minLengthPhysical = Meters.zero();
	public static final Distance maxLengthPhysical = stageExtension.times(movingStageCount);
	public static final Distance maxLengthSoftware = maxLengthPhysical.minus(Inches.of(0));
	public static final Distance minHeightPhysical = Inches.of(26.5);
	public static final Distance maxHeightPhysical = minHeightPhysical.plus(maxLengthPhysical);

	public static final GearRatio motorToMechanism = new GearRatio()
		.planetary(4)
		.planetary(4)
	;
	public static final GearRatio sensorToMechanism = new GearRatio()
		.gear(90)
		.gear(18)
		.axle()
	;
	public static final LinearRelation stage1LinearRelation = new GearRatio()
		.sprocket(16)
		.chain(Inches.of(0.25))
	;
}
