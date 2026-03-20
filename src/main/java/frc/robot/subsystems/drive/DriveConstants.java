package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Arrays;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.HardwareDevices;
import frc.util.hardwareID.can.CANDevice;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public final class DriveConstants {
	public static final double odometryLoopFrequencyHz = 250.0;
	public static final Frequency odometryLoopFrequency = Hertz.of(odometryLoopFrequencyHz);

	/**Distance between the front and back wheels*/
	public static final Distance trackWidthX = Inches.of(25.5);
	/**Distance between the left and right wheels*/
	public static final Distance trackWidthY = Inches.of(25.5);

	public static class ModuleConstants {
		public final String name;
		public final CANDevice driveMotorID;
		public final CANDevice azimuthMotorID;
		public final InvertedValue driveInverted;
		public final Angle encoderZeroOffset;
		public final Transform2d moduleTransform;
		public final Rotation2d positiveRotVec;
		ModuleConstants(String name, CANDevice driveMotorID, CANDevice turnMotorID, InvertedValue driveInverted, Angle encoderZeroOffset, Transform2d moduleTransform) {
			this.name = name;
			this.driveMotorID = driveMotorID;
			this.azimuthMotorID = turnMotorID;
			this.driveInverted = driveInverted;
			this.encoderZeroOffset = encoderZeroOffset;
			this.moduleTransform = moduleTransform;
			this.positiveRotVec = this.moduleTransform.getTranslation().getAngle().plus(Rotation2d.kCCW_90deg);
		}
	}

	public static final ModuleConstants[] moduleConstants = {
		new ModuleConstants(
			"Front Left",
			HardwareDevices.frontLeftDriveMotorID, HardwareDevices.frontLeftAzimuthMotorID,
			InvertedValue.CounterClockwise_Positive,
			Rotations.of(0.3238442),
			new Transform2d(
				new Translation2d(
					trackWidthX.div(+2),
					trackWidthY.div(+2)
				),
				Rotation2d.kCCW_90deg
			)
		),
		new ModuleConstants(
			"Front Right",
			HardwareDevices.frontRightDriveMotorID, HardwareDevices.frontRightAzimuthMotorID,
			InvertedValue.CounterClockwise_Positive,
			Rotations.of(0.8459126),
			new Transform2d(
				new Translation2d(
					trackWidthX.div(+2),
					trackWidthY.div(-2)
				),
				Rotation2d.kZero
			)
		),
		new ModuleConstants(
			"Back Left",
			HardwareDevices.backLeftDriveMotorID, HardwareDevices.backLeftAzimuthMotorID,
			InvertedValue.CounterClockwise_Positive,
			Rotations.of(0.000863050503),
			new Transform2d(
				new Translation2d(
					trackWidthX.div(-2),
					trackWidthY.div(+2)
				),
				Rotation2d.k180deg
			)
		),
		new ModuleConstants(
			"Back Right",
			HardwareDevices.backRightDriveMotorID, HardwareDevices.backRightAzimuthMotorID,
			InvertedValue.CounterClockwise_Positive,
			Rotations.of(0.5872595),
			new Transform2d(
				new Translation2d(
					trackWidthX.div(-2),
					trackWidthY.div(-2)
				),
				Rotation2d.kCW_90deg
			)
		),
	};
	public static final Translation2d[] moduleTranslations = Arrays.stream(moduleConstants).map((a) -> a.moduleTransform.getTranslation()).toArray(Translation2d[]::new);
	public static final Distance driveBaseRadius = Meters.of(Arrays.stream(moduleTranslations).mapToDouble((t) -> t.getNorm()).max().orElse(0.5));

	public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

	public static final LinearRelation wheel = LinearRelation.wheelRadius(Inches.of(1.4835686288396965));

	public static final GearRatio driveMotorToWheelRatio = new GearRatio()
		.gear(14).gear(22).axle()
		.gear(15).gear(45).axle()
	;
	public static final GearRatio azimuthMotorToEncoderRatio = new GearRatio()
		.planetary(GearRatio.ULTRAPLANETARY_3_1)
		.planetary(GearRatio.ULTRAPLANETARY_4_1)
		.gear(14).gear(62).axle()
	;
	public static final GearRatio azimuthEncoderToCarriageRatio = new GearRatio()

	;
	public static final GearRatio azimuthMotorToCarriageRatio = azimuthMotorToEncoderRatio.then(azimuthEncoderToCarriageRatio);

	public static final LinearVelocity maxModuleSpeed = wheel.angularVelocityToLinearVelocity(driveMotorToWheelRatio.applyUnsigned(RadiansPerSecond.of(DCMotor.getFalcon500(1).freeSpeedRadPerSec)));

	public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(6);
	public static final AngularVelocity maxTurnRate = RadiansPerSecond.of(maxDriveSpeed.in(MetersPerSecond) / driveBaseRadius.in(Meters));
}
