package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.math.MathExtraUtil;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheel leftFlywheel;
	public final Flywheel rightFlywheel;
	public final Hood hood;
	public final AimingSystem aimingSystem;

	private static final LoggedTunable<LinearVelocity> flywheelTolerance = LoggedTunable.from("Shooter/Tolerances/Flywheel", MetersPerSecond::of, 0.5);
	private static final LoggedTunable<Distance>        azimuthTolerance = LoggedTunable.from("Shooter/Tolerances/Azimuth", Inches::of, 5);
	private static final LoggedTunable<Distance>          pitchTolerance = LoggedTunable.from("Shooter/Tolerances/Pitch", Inches::of, 5);

	public Command aimLeftFlywheelAtHub() {
		return this.leftFlywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimRightFlywheelAtHub() {
		return this.rightFlywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimHoodAtHub() {
		return this.hood.genAngleCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetHoodAngleRads);
	}
	public Command aimDriveAtHub(Drive.Rotational rotational) {
		return rotational.genHeadingPIDCommand(
			"Aim at Hub",
			LoggedTunable.from("Shooting/Aiming/Rotational PID", new PIDConstants(5.0, 0.0, 0.0)),
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.shootingCalc::getTargetAzimuthHeadingRads
		);
	}

	public Command aimLeftFlywheelToPass() {
		return this.leftFlywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimRightFlywheelToPass() {
		return this.rightFlywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimHoodToPass() {
		return this.hood.genAngleCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetHoodAngleRads);
	}
	public Command aimDriveToPass(Drive.Rotational rotational) {
		return rotational.genHeadingPIDCommand(
			"Aim to Pass",
			LoggedTunable.from("Shooting/Passing/Rotational PID", new PIDConstants(0.0, 0.0, 0.0)),
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.passingCalc::getTargetAzimuthHeadingRads
		);
	}

	public double getAverageMeasuredFlywheelSurfaceVeloMPS() {
		return (this.leftFlywheel.getMeasuredSurfaceVeloMPS() + this.rightFlywheel.getMeasuredSurfaceVeloMPS()) / 2.0;
	}

	public boolean withinTolerance() {
		double targetSurfaceVelo = this.aimingSystem.shootingCalc.getTargetFlywheelSurfaceVeloMPS();
		double targetHoodAngle = this.aimingSystem.shootingCalc.getTargetHoodAngleRads();
		double targetAzimuth = this.aimingSystem.shootingCalc.getTargetAzimuthHeadingRads();

		double avgSurfaceVelo = getAverageMeasuredFlywheelSurfaceVeloMPS();
		boolean withinFlywheelTolerance = avgSurfaceVelo < targetSurfaceVelo + flywheelTolerance.get().in(MetersPerSecond) && avgSurfaceVelo > targetSurfaceVelo - flywheelTolerance.get().in(MetersPerSecond);


		var targetVector = getLaunchVector(targetSurfaceVelo, targetHoodAngle, targetAzimuth);

		double measuredAzimuth = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians();
		var measuredAzimuthVector = new double[] {Math.cos(measuredAzimuth), Math.sin(measuredAzimuth)};
		measuredAzimuthVector = MathExtraUtil.matchVectorLength2d(measuredAzimuthVector, targetVector);
		double azimuthDistance = Math.sqrt(Math.pow(measuredAzimuthVector[0] - targetVector[0], 2) + Math.pow(measuredAzimuthVector[1] - targetVector[1], 2));
		boolean withinAzimuthTolerance = azimuthDistance < azimuthTolerance.get().in(Meters);

		double measuredPitch = Math.PI / 2.0 - this.hood.getMeasuredAngleRads();
		var measuredPitchVector = new double[] {Math.cos(measuredPitch), Math.sin(measuredPitch)};
		var targetVectorSideView = new double[] {Math.sqrt(Math.pow(targetVector[0], 2) + Math.pow(targetVector[1], 2)), targetVector[2]};
		measuredPitchVector = MathExtraUtil.matchVectorLength2d(measuredPitchVector, targetVectorSideView);
		double pitchDistance = Math.sqrt(Math.pow(measuredPitchVector[0] - targetVectorSideView[0], 2) + Math.pow(measuredPitchVector[1] - targetVectorSideView[1], 2));
		boolean withinPitchTolerance = pitchDistance < pitchTolerance.get().in(Meters);

		return withinFlywheelTolerance && withinAzimuthTolerance && withinPitchTolerance;
	}



	public static double[] getLaunchVector(double flywheelSpeedMPS, double hoodAngleRads, double robotAngleRads) {
		double exitVelo = Shooter.getExitVelo(flywheelSpeedMPS, Shooter.getFVelo());
		double x = exitVelo * Math.cos(Math.PI / 2.0 - hoodAngleRads) * Math.cos(robotAngleRads);
		double y = exitVelo * Math.cos(Math.PI / 2.0 - hoodAngleRads) * Math.sin(robotAngleRads);
		double z = exitVelo * Math.sin(Math.PI / 2.0 - hoodAngleRads);
		return new double[] {x, y, z};
	}

	public static double getExitVelo(double flywheelSpeedMPS, double fVelo) {
		return (flywheelSpeedMPS  + flywheelSpeedMPS * fVelo)/2;
	}

	public static double getFlywheelSpeedMPS(double exitVelo) {
		return (2 * exitVelo) / (1 + getFVelo());
	}

	public static double getFVelo() {
		return (FlywheelConstants.flywheelToHood.reductionUnsigned() / FlywheelConstants.wheel.effectiveRadius().in(Meters)) * FlywheelConstants.hoodRoller.effectiveRadius().in(Meters);
	}
}
