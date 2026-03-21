package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.util.PIDGains;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.math.MathExtraUtil;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheel flywheel;
	public final Hood hood;
	public final AimingSystem aimingSystem;

	private static final LoggedTunable<LinearVelocity> flywheelTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Flywheel", MetersPerSecond::of, 1.5);
	private static final LoggedTunable<Distance>        azimuthTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Azimuth", Inches::of, 10.0);
	private static final LoggedTunable<Distance>          pitchTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Pitch", Inches::of, 10.0);
	private static final LoggedTunable<Distance>          translationTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Translation", Inches::of, 4.0);

	public Command aimFlywheelAtHub() {
		return this.flywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}

	public Command aimHoodAtHub() {
		return this.hood.genAngleCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetHoodAngleRads);
	}

	private static final LoggedTunable<PIDGains> rotationalPIDGains = LoggedTunable.from("Shooting/Aiming/Rotational PID", new PIDGains(3.5, 0.0, 0.0));

	public Command aimDriveAtHub(Drive.Rotational rotational) {
		return rotational.genHeadingPIDCommand(
			"Aim at Hub",
			rotationalPIDGains,
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.shootingCalc::getTargetAzimuthHeadingRads
		);
	}

	public Command aimDriveAtHubWithXLock(Drive drive, Supplier<ChassisSpeeds> desiredVeloSupplier) {
		final var shooter = this;
		return new Command() {
			private final PIDController pid = new PIDController(rotationalPIDGains.get().kP(), rotationalPIDGains.get().kI(), rotationalPIDGains.get().kD());

			{
				this.setName("Aim at Hub With Xlock");
				this.addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);

				this.pid.enableContinuousInput(-Math.PI, Math.PI);
			}

			@Override
			public void initialize() {
				if (rotationalPIDGains.hasChanged(this.hashCode())) {
					rotationalPIDGains.get().update(this.pid);
				}
			}

			@Override
			public void execute() {
				var measuredHeadingRads = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians();
				var targetHeadingRads = shooter.aimingSystem.shootingCalc.getTargetAzimuthHeadingRads();
				var pidOut = this.pid.calculate(measuredHeadingRads, targetHeadingRads);

				var translationalVelo = desiredVeloSupplier.get();

				if (Math.hypot(translationalVelo.vxMetersPerSecond, translationalVelo.vyMetersPerSecond) >= 0.1 || pidOut >= 0.1) {
					drive.runRobotSpeeds(
						translationalVelo.vxMetersPerSecond,
						translationalVelo.vyMetersPerSecond,
						pidOut
					);
				} else {
					drive.stopWithX();
				}
			}

			@Override
			public void end(boolean interrupted) {
				drive.stop();
			}
		};
	}

	public Command aimFlywheelToPass() {
		return this.flywheel.genSurfaceVeloCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetFlywheelSurfaceVeloMPS);
	}

	public Command aimHoodToPass() {
		return this.hood.genAngleCommand("Aim to Pass", this.aimingSystem.passingCalc::getTargetHoodAngleRads);
	}

	public Command aimDriveToPass(Drive.Rotational rotational) {
		return rotational.genHeadingPIDCommand(
			"Aim to Pass",
			LoggedTunable.from("Shooting/Passing/Rotational PID", new PIDGains(3.5, 0.0, 0.0)),
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.passingCalc::getTargetAzimuthHeadingRads
		);
	}

	public double getAverageMeasuredFlywheelSurfaceVeloMPS() {
		return this.flywheel.getMeasuredSurfaceVeloMPS();
	}

	public boolean withinTolerance() {
		double targetSurfaceVelo = this.aimingSystem.shootingCalc.getTargetFlywheelSurfaceVeloMPS();
		double targetHoodAngle = this.aimingSystem.shootingCalc.getTargetHoodAngleRads();
		double targetAzimuth = this.aimingSystem.shootingCalc.getTargetAzimuthHeadingRads();
		var shotPose = this.aimingSystem.shootingCalc.getShotPose();

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

		var withinTranslationTolerance = GeomUtil.isNear(shotPose, RobotState.getInstance().getEstimatedGlobalPose().getTranslation(), translationTolerance.get());

		return withinFlywheelTolerance && withinAzimuthTolerance && withinPitchTolerance && withinTranslationTolerance;
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
