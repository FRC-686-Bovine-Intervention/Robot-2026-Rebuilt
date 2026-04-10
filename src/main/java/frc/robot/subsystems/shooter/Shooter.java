package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
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
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheel flywheel;
	public final Hood hood;
	public final AimingSystem aimingSystem;

	private static final LoggedTunable<LinearVelocity>  shooterTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Shooter", MetersPerSecond::of, 1.0);
	private static final LoggedTunable<Distance>        azimuthTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Azimuth", Inches::of, 6.0);
	private static final LoggedTunable<Distance>    translationTolerance = LoggedTunable.from("Shooting/Aiming/Tolerances/Translation", Inches::of, 4.0);

	public Command aimFlywheelAtHub() {
		return this.flywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}

	public Command aimHoodAtHub() {
		return this.hood.genAngleCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetHoodAngleRads);
	}

	private static final LoggedTunable<PIDGains> rotationalPIDGains = LoggedTunable.from("Shooting/Aiming/Azimuth/Rotational PID", new PIDGains(5.0, 0.0, 0.0));
	private static final LoggedTunable<AngularVelocity> rotationalPIDMaxOmega = LoggedTunable.from("Shooting/Aiming/Azimuth/Max Omega", RotationsPerSecond::of, 1.5);

	public Command aimDriveAtHub(Drive.Rotational rotational) {
		return rotational.genHeadingPIDCommand(
			"Aim at Hub",
			rotationalPIDGains,
			() -> rotationalPIDMaxOmega.get().in(RadiansPerSecond),
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

				if (Math.hypot(translationalVelo.vxMetersPerSecond, translationalVelo.vyMetersPerSecond) >= 0.1 || Math.abs(pidOut) >= 0.15) {
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
			LoggedTunable.from("Shooting/Passing/Azimuth/Rotational PID", new PIDGains(5.0, 0.0, 0.0)),
			() -> rotationalPIDMaxOmega.get().in(RadiansPerSecond),
			() -> RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(),
			this.aimingSystem.passingCalc::getTargetAzimuthHeadingRads
		);
	}

	public boolean withinShootingTolerance() {
		final var aimPoint = this.aimingSystem.shootingCalc.getAimPoint();
		final var shotPose = this.aimingSystem.shootingCalc.getShotPose();
		final var targetSurfaceVeloMPS = this.aimingSystem.shootingCalc.getTargetFlywheelSurfaceVeloMPS();
		final var targetHoodAngleRads = this.aimingSystem.shootingCalc.getTargetHoodAngleRads();
		final var targetAzimuthHeadingRads = this.aimingSystem.shootingCalc.getTargetAzimuthHeadingRads();

		return this.withinTolerance(
			aimPoint,
			shotPose,
			targetSurfaceVeloMPS,
			targetHoodAngleRads,
			targetAzimuthHeadingRads
		);
	}

	public boolean withinPassingTolerance() {
		final var aimPoint = this.aimingSystem.passingCalc.getAimPoint();
		final var shotPose = this.aimingSystem.passingCalc.getShotPose();
		final var targetSurfaceVeloMPS = this.aimingSystem.passingCalc.getTargetFlywheelSurfaceVeloMPS();
		final var targetHoodAngleRads = this.aimingSystem.passingCalc.getTargetHoodAngleRads();
		final var targetAzimuthHeadingRads = this.aimingSystem.passingCalc.getTargetAzimuthHeadingRads();

		return this.withinTolerance(
			aimPoint,
			shotPose,
			targetSurfaceVeloMPS,
			targetHoodAngleRads,
			targetAzimuthHeadingRads
		);
	}

	private boolean withinTolerance(
		Translation3d aimPoint,
		Translation2d shotPose,
		double targetSurfaceVeloMPS,
		double targetHoodAngleRads,
		double targetAzimuthHeadingRads
	) {
		final var targetLaunchVector = Shooter.calculateLaunchVector(targetSurfaceVeloMPS, targetHoodAngleRads, targetAzimuthHeadingRads);
		final var targetVectorFloor = Math.hypot(targetLaunchVector.getX(), targetLaunchVector.getY());

		final var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
		final var measuredSurfaceVeloMPS = this.flywheel.getMeasuredSurfaceVeloMPS();
		final var measuredHoodAngleRads = this.hood.getMeasuredAngleRads();
		final var measuredAzimuthHeadingRads = robotPose.getRotation().getRadians();
		final var measuredLaunchVector = Shooter.calculateLaunchVector(measuredSurfaceVeloMPS, measuredHoodAngleRads, measuredAzimuthHeadingRads);
		final var measuredVectorFloor = Math.hypot(measuredLaunchVector.getX(), measuredLaunchVector.getY());


		final var azimuthDot = targetLaunchVector.getX() * measuredLaunchVector.getX() + targetLaunchVector.getY() * measuredLaunchVector.getY();
		final var distanceToTargetMeters = robotPose.getTranslation().getDistance(aimPoint.toTranslation2d());
		final var hubradiusMeters = azimuthTolerance.get().in(Meters);
		final var a = Math.hypot(hubradiusMeters, distanceToTargetMeters);
		final var azimuthToleranceDot = hubradiusMeters / a;

		final var withinAzimuthTolerance = azimuthDot >= azimuthToleranceDot;


		final var errorX = targetVectorFloor - measuredVectorFloor;
		final var errorY = targetLaunchVector.getZ() - measuredLaunchVector.getZ();
		final var errorMPS = Math.hypot(errorX, errorY);

		final var withinShooterTolerance = errorMPS <= shooterTolerance.get().in(MetersPerSecond);

		final var withinTranslationTolerance = GeomUtil.isNear(shotPose, RobotState.getInstance().getEstimatedGlobalPose().getTranslation(), translationTolerance.get().in(Meters));

		Logger.recordOutput("Subsystems/Shooter/Aiming/Tolerances/Within Azimuth Tolerance", withinAzimuthTolerance);
		Logger.recordOutput("Subsystems/Shooter/Aiming/Tolerances/Shooter Tolerance", withinShooterTolerance);
		Logger.recordOutput("Subsystems/Shooter/Aiming/Tolerances/Translation Tolerance", withinTranslationTolerance);

		return withinAzimuthTolerance && withinShooterTolerance && withinTranslationTolerance;
	}



	public static Translation3d calculateLaunchVector(double flywheelSpeedMPS, double hoodAngleRads, double robotAngleRads) {
		double exitVelo = Shooter.calculateExitVeloMPS(flywheelSpeedMPS, Shooter.getFRatio());
		double x = exitVelo * Math.cos(Math.PI / 2.0 - hoodAngleRads) * Math.cos(robotAngleRads);
		double y = exitVelo * Math.cos(Math.PI / 2.0 - hoodAngleRads) * Math.sin(robotAngleRads);
		double z = exitVelo * Math.sin(Math.PI / 2.0 - hoodAngleRads);
		return new Translation3d(x, y, z);
	}

	public static double calculateExitVeloMPS(double flywheelSpeedMPS, double fVelo) {
		return (flywheelSpeedMPS  + flywheelSpeedMPS * fVelo) / 2.0;
	}

	public static double calculateFlywheelSpeedMPS(double exitVelo) {
		return (2.0 * exitVelo) / (1.0 + Shooter.getFRatio());
	}

	public static double getFRatio() {
		return (FlywheelConstants.flywheelToHood.reductionUnsigned() / FlywheelConstants.wheel.effectiveRadius().in(Meters)) * FlywheelConstants.hoodRoller.effectiveRadius().in(Meters);
	}
}
