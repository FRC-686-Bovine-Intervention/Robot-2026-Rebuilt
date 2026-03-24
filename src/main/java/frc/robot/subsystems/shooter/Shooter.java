package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.util.PIDGains;
import frc.util.loggerUtil.tunables.LoggedTunable;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Shooter {
	public final Flywheel leftFlywheel;
	public final Flywheel rightFlywheel;
	public final Hood hood;
	public final AimingSystem aimingSystem;

	private static final LoggedTunable<LinearVelocity> flywheelTolerance = LoggedTunable.from("Shooter/Tolerances/Flywheel", MetersPerSecond::of, 3);
	private static final LoggedTunable<Angle>        azimuthTolerance = LoggedTunable.from("Shooter/Tolerances/Azimuth", Degrees::of, 10.0);
	private static final LoggedTunable<Angle>          pitchTolerance = LoggedTunable.from("Shooter/Tolerances/Pitch", Degrees::of, 10.0);

	public Command aimLeftFlywheelAtHub() {
		return this.leftFlywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
	}
	public Command aimRightFlywheelAtHub() {
		return this.rightFlywheel.genSurfaceVeloCommand("Aim at Hub", this.aimingSystem.shootingCalc::getTargetFlywheelSurfaceVeloMPS);
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
				var targetHeadingRads = aimingSystem.shootingCalc.getTargetAzimuthHeadingRads();
				var pidOut = this.pid.calculate(measuredHeadingRads, targetHeadingRads);

				var translationalVelo = desiredVeloSupplier.get();

				if (Math.hypot(translationalVelo.vxMetersPerSecond, translationalVelo.vyMetersPerSecond) >= 0.1 || pidOut >= 0.1) {
					drive.translationSubsystem.driveVelocity(translationalVelo.vxMetersPerSecond, translationalVelo.vyMetersPerSecond);
					drive.rotationalSubsystem.driveVelocity(pidOut);
				} else {
					drive.translationSubsystem.cancelPostProcessing();
					drive.rotationalSubsystem.cancelPostProcessing();
					drive.stopWithX();
				}
			}

			@Override
			public void end(boolean interrupted) {
				drive.translationSubsystem.stop();
				drive.rotationalSubsystem.stop();
			}
		};
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
			LoggedTunable.from("Shooting/Passing/Rotational PID", new PIDGains(3.5, 0.0, 0.0)),
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


		double measuredAzimuth = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians();
		boolean withinAzimuthTolerance = measuredAzimuth < targetAzimuth + azimuthTolerance.get().in(Radians) && measuredAzimuth > targetAzimuth - azimuthTolerance.get().in(Radians);

		double measuredPitch = this.hood.getMeasuredAngleRads();
		Logger.recordOutput("DEBUG/Shooter/Measured Pitch", measuredPitch);
		Logger.recordOutput("DEBUG/Shooter/TargetPitch", targetHoodAngle);
		boolean withinPitchTolerance = measuredPitch < targetHoodAngle + pitchTolerance.get().in(Radians) && measuredPitch > targetHoodAngle - pitchTolerance.get().in(Radians);

		Logger.recordOutput("DEBUG/Shooter/Flywheel Within Tolerance", withinFlywheelTolerance);
		Logger.recordOutput("DEBUG/Shooter/Azimuth Within Tolerance", withinAzimuthTolerance);
		Logger.recordOutput("DEBUG/Shooter/Pitch Within Tolerance", withinPitchTolerance);
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
