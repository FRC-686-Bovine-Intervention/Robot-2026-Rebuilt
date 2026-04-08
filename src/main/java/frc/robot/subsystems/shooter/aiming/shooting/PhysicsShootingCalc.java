package frc.robot.subsystems.shooter.aiming.shooting;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class PhysicsShootingCalc implements ShootingCalc {
	private double hoodAngleRads;
	private double flywheelSpeedMS;
	private double robotRotationRads;
	private Translation3d aimPoint;
	private Translation2d shotPose;
	private double tofSeconds;

	private static final LoggedTunable<Distance> distanceOffset = LoggedTunable.from("Shooting/Aiming/Distance Offset", Inches::of, 0.0);

	public PhysicsShootingCalc() {
		// Preload polynomials
		ShooterConstants.flywheelPolynomial.evaluate(0.0, 0.0);
		ShooterConstants.hoodPolynomial.evaluate(0.0, 0.0);
	}

	@Override
	public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		this.shotPose = robotPose.getTranslation();
		final var shooterHubSpaceCartesian = new Pose3d(robotPose).transformBy(HoodConstants.hoodBase).getTranslation().toTranslation2d().minus(aimPoint.toTranslation2d());
		final var radiusMeters = shooterHubSpaceCartesian.getNorm() + PhysicsShootingCalc.distanceOffset.get().in(Meters);

		final var radialUnitVectorCartesian = new Translation3d(
			-shooterHubSpaceCartesian.getX() / radiusMeters,
			-shooterHubSpaceCartesian.getY() / radiusMeters,
			0.0
		);
		final var tangentialUnitVectorCartesian = new Translation3d(
			-radialUnitVectorCartesian.getY(),
			+radialUnitVectorCartesian.getX(),
			0.0
		);
		final var robotSpeedsVecMPS = new Translation3d(
			fieldSpeeds.vxMetersPerSecond,
			fieldSpeeds.vyMetersPerSecond,
			0.0
		);

		final var radialVelocityMPS = radialUnitVectorCartesian.dot(robotSpeedsVecMPS);
		final var tangentialVelocityMPS = tangentialUnitVectorCartesian.dot(robotSpeedsVecMPS);

		final var hoodAngleRads = ShooterConstants.hoodPolynomial.evaluate(radiusMeters, radialVelocityMPS);
		// Logger.recordOutput("DEBUG/PhysicsShooting/Before Hood Angle", hoodAngleRads);
		final var flywheelSpeedMPS = ShooterConstants.flywheelPolynomial.evaluate(radiusMeters, radialVelocityMPS);
		// Logger.recordOutput("DEBUG/PhysicsShooting/Before Flywheel Speed", flywheelSpeedMPS);
		this.tofSeconds = ShooterConstants.tofPolynomial.evaluate(radiusMeters, radialVelocityMPS);

		final var hubRobotSpaceCartesian = aimPoint.toTranslation2d().minus(robotPose.getTranslation());
		final var robotAngleRads = Math.atan2(hubRobotSpaceCartesian.getY(), hubRobotSpaceCartesian.getX());

		final var staticAimVectorMPS = Shooter.calculateLaunchVector(flywheelSpeedMPS, hoodAngleRads, robotAngleRads);
		final var radialVectorMPS = radialUnitVectorCartesian.times(radialVelocityMPS);
		final var offsetVectorMPS = tangentialUnitVectorCartesian.times(-tangentialVelocityMPS);
		final var launchVector = staticAimVectorMPS.plus(offsetVectorMPS);
		// Logger.recordOutput("DEBUG/PhysicsShooting/NewVector", new Translation3d(newVector[0], newVector[1], newVector[2]));
		// Logger.recordOutput("DEBUG/PhysicsShooting/staticVector", new Translation3d(staticAimVector[0], staticAimVector[1], staticAimVector[2]));
		// Logger.recordOutput("DEBUG/PhysicsShooting/radialVector", new Translation2d(radialVector[0], radialVector[1]));
		// Logger.recordOutput("DEBUG/PhysicsShooting/offsetVector", new Translation2d(offsetVector[0], offsetVector[1]));
		// Logger.recordOutput("DEBUG/PhysicsShooting/tangentialVeloUnitVec", new Translation2d(tangentialUnitVectorCartesian[0], tangentialUnitVectorCartesian[1]));

		final var exitVeloMPS = launchVector.getNorm();
		final var robotAngleVector = launchVector.plus(radialVectorMPS);

		this.flywheelSpeedMS = Shooter.calculateFlywheelSpeedMPS(exitVeloMPS);
		this.hoodAngleRads = Math.PI / 2.0 - Math.atan2(launchVector.getZ(), Math.hypot(launchVector.getX(), launchVector.getY()));
		this.robotRotationRads = Math.atan2(robotAngleVector.getY(), robotAngleVector.getX());
		// Logger.recordOutput("DEBUG/PhysicsShooting/HoodAngle", this.hoodAngleRads);
		// Logger.recordOutput("DEBUG/PhysicsShooting/FlywheelSpeed", this.flywheelSpeedMS);
		// this.robotRotationRads = robotAngleRads;
		// this.hoodAngleRads = hoodAngleRads;
		// this.flywheelSpeedMS = flywheelSpeedMPS;
		this.aimPoint = aimPoint;

		Logger.recordOutput("Subsystems/Shooter/Aiming/Effective Distance", radiusMeters, Meters);
	}

	@Override
	public double getTargetFlywheelSurfaceVeloMPS() {
		return this.flywheelSpeedMS;
	}

	@Override
	public double getTargetHoodAngleRads() {
		return this.hoodAngleRads;
	}

	@Override
	public double getTargetAzimuthHeadingRads() {
		return this.robotRotationRads;
	}

	@Override
	public Translation3d getAimPoint() {
		return this.aimPoint;
	}

	@Override
	public Translation2d getShotPose() {
		return this.shotPose;
	}

	@Override
	public double getTOFSeconds() {
		return this.tofSeconds;
	}
}
