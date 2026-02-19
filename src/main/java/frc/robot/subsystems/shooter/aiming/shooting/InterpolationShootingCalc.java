package frc.robot.subsystems.shooter.aiming.shooting;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class InterpolationShootingCalc implements ShootingCalc {

	private static final LoggedTunable<Time> lookaheadTime = LoggedTunable.from("Shooter/Aiming/Lookahead Seconds", Seconds::of, 0.035);
	private static final LoggedTunable<Distance> azimuthTolerance = LoggedTunable.from("Shooter/Aiming/Tolerance/Azimuth", Centimeters::of, 100);
	private static final LoggedTunable<Distance> altitudeDegsTolerance = LoggedTunable.from("Shooter/Aiming/Tolerance/Altitude", Centimeters::of, 46);
	private static final LoggedTunable<Angle> customAzimuthOffset = LoggedTunable.from("Shooter/Aiming/Custom Azimuth Offset", Radians::of, 0.0);

	private Translation3d aimPoint;
	// private Pose2d shotPose;
	// private ChassisSpeeds shotSpeeds;
	private double effectiveDistanceMeters;
	private double targetHoodAngleRads;
	private double targetFlywheelVeloMPS;
	private double targetDriveHeadingRads;

	@Override
	public void calculate(Translation2d robotPos, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		this.aimPoint = aimPoint;

		var predictedX = robotPos.getX() + fieldSpeeds.vxMetersPerSecond * lookaheadTime.get().in(Seconds);
		var predictedY = robotPos.getY() + fieldSpeeds.vyMetersPerSecond * lookaheadTime.get().in(Seconds);

		var predictedToTargetX = this.aimPoint.getX() - predictedX;
		var predictedToTargetY = this.aimPoint.getY() - predictedY;

		this.targetDriveHeadingRads = Math.atan2(predictedToTargetY, predictedToTargetX);
		this.effectiveDistanceMeters = Math.hypot(predictedToTargetX, predictedToTargetY);
		Logger.recordOutput("Shooter/Aiming/Effective Distance", this.effectiveDistanceMeters);

		this.targetHoodAngleRads = ShooterConstants.hubTargetHoodAngleRads.get(this.effectiveDistanceMeters);
		this.targetFlywheelVeloMPS = ShooterConstants.hubTargetFlyWheelVeloMPS.get(this.effectiveDistanceMeters);

		Logger.recordOutput("Shooter/Aiming/Aim Point", this.aimPoint);
		Logger.recordOutput("Shooter/Aiming/Shot Pose", new Pose2d(
			new Translation2d(
				predictedX,
				predictedY
			),
			Rotation2d.fromRadians(
				this.targetDriveHeadingRads
			)
		));
	}

	@Override
	public double getTargetFlywheelSurfaceVeloMPS() {
		return this.targetFlywheelVeloMPS;
	}

	@Override
	public double getTargetHoodAngleRads() {
		return this.targetHoodAngleRads;
	}

	@Override
	public double getTargetAzimuthHeadingRads() {
		return this.targetDriveHeadingRads;
	}

	@Override
	public Translation3d getAimPoint() {
		return this.aimPoint;
	}

}
