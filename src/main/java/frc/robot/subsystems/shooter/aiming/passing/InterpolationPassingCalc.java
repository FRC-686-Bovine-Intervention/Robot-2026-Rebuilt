package frc.robot.subsystems.shooter.aiming.passing;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class InterpolationPassingCalc implements PassingCalc {
	private static final LoggedTunable<Time> lookaheadTime = LoggedTunable.from("Subsystems/Shooter/Passing/Lookahead Seconds", Seconds::of, 1.0);
	// private static final LoggedTunable<Distance> azimuthTolerance = LoggedTunable.from("Subsystems/Shooter/Aiming/Tolerance/Azimuth", Centimeters::of, 100);
	// private static final LoggedTunable<Distance> altitudeDegsTolerance = LoggedTunable.from("Subsystems/Shooter/Aiming/Tolerance/Altitude", Centimeters::of, 46);
	// private static final LoggedTunable<Angle> customAzimuthOffset = LoggedTunable.from("Subsystems/Shooter/Aiming/Custom Azimuth Offset", Radians::of, 0.0);

	private Translation3d aimPoint = Translation3d.kZero;
	private Translation2d shotPose = Translation2d.kZero;
	// private ChassisSpeeds shotSpeeds;
	private double effectiveDistanceMeters;
	private double targetHoodAngleRads;
	private double targetFlywheelVeloMPS;
	private double targetDriveHeadingRads;

	public InterpolationPassingCalc() {
		ShooterConstants.passFlyWheelVeloMPS.get(0.0);
	}

	@Override
	public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		this.shotPose = robotPose.getTranslation();
		this.aimPoint = aimPoint;

		var predictedX = this.shotPose.getX() + fieldSpeeds.vxMetersPerSecond * lookaheadTime.get().in(Seconds);
		var predictedY = this.shotPose.getY() + fieldSpeeds.vyMetersPerSecond * lookaheadTime.get().in(Seconds);

		var predictedToTargetX = this.aimPoint.getX() - predictedX;
		var predictedToTargetY = this.aimPoint.getY() - predictedY;

		this.targetDriveHeadingRads = Math.atan2(predictedToTargetY, predictedToTargetX);
		this.effectiveDistanceMeters = Math.hypot(predictedToTargetX, predictedToTargetY);
		Logger.recordOutput("Subsystems/Shooter/Aiming/Effective Distance", this.effectiveDistanceMeters);

		this.targetHoodAngleRads = ShooterConstants.passHoodAngleRads.get(this.effectiveDistanceMeters);
		this.targetFlywheelVeloMPS = ShooterConstants.passFlyWheelVeloMPS.get(this.effectiveDistanceMeters);

		Logger.recordOutput("Subsystems/Shooter/Aiming/Aim Point", this.aimPoint);
		Logger.recordOutput("Subsystems/Shooter/Aiming/Shot Pose", new Pose2d(
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

	@Override
	public Translation2d getShotPose() {
		return this.shotPose;
	}
}
