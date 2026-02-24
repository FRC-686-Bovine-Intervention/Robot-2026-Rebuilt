package frc.robot.subsystems.shooter.aiming.shooting;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.math.MathExtraUtil;

public class PhysicsShootingCalc implements ShootingCalc {
	private double hoodAngleRads;
	private double flywheelSpeedMS;
	private double robotRotationRads;
	private Translation3d aimPoint;

	private static final LoggedTunable<Time> tangentLookahead = LoggedTunable.from("Shooter/Aiming/Shooting/Physics/Tangential Lookahead", Seconds::of, RobotConstants.rioUpdatePeriodSecs);
;
	@Override
	public void calculate(Translation2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		var robotHubSpaceCartesian = robotPose.minus(aimPoint.toTranslation2d());
		double radius = Math.sqrt(Math.pow(robotHubSpaceCartesian.getX(), 2) + Math.pow(robotHubSpaceCartesian.getY(), 2));
		Logger.recordOutput("DEBUG/PhysicsShootingCalc/RadiusMeters", radius);

		var radialUnitVectorCartesian = new double[] {-robotHubSpaceCartesian.getX() / radius, -robotHubSpaceCartesian.getY() / radius};
		var tangentialUnitVectorCartesian = new double[] {-radialUnitVectorCartesian[1], radialUnitVectorCartesian[0]};
		var robotSpeedsArray = new double[] { fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond };

		double radialVelocity = MathExtraUtil.dotProduct(radialUnitVectorCartesian, robotSpeedsArray);
		Logger.recordOutput("DEBUG/PhysicsShootingCalc/RadialVelocityMPS", radialVelocity);
		double tangentialVelocity = MathExtraUtil.dotProduct(tangentialUnitVectorCartesian, robotSpeedsArray);
		Logger.recordOutput("DEBUG/PhysicsShootingCalc/TangentialVelocityMPS", tangentialVelocity);

		var hoodAngleRads = ShooterConstants.hoodPolynomial.evaluate(radius, radialVelocity);
		Logger.recordOutput("DEBUG/PhysicsShootingCalc/HoodAngleRads", hoodAngleRads);
		var flywheelSpeedMPS = ShooterConstants.flywheelPolynomial.evaluate(radius, radialVelocity);
		Logger.recordOutput("DEBUG/PhysicsShootingCalc/flywheelSpeedMPS", flywheelSpeedMPS);

		double angleOffsetRads = Math.atan2(-tangentialVelocity * tangentLookahead.get().in(Seconds), radius); //NEED A DOUBLE-CHECK ON THAT
		Logger.recordOutput("DEBUG/PhysicsShootingCalc/AngleOffsetRads", angleOffsetRads);

		double robotAngleRads = Math.atan2(robotHubSpaceCartesian.getY(), robotHubSpaceCartesian.getX());

		this.robotRotationRads = angleOffsetRads + robotAngleRads + Math.PI;
		this.hoodAngleRads = hoodAngleRads;
		this.flywheelSpeedMS = flywheelSpeedMPS;
		this.aimPoint = aimPoint;
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
}
