package frc.robot.subsystems.shooter.aiming.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.math.MathExtraUtil;

public class PhysicsShootingCalc implements ShootingCalc {
	private double hoodAngleRads;
	private double flywheelSpeedMS;
	private double robotRotationRads;
	private Translation3d aimPoint;

	@Override
	public void calculate(Translation2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		var robotHubSpaceCartesian = robotPose.minus(aimPoint.toTranslation2d());
		double radius = Math.sqrt(Math.pow(robotHubSpaceCartesian.getX(), 2) + Math.pow(robotHubSpaceCartesian.getY(), 2));

		var radialUnitVectorCartesian = new double[] {-robotHubSpaceCartesian.getX() / radius, -robotHubSpaceCartesian.getY() / radius};
		var tangentialUnitVectorCartesian = new double[] {-radialUnitVectorCartesian[1], radialUnitVectorCartesian[0]};
		var robotSpeedsArray = new double[] { fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond };

		double radialVelocity = MathExtraUtil.dotProduct(radialUnitVectorCartesian, robotSpeedsArray);
		double tangentialVelocity = MathExtraUtil.dotProduct(tangentialUnitVectorCartesian, robotSpeedsArray);

		var vals = ShooterConstants.aimingPolynomial.evaluate(new double[] {radius, radialVelocity});
		double angleOffsetRads = Math.atan2(tangentialVelocity, -Math.abs(radialVelocity)); //NEED A DOUBLE-CHECK ON THAT

		double robotAngleRads = Math.atan2(robotHubSpaceCartesian.getY(), robotHubSpaceCartesian.getX());

		this.robotRotationRads = angleOffsetRads + robotAngleRads;
		this.hoodAngleRads = vals[0];
		this.flywheelSpeedMS = vals[1];
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
