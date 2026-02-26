package frc.robot.subsystems.shooter.aiming.shooting;

import static edu.wpi.first.units.Units.Seconds;


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
	private double tofSeconds;

	private static final LoggedTunable<Time> tangentLookahead = LoggedTunable.from("Shooter/Aiming/Shooting/Physics/Tangential Lookahead", Seconds::of, RobotConstants.rioUpdatePeriodSecs);
;
	@Override
	public void calculate(Translation2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		var robotHubSpaceCartesian = robotPose.minus(aimPoint.toTranslation2d());
		double radius = Math.sqrt(Math.pow(robotHubSpaceCartesian.getX(), 2) + Math.pow(robotHubSpaceCartesian.getY(), 2));

		var radialUnitVectorCartesian = new double[] {-robotHubSpaceCartesian.getX() / radius, -robotHubSpaceCartesian.getY() / radius};
		var tangentialUnitVectorCartesian = new double[] {-radialUnitVectorCartesian[1], radialUnitVectorCartesian[0]};
		var robotSpeedsArray = new double[] { fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond };

		double radialVelocity = MathExtraUtil.dotProduct(radialUnitVectorCartesian, robotSpeedsArray);
		double tangentialVelocity = MathExtraUtil.dotProduct(tangentialUnitVectorCartesian, robotSpeedsArray);

		var hoodAngleRads = ShooterConstants.hoodPolynomial.evaluate(radius, radialVelocity);
		var flywheelSpeedMPS = ShooterConstants.flywheelPolynomial.evaluate(radius, radialVelocity);
		var tofSeconds = ShooterConstants.tofPolynomial.evaluate(radius, radialVelocity);

		double angleOffsetRads = Math.atan2(-tangentialVelocity * tangentLookahead.get().in(Seconds), radius);

		double robotAngleRads = Math.atan2(robotHubSpaceCartesian.getY(), robotHubSpaceCartesian.getX());

		this.robotRotationRads = angleOffsetRads + robotAngleRads + Math.PI;
		this.hoodAngleRads = hoodAngleRads;
		this.flywheelSpeedMS = flywheelSpeedMPS;
		this.aimPoint = aimPoint;
		this.tofSeconds = tofSeconds;
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
	public double getTOFSeconds() {
		return this.tofSeconds;
	}
}
