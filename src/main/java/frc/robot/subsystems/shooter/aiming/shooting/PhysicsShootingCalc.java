package frc.robot.subsystems.shooter.aiming.shooting;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.util.math.MathExtraUtil;

public class PhysicsShootingCalc implements ShootingCalc {
	private double hoodAngleRads;
	private double flywheelSpeedMS;
	private double robotRotationRads;
	private Translation3d aimPoint;
	private double tofSeconds;

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

		double robotAngleRads = Math.atan2(robotHubSpaceCartesian.getY(), robotHubSpaceCartesian.getX());

		var staticAimVector = getLaunchVector(flywheelSpeedMPS, hoodAngleRads, robotAngleRads);
		var newVector = MathExtraUtil.addVectors(staticAimVector, MathExtraUtil.scalarMultiply(tangentialUnitVectorCartesian, -tangentialVelocity));
		var launchValues = getLaunchValues(newVector);

		this.robotRotationRads = launchValues[2];
		this.hoodAngleRads = launchValues[1];
		this.flywheelSpeedMS = launchValues[0];
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

	private static double[] getLaunchVector(double flywheelSpeedMPS, double hoodAngleRads, double robotAngleRads) {
		double exitVelo = getExitVelo(flywheelSpeedMPS, getFVelo());
		double x = exitVelo * Math.cos(hoodAngleRads + Math.PI/2) * Math.cos(robotAngleRads);
		double y = exitVelo * Math.cos(hoodAngleRads + Math.PI/2) * Math.sin(robotAngleRads);
		double z = exitVelo * Math.sin(hoodAngleRads + Math.PI/2);
		return new double[] {x, y, z};
	}

	private static double[] getLaunchValues(double[] launchVector) {
		double exitVelo = Math.sqrt(Math.pow(launchVector[0], 2) + Math.pow(launchVector[1], 2) + Math.pow(launchVector[2], 2));
		double robotAngleRads = Math.atan2(launchVector[1], launchVector[0]);
		double hoodAngleRads = Math.atan2(launchVector[2], Math.sqrt(Math.pow(launchVector[0], 2) + Math.pow(launchVector[1], 2))) - Math.PI/2;
		return new double[] {getFlywheelSpeedMPS(exitVelo), hoodAngleRads, robotAngleRads};
	}

	private static double getExitVelo(double flywheelSpeedMPS, double fVelo) {
		return (flywheelSpeedMPS  + flywheelSpeedMPS * fVelo)/2;
	}

	private static double getFlywheelSpeedMPS(double exitVelo) {
		return (2 * exitVelo) / (1 + getFVelo());
	}

	private static double getFVelo() {
		return (FlywheelConstants.flywheelToHood.reductionUnsigned() / FlywheelConstants.wheel.effectiveRadius().in(Meters)) * FlywheelConstants.hoodRoller.effectiveRadius().in(Meters);
	}
}
