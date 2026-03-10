package frc.robot.subsystems.shooter.aiming.shooting;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.util.math.MathExtraUtil;

public class PhysicsShootingCalc implements ShootingCalc {
	private double hoodAngleRads;
	private double flywheelSpeedMS;
	private double robotRotationRads;
	private Translation3d aimPoint;
	private double tofSeconds;

	private double[] radialUnitVectorCartesian = new double[] {0.0, 0.0};
	private double[] tangentialUnitVectorCartesian = new double[] {0.0, 0.0};
	private double[] robotSpeedsArray = new double[] {0.0, 0.0};

	private double[] shooterFieldSpaceCartesian = new double[] {0.0, 0.0, 0.0};
	private double[] shooterHubSpaceCartesian = new double[] {0.0, 0.0};
	private double[] hubRobotSpaceCartesian = new double[] {0.0, 0.0};

	private double[] staticAimVector = new double[] {0.0, 0.0, 0.0};
	private double[] radialVector = new double[] {0.0, 0.0};
	private double[] offsetVector = new double[] {0.0, 0.0};
	private double[] newLaunchVector = new double[] {0.0, 0.0, 0.0};
	private double[] launchValues = new double[] {0.0, 0.0, 0.0};

	@Override
	public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		MathExtraUtil.transformBy(shooterFieldSpaceCartesian, robotPose, HoodConstants.hoodBase);
		MathExtraUtil.putVector(shooterHubSpaceCartesian, shooterFieldSpaceCartesian[0] - aimPoint.getX(), shooterFieldSpaceCartesian[1] - aimPoint.getY());
		double radius = MathExtraUtil.norm(shooterHubSpaceCartesian);

		MathExtraUtil.putVector(radialUnitVectorCartesian, -shooterHubSpaceCartesian[0] / radius, -shooterHubSpaceCartesian[1] / radius);
		MathExtraUtil.putVector(tangentialUnitVectorCartesian, -radialUnitVectorCartesian[1], radialUnitVectorCartesian[0]);
		MathExtraUtil.putVector(robotSpeedsArray, fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

		double radialVelocity = MathExtraUtil.dotProduct(radialUnitVectorCartesian, robotSpeedsArray);
		double tangentialVelocity = MathExtraUtil.dotProduct(tangentialUnitVectorCartesian, robotSpeedsArray);

		var hoodAngleRads = ShooterConstants.hoodPolynomial.evaluate(radius, radialVelocity);
		var flywheelSpeedMPS = ShooterConstants.flywheelPolynomial.evaluate(radius, radialVelocity);
		var tofSeconds = ShooterConstants.tofPolynomial.evaluate(radius, radialVelocity);

		MathExtraUtil.putVector(hubRobotSpaceCartesian, aimPoint.getX() - robotPose.getX(), aimPoint.getY() - robotPose.getY());
		double robotAngleRads = Math.atan2(hubRobotSpaceCartesian[1], hubRobotSpaceCartesian[0]);

		getLaunchVector(staticAimVector, flywheelSpeedMPS, hoodAngleRads, robotAngleRads);
		MathExtraUtil.scalarMultiply(radialVector, radialUnitVectorCartesian, radialVelocity);
		MathExtraUtil.scalarMultiply(offsetVector, tangentialUnitVectorCartesian, -tangentialVelocity);
		MathExtraUtil.addVectors(newLaunchVector, staticAimVector, offsetVector);
		getLaunchValues(launchValues, newLaunchVector, radialVector);

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

	private static void getLaunchVector(double[] out, double flywheelSpeedMPS, double hoodAngleRads, double robotAngleRads) {
		double exitVelo = getExitVelo(flywheelSpeedMPS, getFVelo());
		double x = exitVelo * Math.cos(Math.PI / 2.0 - hoodAngleRads) * Math.cos(robotAngleRads);
		double y = exitVelo * Math.cos(Math.PI / 2.0 - hoodAngleRads) * Math.sin(robotAngleRads);
		double z = exitVelo * Math.sin(Math.PI / 2.0 - hoodAngleRads);
		out[0] = x;
		out[1] = y;
		out[2] = z;
	}

	private static void getLaunchValues(double[] out, double[] launchVector, double[] radialVelo) {
		double exitVelo = Math.sqrt(Math.pow(launchVector[0], 2) + Math.pow(launchVector[1], 2) + Math.pow(launchVector[2], 2));
		var robotAngleVector = MathExtraUtil.addVectors(launchVector, radialVelo);
		double robotAngleRads = Math.atan2(robotAngleVector[1], robotAngleVector[0]);
		double hoodAngleRads = Math.PI / 2.0 - Math.atan2(launchVector[2], Math.sqrt(Math.pow(launchVector[0], 2) + Math.pow(launchVector[1], 2)));
		out[0] = getFlywheelSpeedMPS(exitVelo);
		out[1] = hoodAngleRads;
		out[2] = robotAngleRads;
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
