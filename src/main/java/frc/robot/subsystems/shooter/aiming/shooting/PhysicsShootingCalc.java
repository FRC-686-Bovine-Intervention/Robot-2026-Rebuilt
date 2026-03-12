package frc.robot.subsystems.shooter.aiming.shooting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.util.math.MathExtraUtil;

public class PhysicsShootingCalc implements ShootingCalc {
	private double hoodAngleRads;
	private double flywheelSpeedMS;
	private double robotRotationRads;
	private Translation3d aimPoint;
	private double tofSeconds;

	public PhysicsShootingCalc() {
		// Preload polynomials
		ShooterConstants.flywheelPolynomial.evaluate(0.0, 0.0);
		ShooterConstants.hoodPolynomial.evaluate(0.0, 0.0);
	}

	@Override
	public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
		var shooterHubSpaceCartesian = new Pose3d(robotPose).transformBy(HoodConstants.hoodBase).getTranslation().toTranslation2d().minus(aimPoint.toTranslation2d());
		double radius = shooterHubSpaceCartesian.getNorm();

		var radialUnitVectorCartesian = new double[] {-shooterHubSpaceCartesian.getX() / radius, -shooterHubSpaceCartesian.getY() / radius};
		var tangentialUnitVectorCartesian = new double[] {-radialUnitVectorCartesian[1], radialUnitVectorCartesian[0]};
		var robotSpeedsArray = new double[] { fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond };

		double radialVelocity = MathExtraUtil.dotProduct(radialUnitVectorCartesian, robotSpeedsArray);
		double tangentialVelocity = MathExtraUtil.dotProduct(tangentialUnitVectorCartesian, robotSpeedsArray);

		var hoodAngleRads = ShooterConstants.hoodPolynomial.evaluate(radius, radialVelocity);
		var flywheelSpeedMPS = ShooterConstants.flywheelPolynomial.evaluate(radius, radialVelocity);
		var tofSeconds = ShooterConstants.tofPolynomial.evaluate(radius, radialVelocity);

		var hubRobotSpaceCartesian = aimPoint.toTranslation2d().minus(robotPose.getTranslation());
		double robotAngleRads = Math.atan2(hubRobotSpaceCartesian.getY(), hubRobotSpaceCartesian.getX());

		var staticAimVector = Shooter.getLaunchVector(flywheelSpeedMPS, hoodAngleRads, robotAngleRads);
		var radialVector = MathExtraUtil.scalarMultiply(radialUnitVectorCartesian, radialVelocity);
		var offsetVector = MathExtraUtil.scalarMultiply(tangentialUnitVectorCartesian, -tangentialVelocity);
		var newVector = MathExtraUtil.addVectors(staticAimVector, offsetVector);
		Logger.recordOutput("DEBUG/PhysicsShooting/NewVector", new Translation3d(newVector[0], newVector[1], newVector[2]));
		Logger.recordOutput("DEBUG/PhysicsShooting/staticVector", new Translation3d(staticAimVector[0], staticAimVector[1], staticAimVector[2]));
		Logger.recordOutput("DEBUG/PhysicsShooting/radialVector", new Translation2d(radialVector[0], radialVector[1]));
		Logger.recordOutput("DEBUG/PhysicsShooting/offsetVector", new Translation2d(offsetVector[0], offsetVector[1]));
		Logger.recordOutput("DEBUG/PhysicsShooting/tangentialVeloUnitVec", new Translation2d(tangentialUnitVectorCartesian[0], tangentialUnitVectorCartesian[1]));
		var launchValues = getLaunchValues(newVector, radialVector);

		this.robotRotationRads = launchValues[2];
		this.hoodAngleRads = launchValues[1];
		Logger.recordOutput("DEBUG/PhysicsShooting/HoodAngle", this.hoodAngleRads);
		this.flywheelSpeedMS = launchValues[0];
		Logger.recordOutput("DEBUG/PhysicsShooting/FlywheelSpeed", this.flywheelSpeedMS);
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

	private static double[] getLaunchValues(double[] launchVector, double[] radialVelo) {
		double exitVelo = Math.sqrt(Math.pow(launchVector[0], 2) + Math.pow(launchVector[1], 2) + Math.pow(launchVector[2], 2));
		var robotAngleVector = MathExtraUtil.addVectors(launchVector, radialVelo);
		double robotAngleRads = Math.atan2(robotAngleVector[1], robotAngleVector[0]);
		double hoodAngleRads = Math.PI / 2.0 - Math.atan2(launchVector[2], Math.sqrt(Math.pow(launchVector[0], 2) + Math.pow(launchVector[1], 2)));
		return new double[] {Shooter.getFlywheelSpeedMPS(exitVelo), hoodAngleRads, robotAngleRads};
		// return new double[] {exitVelo, hoodAngleRads, robotAngleRads};
	}
}
