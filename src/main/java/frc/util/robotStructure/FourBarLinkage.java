package frc.util.robotStructure;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FourBarLinkage {
	private final double driverLength;
	private final double followerLength;
	private final double couplerLength;
	private final boolean useNegative;

	private final double baseFollowerJointX;
	private final double baseFollowerJointY;

	private double driverCouplerJointX;
	private double driverCouplerJointY;
	private double followerCouplerJointX;
	private double followerCouplerJointY;

	public FourBarLinkage(
		Translation2d baseDriverJoint,
		Translation2d baseFollowerJoint,
		Distance driverLength,
		Distance followerLength,
		Distance couplerLength,
		boolean useNegative
	) {
		this.baseFollowerJointX = baseFollowerJoint.getX() - baseDriverJoint.getX();
		this.baseFollowerJointY = baseFollowerJoint.getY() - baseDriverJoint.getY();

		this.driverLength = driverLength.in(Meters);
		this.followerLength = followerLength.in(Meters);
		this.couplerLength = couplerLength.in(Meters);

		this.useNegative = useNegative;
	}

	public FourBarLinkage(
		Translation2d baseDriverJoint,
		Translation2d baseFollowerJoint,
		Translation2d driverCouplerJoint,
		Translation2d followerCouplerJoint,
		boolean useNegative
	) {
		this(
			baseDriverJoint,
			baseFollowerJoint,
			Meters.of(baseDriverJoint.getDistance(driverCouplerJoint)),
			Meters.of(baseFollowerJoint.getDistance(followerCouplerJoint)),
			Meters.of(driverCouplerJoint.getDistance(followerCouplerJoint)),
			useNegative
		);

		this.driverCouplerJointX = driverCouplerJoint.getX() - baseDriverJoint.getX();
		this.driverCouplerJointY = driverCouplerJoint.getY() - baseDriverJoint.getY();
		this.followerCouplerJointX = followerCouplerJoint.getX() - baseDriverJoint.getX();
		this.followerCouplerJointY = followerCouplerJoint.getY() - baseDriverJoint.getY();
	}

	public void setDriverAngleRads(double driverRads) {
		this.driverCouplerJointX = Math.cos(driverRads) * this.driverLength;
		this.driverCouplerJointY = Math.sin(driverRads) * this.driverLength;

		var baseFollowerJointToDriverCouplerJointDistance = Math.hypot(
			this.driverCouplerJointX - this.baseFollowerJointX,
			this.driverCouplerJointY - this.baseFollowerJointY
		);

		var f = (2.0 / baseFollowerJointToDriverCouplerJointDistance) * Math.sqrt(
			0.0625
			* (+baseFollowerJointToDriverCouplerJointDistance + this.followerLength + this.couplerLength)
			* (-baseFollowerJointToDriverCouplerJointDistance + this.followerLength + this.couplerLength)
			* (+baseFollowerJointToDriverCouplerJointDistance - this.followerLength + this.couplerLength)
			* (+baseFollowerJointToDriverCouplerJointDistance + this.followerLength - this.couplerLength)
		);

		var asin = (this.useNegative ? -1.0 : 1.0) *  Math.asin(f / this.followerLength);

		var atan = Math.atan2(
			this.driverCouplerJointY - this.baseFollowerJointY,
			this.driverCouplerJointX - this.baseFollowerJointX
		);

		var followerAngleRads = asin + atan;

		this.followerCouplerJointX = Math.cos(followerAngleRads) * this.followerLength + this.baseFollowerJointX;
		this.followerCouplerJointY = Math.sin(followerAngleRads) * this.followerLength + this.baseFollowerJointY;


		Logger.recordOutput("DEBUG/FOURBAR/box", new Translation2d(), new Translation2d(this.driverCouplerJointX, this.driverCouplerJointY), new Translation2d(this.followerCouplerJointX, this.followerCouplerJointY), new Translation2d(this.baseFollowerJointX, this.baseFollowerJointY), new Translation2d());
	}

	public double getHorizonBaseDriverCouplerAngleRads() {
		return Math.atan2(
			this.driverCouplerJointY,
			this.driverCouplerJointX
		);
	}

	public double getHorizonBaseFollowerCouplerAngleRads() {
		return Math.atan2(
			this.followerCouplerJointY - this.baseFollowerJointY,
			this.followerCouplerJointX - this.baseFollowerJointX
		);
	}

	/**
	 * Use this to get the angle for a coupler arm mech if it is attached to the driver arm mech
	 */
	public double getDriverRelativeCouplerAngleRads() {
		// Complex multiply vectors to rotate, the scaling effect doesn't matter because angle is unaffected
		// Implicitly negate the driverCouplerJoint's angle to achieve angle subtraction
		var driverCouplerJointRelFollowerCouplerJointX = this.followerCouplerJointX - this.driverCouplerJointX;
		var driverCouplerJointRelFollowerCouplerJointY = this.followerCouplerJointY - this.driverCouplerJointY;
		var driverRelFollowerCouplerJointX = +driverCouplerJointRelFollowerCouplerJointX * this.driverCouplerJointX + driverCouplerJointRelFollowerCouplerJointY * this.driverCouplerJointY;
		var driverRelFollowerCouplerJointY = -driverCouplerJointRelFollowerCouplerJointX * this.driverCouplerJointY + driverCouplerJointRelFollowerCouplerJointY * this.driverCouplerJointX;
		return Math.atan2(
			driverRelFollowerCouplerJointY,
			driverRelFollowerCouplerJointX
		);
	}
}
