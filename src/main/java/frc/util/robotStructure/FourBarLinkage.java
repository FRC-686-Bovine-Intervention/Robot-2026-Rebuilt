package frc.util.robotStructure;

public class FourBarLinkage {
	private final double frameLength;
	private final double driverLength;
	private final double followerLength;
	private final double couplerLength;
	private final boolean useNegative;

	private double driverAngleRads = 0.0;
	private double followerAngleRads = 0.0;
	private double couplerAngleRads = 0.0;

	public FourBarLinkage(
		double frameLength,
		double driverLength,
		double followerLength,
		double couplerLength,
		boolean driverOnTop,
		boolean useNegative
	) {
		this.frameLength = (driverOnTop ? -1.0 : 1.0) *  frameLength;
		this.driverLength = driverLength;
		this.followerLength = followerLength;
		this.couplerLength = couplerLength;
		this.useNegative = useNegative ^ driverOnTop;
	}

	public void setDriverAngleRads(double driverRads) {
		this.driverAngleRads = driverRads;

		var driverX = Math.cos(this.driverAngleRads) * this.driverLength;
		var driverY = Math.sin(this.driverAngleRads) * this.driverLength;

		var frameToDriverDistance = Math.hypot(driverX, driverY - this.frameLength);

		var f = (2.0 / frameToDriverDistance) * Math.sqrt(
			0.0625
			* (+frameToDriverDistance + this.followerLength + this.couplerLength)
			* (-frameToDriverDistance + this.followerLength + this.couplerLength)
			* (+frameToDriverDistance - this.followerLength + this.couplerLength)
			* (+frameToDriverDistance + this.followerLength - this.couplerLength)
		);

		var asin = (this.useNegative ? -1.0 : 1.0) *  Math.asin(f / this.followerLength);

		var atan = Math.atan2(driverY - this.frameLength, driverX);

		this.followerAngleRads = asin + atan;

		var followerX = Math.cos(this.followerAngleRads) * this.followerLength;
		var followerY = Math.sin(this.followerAngleRads) * this.followerLength + this.frameLength;

		this.couplerAngleRads = Math.atan2(followerY - driverY, followerX - driverX);
	}

	public double getDriverAngleRads() {
		return this.driverAngleRads;
	}
	public double getFollowerAngleRads() {
		return this.followerAngleRads;
	}
	public double getCouplerAngleRads() {
		return this.couplerAngleRads;
	}
}
