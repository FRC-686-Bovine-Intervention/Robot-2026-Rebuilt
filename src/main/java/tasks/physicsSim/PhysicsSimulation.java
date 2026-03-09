package tasks.physicsSim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.google.gson.Gson;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;

public class PhysicsSimulation {
	private static final boolean SHOULD_SIM_DURING_BUILD = false;
	public static void main(String[] args) throws InterruptedException, IOException {
		boolean isWindows = System.getProperty("os.name").toLowerCase().contains("win");
		boolean isLinux = System.getProperty("os.name").toLowerCase().contains("lin");

		if (SHOULD_SIM_DURING_BUILD) {
			// ShooterConfig oldConfig = fromFile("./tools/physicsSim/shooter.json");

			ShooterConfig config = new ShooterConfig(
				HoodConstants.hoodBase.getZ(),
				FlywheelConstants.wheel.effectiveRadius().in(Meters),
				FlywheelConstants.hoodRoller.effectiveRadius().in(Meters),
				HoodConstants.hoodRadius.in(Meters),
				(FlywheelConstants.flywheelToHood.reductionUnsigned() / FlywheelConstants.wheel.effectiveRadius().in(Meters)) * FlywheelConstants.hoodRoller.effectiveRadius().in(Meters),
				FlywheelConstants.wheel.rotationsToMeters(FlywheelConstants.motorToMechanism.reductionSigned() * 6000) / 60,
				0.0,
				30,
				HoodConstants.minAngle.in(Degrees),
				HoodConstants.maxAngle.in(Degrees),
				20,
				-DriveConstants.maxDriveSpeed.in(MetersPerSecond),
				DriveConstants.maxDriveSpeed.in(MetersPerSecond),
				20,
				2,
				10.0,
				30,
				0.5,
				0.5,
				1.0,
				150.0
			);

			saveShooterConfig(config);
		}
		if (isWindows && SHOULD_SIM_DURING_BUILD) {
			Path exePath = Paths.get("./tools/physicsSim/windows/BBP-Headless.exe").toAbsolutePath();

			ProcessBuilder pb = new ProcessBuilder(
				exePath.toString(),
				"--inputpath",
				"../shooter.json",
				"--outputdir",
				"../../../src/main/deploy/"
			);

			pb.directory(new File("tools/physicsSim/windows"));
			pb.inheritIO();

			Process process = pb.start();
			int exitCode = process.waitFor();

			if (exitCode != 0) {
				throw new RuntimeException("Physics tool failed with exit code " + exitCode);
			}

			System.out.println("Physics tool completed successfully.");
		}
		if (isLinux && SHOULD_SIM_DURING_BUILD) {
			Path exePath = Paths.get("./tools/physicsSim/linux/BBP-Headless").toAbsolutePath();

			ProcessBuilder pb = new ProcessBuilder(
				exePath.toString(),
				"--inputpath",
				"../shooter.json",
				"--outputdir",
				"../../../src/main/deploy/"
			);

			pb.directory(new File("tools/physicsSim/linux"));
			pb.inheritIO();

			Process process = pb.start();
			int exitCode = process.waitFor();

			if (exitCode != 0) {
				throw new RuntimeException("Physics tool failed with exit code " + exitCode);
			}

			System.out.println("Physics tool completed successfully.");
		}
	}

	private static ShooterConfig fromFile(String jsonPath) {
		Gson parser = new Gson();

		try {
			System.out.println("Loading polynomial from " + jsonPath);
			return parser.fromJson(new FileReader(jsonPath), ShooterConfig.class);
		} catch (JsonSyntaxException e) {
			System.out.println("Failed to load polynomial from " + jsonPath);
			e.printStackTrace();
		} catch (JsonIOException e) {
			System.out.println("Failed to load polynomial from " + jsonPath);
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			System.out.println("Failed to load polynomial from " + jsonPath);
			e.printStackTrace();
		}
		return null;
	}

	private static void saveShooterConfig(ShooterConfig config) throws JsonIOException, IOException {
		Gson parser = new Gson();
		String json = parser.toJson(config);
		FileWriter writer = new FileWriter("./tools/physicsSim/shooter.json");
		writer.write(json);
		writer.flush();
		writer.close();
	}


	public static class ShooterConfig {
		private final double shooterHeight;
		private final double rFly;
		private final double rRol;
		private final double rHood;
		private final double fVelo;
		private final double maxVFly;
		private final double minVFly;
		private final int vFlyMaxTries;
		private final double minAngle;
		private final double maxAngle;
		private final int angleRes;
		private final double minVX;
		private final double maxVX;
		private final int vxRes;
		private final double minX;
		private final double maxX;
		private final int xRes;
		private final double angleDev;
		private final double vFlyDev;
		private final double robustnessFactor;
		private final double heightFactor;

		public ShooterConfig(double shooterHeight, double rFly, double rRol, double rHood, double fVelo, double maxVFly, double minVFly, int vFlyMaxTries, double minAngle, double maxAngle, int angleRes, double minVX, double maxVX, int vxRes, double minX, double maxX, int xRes, double angleDev, double vFlyDev, double robustnessFactor, double heightFactor) {
			this.shooterHeight = shooterHeight;
			this.rFly = rFly;
			this.rRol = rRol;
			this.rHood = rHood;
			this.fVelo = fVelo;
			this.maxVFly = maxVFly;
			this.minVFly = minVFly;
			this.vFlyMaxTries = vFlyMaxTries;
			this.minAngle = minAngle;
			this.maxAngle = maxAngle;
			this.angleRes = angleRes;
			this.minVX = minVX;
			this.maxVX = maxVX;
			this.vxRes = vxRes;
			this.minX = minX;
			this.maxX = maxX;
			this.xRes = xRes;
			this.angleDev = angleDev;
			this.vFlyDev = vFlyDev;
			this.robustnessFactor = robustnessFactor;
			this.heightFactor = heightFactor;
		}
	}
}
