package tasks.physicsSim;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class PhysicsSimulation {
	public static void main(String[] args) throws InterruptedException, IOException {
		System.out.println(System.getProperty("os.name"));
		boolean isWindows = System.getProperty("os.name").toLowerCase().contains("win");

		if (isWindows) {
			Path exePath = Paths.get("./tools/Headless.exe").toAbsolutePath();

			ProcessBuilder pb = new ProcessBuilder(
				exePath.toString(),
				"--outputdir",
				"../src/main/deploy/"
			);

			pb.directory(new File("tools"));
			pb.inheritIO();

			Process process = pb.start();
			int exitCode = process.waitFor();

			if (exitCode != 0) {
				throw new RuntimeException("Physics tool failed with exit code " + exitCode);
			}

			System.out.println("Physics tool completed successfully.");
		}
	}
}
