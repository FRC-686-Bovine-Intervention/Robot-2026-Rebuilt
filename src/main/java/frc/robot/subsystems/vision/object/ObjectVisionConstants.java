package frc.robot.subsystems.vision.object;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;

public class ObjectVisionConstants {
    public static class TrackableObject {
        public final Distance height;
        public final int classId;
        public final String className;

        private TrackableObject(Distance height, int classId, String className) {
            this.height = height;
            this.classId = classId;
            this.className = className;
        }
    }

    public static final TrackableObject[] trackableObjects = {
        new TrackableObject(FieldConstants.fuelHeight, 0, "Fuel")
    };
}
