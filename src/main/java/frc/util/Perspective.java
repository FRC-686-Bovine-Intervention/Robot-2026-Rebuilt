package frc.util;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.util.flipping.AllianceFlipped;

public class Perspective {
	protected Rotation2d forwardDirection;
	protected Matrix<N2, N2> perspectiveToField;
	protected Matrix<N2, N2> fieldToPerspective;

	private Perspective(Rotation2d forwardDirection) {
		this.forwardDirection = forwardDirection;
		this.perspectiveToField = this.forwardDirection.toMatrix();
		this.fieldToPerspective = this.perspectiveToField.inv();
	}

	public Rotation2d getForwardDirection() {
		return this.forwardDirection;
	}

	public Matrix<N2, N2> getPerspectiveToField() {
		return this.perspectiveToField;
	}

	public Matrix<N2, N2> getFieldToPerspective() {
		return this.fieldToPerspective;
	}

	private static final Perspective posX = new Perspective(Rotation2d.kZero);
	private static final Perspective negX = new Perspective(Rotation2d.k180deg);
	private static final Perspective posY = new Perspective(Rotation2d.kCCW_90deg);
	private static final Perspective negY = new Perspective(Rotation2d.kCW_90deg);
	private static final Perspective custom = new Perspective(Rotation2d.kZero) {
		private final LoggedNetworkNumber customDegrees = new LoggedNetworkNumber("SmartDashboard/Perspective/Custom", 0.0);

		private boolean hasChanged() {
			return this.customDegrees.get() != this.forwardDirection.getDegrees();
		}

		private void setPerspectiveDegs(double degrees) {
			this.forwardDirection = Rotation2d.fromDegrees(degrees);
			this.perspectiveToField = this.forwardDirection.toMatrix();
			this.fieldToPerspective = this.perspectiveToField.inv();
		}

		private void updateIfChanged() {
			if (this.hasChanged()) {
				this.setPerspectiveDegs(this.customDegrees.get());
			}
		}

		@Override
		public Rotation2d getForwardDirection() {
			this.updateIfChanged();
			return super.getForwardDirection();
		}
	};

	private static final String key = "SmartDashboard/Perspective/Chooser";
	private static final StringPublisher namePublisher;
	private static final StringPublisher typePublisher;
	private static final StringArrayPublisher optionsPublisher;
	private static final StringPublisher defaultPublisher;
	private static final StringPublisher activePublisher;
	private static final StringEntry selectedEntry;
	private static String[] ntArray;
	private static final LoggableInputs inputs = new LoggableInputs() {
		@Override
		public void toLog(LogTable table) {
			table.put(key, ntArray);
		}

		@Override
		public void fromLog(LogTable table) {
			ntArray = table.get(key, ntArray);
		}
	};

	private static final AllianceFlipped<String> alliancePerspectiveName;
	private static final Map<String, Perspective> map;
	private static int selectionPriority;
	private static String selectedName;
	private static Perspective selectedValue;

	private static final Alert compNotAllianceAlert = new Alert("Competition Environment detected, but selected Perspective does not match the Alliance", AlertType.kWarning);

	static {
		var posXName = "Blue Alliance (+X)";
		var negXName = "Red Alliance (-X)";
		var posYName = "Blue Left (+Y)";
		var negYName = "Red Left (-Y)";
		var customName = "Custom";

		alliancePerspectiveName = new AllianceFlipped<>(posXName, negXName);

		map = new HashMap<>(5);
		map.put(posXName, posX);
		map.put(negXName, negX);
		map.put(posYName, posY);
		map.put(negYName, negY);
		map.put(customName, custom);

		selectedName = posYName;

		var table = NetworkTableInstance.getDefault().getTable(key);
		namePublisher = table.getStringTopic(".name").publish();
		typePublisher = table.getStringTopic(".type").publish();
		optionsPublisher = table.getStringArrayTopic("options").publish();
		defaultPublisher = table.getStringTopic("default").publish();
		activePublisher = table.getStringTopic("active").publish();
		selectedEntry = table.getStringTopic("selected").getEntry(selectedName, PubSubOption.excludeSelf(true));

		namePublisher.set(key);
		typePublisher.set("String Chooser");

		optionsPublisher.set(new String[] {
			posXName,
			negXName,
			posYName,
			negYName,
			customName
		});

		defaultPublisher.set(selectedName);
		activePublisher.set(selectedName);
		selectedEntry.set(selectedName);

		selectionPriority = 0;
	}

	public static void periodic() {
		ntArray = selectedEntry.readQueueValues();
		Logger.processInputs("NetworkInputs", inputs);
		for (var selected : ntArray) {
			selectedName = selected;
			selectionPriority = 2;
		}
		if (selectionPriority <= 1 && Environment.isCompetition() && !selectedName.equals(alliancePerspectiveName.getOurs())) {
			selectedName = alliancePerspectiveName.getOurs();
			selectedEntry.set(selectedName);
			selectionPriority = 1;
		}
		selectedValue = map.get(selectedName);
		activePublisher.set(selectedName);

		compNotAllianceAlert.set(Environment.isCompetition() && !alliancePerspectiveName.getOurs().equals(selectedName));
	}

	public static Perspective getCurrent() {
		return selectedValue;
	}
}
