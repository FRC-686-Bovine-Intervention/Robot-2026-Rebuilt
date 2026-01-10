package frc.robot.auto;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.SwitchableChooser;
import frc.util.VirtualSubsystem;
import frc.util.flipping.AllianceFlipUtil;

public class AutoSelector extends VirtualSubsystem {
	private final LoggedDashboardChooser<AutoRoutine> routineChooser;
	private final List<StringPublisher> questionPublishers;
	private final List<SwitchableChooser> responseChoosers;
	private final StringPublisher configPublisher;
	private final LoggedNetworkNumber initialDelaySubscriber;
	private final String key;

	private static final AutoRoutine idleRoutine = new AutoRoutine("Do Nothing", List.of()) {
		public Command generateCommand() {
			return Commands.none();
		}
	};
	private final String questionPlaceHolder = "NA";

	private Command lastCommand;
	private AutoConfiguration lastConfiguration;

	public AutoSelector(String key) {
		this.key = key;
		this.routineChooser = new LoggedDashboardChooser<>(this.key + "/Routine");
		this.questionPublishers = new ArrayList<>();
		this.responseChoosers = new ArrayList<>();
		this.configPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(this.key).getStringTopic("Configuration").publish();
		this.initialDelaySubscriber = new LoggedNetworkNumber("SmartDashboard/" + this.key + "/Initial Delay", 0);
		this.addDefaultRoutine(idleRoutine);
	}

	private void populateQuestions(AutoRoutine routine) {
		for (int i = this.questionPublishers.size(); i < routine.questions.size(); i++) {
			var questionPublisher = NetworkTableInstance.getDefault()
				.getStringTopic("/SmartDashboard/" + this.key + "/Question #" + Integer.toString(i + 1))
				.publish()
			;
			questionPublisher.set(this.questionPlaceHolder);
			this.questionPublishers.add(questionPublisher);
			this.responseChoosers.add(new SwitchableChooser(this.key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
		}
	}

	public void addRoutine(AutoRoutine routine) {
		this.populateQuestions(routine);
		this.routineChooser.addOption(routine.name, routine);
	}

	public void addDefaultRoutine(AutoRoutine routine) {
		this.populateQuestions(routine);
		this.routineChooser.addDefaultOption(routine.name, routine);

		this.doThingy(routine, true);
	}

	@Override
	public void periodic() {
		var selectedRoutine = this.routineChooser.get();
		if (selectedRoutine == null) return;

		this.doThingy(
			selectedRoutine,
			selectedRoutine.name != this.lastConfiguration.routine()
			|| AllianceFlipUtil.getAlliance() != this.lastConfiguration.alliance()
		);
	}

	private void doThingy(AutoRoutine routine, boolean configurationChanged) {
		var alliance = AllianceFlipUtil.getAlliance();
		var config = new AutoConfiguration(alliance, routine.name, this.initialDelaySubscriber.get());

		var questions = routine.questions;
		for (int i = 0; i < this.responseChoosers.size(); i++) {
			var questionPublisher = this.questionPublishers.get(i);
			var responseChooser = this.responseChoosers.get(i);

			if (i < questions.size()) {
				var question = questions.get(i);
				questionPublisher.set(question.name);

				if (configurationChanged) {
					var settings = question.updateSettings();
					var defaultOption = settings.defaultOption().getKey();
					responseChooser.setOptions(settings.getOptionNames());
					responseChooser.setDefault(defaultOption);
					var all = true;
					for (var option : settings.getOptionNames()) {
						if (responseChooser.getSelected() == option) {
							all = false;
							break;
						}
					}
					if (all) {
						responseChooser.setSelected(defaultOption);
					}
				} else {
					var response = responseChooser.getSelected();
					var last = lastConfiguration.questions().get(question.name);
					configurationChanged = !Objects.equals(response, last);
				}

				var selectedResponse = responseChooser.getSelected();
				responseChooser.setActive(question.setResponse(selectedResponse));
				config.addQuestion(question.name, responseChooser.getActive());
			} else {
				questionPublisher.set("");
				responseChooser.setOptions();
				responseChooser.setDefault(this.questionPlaceHolder);
				responseChooser.setSelected(this.questionPlaceHolder);
				responseChooser.setActive(this.questionPlaceHolder);
			}
		}
		if (configurationChanged) {
			System.out.println("[AutoSelector] Generating new command\n" + config);
			this.lastCommand = AutoManager.generateAutoCommand(routine, config.initialDelaySeconds());
		}
		lastConfiguration = config;
		this.configPublisher.set(lastConfiguration.toString());
	}

	public Command getSelectedAutoCommand() {
		return this.lastCommand;
	}

	public static record AutoConfiguration (
		Alliance alliance,
		String routine,
		double initialDelaySeconds,
		Map<String, String> questions
	) {
		public AutoConfiguration(Alliance alliance, String routine, double initialDelaySeconds) {
			this(alliance, routine, initialDelaySeconds, new LinkedHashMap<>());
		}
		public void addQuestion(String question, String response) {
			this.questions.put(question, response);
		}

		public String toString() {
			var builder = new StringBuilder()
				.append("\t").append("Alliance: ").append(this.alliance).append("\n")
				.append("\t").append("Routine: ").append(this.routine)
				.append("\t").append("Delay: ").append(this.initialDelaySeconds)
			;
			for (var entry : this.questions.entrySet()) {
				builder.append("\n\t").append(entry.getKey()).append(": ").append(entry.getValue());
			}
			return builder.toString();
		}
	}
}
