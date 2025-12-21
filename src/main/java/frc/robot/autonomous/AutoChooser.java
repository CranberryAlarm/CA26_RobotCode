package frc.robot.autonomous;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRunner.AutoMode;

public class AutoChooser {
  private static AutoChooser m_autoChooser = null;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static AutoChooser getInstance() {
    if (m_autoChooser == null) {
      m_autoChooser = new AutoChooser();
    }
    return m_autoChooser;
  }

  private AutoChooser() {
    // Populate the chooser with all the available autos
    for (AutoMode mode : AutoRunner.AutoMode.values()) {
      m_chooser.addOption(mode.name(), mode.name());
    }

    SmartDashboard.putData("Auto picker", m_chooser);
  }

  public void setDefaultOption(String option) {
    m_chooser.setDefaultOption(option, option);
  }

  public void setOnChangeCallback(Consumer<String> listener) {
    m_chooser.onChange(listener);
  }
}
