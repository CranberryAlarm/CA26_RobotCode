package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = Constants.LEDs.k_totalLength;

  // Main sections
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_ledStripColor = LEDModes
      .setColor(Color.kRed);

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    super("LEDs");
    m_led = new AddressableLED(Constants.LEDs.k_PWMId);
    m_led.setLength(m_ledTotalLength);
    m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    setColorMode();

    m_led.setData(m_buffer);
  }

  public void setColor(Color color) {
    m_ledStripColor = LEDModes.setColor(color);
  }

  public void defaultLEDS() {
    breathe();
  }

  public void chase() {
    m_ledStripColor = LEDModes.redChase;
  }

  public void breathe() {
    m_ledStripColor = LEDModes.redBreathe;
  }

  public void rainbow() {
    m_ledStripColor = LEDModes.rainbow;
  }

  public void setColorMode() {
    m_buffer = m_ledStripColor.apply(0).apply(Constants.LEDs.k_totalLength).apply(m_buffer);
  }

  @Override
  public void stop() {
  }

  @Override
  public void reset() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
  }
}
