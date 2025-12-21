package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Helpers;

public final class LEDModes {
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> setColor(
      Color color) {
    return (
        start) -> {
      return (length) -> {
        return (buffer) -> {
          for (int i = start; i < (start + length); i++) {
            if (color == Color.kYellow) {
              // Custom yellow
              buffer.setRGB(i, 255, (int) (255 * 0.50), 0);
            } else {
              buffer.setLED(i, color);
            }
          }
          return buffer;
        };
      };
    };
  }

  private static double rainbowSpeed = 100;
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = (firstPixelHue + (i * 180 / length)) % 180;
          buffer.setHSV(i, hue, 255, 128);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redChase = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = Helpers.clamp((firstPixelHue + (i * 180 / length)) % 180, 51, 255);
          buffer.setRGB(i, hue, 0, 0);
        }
        return buffer;
      };
    };
  };

  private static double breatheSpeed = 205;
  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redBreathe = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        int r = (int) (Math.pow(Math.sin(System.currentTimeMillis() / 1000.0), 2) * breatheSpeed) + 50;
        for (int i = start; i < (start + length); i++) {
          buffer.setRGB(i, r, 0, 0);
        }
        return buffer;
      };
    };
  };
}
