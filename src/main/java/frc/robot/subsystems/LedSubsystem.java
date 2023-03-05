package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstans;

public class LedSubsystem extends SubsystemBase {

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  private static int m_rainbowFirstPixelHue;

  private static int bluePulseBrightness = 0;
  private static int blueStreakLED = 3;
  private static int numLoops = 3;

  private static int yellowPulseBrightness = 0;
 // private static int yellowStreakLED = 3;

  private static int purplePulseBrightness = 0;
 // private static int purpleStreakLED = 3;


  public LedSubsystem() {

    m_led = new AddressableLED(LedConstans.led_port);

    m_ledBuffer = new AddressableLEDBuffer(LedConstans.led_uzunluk);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {

  }

  public static void rainbow() {

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 3;

    m_rainbowFirstPixelHue %= 180;

    m_led.setData(m_ledBuffer);
  }

  public static void red() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 255, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public static void blue() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 0, 0, 255);
    }

    m_led.setData(m_ledBuffer);
  }

  public static void green() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 0, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }
  public static void yellow() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 255, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public static void purple() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 148, 0, 211);
    }

    m_led.setData(m_ledBuffer);
  }

  public static void bluePulse() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 0, 0, bluePulseBrightness);
    }

    bluePulseBrightness += 10;

    bluePulseBrightness %= 255;

    m_led.setData(m_ledBuffer);

  }
  public static void yellowPulse() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, yellowPulseBrightness, yellowPulseBrightness, 0);
    }

    yellowPulseBrightness += 10;

    yellowPulseBrightness %= 255;

    m_led.setData(m_ledBuffer);

  }
  public static void purplePulse() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 0, purplePulseBrightness, purplePulseBrightness);
    }

    yellowPulseBrightness += 10;

    yellowPulseBrightness %= 128;

    m_led.setData(m_ledBuffer);

  }

  public static void blueStreak() {
    RobotContainer.LedInterrupt = true;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      m_ledBuffer.setRGB(i, 0, 0, 255);
    }

    m_ledBuffer.setRGB(blueStreakLED, 255, 0, 0);

    if (numLoops % 3 == 0) {
      blueStreakLED += 1;

      blueStreakLED %= m_ledBuffer.getLength();
    }

    m_led.setData(m_ledBuffer);

    numLoops += 1;
    // Timer.delay(0.2);

  }



}