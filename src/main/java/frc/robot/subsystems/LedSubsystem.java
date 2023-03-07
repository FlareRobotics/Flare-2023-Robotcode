package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstans;

public class LedSubsystem extends SubsystemBase {

    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;

    private static int m_rainbowFirstPixelHue;

    private static int yellowPulseBrightness = 0;

    private static int purplePulseBrightness = 0;

    public LedSubsystem() {

        m_led = new AddressableLED(LedConstans.led_port);

        m_ledBuffer = new AddressableLEDBuffer(LedConstans.led_uzunluk);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void rainbow() {

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;

        m_rainbowFirstPixelHue %= 180;

        m_led.setData(m_ledBuffer);
    }

    public void red() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 255, 0, 0);
        }

        m_led.setData(m_ledBuffer);
    }

    public void green() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 0, 255, 0);
        }

        m_led.setData(m_ledBuffer);
    }

    public void yellow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 255, 255, 0);
        }

        m_led.setData(m_ledBuffer);
    }

    public void purple() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 148, 0, 211);
        }

        m_led.setData(m_ledBuffer);
    }

    public void yellowPulse() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, yellowPulseBrightness, yellowPulseBrightness, 0);
        }

        yellowPulseBrightness += 10;

        yellowPulseBrightness %= 255;

        m_led.setData(m_ledBuffer);

    }

    public void purplePulse() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 0, purplePulseBrightness, purplePulseBrightness);
        }

        yellowPulseBrightness += 10;

        yellowPulseBrightness %= 128;

        m_led.setData(m_ledBuffer);

    }
}
