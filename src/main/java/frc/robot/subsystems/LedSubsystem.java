package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstans;
import frc.robot.commands.Claw.ClawSet;

public class LedSubsystem extends SubsystemBase {

    private static AddressableLED m_led;

    private static AddressableLEDBuffer m_ledBuffer;

    private static int m_rainbowFirstPixelHue;

    private static int yellowPulseBrightness = 0;

    private static int purplePulseBrightness = 0;

    public LedSubsystem() {

        m_led = new AddressableLED(LedConstans.led_port_sag);

        m_ledBuffer = new AddressableLEDBuffer(LedConstans.led_uzunluk);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);

        m_led.start();

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("RobotState", RobotContainer.currentState.toString());
        SmartDashboard.putString("LastState", ClawSet.lastState.toString());

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

            m_ledBuffer.setRGB(i, 246, 190, 0);

        }

        m_led.setData(m_ledBuffer);
    }

    public void yellowAnimated() {
        for (int a = 0; a < 4; a++) {
            m_ledBuffer.setRGB(a, 255, 255, 0);
        }

        for (var u = 4; u < m_ledBuffer.getLength(); u++) {
            m_ledBuffer.setRGB(u, 255, 255, 0);
            m_ledBuffer.setRGB(u - 4, 0, 0, 0);

            if (u == m_ledBuffer.getLength() - 1) {
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                    m_ledBuffer.setRGB(i - 4, 255, 255, 0);
                }
            }
        }

        m_led.setData(m_ledBuffer);

    }

    public void purple() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 148, 0, 211);

        }

        m_led.setData(m_ledBuffer);

    }

    public void purpleAnimated() {
        for (int a = 0; a < 4; a++) {
            m_ledBuffer.setRGB(a, 148, 0, 211);
        }

        for (var u = 4; u < m_ledBuffer.getLength(); u++) {
            m_ledBuffer.setRGB(u, 148, 0, 211);
            m_ledBuffer.setRGB(u - 4, 0, 0, 0);

            if (u == m_ledBuffer.getLength() - 1) {
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                    m_ledBuffer.setRGB(i - 4, 148, 0, 211);
                }
            }
        }

        m_led.setData(m_ledBuffer);

    }

    public void yellowPulse() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, yellowPulseBrightness, yellowPulseBrightness, 0);

        }

        yellowPulseBrightness += 25;

        yellowPulseBrightness %= 255;

        m_led.setData(m_ledBuffer);

    }

    public void purplePulse() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, purplePulseBrightness, 0, purplePulseBrightness);

        }

        purplePulseBrightness += 10;

        purplePulseBrightness %= 128;

        m_led.setData(m_ledBuffer);
    }

    public void close() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            m_ledBuffer.setRGB(i, 0, 0, 0);

        }

        m_led.setData(m_ledBuffer);
    }
}
