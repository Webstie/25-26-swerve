// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Candle.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANDLEID, "canivore");
    private final int LedCount = 80;
    private static final double DEFAULT_STROBE_SPEED = 0.5;
    private static final double DEFAULT_BRIGHTNESS = 0.1;

    private Animation m_toAnimate = null;
    private boolean m_manualColorEnabled = false;
    private int m_manualR = 0;
    private int m_manualG = 0;
    private int m_manualB = 0;

    public CANdleSystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public Command setRgb(int r, int g, int b, boolean blink) {
        return setRgb(r, g, b, DEFAULT_BRIGHTNESS, blink);
    }

    public Command setRgb(int r, int g, int b, double brightness, boolean blink) {
        int red = clampColor(r);
        int green = clampColor(g);
        int blue = clampColor(b);
        double scalar = clampBrightness(brightness);
        return new InstantCommand(() -> {
            m_candle.configBrightnessScalar(scalar, 0);
            if (blink) {
                m_manualColorEnabled = false;
                m_toAnimate = new StrobeAnimation(red, green, blue, 0, DEFAULT_STROBE_SPEED, LedCount);
            } else {
                m_candle.clearAnimation(0);
                m_toAnimate = null;
                m_manualColorEnabled = true;
                m_manualR = red;
                m_manualG = green;
                m_manualB = blue;
                m_candle.setLEDs(m_manualR, m_manualG, m_manualB);
            }
        }, this);
    }

    public Command setRainbow() {
        return new InstantCommand(() -> {
            m_manualColorEnabled = false;
            m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        }, this);
    }

    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(clampBrightness(percent), 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    private static int clampColor(int value) {
        return Math.max(0, Math.min(255, value));
    }

    private static double clampBrightness(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_toAnimate == null) {
            if (m_manualColorEnabled) {
                m_candle.setLEDs(m_manualR, m_manualG, m_manualB);
            }
        } else {
            m_candle.animate(m_toAnimate);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
