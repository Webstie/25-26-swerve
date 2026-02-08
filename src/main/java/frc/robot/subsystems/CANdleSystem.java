package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Candle.*;
import frc.robot.Constants.LightState;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CANdleSystem extends SubsystemBase {
    
    // 如果你的 Constants 里没有定义这个，请保留下面这段枚举定义
    // 如果 Constants 里已经有了，请删除这段并确保 import frc.robot.Constants.LightState;
    public enum LightState {
        OFF, INTAKING, OUTTAKING, SHOOTING, CLIMBING
    }

    private final CANdle m_candle = new CANdle(1, "canivore"); 
    private final int LedCount = 80;
    private static final double DEFAULT_STROBE_SPEED = 0.5;
    private static final double DEFAULT_BRIGHTNESS = 0.2;

    private Animation m_toAnimate = null;
    private boolean m_manualColorEnabled = false;
    private int m_manualR = 0;
    private int m_manualG = 0;
    private int m_manualB = 0;
    private boolean m_manualOverride = false;
    private LightState m_desiredState = LightState.OFF;
    private LightState m_appliedState = null;

    public CANdleSystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = DEFAULT_BRIGHTNESS;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        
        m_candle.configAllSettings(configAll, 100);
        // Default to steady white on boot
        m_manualColorEnabled = true;
        m_manualR = 0;
        m_manualG = 0;
        m_manualB = 0;
        m_candle.setLEDs(m_manualR, m_manualG, m_manualB);
    }

    public void setLightState(LightState state) {
        m_desiredState = state;
        m_manualOverride = false;
        m_appliedState = null;
        applyLightState();
    }

    public Command setLightStateCommand(LightState state) {
        return new InstantCommand(() -> setLightState(state));
    }

    public Command holdLightState(LightState state) {
        return startEnd(
            () -> setLightState(state),
            () -> setLightState(LightState.OFF)
        );
    }

    public void setLightState(LightState state) {
        m_manualOverride = false; 
        if (m_desiredState != state) {
            m_desiredState = state;
            m_appliedState = null; 
        }
    }

    public Command setLightStateCommand(LightState state) {
        return new InstantCommand(() -> setLightState(state), this);
    }

    public Command setRgbCommand(int r, int g, int b, boolean blink) {
        return new InstantCommand(() -> {
            m_manualOverride = true;
            m_candle.configBrightnessScalar(scalar, 0);
            if (blink) {
                m_toAnimate = new StrobeAnimation(clamp(r), clamp(g), clamp(b), 0, DEFAULT_STROBE_SPEED, LedCount);
            } else {
                m_toAnimate = null;
                updateColorVariables(clamp(r), clamp(g), clamp(b));
            }
        }, this);
    }

    public Command setRainbowCommand() {
        return new InstantCommand(() -> {
            m_manualOverride = true;
            m_manualColorEnabled = false;
            m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        }, this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (!m_manualOverride) {
            applyLightState();
        }
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

    private void applyLightState() {
        if (m_desiredState == m_appliedState) {
            return;
        }
        m_appliedState = m_desiredState;
        m_toAnimate = null;
        m_manualColorEnabled = true;
        m_candle.configBrightnessScalar(DEFAULT_BRIGHTNESS, 0);
        switch (m_desiredState) {
            case OFF:
                m_manualR = 0;
                m_manualG = 0;
                m_manualB = 0;
                break;
            case INTAKING:
                m_manualR = 0;
                m_manualG = 255;
                m_manualB = 0;
                break;
            case OUTTAKING:
                m_manualR = 255;
                m_manualG = 255;
                m_manualB = 0;
                break;
            case SHOOTING:
                m_manualR = 255;
                m_manualG = 0;
                m_manualB = 0;
                break;
            case CLIMBING:
                m_manualR = 0;
                m_manualG = 0;
                m_manualB = 255;
                break;
        }
        m_candle.setLEDs(m_manualR, m_manualG, m_manualB);
    }
}
