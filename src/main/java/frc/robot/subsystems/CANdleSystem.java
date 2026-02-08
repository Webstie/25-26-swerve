package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Candle.*;

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
    private Animation m_lastAppliedAnimate = null; 
    
    private boolean m_manualOverride = false;
    private LightState m_desiredState = LightState.OFF;
    private LightState m_appliedState = null;

    private int m_curR = 0;
    private int m_curG = 0;
    private int m_curB = 0;

    public CANdleSystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = DEFAULT_BRIGHTNESS;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        
        m_candle.configAllSettings(configAll, 100);
        
        setLightState(LightState.OFF);
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
            m_toAnimate = new RainbowAnimation(1, 0.5, LedCount);
        }, this);
    }

    @Override
    public void periodic() {
        if (!m_manualOverride) {
            processAutoLogic();
        }
        handleHardwareUpdate();
    }

    private void processAutoLogic() {
        if (m_desiredState == m_appliedState) return;
        m_appliedState = m_desiredState;

        m_toAnimate = null; 

        switch (m_desiredState) {
            case INTAKING:  updateColorVariables(0, 255, 0);   break; 
            case OUTTAKING: updateColorVariables(255, 255, 0); break; 
            case SHOOTING:  updateColorVariables(255, 0, 0);   break; 
            case CLIMBING:  updateColorVariables(0, 0, 255);   break; 
            case OFF:
            default:        updateColorVariables(0, 0, 0);     break;
        }
    }

    private void handleHardwareUpdate() {
        if (m_toAnimate != null) {
            if (m_toAnimate != m_lastAppliedAnimate) {
                m_candle.animate(m_toAnimate);
                m_lastAppliedAnimate = m_toAnimate;
            }
        } else {
            if (m_lastAppliedAnimate != null) {
                m_candle.clearAnimation(0); 
                m_lastAppliedAnimate = null;
                m_candle.setLEDs(m_curR, m_curG, m_curB);
            }
        }
    }

    private void updateColorVariables(int r, int g, int b) {
        m_curR = r; m_curG = g; m_curB = b;
        m_candle.setLEDs(m_curR, m_curG, m_curB);
    }

    private int clamp(int val) {
        return Math.max(0, Math.min(255, val));
    }

    public double getBusVoltage() { return m_candle.getBusVoltage(); }
}