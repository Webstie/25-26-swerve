package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightState;


import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CANdleSystem extends SubsystemBase {

    private final CANdle m_candle_L = new CANdle(1, "canivore"); 
    private final CANdle m_candle_R = new CANdle(2, "canivore"); 
    
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
        
        m_candle_L.configAllSettings(configAll, 100);
        m_candle_R.configAllSettings(configAll, 100);
        
        // Initial state
        setLightState(LightState.OFF);
    }

    /** Sets the state and clears any manual overrides. */
    public void setLightState(LightState state) {
        m_manualOverride = false;
        if (m_desiredState != state) 
            m_desiredState = state;
            m_appliedState = null; // Force update in periodic
        }
    

    /** Command to set the state. Includes 'this' requirement to interrupt other LED commands. */
    public Command setLightStateCommand(LightState state) {
        return new InstantCommand(() -> setLightState(state), this);
    }

    /** Temporarily sets a state, returns to OFF when the command ends. */
    public Command holdLightState(LightState state) {
        return startEnd(
            () -> setLightState(state),
            () -> setLightState(LightState.OFF)
        );
    }

    /** Manual RGB control. Overrides the state machine. */
    public Command setRgbCommand(int r, int g, int b, boolean blink) {
        return new InstantCommand(() -> {
            m_manualOverride = true;
            m_candle_L.configBrightnessScalar(DEFAULT_BRIGHTNESS, 0);
            m_candle_R.configBrightnessScalar(DEFAULT_BRIGHTNESS, 0);
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
            m_toAnimate = new LarsonAnimation(1, 1, LedCount);
        }, this);
    }

    @Override
    public void periodic() {
        if (!m_manualOverride) {
            applyLightState();
        }

        if (m_toAnimate != null) {
            m_candle_L.animate(m_toAnimate);
            m_candle_R.animate(m_toAnimate);
        } else if (m_manualColorEnabled) {
            m_candle_L.setLEDs(m_manualR, m_manualG, m_manualB);
            m_candle_R.setLEDs(m_manualR, m_manualG, m_manualB);
        }
    }

    private void applyLightState() {
        if (m_desiredState == m_appliedState) return;
        
        m_appliedState = m_desiredState;
        m_toAnimate = null;
        m_manualColorEnabled = true;

        switch (m_desiredState) {
            case INTAKING:   updateColorVariables(0, 255, 0);   break; // Green
            case OUTTAKING:  updateColorVariables(255, 255, 0); break; // Yellow
            case SHOOTING:   updateColorVariables(255, 0, 0);   break; // Red
            case CLIMBING:   updateColorVariables(0, 0, 255);   break; // Blue
            case OFF:
            default:         updateColorVariables(0, 0, 0);     break;
        }
    }

    // --- Helper Methods ---

    private void updateColorVariables(int r, int g, int b) {
        m_manualR = r;
        m_manualG = g;
        m_manualB = b;
        m_manualColorEnabled = true;
    }

    private int clamp(int value) {
        return Math.max(0, Math.min(255, value));
    }
}
