// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("removal")
public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle1 = new CANdle(Constants.CANDLE.CANdleID1, "rio");
    private final CANdle m_candle2 = new CANdle(Constants.CANDLE.CANdleID2, "rio");
    private final int LedCount = 300;

    private Constants.RobotState.State m_current_state = Constants.RobotState.State.Idle;
    private Constants.RobotState.State m_background_state = Constants.RobotState.State.Idle;

    public CANdleSystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle1.configAllSettings(configAll, 100);
        m_candle2.configAllSettings(configAll, 100);
    }

    /** Sets a temporary foreground state (does not affect background). */
    public void changeColor(Constants.RobotState.State state) {
        m_current_state = state;
    }

    /** Sets a persistent background state and updates current state. */
    public void setBackgroundState(Constants.RobotState.State state) {
        m_background_state = state;
        m_current_state = state;
    }

    /** Restores current state to the persistent background after a temporary action ends. */
    public void restoreBackground() {
        m_current_state = m_background_state;
    }

    private void applyColor(int r, int g, int b) {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(r, g, b);
        m_candle2.setLEDs(r, g, b);
    }

    @Override
    public void periodic() {
        switch (m_current_state) {
            case Idle:
                applyColor(0, 0, 0);
                break;
            case VisionFusion:
                applyColor(255, 255, 0);
                break;
            case Shooting:
                applyColor(245, 140, 245);
                break;
            case Intaking:
                applyColor(255, 0, 0);
                break;
            case Outtaking:
                applyColor(0, 0, 255);
                break;
            case ClimbingUp:
                applyColor(0, 255, 255);
                break;
            case ClimbingDown:
                m_candle1.animate(new RainbowAnimation(1, 1, LedCount));
                m_candle2.animate(new RainbowAnimation(1, 1, LedCount));
                break;
            default:
                applyColor(0, 0, 0);
                break;
        }
    }
}
