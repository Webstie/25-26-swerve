// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("removal")
public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle1 = new CANdle(Constants.CANDLE.CANdleID1, "rio");
    private final CANdle m_candle2 = new CANdle(Constants.CANDLE.CANdleID2, "rio");
    private final int LedCount = 300;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff, SetAll
    }
    private AnimationTypes m_currentAnimation;

    private Constants.RobotState.State m_current_state = Constants.RobotState.State.Idle;
    private Constants.RobotState.State m_background_state = Constants.RobotState.State.Idle;

    public CANdleSystem() {
        changeAnimation(AnimationTypes.SetAll);
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
    public void Changecolor(Constants.RobotState.State state) {
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

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat1() { return m_candle1.getBusVoltage(); }
    public double getVbat2() { return m_candle2.getBusVoltage(); }
    public double get5V1() { return m_candle1.get5VRailVoltage(); }
    public double get5V2() { return m_candle2.get5VRailVoltage(); }
    public double getCurrent1() { return m_candle1.getCurrent(); }
    public double getCurrent2() { return m_candle2.getCurrent(); }
    public double getTemperature1() { return m_candle1.getTemperature(); }
    public double getTemperature2() { return m_candle2.getTemperature(); }
    public void configBrightness(double percent) { m_candle1.configBrightnessScalar(percent, 0); m_candle2.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle1.configLOSBehavior(disableWhenLos, 0); m_candle2.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle1.configLEDType(type, 0); m_candle2.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle1.configStatusLedState(offWhenActive, 0); m_candle2.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        switch (toChange) {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
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
