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


public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle1 = new CANdle(Constants.CANDLE.CANdleID1, "canivore");
    private final CANdle m_candle2 = new CANdle(Constants.CANDLE.CANdleID2, "canivore");
    private final int LedCount = 300;
    //private CommandXboxController joystick;

    private Animation m_toAnimate = null;

    public  boolean hasSetFlowEffect = false; 

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,

    }
    
    private AnimationTypes m_currentAnimation;

    public boolean isflow = false;
    public Constants.RobotState.State m_current_state = Constants.RobotState.State.STATE1;

    public CANdleSystem() {
        //this.joystick = joy;
        changeAnimation(AnimationTypes.SetAll);//默认不亮灯
        CANdleConfiguration configAll = new CANdleConfiguration();

        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        
        m_candle1.configAllSettings(configAll, 100);
        m_candle2.configAllSettings(configAll, 100);
    }

    public void incrementAnimation() {
        m_current_state = Constants.RobotState.State.STATE1;
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        m_current_state = Constants.RobotState.State.STATE2;
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void Changecolor(Constants.RobotState.State state) {//change color according to the position of elevator
        m_current_state = state;
 
    }

    public void setOff() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 0, 0);
        m_candle2.setLEDs(0, 0, 0);

    }
    

    public void setWhite() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 255, 255);
        m_candle2.setLEDs(255, 255, 255);
        
    }
    public void setRed() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 0, 0);
        m_candle2.setLEDs(255, 0, 0);

    }
    public void setGreen() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 255, 0);
        m_candle2.setLEDs(0, 255, 0);
 
    }
    public void setBlue() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 0, 255);
        m_candle2.setLEDs(0, 0, 255);
    }
    public void setYellow(){ 
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 255, 0);
        m_candle2.setLEDs(255, 255, 0);

    }
    public void setCyan() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(0, 255, 255);
        m_candle2.setLEDs(0, 255, 255);
    }
    public void setMagenta() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 0, 255);
        m_candle2.setLEDs(255, 0, 255);

    }
    public void setOrange() {
        m_candle1.animate(null);
        m_candle2.animate(null);
        m_candle1.setLEDs(255, 165, 0);
        m_candle2.setLEDs(255, 165, 0);

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

    public void configBrightness(double percent) { m_candle1.configBrightnessScalar(percent, 0);
                                                   m_candle2.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle1.configLOSBehavior(disableWhenLos, 0); 
                                                    m_candle2.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle1.configLEDType(type, 0); 
                                                   m_candle2.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle1.configStatusLedState(offWhenActive, 0); 
                                                                 m_candle2.configStatusLedState(offWhenActive, 0); }



    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
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
    //机器人实际状态不同，对应的灯效也不同
    @Override
    public void periodic() {
        //System.out.println("m_currentstate: " + m_elevator.m_current_state);
        switch(m_current_state) {
            case STATE1:
                setOff();
                break;
            case STATE2:
                setOrange();
                break;
            case STATE3:
                setRed();
                break;
            case STATE4:
                m_candle1.animate(new RainbowAnimation(1, 1, LedCount));//single showoff lights
                m_candle2.animate(new RainbowAnimation(1, 1, LedCount));
                break;
            default:
                setOff();
                break;
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
