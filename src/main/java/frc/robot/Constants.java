package frc.robot;

public class Constants {
    public static final class Climber{
        public static final double ClimberTopPosition = 120.0;
        public static final double ClimbPosition = 1.0;

        public static final int CLIMBER_MOTOR_ID = 1;
    }

    public static final class Shooter {
        public static final double Frictionwheelshootspeed = 70.0;
        public static final double Intakeballspeed = 30.0;

        public static final int FEEDER_MOTOR_ID = 2;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 3;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 4;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 5;
    }

    public static final class Intake{
        public static final double IntakeSwingDownPosition = -14.0;
        public static final double IntakeSwingUpPosition = -5.0;
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = -16.0;
        public static final double SwingWaitTime = 0.25;
        public static final double OuttakeVelocity = 20.0;
        public static final double IntakeVelocity = -50.0;
        
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int INTAKE_PITCH_MOTOR_ID = 7;
    }

    public static final class Transport{
        public static int TransportSpeed = -30;

        public static int TRANSPORT_MOTOR_ID = 8;
    }

    public static final class Candle{
        public static final int CANDLEID = 1;
    }
}