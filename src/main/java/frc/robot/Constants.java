package frc.robot;

public class Constants {
    public static final class Climber{
        public static final int CLIMBER_MOTOR_ID = 1;
        public static final double ClimberTopPosition = 120.0;
        public static final double ClimbPosition = 1.0;
    }

    public static final class Shooter {
        public static final double Frictionwheelshootspeed= 100.0;
        public static final double Intakeballspeed = 10;
        public static final int INTAKE_BALL_MOTOR_ID = 2;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 3;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 4;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 5;
    }

    public static final class Intake{
        public static final double IntakeUpPosition = -14.0;
        public static final double IntakeDownPosition = -5.0;
        public static final double OuttakePosition = -10.0;
        public static final double SwingWaitTime = 0.5;
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int INTAKE_PITCH_MOTOR_ID = 7;
        public static final int INTAKE_PITCH_ENCODER_ID = 11;
    }

    public static final class Transport{
        public static int TRANSPORT_MOTOR_ID = 8;
        public static int TransportSpeed = 15;
    }
}