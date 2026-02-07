package frc.robot;

public class Constants {
    public static final class Climber{
        public static final int CLIMBER_MOTOR_ID = 12;
        public static final double ClimberTopPosition = 20.0;
        public static final double ClimbPosition = 0.0;
    }

    public static final class Shooter {
        public static final double Frictionwheelshootspeed= 0.0;
        public static final double Intakeballspeed = 10;
        public static final int INTAKE_BALL_MOTOR_ID = 13;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 14;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 15;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 5;
    }

    public static final class Intake{
        public static final double IntakeUpPosition = 20.0;
        public static final double IntakeDownPosition = 0.0;
        public static final double SwingWaitTime = 0.5;
        public static final int INTAKE_MOTOR_ID = 9;
        public static final int INTAKE_PITCH_MOTOR_ID = 10;
        public static final int INTAKE_PITCH_ENCODER_ID = 11;
    }

    public static final class Transport{
        public static int TRANSPORT_MOTOR_ID = 16;
        public static int TransportSpeed = 10;
    }
}
