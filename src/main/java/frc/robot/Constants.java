package frc.robot;

public class Constants {
    public static final class Climber{
        public static final int CLIMBER_MOTOR_ID = 12;
        public static final double ClimberTopPosition = 20.0;
        public static final double ClimbPosition = 0.0;
    }

    public static final class Shooter {
        //Speeds 
        //Frictionwheel speed
        public static final double Frictionwheelshootspeed= 0.0;
        public static final double Frictionwheelretreatspeed = 0.0;
        public static final double Frictionwheelstopspeed= 0.0;
        //Intakeball speed
        public static final double Intakeballspeed = 0.0;
        public static final double IntakeBallRetreatSpeed = 0.0;
        public static final double Outtakeballspeed = 0.0;
        public static final double Outtakeballstopspeed = 0.0;
        //motor IDs
        public static final int INTAKE_BALL_MOTOR_ID = 13;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 14;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 15;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 16;
    }

    public static final class Intake{
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = 0.0;
        public static final double SwingIntakePosition1 = 0.0;
        public static final double SwingIntakePosition2 = 0.0;
        public static final double SwingWaitTime = 0.5;
        public static final int INTAKE_MOTOR_ID = 9;
        public static final int INTAKE_PITCH_MOTOR_ID = 10;
        public static final int INTAKE_PITCH_ENCODER_ID = 5;
    }

    public static final class Transport{
        public static int TRANSPORT_MOTOR_ID = 11;
    }
}
