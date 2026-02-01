package frc.robot;
import edu.wpi.first.math.util.Units;
public class Constants {
    public static final class Climber{
        public static final int CLIMBER_MOTOR_ID = 1;
        public static final double ClimberTopPosition = 20.0;
        public static final double ClimbPosition = 0.0;
    }

    public static final class Shooter {
        //Speeds 
        //Frictionwheel speed
        public static final double Frictionwheelshootspeed= 20;
        public static final double Frictionwheelretreatspeed =-20;
        public static final double Frictionwheelstopspeed= 0.0;
        //Intakeball speed
        public static final double Intakeballspeed= 20;
        public static final double IntakeBallRetreatSpeed= -20.0;
        public static final double Outtakeballspeed =0.0;
        public static final double Outtakeballstopspeed =0.0;
        //motor IDs
        public static final int IntakeBallMotorID = 3;
        public static final int LeftFrictionwheelMotorID = 4;
        public static final int MiddleFrictionwheelMotorID = 5;
        public static final int RightFrictionwheelMotorID = 6;
    }

    public static final class Intake{
        public static final double IntakeUpPosition = 10;
        public static final double IntakeDownPosition = 0.0;
        public static final double SwingIntakePosition1 = 0.0;
        public static final double SwingIntakePosition2 = 0.0;
        public static final double SwingWaitTime = 0.5;
    }
}
