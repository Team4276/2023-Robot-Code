package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

import frc.auto.AutoScoringFunctions;
//import frc.utilities.Xbox;

public class Intake {

    private static CANSparkMax intakeDrive;

    private static double deadband = 0.2;

    public static intake intakeState = intake.OFF;

    public enum intake{
        OFF,
        INTAKE,
        OUTTAKE,
    }

    public Intake(int port) {
        intakeDrive = new CANSparkMax(port, MotorType.kBrushless);
    }

    // Speed inrange -1.0 to +1.0
    public static void setSpeed(double speed) {
        intakeDrive.set(speed);
    }

    public void updatePeriodic() {
        double speed = Robot.xboxController.getRightY();

        if ((speed < -deadband)/* || (intakeState == intake.INTAKE)*/) {
            // Intake
            setSpeed(0.65);
        } else if ((speed > deadband) /*|| (AutoScoringFunctions.usingIntake) || (intakeState == intake.OUTTAKE)*/) { // -1 for deadband in opposite
                                                                                       // direction
            // Outtake
            setSpeed(-0.65);
        } else {
            setSpeed(0.0);
        }
    }

}
