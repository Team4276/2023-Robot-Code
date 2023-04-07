package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

import frc.auto.AutoScoringFunctions;

public class Intake {

    private static CANSparkMax intakeDrive;

    private static double deadband = 0.2;

    public Intake(int port) {
        intakeDrive = new CANSparkMax(port, MotorType.kBrushless);
    }

    // Speed inrange -1.0 to +1.0
    public static void setSpeed(double speed) {
        intakeDrive.set(speed);
    }

    public void updatePeriodic() {

        if (Robot.xboxController.getRightTriggerAxis() > deadband) {
            // Intake
            setSpeed(0.65);
        } else if ((Robot.xboxController.getLeftTriggerAxis() > deadband) || (AutoScoringFunctions.usingIntake)) { // -1 for deadband in opposite
                                                                                       // direction
            // Outtake
            setSpeed(-0.65);
        } else {
            setSpeed(0.0);
        }
    }

}
