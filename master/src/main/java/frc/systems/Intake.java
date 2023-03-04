package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
//import frc.utilities.Xbox;

public class Intake {

    private static CANSparkMax intakeDrive;

    private static double deadband = 0.2;

    public Intake(int port) {
        intakeDrive = new CANSparkMax(port, MotorType.kBrushless);
    }

    // Speed inrange -1.0 to +1.0
    public void setSpeed(double speed) {
        intakeDrive.set(speed);
    }

    public void updatePeriodic() {
        /* if (Robot.xboxController.getRawButton(Xbox.A)) {
            // Intake
            setSpeed(0.9);

        } else if (Robot.xboxController.getRawButton(Xbox.Y)) {
            // Outtake
            setSpeed(-0.9);
        } else {
            setSpeed(0.0);
        } */

        if (Robot.xboxController.getRightY() > deadband) {
            // Intake
            setSpeed(0.9);
        } else if (Robot.xboxController.getRightY() < deadband * -1) { // -1 for deadband in opposite direction
            // Outtake
            setSpeed(-0.9);
        } else {
            setSpeed(0.0);
        }

    }

}
