package frc.systems;

import frc.utilities.Xbox;
import frc.robot.Robot;

public class Balance{
    static private final double SLOW_ZONE = 7.5; // Degree
    static private final double DEAD_ZONE = 3.0;

    static private final double MAX_POWER = 0.3; // Percent motor output
    static private final double SLOW_POWER = 0.125;
    static private final double HOLD_POSITION = 0.0;

    public static void balance(double Pitch) {
        if (Robot.xboxController.getRawButton(Xbox.B)){
            if (Pitch > DEAD_ZONE){
                if (Pitch > SLOW_ZONE){
                Drivetrain.assignMotorPower(MAX_POWER,-1 * MAX_POWER);
                }
                else if (Pitch < SLOW_ZONE){
                Drivetrain.assignMotorPower(SLOW_POWER,-1 * SLOW_POWER);
                }
                else {
                    Drivetrain.assignMotorPower(HOLD_POSITION,HOLD_POSITION);
                }
            } else if (Pitch < (-1.0 * DEAD_ZONE)){
                if (Pitch > (-1.0 * SLOW_ZONE)){
                    Drivetrain.assignMotorPower(-1.0 * SLOW_POWER,SLOW_POWER);
                }
                else if (Pitch < (-1.0 * SLOW_ZONE)){
                    Drivetrain.assignMotorPower(-1.0 * MAX_POWER,MAX_POWER);
                }
                else {
                    Drivetrain.assignMotorPower(HOLD_POSITION,HOLD_POSITION);
                }
            } else {
                Drivetrain.assignMotorPower(HOLD_POSITION,HOLD_POSITION);
            }
        }
    }
}