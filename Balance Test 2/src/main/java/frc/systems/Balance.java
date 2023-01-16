package frc.systems;

import frc.utilities.Xbox;
import frc.robot.Robot;

public class Balance{
    static private final double SLOW_ZONE_ANGLE = 7.5; // Degree
    static private final double DEAD_ZONE_ANGLE = 1.0;

    static private double initialRightDistance = 0.0;
    static private double initialLeftDistance = 0.0;
    static private double currentRightDistance = 0.0;
    static private double currentLeftDistance = 0.0;

    static private final double MAX_POWER = 0.01; // Percent motor output
    static private final double SLOW_POWER = 0.01;
    static private final double HOLD_POSITION = 0.0;

    static private final double PLATFORM_LENGTH = 999999999;

    public static double LinearCalc(double initialDistance, double currentDistance) {
        double distance = Math.abs(currentDistance - initialDistance);

        double power = 0.01 * (1 - (distance/PLATFORM_LENGTH));
        
        return power;

    }

    public static void balance(double Pitch) {
        if (Robot.xboxController.getRawButton(Xbox.B)){
            if (Pitch > DEAD_ZONE_ANGLE){
                if (Pitch > SLOW_ZONE_ANGLE){
                    Drivetrain.assignMotorPower(MAX_POWER,MAX_POWER);

                    initialRightDistance = Drivetrain.getRightEncoderDistance();
                    initialLeftDistance = Drivetrain.getLeftEncoderDistance();
                }

                else if (Pitch < SLOW_ZONE_ANGLE){
                    currentRightDistance = Drivetrain.getRightEncoderDistance();
                    currentLeftDistance = Drivetrain.getLeftEncoderDistance();

                    double rightPower = LinearCalc(initialRightDistance, currentRightDistance);
                    double leftPower = LinearCalc(initialLeftDistance, currentLeftDistance);

                    Drivetrain.assignMotorPower(rightPower,leftPower);
                }

                else {
                    Drivetrain.assignMotorPower(HOLD_POSITION,HOLD_POSITION);

                }



            } else if (Pitch < (-1.0 * DEAD_ZONE_ANGLE)){
                if (Pitch > (-1.0 * SLOW_ZONE_ANGLE)){
                    Drivetrain.assignMotorPower(-1.0 * SLOW_POWER, -1.0 * SLOW_POWER);

                    initialRightDistance = Drivetrain.getRightEncoderDistance();
                    initialLeftDistance = Drivetrain.getLeftEncoderDistance();

                }

                else if (Pitch < (-1.0 * SLOW_ZONE_ANGLE)){
                    currentRightDistance = Drivetrain.getRightEncoderDistance();
                    currentLeftDistance = Drivetrain.getLeftEncoderDistance();

                    double rightPower = LinearCalc(initialRightDistance, currentRightDistance);
                    double leftPower = LinearCalc(initialLeftDistance, currentLeftDistance);

                    Drivetrain.assignMotorPower(-1.0 * rightPower, -1.0 * leftPower);                }

                else {
                    Drivetrain.assignMotorPower(HOLD_POSITION,HOLD_POSITION);
                }
            } else {
                Drivetrain.assignMotorPower(HOLD_POSITION,HOLD_POSITION);
            }
        }
    }
}