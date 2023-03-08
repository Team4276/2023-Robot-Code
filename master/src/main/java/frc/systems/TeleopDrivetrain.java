
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Robot;
import frc.utilities.LogJoystick;
import frc.utilities.Toggler;

public class TeleopDrivetrain extends BaseDrivetrain {
    private static boolean brakeModeisEngaged = true;

    private static DriveMode currentMode = DriveMode.TANK;
    private static String currentMode_s = "Tank";

    private static double leftPower = 0;
    private static double rightPower = 0;
    private Toggler brakeModeToggler;

    private double deadband = 0.05;

    public TeleopDrivetrain(int FLport, int BLport, int FRport, int BRport) {

        super(FLport, BLport, FRport, BRport);

        brakeModeToggler = new Toggler(LogJoystick.B1);
        brakeModeToggler.setMechanismState(true); // sets to brake mode
    }

    /**
     * 
     * @param rightPow right motor power
     * @param leftPow  left motor power
     */

    public static void assignMotorPower(double rightPow, double leftPow) {

        flDriveX.set(leftPow);
        blDriveX.set(leftPow);
        frDriveX.set(rightPow);
        brDriveX.set(rightPow);

        rightPower = rightPow;
        leftPower = leftPow;
    }

    public void operatorDrive() {

        changeMode();
        if (currentMode == DriveMode.ARCADE) {
            currentMode_s = "Arcade";
        } else {
            currentMode_s = "Tank";
        }

        double leftY = 0;
        double rightY = 0;

        switch (currentMode) {

            case ARCADE:
                double linear = 0;
                double turn = 0;

                if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
                    linear = -Robot.rightJoystick.getY();
                }
                if (Math.abs(Robot.leftJoystick.getX()) > deadband) {
                    turn = Math.pow(Robot.leftJoystick.getX(), 3);
                }

                leftY = -linear - turn;
                rightY = linear - turn;
                assignMotorPower(rightY, leftY);
                break;

            case TANK:
                if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
                    rightY = Math.pow(Robot.rightJoystick.getY(), 3 / 2);
                } else {
                    rightY = 0;
                }
                if (Math.abs(Robot.leftJoystick.getY()) > deadband) {
                    leftY = -Math.pow(Robot.leftJoystick.getY(), 3 / 2);
                } else {
                    leftY = 0;
                }
                assignMotorPower(rightY, leftY);
                break;

            default:
                break;
        }
    }

    public enum DriveMode {
        TANK, ARCADE
    }

    public void changeMode() {
        brakeModeToggler.updateMechanismStateLJoy();
        brakeModeisEngaged = brakeModeToggler.getMechanismState();
        if (brakeModeisEngaged) {
            flDriveX.setIdleMode(IdleMode.kBrake);
            blDriveX.setIdleMode(IdleMode.kBrake);
            frDriveX.setIdleMode(IdleMode.kBrake);
            brDriveX.setIdleMode(IdleMode.kBrake);
        } else {
            flDriveX.setIdleMode(IdleMode.kCoast);
            blDriveX.setIdleMode(IdleMode.kCoast);
            frDriveX.setIdleMode(IdleMode.kCoast);
            brDriveX.setIdleMode(IdleMode.kCoast);
        }
    }

}
