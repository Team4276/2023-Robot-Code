
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import frc.utilities.Constants;
import frc.utilities.LogJoystick;
import frc.robot.Robot;
import frc.utilities.SoftwareTimer;
import frc.utilities.Toggler;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class TeleopDrivetrain {
    private final int HI_SHIFTER = 4;
    private final int LO_SHIFTER = 3;
    private Gear currentGear = Gear.HI;
    private DoubleSolenoid gearShifter;
    private SoftwareTimer shiftTimer;
    private boolean shiftInit = true;
    private boolean isShifting = false;
    private boolean brakeModeisEngaged = true;

    private DriveMode currentMode = DriveMode.TANK;
    private String currentMode_s = "Tank";

    private static CANSparkMax flDriveX = BaseDrivetrain.flDriveX;
    private static CANSparkMax blDriveX = BaseDrivetrain.blDriveX;
    private static CANSparkMax frDriveX = BaseDrivetrain.frDriveX;
    private static CANSparkMax brDriveX = BaseDrivetrain.brDriveX;

    private static double leftPower = 0;
    private static double rightPower = 0;
    private Toggler brakeModeToggler;

    private double deadband = 0.05;

    public TeleopDrivetrain() {

        brakeModeToggler = new Toggler(LogJoystick.B1);
        brakeModeToggler.setMechanismState(true); // sets to brake mode

        shiftTimer = new SoftwareTimer();
    }

    /**
     * 
     * @param rightPow right motor power
     * @param leftPow  left motor power
     */

    public static void assignMotorPower(double rightPow, double leftPow) {
        SmartDashboard.putBoolean("Drive check", true);

        flDriveX.set(leftPow);
        blDriveX.set(leftPow);
        frDriveX.set(rightPow);
        brDriveX.set(rightPow);

        rightPower = rightPow;
        leftPower = leftPow;
    }

    public void operatorDrive() {

        changeMode();
        checkForGearShift();
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
                if (!isShifting) {
                    assignMotorPower(rightY, leftY);
                } else {

                    assignMotorPower(0, 0);
                }

                break;

            case TANK:
                if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
                    rightY = Math.pow(Robot.rightJoystick.getY(), 3 / 2);
                }
                if (Math.abs(Robot.leftJoystick.getY()) > deadband) {
                    leftY = -Math.pow(Robot.leftJoystick.getY(), 3 / 2);
                }
                if (!isShifting) {
                    assignMotorPower(rightY, leftY);
                } else {

                    assignMotorPower(0, 0);
                }
                break;

            default:
                break;
        }

        updateTelemetry();
    }

    /**
     * Checks for joystick input to shift gears. Manages to logic and timing to not
     * power drive motors while shifting
     */
    public void checkForGearShift() {
        boolean shiftHi = Robot.leftJoystick.getRawButton(HI_SHIFTER);
        boolean shiftLo = Robot.leftJoystick.getRawButton(LO_SHIFTER);

        if (shiftHi) {
            currentGear = Gear.HI;
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.HI_GEAR_VALUE);
        } else if (shiftLo) {
            currentGear = Gear.LO;
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.LO_GEAR_VALUE);
        } else {
            isShifting = false;
        }

    }

    /**
     * current gear status
     */

    public enum Gear {
        HI, LO
    }

    public enum DriveMode {
        TANK, ARCADE
    }

    /**
     * 
     * @param shiftTo desired gear
     */
    public void shiftGear(Gear shiftTo) {
        boolean shiftHi = false;
        boolean shiftLo = false;

        currentGear = shiftTo;

        if (shiftTo == Gear.HI) {
            shiftHi = true;
        } else {
            shiftLo = true;
        }

        if (shiftHi) {
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.HI_GEAR_VALUE);
        } else if (shiftLo) {
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.LO_GEAR_VALUE);
        } else {
            isShifting = false;
        }

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

    /**
     * updates smartdashboard
     */
    public void updateTelemetry() {
        // shifting status
        SmartDashboard.putBoolean("Shifting", isShifting);
        SmartDashboard.putString("Drive Mode", currentMode_s);
        // current gear
        SmartDashboard.putBoolean("HI Gear", (currentGear == Gear.HI));
        SmartDashboard.putBoolean("LOW Gear", (currentGear == Gear.LO));
        // power outputs
        SmartDashboard.putNumber("Right Power", rightPower);
        SmartDashboard.putNumber("Left Power", leftPower);
        // Check Coast/Brake
        SmartDashboard.putBoolean("Brake Mode", brakeModeisEngaged);
    }

}
