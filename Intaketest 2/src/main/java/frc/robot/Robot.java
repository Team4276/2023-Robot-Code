// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.systems.AutoDrivetrain;
import frc.systems.TeleopDrivetrain;
import frc.systems.BaseDrivetrain;
import frc.systems.Intake;
import frc.systems.Shoulder;
import frc.utilities.RoboRioPorts;
import frc.utilities.Xbox;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  public static Joystick leftJoystick;
  public static Joystick rightJoystick;

  Notifier driveRateGroup;
  public static BaseDrivetrain mBaseDrivetrain;
  public static TeleopDrivetrain mTeleopDrivetrain;

  public static Timer systemTimer;

  public static Intake mIntake;
  public static Shoulder mShoulder;
  public static Wrist mWrist;

  public static XboxController xboxController;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);

    mBaseDrivetrain = new BaseDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2, RoboRioPorts.DIO_DRIVE_RIGHT_A,
        RoboRioPorts.DIO_DRIVE_RIGHT_B, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B);

    mTeleopDrivetrain = new TeleopDrivetrain();

    AutoDrivetrain.PIDDrivetrainInit();

    mIntake = new Intake(RoboRioPorts.CAN_INTAKE);
    mShoulder = new Shoulder(RoboRioPorts.CAN_SHOULDER_R, RoboRioPorts.CAN_SHOULDER_L);
    mWrist = new Wrist(RoboRioPorts.CAN_WRIST);

    // Drive train motor control is done on its own timer driven thread regardless
    // of disabled/teleop/auto mode selection
    driveRateGroup = new Notifier(mTeleopDrivetrain::operatorDrive);
    driveRateGroup.startPeriodic(0.05);

    xboxController = new XboxController(2);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    AutoDrivetrain.PIDDrivetrainUpdate();

  }

  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    mShoulder.updatePeriodic();

    if (Robot.xboxController.getRawButton(Xbox.A)) {
      // Intake
      mIntake.setSpeed(0.3);

    } else if (Robot.xboxController.getRawButton(Xbox.Y)) {
      // Outtake
      mIntake.setSpeed(-0.3);
    } else {
      mIntake.setSpeed(0.0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
