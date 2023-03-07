// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.systems.Balance;
import frc.systems.Intake;
import frc.systems.PIDDrivetrain;
import frc.systems.PIDElbow;
import frc.systems.PIDShoulder;
import frc.systems.TeleopDrivetrain;
import frc.utilities.Gyroscope;
import frc.utilities.LedStripControl;
import frc.utilities.Location4276;
import frc.utilities.RoboRioPorts;
import frc.utilities.Xbox;

public class Robot extends TimedRobot {

  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static XboxController xboxController;

  Notifier driveRateGroup;
  public static TeleopDrivetrain mTeleopDrivetrain;
  public static PIDDrivetrain mPIDDrivetrain;

  public static PIDShoulder mShoulder;
  public static PIDElbow mElbow;
  public static Intake mIntake;

  public static LedStripControl myLedStrip;

  public static Timer systemTimer;

  public static double initialPitch = 0;

  public static boolean isCAN = true;

  public static Location4276 myLocation;

  public static double pov;

  public static double deadband = 0.05;

  public static boolean isTestMode = false;

  public static void timedDrive() {
    if ((Math.abs(Robot.rightJoystick.getY()) > deadband)
        || (Math.abs(Robot.leftJoystick.getY()) > deadband)) {
      SmartDashboard.putNumber("Right Joystick output check", Robot.rightJoystick.getY());
      PIDDrivetrain.newPositiontohold = true;
      PIDDrivetrain.holdPosition = false;
      mTeleopDrivetrain.operatorDrive();
      TeleopDrivetrain.updateTelemetry();

    } else if (Robot.xboxController.getRawButton(Xbox.X)
        || Robot.rightJoystick.getRawButton(3)) {
      PIDDrivetrain.holdPosition = true;
      PIDDrivetrain.PIDDrivetrainUpdate();
      PIDDrivetrain.updateTelemetry();

    } else if (Robot.xboxController.getRawButton(Xbox.B)) {
      Balance.balance(Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));
      PIDDrivetrain.PIDDrivetrainUpdate();
      PIDDrivetrain.updateTelemetry();

    } else {
      TeleopDrivetrain.assignMotorPower(0, 0);
      TeleopDrivetrain.updateTelemetry();

    }

    mIntake.updatePeriodic();

    PIDElbow.PIDElbowUpdate();
    PIDElbow.updateTelemetry();

    PIDShoulder.PIDShoulderUpdate();
    PIDShoulder.updateTelemetry();

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    isTestMode = false;

    CameraServer.startAutomaticCapture();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxController = new XboxController(2);

    mTeleopDrivetrain = new TeleopDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    mPIDDrivetrain = new PIDDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    PIDDrivetrain.PIDDrivetrainInit();

    mShoulder = new PIDShoulder(RoboRioPorts.CAN_SHOULDER_R, RoboRioPorts.CAN_SHOULDER_L);
    mElbow = new PIDElbow(RoboRioPorts.CAN_ELBOW);
    mIntake = new Intake(RoboRioPorts.CAN_INTAKE);

    // Drive train motor control is done on its own timer driven thread regardless
    // of disabled/teleop/auto mode selection
    driveRateGroup = new Notifier(Robot::timedDrive);
    driveRateGroup.startPeriodic(0.05);

    myLocation = new Location4276();

    myLedStrip = new LedStripControl();
    myLedStrip.setMode(LedStripControl.LED_MODE.LED_OFF);

    PIDElbow.PIDElbowInit();
    PIDShoulder.PIDShoulderInit();
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
    Gyroscope.gyroscopeUpdate();
    pov = xboxController.getPOV();
    SmartDashboard.putNumber("POV", pov);

    SmartDashboard.putNumber("LJoystickPos", Math.abs(Robot.xboxController.getLeftY()));

    myLocation.updatePosition();

  }

  @Override
  public void autonomousInit() {
    isTestMode = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_AUTO);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    isTestMode = false;
    initialPitch = Gyroscope.GetPitch();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_TELEOP_NORMAL);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    isTestMode = false;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_OFF);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    isTestMode = true;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_OFF);
  }
}
