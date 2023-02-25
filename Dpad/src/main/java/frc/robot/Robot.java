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
import frc.systems.Elbow;
import frc.systems.PIDElbow;
import frc.systems.PIDDrivetrain;
import frc.systems.Shoulder;
import frc.systems.TeleopDrivetrain;
import frc.utilities.Gyroscope;
import frc.utilities.RoboRioPorts;
import frc.utilities.Xbox;

public class Robot extends TimedRobot {

  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static XboxController xboxController;

  Notifier driveRateGroup;
  public static TeleopDrivetrain mTeleopDrivetrain;
  public static PIDDrivetrain mPIDDrivetrain;

  public static Shoulder mShoulder;
  public static Elbow mElbow;

  public static Timer systemTimer;

  public static double initialPitch = 0;

  public static boolean isCAN = true;
  public static boolean usingPIDDrivetrain = false;

  public static double pov;

  public static double armSafeZone = 0;

  public static void timedDrive() {
    if (SmartDashboard.getNumber("Encoder_W_Pos", 0) > armSafeZone){
      TeleopDrivetrain.assignMotorPower(0, 0);
    } else if (Robot.xboxController.getRawButton(Xbox.B)) {
      Robot.usingPIDDrivetrain = true;
      PIDDrivetrain.PIDDrivetrainUpdate();
      Balance.balance(Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));
      PIDDrivetrain.updateTelemetry();
    } else if (Robot.xboxController.getRawButton(Xbox.X)) {
      PIDDrivetrain.holdPosition = true;
      PIDDrivetrain.PIDDrivetrainUpdate();
    } else {
      Robot.usingPIDDrivetrain = false;
      Balance.stopBalance();
      mTeleopDrivetrain.operatorDrive();
      TeleopDrivetrain.updateTelemetry();
    }

    mShoulder.updatePeriodic();
    mElbow.updatePeriodic();

    PIDElbow.PIDElbowUpdate();
    PIDElbow.updateTelemetry();
  }


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxController = new XboxController(2);

    mTeleopDrivetrain = new TeleopDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    mPIDDrivetrain = new PIDDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    PIDDrivetrain.PIDDrivetrainInit();

        mShoulder = new Shoulder(RoboRioPorts.CAN_SHOULDER_R, RoboRioPorts.CAN_SHOULDER_L);
        mElbow = new Elbow(RoboRioPorts.CAN_ELBOW);
   
    // Drive train motor control is done on its own timer driven thread regardless
    // of disabled/teleop/auto mode selection
    driveRateGroup = new Notifier(Robot::timedDrive);
    driveRateGroup.startPeriodic(0.05);
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
    SmartDashboard.putBoolean("Using Joystick", TeleopDrivetrain.usingJoystick);
    pov = xboxController.getPOV();
    SmartDashboard.putNumber("POV", pov);
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
    initialPitch = Gyroscope.GetPitch();
    PIDElbow.PIDElbowInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
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
