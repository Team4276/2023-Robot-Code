// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auto.AutoScoringFunctions;
import frc.auto.BabyAuto;
import frc.auto.MainAutoFunctions;
import frc.systems.Balance;
import frc.systems.FeederFinder;
import frc.systems.Intake;
import frc.systems.PIDDrivetrain;
import frc.systems.PIDElbow;
import frc.systems.TeleopDrivetrain;
import frc.utilities.Gyroscope;
import frc.utilities.LedStripControl;
import frc.utilities.Location4276;
import frc.utilities.LogFile;
import frc.utilities.LogJoystick;
import frc.utilities.Pathing;
import frc.utilities.RoboRioPorts;
import frc.utilities.RobotMode;
import frc.utilities.SoftwareTimer;
import frc.utilities.Xbox;

public class Robot extends TimedRobot {

  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static XboxController xboxController;

  Notifier driveRateGroup;
  public static TeleopDrivetrain mTeleopDrivetrain;
  public static PIDDrivetrain mPIDDrivetrain;

  Notifier armRateGroup;
  public static PIDElbow mElbow;
  public static Intake mIntake;

  public static FeederFinder mFeederFinder;

  public static NetworkTable ntLimelight;

  public static LedStripControl myLedStrip;

  public static LogFile myLogFile;

  public static Timer systemTimer;
  public static SoftwareTimer testTimer;

  public static double initialPitch = 0;

  public static boolean isCAN = true;

  public static Location4276 myLocation;

  public static double pov;

  public static double deadband = 0.05;

  private static boolean firstRun = true;

  private static DigitalInput Switch1;
  private static DigitalInput Switch2;
  private static DigitalInput Switch3;

  private static boolean auto0 = false;
  private static boolean auto1 = false;
  private static boolean auto2 = false;
  private static boolean auto3 = false;
  private static boolean auto4 = false;

  public static int autoselector = 0;

  private static boolean firstRunTimer4 = true;

  public static RobotMode mRobotMode;

  public static boolean isTestMode = false;

  public static boolean isTeleop = true;

  
  public static boolean isJoystickInReverse() {
    return  (Robot.rightJoystick.getY() > deadband);
  }

  public static void timedDrive() {
    boolean goDrive = false;
    if (TeleopDrivetrain.currentMode == TeleopDrivetrain.DriveMode.ARCADE) {
      if ((Math.abs(Robot.rightJoystick.getY()) > deadband)
          || (Math.abs(Robot.rightJoystick.getZ()) > deadband)) {
        goDrive = true;
      }
    } else { // TANK drive}
      if ((Math.abs(Robot.rightJoystick.getY()) > deadband)
          || (Math.abs(Robot.leftJoystick.getY()) > deadband)) {
        goDrive = true;
      }
    }

    SmartDashboard.putNumber("R joystick Z: ", Math.abs(Robot.rightJoystick.getZ()));

    if (goDrive) {
      PIDDrivetrain.newPositiontohold = true;
      PIDDrivetrain.holdPosition = false;
      mTeleopDrivetrain.operatorDrive();

    } else if (Robot.xboxController.getRawButton(Xbox.X)
        || Robot.rightJoystick.getRawButton(LogJoystick.B1)) {

      PIDDrivetrain.holdPosition = true;
      PIDDrivetrain.PIDDrivetrainUpdate();

    } else if (Robot.xboxController.getRawButton(Xbox.B) || (BabyAuto.balance)
        || (Robot.leftJoystick.getRawButton(LogJoystick.B1))) {
      Balance.balance(Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));
      if (!Balance.pause) {
        PIDDrivetrain.PIDDrivetrainUpdate();
      }

    } else if (Robot.rightJoystick.getRawButton(LogJoystick.B7)) {
      FeederFinder.updatePeriodic();
    } else {
      if (isTeleop) {
        TeleopDrivetrain.assignMotorPower(0, 0);
        Balance.pause = false;
      }

    }

    if (!((Robot.xboxController.getRawButton(Xbox.X)
        || (!Robot.rightJoystick.getRawButton(LogJoystick.B1))))) {
      PIDDrivetrain.newPositiontohold = true;
      PIDDrivetrain.holdPosition = false;
    }

    if (!isTeleop) {
      if (BabyAuto.usingDrivetrainMotorsNOPOWER) {
        TeleopDrivetrain.assignMotorPower(0, 0);
      } else if (BabyAuto.usingDrivetrainMotorsForward) {
        TeleopDrivetrain.assignMotorPower(-1 * BabyAuto.MOTORPOWER, BabyAuto.MOTORPOWER);
      } else if (BabyAuto.usingDrivetrainMotorsBackward) {
      TeleopDrivetrain.assignMotorPower(BabyAuto.MOTORPOWER, -1 * BabyAuto.MOTORPOWER);
      } else if (firstRun) {
        TeleopDrivetrain.assignMotorPower(0, 0);
        firstRun = false;
      }
    }
  }

  public static void timedArm() {

    mIntake.updatePeriodic();
    PIDElbow.PIDElbowUpdate();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    mRobotMode = new RobotMode();
    myLogFile = new LogFile();

    Pathing.IntiateServer();

    isTestMode = false;

    CameraServer.startAutomaticCapture();

    testTimer = new SoftwareTimer();

    Switch1 = new DigitalInput(5);
    Switch2 = new DigitalInput(6);
    Switch3 = new DigitalInput(7);

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxController = new XboxController(2);

    mTeleopDrivetrain = new TeleopDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    mPIDDrivetrain = new PIDDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    PIDDrivetrain.PIDDrivetrainInit();

    // Drive train motor control is done on its own timer driven thread regardless
    // of disabled/teleop/auto mode selection
    driveRateGroup = new Notifier(Robot::timedDrive);
    driveRateGroup.startPeriodic(0.05);

    mElbow = new PIDElbow(RoboRioPorts.CAN_ELBOW);
    mIntake = new Intake(RoboRioPorts.CAN_INTAKE);

    armRateGroup = new Notifier(Robot::timedArm);
    armRateGroup.startPeriodic(0.05);

    mFeederFinder = new FeederFinder();

    ntLimelight = NetworkTableInstance.getDefault().getTable("limelight");

    myLocation = new Location4276();

    myLedStrip = new LedStripControl();

    Balance.balanceinit();

    myLedStrip.setMode(frc.utilities.LedStripControl.LED_MODE.LED_OFF);

    SmartDashboard.putString("Set Robot Mode: ", "***");
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

    myLocation.updatePosition();
    myLocation.updateTelemetry();

    pov = xboxController.getPOV();

    SmartDashboard.putString("Robot Mode: ", RobotMode.getString(mRobotMode));

    SmartDashboard.putNumber("Pitch", Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));

    autoselector = 0;

    if (!Switch1.get())
      autoselector += 1;
    if (!Switch2.get())
      autoselector += 2;
    if (!Switch3.get())
      autoselector += 4;
    SmartDashboard.putNumber("Auto Mode", autoselector);

    Pathing.SetSimOrinitation();

    if (firstRunTimer4) {
      testTimer.setTimer(1);
      firstRunTimer4 = false;
    }

    if (testTimer.isExpired()) {
      // System.out.println(PIDElbow.driveElbow.getAppliedOutput());
      firstRunTimer4 = true;
    }

  }

  @Override
  public void autonomousInit() {

    autoselector = 0;

    if (!Switch1.get())
      autoselector += 1;
    if (!Switch2.get())
      autoselector += 2;
    if (!Switch3.get())
      autoselector += 4;

    if (autoselector == 0)
      auto0 = true;
    if (autoselector == 1)
      auto1 = true;
    if (autoselector == 2)
      auto2 = true;
    if (autoselector == 4)
      auto3 = true;
    if (autoselector == 7)
      auto4 = true;

    isTestMode = false;
    AutoScoringFunctions.AutoScoringFunctionsInit();
    BabyAuto.BabyAutoInit();
    isTeleop = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (auto0) {
      // do nothing
    } else if (auto1) {
      MainAutoFunctions.auto_shoot_backup();
    } else if (auto2) {
      MainAutoFunctions.auto_shoot();
    } else if (auto3) {
      MainAutoFunctions.auto_balance();
    } else if (auto4) {
      MainAutoFunctions.auto_shoot_balance();
    }

    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_AUTO);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    isTestMode = false;
    initialPitch = Gyroscope.GetPitch();
    isTeleop = true;
    BabyAuto.balance = false;
    Balance.pause = false;
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
