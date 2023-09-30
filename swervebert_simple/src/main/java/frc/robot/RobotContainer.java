// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoPicker;
import frc.robot.commands.SpinnyWEWEE;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PIDElbow;
import frc.utils.BetterXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private static final XboxController m_opController = new XboxController(OIConstants.kopControllerPort);

    private static final BetterXboxController m_BetterXboxController = new BetterXboxController(m_opController);

    private final AutoPicker chooser = new AutoPicker(m_robotDrive);

    private static PIDElbow pidElbow;

    Notifier armRateGroup;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        pidElbow = new PIDElbow(ElbowConstants.ElbowID);

        armRateGroup = new Notifier(RobotContainer::timedArm);
        armRateGroup.startPeriodic(0.05);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));

        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));
        

        new JoystickButton(m_driverController, Button.kL1.value)
                .whileTrue(new SpinnyWEWEE(m_robotDrive));

        new JoystickButton(m_opController, Button.kCross.value)
                .onTrue(new InstantCommand(() -> {PIDElbow.setZero();}));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getAutoCommand();

        }

        public static void timedArm(){
                pidElbow.PIDElbowUpdate(m_opController.getLeftY(), m_BetterXboxController.getPOV());
        }
}
