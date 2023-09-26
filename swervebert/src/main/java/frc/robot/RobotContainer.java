// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoPicker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.utils.BetterController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final LimeLightSubsystem m_LightSubsystem = new LimeLightSubsystem();

    // The driver's controller
    XboxController m_driverCon = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_opCon = new XboxController(OIConstants.kSubsystemControllerPort);

    private final BetterController m_bopCon = new BetterController(m_opCon);

    private final AutoPicker chooser = new AutoPicker(m_driveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_LightSubsystem, m_opCon);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_driveSubsystem.drive(
                                -MathUtil.applyDeadband(m_driverCon.getLeftY(), OIConstants.kJoystickDeadband),
                                -MathUtil.applyDeadband(m_driverCon.getLeftX(), OIConstants.kJoystickDeadband),
                                -MathUtil.applyDeadband(m_driverCon.getRightX(), OIConstants.kJoystickDeadband),
                                true, true, m_driveSubsystem.driveMode.getSelected()),
                        m_driveSubsystem));

        m_ArmSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> m_ArmSubsystem.set(ArmSubsystemConstants.stow), 
                        m_ArmSubsystem)

        );

        m_IntakeSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> m_IntakeSubsystem.idle(), 
                        m_IntakeSubsystem)
        );
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
        new Trigger(m_driverCon::getRightBumper)
                .whileTrue(new RunCommand(
                        () -> m_driveSubsystem.setX(),
                        m_driveSubsystem));

        new Trigger(m_opCon::getBButton)
                .whileTrue(new RunCommand(
                        () -> m_ArmSubsystem.setElbowVelCmd(m_opCon.getLeftY()),
                        m_ArmSubsystem));

        new Trigger(m_bopCon::getLeftTriggerPressed)
                .whileTrue(new RunCommand(
                        () -> m_IntakeSubsystem.in(),
                        m_IntakeSubsystem));

        new Trigger(m_bopCon::getRightTriggerPressed)
                .whileTrue(new RunCommand(
                        () -> m_IntakeSubsystem.out(),
                        m_IntakeSubsystem));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getAutoCommand();

        }
}
