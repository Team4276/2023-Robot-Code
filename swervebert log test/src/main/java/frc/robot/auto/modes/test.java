package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class test extends SequentialCommandGroup {
    test(){
        addCommands(
            new WaitUntilCommand(null)
        );
    }
}
