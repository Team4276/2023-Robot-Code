package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

public class BetterController {
    XboxController controller;

    public BetterController(XboxController controller){
        this.controller = controller;
    }

    public boolean getRightTriggerPressed(){
        if(controller.getRightTriggerAxis() > OIConstants.kTriggerDeadband){
            return true;
        } else {
            return false;
        }
    }

    
    public boolean getLeftTriggerPressed(){
        if(controller.getLeftTriggerAxis() > OIConstants.kTriggerDeadband){
            return true;
        } else {
            return false;
        }
    }



    
    
}
