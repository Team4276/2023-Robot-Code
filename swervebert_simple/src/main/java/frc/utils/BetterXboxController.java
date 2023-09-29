package frc.utils;

import edu.wpi.first.wpilibj.XboxController;

public class BetterXboxController {
    
    XboxController xboxController;

    public BetterXboxController(XboxController xboxController){

        this.xboxController = xboxController;
    }

    /** Returns 0 for neutral 1-4 from top to left going clockwise and 5 for when it isn't one of these values */
    public int getPOV(){
        int pov;
        if (xboxController.getPOV() == -1){
            pov = 0;
        } else if (xboxController.getPOV() == 0){
            pov = 1;
        } else if (xboxController.getPOV() == 90){
            pov = 2;
        } else if (xboxController.getPOV() == 180){
            pov = 3;
        } else if (xboxController.getPOV() == 270){
            pov = 4;
        } else {
            pov = 5;
        }

        return pov;
    }
}
