package frc.utilities;

import frc.systems.BaseDrivetrain;

public class DistanceLog extends BaseDrivetrain {
    public DistanceLog(int FLport, int BLport, int FRport, int BRport) {
        super(FLport, BLport, FRport, BRport);
    }
    public static double WheelCircumfrence = 0.1524*Math.PI;//in Meters
    public static double DistanceTracker(){

    double FrDistanceTravled = ((frDriveX.getEncoder().getPosition() * WheelCircumfrence)*-1)/42;
    double BrDistanceTravled = ((brDriveX.getEncoder().getPosition() * WheelCircumfrence)*-1)/42;
    double BlDistanceTravled = (blDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    double FlDistanceTravled = (flDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;        
    double distanceTravledstart = (FlDistanceTravled + FrDistanceTravled + BrDistanceTravled + BlDistanceTravled)/4;
        
        return distanceTravledstart;
    }
}
