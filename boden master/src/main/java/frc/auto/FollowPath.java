package frc.auto;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import frc.systems.BaseDrivetrain;
import frc.utilities.Gyroscope;



public class FollowPath extends BaseDrivetrain{
    
    public FollowPath(int FLport, int BLport, int FRport, int BRport) {
        super(FLport, BLport, FRport, BRport);
    }

    public static List<Double> Angles;
    public static List<Double> Distances;
    public static double WheelCircumfrence = 0.1524*Math.PI;//in Meters

    public static boolean angleReached = false;
    public static boolean pointReached = false;

//throws an indexoutofboundsexception if no path has been put into unity or unity is not open make sure this is run in a try and catch
    
public static void Follow() throws IndexOutOfBoundsException, NullPointerException
     {
    int i = 0;
    
    
    double distanceTravledstart = 0;//meters again
    double FrDistanceTravled = 0;
    double BrDistanceTravled = 0;
    double FlDistanceTravled = 0;
    double BlDistanceTravled = 0;

    FrDistanceTravled = (frDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    BrDistanceTravled = (brDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    BlDistanceTravled = (blDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    FlDistanceTravled = (flDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;

    distanceTravledstart = (FrDistanceTravled + BrDistanceTravled + FlDistanceTravled + BlDistanceTravled)/4;//average all the wheels distance so that turning in place returns 0 and incase 1 - 3 encoders are off a bit it will be closer to the real value 


    List<Double> FullPath = Pathing.receivePath();
        if (Angles != null && Distances != null){
    while (i <= FullPath.size()/2){
    
    double distanceTravled = (FrDistanceTravled + BrDistanceTravled + FlDistanceTravled + BlDistanceTravled)/4 - distanceTravledstart;
    double correctedYaw = Gyroscope.GetYaw();
    if (correctedYaw < 0){
        correctedYaw += 360;
        
    }

    for (int k = 0; k < FullPath.size()/2; k++){
        Angles.add(FullPath.get(k));
    }
    for (int k = FullPath.size()/2; k < FullPath.size(); k++){
        Distances.add(FullPath.get(k));
    }
    double dead_zone_range_upper = Angles.get(i) + 10;//in degrees 
    double dead_zone_range_lower = Angles.get(i) - 10;//in degrees 

    //turn towards point
    if (  360 - correctedYaw > Angles.get(i)){
        frDriveX.set(-0.1);
        brDriveX.set(-0.1);
        flDriveX.set(0.1);
        blDriveX.set(0.1);
        angleReached = false;
    }
    if ( 360 - correctedYaw < Angles.get(i)){
        frDriveX.set(0.1);
        brDriveX.set(0.1);
        frDriveX.set(-0.1);
        brDriveX.set(-0.1);
        angleReached = false;
    }
    if ( 360 - correctedYaw < dead_zone_range_upper && 360 - correctedYaw > dead_zone_range_lower){
        frDriveX.set(0);
        brDriveX.set(0);
        frDriveX.set(0);
        brDriveX.set(0);
        angleReached = true;
    }
    //drive towards point
    if (distanceTravled < Distances.get(i)){
        frDriveX.set(0.3);
        brDriveX.set(0.3);
        flDriveX.set(0.3);
        blDriveX.set(0.3);
        pointReached = false;
    }

    if (distanceTravled > Distances.get(i)){
        frDriveX.set(-0.1);
        brDriveX.set(-0.1);
        flDriveX.set(-0.1);
        blDriveX.set(-0.1);
        pointReached = false;
    }

    if ((distanceTravled - Distances.get(i)) > 0.5 && (distanceTravled - Distances.get(i)) < 1 || distanceTravled - Distances.get(i) < 0.5 && distanceTravled - Distances.get(i) > 1){
        frDriveX.set(0);
        brDriveX.set(0);
        frDriveX.set(0);
        brDriveX.set(0);
    }
    if (pointReached && angleReached){
        i++;
    }
    //drive towards point
}   
}else {
    DriverStation.reportWarning("FullPath is null this is likely because unity is not running or no path was inputed", false);
}
} 
}
