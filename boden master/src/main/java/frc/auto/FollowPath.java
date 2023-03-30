package frc.systems;
import java.util.List;
import frc.utilities.*;


public class FollowPath extends BaseDrivetrain{
    
    public FollowPath(int FLport, int BLport, int FRport, int BRport) {
        super(FLport, BLport, FRport, BRport);
    }

    public static List<Double> Angles;
    public static List<Double> Distances;
    
public static void Follow (){
    int i = 0;
    List<Double> FullPath = Pathing.receivePath();

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

    if (  360 - correctedYaw > Angles.get(i)){
        frDriveX.set(-0.1);
        brDriveX.set(-0.1);
        flDriveX.set(0.1);
        blDriveX.set(0.1);
    }
    if ( 360 - correctedYaw < Angles.get(i)){
        frDriveX.set(0.1);
        brDriveX.set(0.1);
        frDriveX.set(-0.1);
        brDriveX.set(-0.1);
    }
    if ( 360 - correctedYaw < dead_zone_range_upper && 360 - correctedYaw > dead_zone_range_lower){
        frDriveX.set(0);
        brDriveX.set(0);
        frDriveX.set(0);
        brDriveX.set(0);
    }
}
}
