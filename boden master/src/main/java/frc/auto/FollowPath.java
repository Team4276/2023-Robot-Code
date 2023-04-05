package frc.auto;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.systems.BaseDrivetrain;
import frc.utilities.Gyroscope;
import frc.utilities.DistanceLog;


public class FollowPath extends BaseDrivetrain{
    
    public FollowPath(int FLport, int BLport, int FRport, int BRport) {
        super(FLport, BLport, FRport, BRport);
    }

    private static List<Double> Angles = new ArrayList<>();
    public static List<Double> Distances = new ArrayList<>();
    
    public static double WheelCircumfrence = 0.1524*Math.PI;//in Meters
    public static double DistancePerPoint = 0;
    public static double currentdistance = 0;

    public static boolean angleReached = false;
    public static boolean pointReached = false;
    public static boolean xyz = true; // no idea what to call this variable but it resets the distance evertime the point is incremneted 


    //throws an indexoutofboundsexception if no path has been put into unity or unity is not open make sure this is run in a try and catch
     public static void Follow() throws IndexOutOfBoundsException, NullPointerException 
     {

        int i = 0;
    
    
    double FrDistanceTravled = (frDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    double BrDistanceTravled = (brDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    double BlDistanceTravled = (blDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;
    double FlDistanceTravled = (flDriveX.getEncoder().getPosition() * WheelCircumfrence)/42;  




    List<Double> FullPath = Pathing.receivePath();
        if (FullPath != null){

         if (xyz == true){
            currentdistance = DistanceLog.DistanceTracker();
            xyz = false;
            }
    
    
    double distanceTravled = (FrDistanceTravled + BrDistanceTravled + FlDistanceTravled + BlDistanceTravled)/4 - currentdistance;
    

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
    double dead_zone_range_upper = Angles.get(i) + 2;//in degrees 
    double dead_zone_range_lower = Angles.get(i) - 2;//in degrees 

    //turn towards point
    if (angleReached != true){
        
        if ( 360 - correctedYaw < dead_zone_range_upper && 360 - correctedYaw > dead_zone_range_lower){

            FollowPath.stop();
            
            angleReached = true;
            System.out.println("done");
    
        }
    
        if (  360 - correctedYaw > Angles.get(i)){
            frDriveX.set(0.05);
            brDriveX.set(0.05);
            flDriveX.set(0.05);
            blDriveX.set(0.05);
            System.out.println("turning right");
            angleReached = false;
        }
        
        if ( 360 - correctedYaw < Angles.get(i)){
            frDriveX.set(-0.05);
            brDriveX.set(-0.05);
            flDriveX.set(-0.05);
            blDriveX.set(-0.05);
            angleReached = false;
            System.out.println("turning left");

        }

} else if (angleReached == true && pointReached != true){//drive towards point
     
   if (distanceTravled < Distances.get(i)){
        frDriveX.set(-0.05); 
        brDriveX.set(-0.05);
        flDriveX.set(0.05);
        blDriveX.set(0.05);
        pointReached = false;
        System.out.println("going staright");
    }


    if (distanceTravled > Distances.get(i)){
        frDriveX.set(0.05);
        brDriveX.set(0.05);
        flDriveX.set(-0.05);
        blDriveX.set(-0.05);
        pointReached = false;
        System.out.println("going back");

    }

    if ((distanceTravled - Distances.get(i)) > 0.5 && (distanceTravled - Distances.get(i)) < 1 || distanceTravled - Distances.get(i) < 0.5 && distanceTravled - Distances.get(i) > 1){

        FollowPath.stop();

        pointReached = true;
        System.out.println("done");

    }
}
    if (pointReached && angleReached){
        i++;//increments to next point
        
        angleReached = false;//resets angle
        pointReached = false;//resets distance 
        xyz = true;//updates pos 

    }
    //drive towards point
}   else {
    DriverStation.reportWarning("FullPath is null this is likely because unity is not running or no path was inputed", false);
}
     
     }
     public static void stop(){
        frDriveX.set(0);
        brDriveX.set(0);
        frDriveX.set(0);
        brDriveX.set(0);
     }
    
    }

