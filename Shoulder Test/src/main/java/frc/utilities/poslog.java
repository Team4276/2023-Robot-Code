package frc.utilities;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.systems.BaseDrivetrain;

public class poslog {
    public static void logpos(){
      try {
        double [] test =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpos").getDoubleArray(new double[6]);
      Vector3 pos = new Vector3(test[1], test[2], test[3]);
      Vector3 rotation = new Vector3(test[4], test[5], test[6]);
      /*System.out.print (pos.x);
      System.out.print (",");
      System.out.print(pos.y);
      System.out.print (",");
      System.out.print(pos.z);
      */


    }
    catch (Exception e){
        //System.out.println("No apriltags in view!");
    }
}
}
 