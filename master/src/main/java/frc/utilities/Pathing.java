package frc.utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//may not work still debugging will build though

public class Pathing {
    
    public static NetworkTableInstance inst = NetworkTableInstance.getDefault();//define the instance 
    public static NetworkTable table = inst.getTable("PathCorners");// get table

     
    
    public static void IntiateServer(){
        inst.startClient3("Roborioclient");//not always needed but is needed for some edge cases. just starts a client
        inst.startDSClient(1735);//Get driver station ip and uses the driver station as a server
        inst.setServer("", 1735);//sets the adress and port for the client
        inst.setServerTeam(4276, 1735);
        SmartDashboard.putNumber("X target", 0);
        SmartDashboard.putNumber("Y target", 0);
        SmartDashboard.putNumber("Z target", 0);

    }    

    public static void SetSimOrinitation(){
        table.getEntry("pitch").setDouble(Gyroscope.GetPitch());
        table.getEntry("roll").setDouble(Gyroscope.GetRoll());
        table.getEntry("yaw").setDouble(Gyroscope.GetYaw());
    }
    
    public static void receivePath(){
        Vector3 target = new Vector3(SmartDashboard.getNumber("X target", 0),SmartDashboard.getNumber("Y target", 0),SmartDashboard.getNumber("Z target", 0) );
        table.getEntry("X target").setDouble(target.x);
        table.getEntry("Y target").setDouble(target.y);
        table.getEntry("Z target").setDouble(target.z);
        try{
       if (SmartDashboard.getBoolean("Get path" , false) == true){
        //System.out.print("1");

        // Get the entry containing the path corners
        NetworkTableEntry entry = table.getEntry("Corners");
        List<Double> cornerList = new ArrayList<>();
        //System.out.println(cornerList);
        //System.out.print(inst.isConnected());
        //System.out.print("2");

        // Get the corner list from NetworkTables
        double[] cornerArray = entry.getDoubleArray(new double[]{});
        double distance = table.getEntry("Distance").getDouble(0); // gets the distance of the path (note this is not the direct distance to the target just the length of the path)
        //System.out.print("3");
        //System.out.println(cornerArray);
        //System.out.println(cornerArray[0]);

        for (double corner : cornerArray) {
            cornerList.add(corner);
           // System.out.print("4");
        }

        // Convert the corner list to a list of Vector3 objects
        List<Vector3> corners = new ArrayList<>();
        for (int i = 0; i < cornerList.size(); i += 3) {
            Vector3 corner = new Vector3(cornerList.get(i), cornerList.get(i + 1), cornerList.get(i + 2));
            corners.add(corner);
        }
      
        for (int k = 0; k < corners.size(); k++) {
            //just kept because i dont wanna rewrite this later
            
        }
        //System.out.print("6");
    SmartDashboard.putNumber("Distance",distance );
    SmartDashboard.putBoolean("Get path", false);


    
    }
}
    catch(Exception e) {
    
         }
}


}
