package frc.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utilities.Gyroscope;
import frc.utilities.Vector3;



//may not work still debugging will build though

public class Pathing {
    
    public static NetworkTableInstance inst = NetworkTableInstance.getDefault();//define the instance 
    public static NetworkTable table = inst.getTable("PathCorners");// get table
    public static List<Double> angles;
     
    
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
    
    public static List<Double> receivePath(){
        Vector3 target = new Vector3(SmartDashboard.getNumber("X target", 0),SmartDashboard.getNumber("Y target", 0),SmartDashboard.getNumber("Z target", 0) );
        table.getEntry("X target").setDouble(target.x);
        table.getEntry("Y target").setDouble(target.y);
        table.getEntry("Z target").setDouble(target.z);
        try{

        //System.out.print("1");

        // Get the entry containing the path corners
        NetworkTableEntry entry = table.getEntry("Corners");
        List<Double> cornerList = new ArrayList<>();
        //System.out.println(cornerList);
        //System.out.print(inst.isConnected());
        //System.out.print("2");

        // Get the corner list from NetworkTables
        double[] cornerArray = entry.getDoubleArray(new double[]{});
        //System.out.print(cornerArray[0]);
        double distance = table.getEntry("Distance").getDouble(0); // gets the distance of the path (note this is not the direct distance to the target just the length of the path)
        //System.out.print("3");
        //System.out.println(cornerArray);
        //System.out.println(cornerArray[0]);
        

        for (double corner : cornerArray) {
            cornerList.add(corner);
            //System.out.print("4");
        }

        // Convert the corner list to a list of Vector3 objects
        List<Vector3> corners = new ArrayList<>();
        for (int i = 0; i < cornerList.size(); i += 3) {
            Vector3 corner = new Vector3(cornerList.get(i), cornerList.get(i + 1), cornerList.get(i + 2));
            corners.add(corner);
        }
        //System.out.print("5");
      
        /*for (int k = 0; k < corners.size(); k++) {
            System.out.println(corners.get(k).x);
            System.out.println(corners.get(k).y);
            System.out.println(corners.get(k).z);
            System.out.println("");
            
        }*/
        //System.out.print(corners.get(1).x);
        angles = getCornerAngles(corners);
        angles.addAll(getDistance(corners));
        return (angles);
    }

    
    


    catch(Exception e) {
        System.out.print("exception caught at pathing.receivePath");
        List<Double> exception = new ArrayList<>(1);      
        return (exception);
        }
    
}



//method to list the heading of each point
public static List<Double> getCornerAngles(List<Vector3> corners) {
    try{List<Double> angles = new ArrayList<>();
    for (int i = 0; i < corners.size() - 1; i++) {
        Vector3 currentCorner = corners.get(i);
        Vector3 nextCorner = corners.get(i + 1);
        
        angles.add(nextCorner.angle(currentCorner));

    }
    return angles;
}
catch(Exception e){
    System.out.print("exception caught at pathing.getCornerAngles");
    List<Double> exception = new ArrayList<>(1);      
    
    return (exception);
}
}



public static List<Double> getDistance(List<Vector3> corners1) {
   
   
   
    //please note that this returns a vaule in meters



    try{List<Double> Distance = new ArrayList<>();
    for (int i = 0; i < corners1.size() - 1; i++) {
        Vector3 currentCorner1 = corners1.get(i);
        Vector3 nextCorner1 = corners1.get(i + 1);
        double distance = currentCorner1.distance(nextCorner1);
        Distance.add(distance);
    }
    return Distance;
}
catch(Exception e){
    System.out.print("exception caught at pathing.getDistance");
    List<Double> exception = new ArrayList<>(1);      
    return (exception);
}
}
}


