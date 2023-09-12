package frc.systems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utilities.Vector3;
import frc.robot.Robot;



public class LogPos {

    
    
    public static Vector3 log(){    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable("limelight");
        
        try{
        double[] errorhandle = new double [6];
        double[] pos = table.getEntry("botpos").getDoubleArray(errorhandle);
        Vector3 vectorpos = new Vector3(pos[0], pos[1], pos[2]);
        Robot.wr.write(String.valueOf(vectorpos.x));
        Robot.wr.write(String.valueOf(vectorpos.y));
        Robot.wr.write(String.valueOf(vectorpos.z));
    
        return vectorpos;
        }
        
        catch(Exception e){
            Vector3 Exception = new Vector3(0, 0, 0);
            return Exception;
        }
    }
}
         
    
    


