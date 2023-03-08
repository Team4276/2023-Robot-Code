package frc.utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReceivePath {
    public static void main(String[] args) {
        //get table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("PathCorners");

        // Get the entry containing the path corners
        NetworkTableEntry entry = table.getEntry("Corners");
        List<Double> cornerList = new ArrayList<>();

        // Get the corner list from NetworkTables
        double[] cornerArray = entry.getDoubleArray(new double[]{});

        for (double corner : cornerArray) {
            cornerList.add(corner);
        }

        // Convert the corner list to a list of Vector3 objects
        List<Vector3> corners = new ArrayList<>();
        for (int i = 0; i < cornerList.size(); i += 3) {
            Vector3 corner = new Vector3(cornerList.get(i), cornerList.get(i + 1), cornerList.get(i + 2));
            corners.add(corner);
        }
        System.out.print(cornerArray[0]);
        for (int k = 0; k < corners.size(); k++) {
            SmartDashboard.putNumber("Corner " + k + " X", corners.get(k).x);
            SmartDashboard.putNumber("Corner " + k + " Y", corners.get(k).y);
            SmartDashboard.putNumber("Corner " + k + " Z", corners.get(k).z);
        }
    }
}
