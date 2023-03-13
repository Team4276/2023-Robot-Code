package frc.utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
    }
}
