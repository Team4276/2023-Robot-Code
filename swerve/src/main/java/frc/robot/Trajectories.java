/*package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.io.File;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;



public class Trajectories {
   public static ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

   public Trajectories(){
      path_init();
   }

   public static void path_init(){
      File paths = new File("paths");

      for (File path : paths.listFiles()){
         String trajectoryJSON = path.getName();

         try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            trajectories.add(trajectory);

         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
         }

      }
   }
}*/
