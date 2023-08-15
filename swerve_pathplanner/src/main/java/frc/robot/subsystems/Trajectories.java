package frc.robot.subsystems;

import java.util.LinkedHashMap;

import java.io.IOException;

import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;


public class Trajectories {
   private static LinkedHashMap<String, Trajectory> paths = new LinkedHashMap<String, Trajectory>();

   public static void loadPaths(){
      paths.put("Test", getTraj("Test"));
      paths.put("Test2", getTraj("Test2"));

   }
   
   private static Trajectory getTraj(String name){
      try{ 
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
            .resolve("/home/lvuser/deploy/paths/" + name + ".wpilib.json");
         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

         return trajectory;

      }catch(IOException e){
          System.out.println("!!!!!!!!!! IO Exception on Reading Traj !!!!!!!!!!");
          return null;
      }
   }

   public static Trajectory getPath(String path_key){
      Trajectory path = paths.get(path_key);

      return path;
   }
}
