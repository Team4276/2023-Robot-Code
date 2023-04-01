package frc.utilities;

public class RobotMode {

  public enum ROBOT_MODE {
    NOT_INITIALIZED,
    TELEOP_DRIVING,
    TELEOP_APPROACHING_PATH_START,
    TELEOP_FOLLOWING_PATH,
    BALANCING,
    HOLD_POSITION,
    IDLING,
    AUTO_DRIVING,
    AUTO_BALANCING
  };

  private static ROBOT_MODE myRobotMode;

  public static void RobotModeInit() {
    myRobotMode = ROBOT_MODE.NOT_INITIALIZED;
  }

  public static ROBOT_MODE get() {
    return myRobotMode;
  }

  public static void set(ROBOT_MODE val) {
    myRobotMode = val;
  }

  public static String getString() {

    switch (myRobotMode.ordinal()) {
      case 0:
        return "NOT_INITIALIZED";
      case 1:
        return "TELEOP_DRIVING";
      case 2:
        return "TELEOP_APPROACHING_PATH_START";
      case 3:
        return "TELEOP_FOLLOWING_PATH";
      case 4:
        return "BALANCING";
      case 5:
        return "HOLD_POSITION";
      case 6:
        return "IDLING";
      case 7:
        return "AUTO_DRIVING";
      case 8:
        return "AUTO_BALANCING";


      default:
        break;
    }
    return "*****";
  }
}
