package frc.utilities;

public class RobotMode {

  public enum ROBOT_MODE {
    NOT_INITIALIZED,
    AUTO_1_MOBILITY,
    AUTO_2_BALANCE,
    TELEOP_NORMAL,
    TELEOP_BALANCE,
    TELEOP_HOLD_POSITION,
    TELEOP_APPROACHING_PATH_START,
    TELEOP_FOLLOWING_PATH
  };

  private static ROBOT_MODE myRobotMode;

  public RobotMode() {
    myRobotMode = ROBOT_MODE.NOT_INITIALIZED;
  }

  public static ROBOT_MODE get() {
    return myRobotMode;
  }

  public static void set(ROBOT_MODE val) {
    myRobotMode = val;
  }

  public static String getString(RobotMode mRobotMode) {

    switch (myRobotMode.ordinal()) {
      case 0:
        return "NOT_INITIALIZED";
      case 1:
        return "AUTO_1_MOBILITY";
      case 2:
        return "AUTO_2_BALANCE";
      case 3:
        return "TELEOP_NORMAL";
      case 4:
        return "TELEOP_BALANCE";
      case 5:
        return "TELEOP_HOLD_POSITION";
      case 6:
        return "TELEOP_APPROACHING_PATH_START";
      case 7:
        return "TELEOP_FOLLOWING_PATH";

      default:
        break;
    }
    return "*****";
  }
};
