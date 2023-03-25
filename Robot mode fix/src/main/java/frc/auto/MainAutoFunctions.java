package frc.auto;

public class MainAutoFunctions{
    public static enum AUTOS{
        NO_BANANA,
        SHOOT_BACKUP,
        SHOOT_BALANCE,
        SHOOT,
        BALANCE,
    }

    private static AUTOS Auto;

    public static void MainAutoFunctionsInit(){
        Auto = AUTOS.NO_BANANA;
    }

    public static AUTOS get() {
        return Auto;
    }
    
    public static void set(AUTOS val) {
        Auto = val;
    }

    public static String getString() {

        switch (Auto.ordinal()) {
          case 0:
            return "NO_BANANA";
          case 1:
            return "SHOOT_BACKUP";
          case 2:
            return "SHOOT_BALANCE";
          case 3:
            return "SHOOT";
          case 4:
            return "BALANCE";
    
          default:
            break;
        }
        return "*****";
      }
    
    public static void auto_shoot_backup(){
        if (!AutoScoringFunctions.taskIsFinished){
            AutoScoringFunctions.scoreLow();
        } else if (!BabyAuto.taskIsFinished) {
            BabyAuto.ScoreMobility();
        } else {
            // idk somtin im probably missing
        }
    }

    public static void auto_shoot(){
        if (!AutoScoringFunctions.taskIsFinished){
            AutoScoringFunctions.scoreLow();
        }
    }

    public static void auto_balance(){
        if (!BabyAuto.taskIsFinished) {
            BabyAuto.middleBalance(true);
        } else {
            // idk somtin im probably missing
        }

    }

    public static void auto_shoot_balance(){
        if (!AutoScoringFunctions.taskIsFinished){
            AutoScoringFunctions.scoreLow();
        } else if (!BabyAuto.taskIsFinished) {
            BabyAuto.middleBalance(false);
        } else {
            // idk somtin im probably missing
        }
    }


}