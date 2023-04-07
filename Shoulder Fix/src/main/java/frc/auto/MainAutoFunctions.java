package frc.auto;



public class MainAutoFunctions{
    
    public static void auto_shoot_backup(){
        if (!AutoScoringFunctions.taskIsFinished){
            AutoScoringFunctions.scoreLow();
        } else if (!BabyAuto.taskIsFinished) {
            BabyAuto.leftScoreMobility();
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