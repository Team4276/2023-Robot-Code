package frc.utils;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;


//log string to standard log location
public class logPos {
   public static Exception logString(String textToLog){
    Logger logger = Logger.getLogger("MyLog.txt");  
    FileHandler fh;
    

    try {  

        // This block configure the logger with handler and formatter  
        fh = new FileHandler("/home/admin");  
        logger.addHandler(fh);
        SimpleFormatter formatter = new SimpleFormatter();  
        fh.setFormatter(formatter);  

        // the following statement is used to log any messages  
        logger.info(textToLog);  

    } catch (SecurityException e) {  
        return e; 
    } catch (IOException ei) {  
        return ei;
    }  
    return null;
}

}
