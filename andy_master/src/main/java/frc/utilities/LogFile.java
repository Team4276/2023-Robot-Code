package frc.utilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalTime;

public class LogFile {

    private FileWriter logFileWriter;

    public LogFile() {
    }

    public boolean init() {
        try {
            // Create log file with unique name
            LocalTime currentTime = java.time.LocalTime.now();

            // Limit number of files to 10 - delete oldest if needed
            long oldestFileTime = Long.MAX_VALUE;
            ;
            File oldestFile = null;
            File targetDir = new File("/home/admin/");
            if (targetDir.list().length > 9) {
                File[] files = targetDir.listFiles();
                for (File file : files) {
                    if (file.lastModified() < oldestFileTime) {
                        oldestFile = file;
                        oldestFileTime = file.lastModified();
                    }
                }
                if (oldestFile != null) {
                    oldestFile.delete();
                }
            }

            logFileWriter = new FileWriter("/home/admin/logpos_" + currentTime.toString() + ".txt");

        } catch (Exception e) {
            return false;
        }
        return true;
    }

    public boolean write(String val) {
        try {
            logFileWriter.write(val);
        } catch (IOException e) {
            return false;
        }
        return true;
    }
}
