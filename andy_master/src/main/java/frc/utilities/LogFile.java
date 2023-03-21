package frc.utilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalTime;

public class LogFile {

    private final int MAX_NUMBER_OF_LOG_FILES = 10;
    private final int MAX_SIZE_OF_ONE_LOG_FILE = 300000; // 10 * 300KB approx 3MB total max for log files

    private int writeCount = 0;
    private String path;
    private FileWriter logFileWriter;

    public LogFile() {
    }

    public boolean init() {
        try {
            // Create log file with unique name
            LocalTime currentTime = java.time.LocalTime.now();

            // Limit number of files to 10 - delete oldest if needed
            long oldestFileTime = Long.MAX_VALUE;

            File oldestFile = null;
            File targetDir = new File("/home/admin/");
            if (targetDir.list().length > MAX_NUMBER_OF_LOG_FILES - 1) {
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

            path = "/home/admin/logpos_" + currentTime.toString() + ".txt";
            logFileWriter = new FileWriter(path);

        } catch (Exception e) {
            return false;
        }
        return true;
    }

    public boolean write(String val) {
        try {
            writeCount++;
            if (writeCount > 100) {
                writeCount = 0;
                File file = new File(path);
                if (file.length() > MAX_SIZE_OF_ONE_LOG_FILE) {
                    logFileWriter.close();
                    init();
                }
            }
            logFileWriter.write(val);
        } catch (IOException e) {
            return false;
        }
        return true;
    }
}
