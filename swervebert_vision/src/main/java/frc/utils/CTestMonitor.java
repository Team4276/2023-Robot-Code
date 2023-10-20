//*****************************************************************************************//
// The MIT License (MIT)                                                                   //
//                                                                                         //
// Copyright (c) 2017 - Marina High School FIRST Robotics Team 4276 (Huntington Beach, CA) //
//                                                                                         //
// Permission is hereby granted, free of charge, to any person obtaining a copy            //
// of this software and associated documentation files (the "Software"), to deal           //
// in the Software without restriction, including without limitation the rights            //
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell               //
// copies of the Software, and to permit persons to whom the Software is                   //
// furnished to do so, subject to the following conditions:                                //
//                                                                                         //
// The above copyright notice and this permission notice shall be included in              //
// all copies or substantial portions of the Software.                                     //
//                                                                                         //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR              //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE             //
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                  //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,           //
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN               //
// THE SOFTWARE.                                                                           //
//*****************************************************************************************//
//*****************************************************************************************//
// We are a high school robotics team and always in need of financial support.             //
// If you use this software for commercial purposes please return the favor and donate     //
// (tax free) to "Marina High School Educational Foundation, attn: FRC team 4276"          //
// (Huntington Beach, CA)                                                                  //
//*****************************************************************************************//
package frc.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import static org.opencv.imgcodecs.Imgcodecs.imwrite;
import org.opencv.videoio.VideoWriter;

public final class CTestMonitor {

    final String HOME_NAME = "pi";

    int m_nNextFile;
    int m_nMaxFileNumber;
    String m_sBaseFileName;
    String m_sLogFolder;
    String m_sLogVideoFolder;
    VideoWriter m_outVideo;
    Boolean m_isVideoRecording = false;
    Boolean m_isPrintEnabled = false;
    Boolean m_isMonitorEnabled = true;
    MatOfByte compressionBuffer;

    public CTestMonitor() {
        init();
    }

    public void init() {
        m_nNextFile = 0;
        m_nMaxFileNumber = 1000;

        Date date = new Date();
        Calendar cal = Calendar.getInstance();
        cal.setTime(date);

        m_sBaseFileName = numberToText00(cal.get(Calendar.YEAR) + 1900);
        m_sBaseFileName += numberToText00(cal.get(Calendar.MONTH) + 1);
        m_sBaseFileName += numberToText00(cal.get(Calendar.DAY_OF_MONTH) + 1);
        m_sBaseFileName += "-";
        m_sBaseFileName += numberToText00(cal.get(Calendar.HOUR_OF_DAY));
        m_sBaseFileName += "-";
        m_sBaseFileName += numberToText00(cal.get(Calendar.MINUTE));
        m_sBaseFileName += "-";
        m_sBaseFileName += numberToText00(cal.get(Calendar.SECOND));

        m_sLogFolder = "/home/";
        m_sLogFolder += HOME_NAME;
        m_sLogFolder += "/log";
        m_sLogVideoFolder = "/home/";
        m_sLogVideoFolder += HOME_NAME;
        m_sLogVideoFolder += "/logVideo";
        m_nNextFile = getNextFileNumber(m_sLogFolder);
    }

    public void initVideo(int framesPerSec, int height, int width, int codec) {
        m_isVideoRecording = false; //enableVideoCollection(true, framesPerSec, height, width, codec);
    }

    public String numberToText(int n) {
        Integer i = n;
        return i.toString();
    }

    public String numberToText00(int n) {
        return String.format("%02d", n);
    }

    public String numberToText0000(int n) {
        return String.format("%04d", n);
    }

    public int atoi(String str) {
        if (str == null || str.length() < 1) {
            return 0;
        }

        // trim white spaces
        str = str.trim();

        char flag = '+';

        // check negative or positive
        int i = 0;
        if (str.charAt(0) == '-') {
            flag = '-';
            i++;
        } else if (str.charAt(0) == '+') {
            i++;
        }
        // use double to store result
        double result = 0;

        // calculate value
        while (str.length() > i && str.charAt(i) >= '0' && str.charAt(i) <= '9') {
            result = result * 10 + (str.charAt(i) - '0');
            i++;
        }

        if (flag == '-') {
            result = -result;
        }

        // handle max and min
        if (result > Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }

        if (result < Integer.MIN_VALUE) {
            return Integer.MIN_VALUE;
        }

        return (int) result;
    }

    public int getNextFileNumber(String sFolderPath) {
        int uiRet = 0;
        Boolean bFound = false;
        long newestTime = 0;
        String sNewestFileName = "";
        File dir = new File(sFolderPath);
        File[] directoryListing = dir.listFiles();
        if (directoryListing != null) {
            for (File child : directoryListing) {
                if (newestTime < child.lastModified()) {
                    newestTime = child.lastModified();
                    sNewestFileName = child.getName();
                }
            }
        }
        if (bFound) {
            if (sNewestFileName.length() > 3) {
                sNewestFileName = sNewestFileName.substring(0, 3);
                uiRet = atoi(sNewestFileName);
                if (uiRet >= m_nMaxFileNumber) {
                    uiRet = 0;
                }
            }
        }
        m_nNextFile = uiRet;
        return uiRet;
    }

    public String getNextFilePath(String sFolderPath) {
        String sRet = sFolderPath;
        sRet += "/";
        sRet += numberToText0000(m_nNextFile++);
        sRet += "-";
        sRet += m_sBaseFileName;
        return sRet;
    }

    public String getLogFilePath() {
        String sRet = m_sLogFolder;
        sRet += "/log-";
        sRet += m_sBaseFileName;
        sRet += ".txt";
        return sRet;
    }

    public void deleteFileByNumberIfExists(int nFile, String sFolderPath) {
        String sFile = sFolderPath;
        sFile += "/";
        sFile += numberToText0000(nFile);
        sFile += "*.*";
        File f = new File(sFile);
        f.delete();
    }

    public void dbgMsg_s(String str) {
        System.out.print(str);
    }

    public Boolean logWrite(String sLine) {
        if (m_isPrintEnabled) {
            dbgMsg_s(sLine);
        }
        if (m_isMonitorEnabled) {
            OutputStreamWriter sw;
            try {
                sw = new OutputStreamWriter(new FileOutputStream(getLogFilePath()));
                Writer writer = new BufferedWriter(sw);
                try {
                    writer.write(sLine);
                } catch (IOException ex) {
                    Logger.getLogger(CTestMonitor.class.getName()).log(Level.SEVERE, null, ex);
                }
                FileOutputStream out = new FileOutputStream(getLogFilePath());
            } catch (FileNotFoundException ex) {
                Logger.getLogger(CTestMonitor.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        return true;
    }

    public Boolean enableVideoCollection(Boolean bEnable, int framesPerSec, int height, int width, int codec) {
        int nColor = 1;
        Size mySize = new Size(width, height);
        String sFilePath = getNextFilePath(m_sLogVideoFolder);
        sFilePath += ".avi";

        //make output video file  
        m_outVideo.open(sFilePath, codec, framesPerSec, mySize);
        return m_outVideo.isOpened();
    }

    public void saveVideoFrame(Mat frame) {
        if (m_isMonitorEnabled && m_isVideoRecording) {
            m_outVideo.write(frame);
        }
    }

    public Boolean saveFrameToJpeg(Mat frame) {
        if (m_isMonitorEnabled) {
            try {
                MatOfInt myQualityParams = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 90);
                int bufferSize = frame.channels() * frame.cols() * frame.rows();
                byte[] outputBuffer = new byte[bufferSize];
                compressionBuffer = new MatOfByte(outputBuffer);
                deleteFileByNumberIfExists(m_nNextFile, m_sLogFolder);
                String sFilePath = getNextFilePath(m_sLogFolder);
                sFilePath += ".jpg";
                if (Imgcodecs.imwrite(sFilePath, frame, myQualityParams)) {
                    return true;
                } else {
                    dbgMsg_s("CTestMonitor:  Write failed");
                }
            } catch (Exception e) {
                System.out.println("Error: " + e.getMessage());
            }
        }
        return false;
    }

    public long getTicks() {
        return System.currentTimeMillis();
    }

    public long getDeltaTimeSeconds(long timeStart, long timeEnd) {
        long dTemp = getDeltaTimeMilliseconds(timeStart, timeEnd);
        dTemp /= 1000.0;
        return dTemp;
    }

    public long getDeltaTimeMilliseconds(long timeStart, long timeEnd) {
        return timeEnd - timeStart;
    }

    public void padString(String str, int desiredLength) {
        while (str.length() < desiredLength) {
            str += " ";
        }
    }
}
