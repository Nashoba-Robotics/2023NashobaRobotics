package frc.robot;

import java.util.HashMap;

import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public final class LogManager {

    private static HashMap<String, LogEntry> filepathToLogEntry = new HashMap<>(); 

    // Use instead of System.out.println();
    public static void logMessage(String message) {
        DataLogManager.log(message);
    }

    public static void appendToLog(Object valueToAppend, String path) {
        if (filepathToLogEntry.containsKey(path)) {
            filepathToLogEntry.get(path).appendToLog(valueToAppend);
        }
        else {
            LogEntry newLogEntry = new LogEntry(path, valueToAppend);
            filepathToLogEntry.put(path, newLogEntry);
        }
    }
}

class LogEntry {
    enum LogType {
        BOOLEAN,
        BOOLEANARR,
        DOUBLE,
        DOUBLEARR,
        STRING,
        STRINGARR,
        INTEGER,
        INTEGERARR,
        BYTEARR
    }

    BooleanLogEntry booleanLog;
    BooleanArrayLogEntry booleanArrLog;
    DoubleLogEntry doubleLog;
    DoubleArrayLogEntry doubleArrLog;
    StringLogEntry stringLog;
    StringArrayLogEntry stringArrLog;
    IntegerLogEntry integerLog;
    IntegerArrayLogEntry integerArrLog;
    RawLogEntry byteArrLog;
    
    LogType thisEntryType;

    String path;

    public LogEntry(String path, Object valueToAppend) {
        this.path = path;
        DataLog log = DataLogManager.getLog();

        if (valueToAppend instanceof Boolean) {
            booleanLog = new BooleanLogEntry(log, path);
            thisEntryType = LogType.BOOLEAN;
        }
        else if (valueToAppend instanceof boolean[]) {
            booleanArrLog = new BooleanArrayLogEntry(log, path);
            thisEntryType = LogType.BOOLEANARR;
        }
        else if (valueToAppend instanceof Double) {
            doubleLog = new DoubleLogEntry(log, path);
            thisEntryType = LogType.DOUBLE;
        }
        else if (valueToAppend instanceof double[]) {
            doubleArrLog = new DoubleArrayLogEntry(log, path);
            thisEntryType = LogType.DOUBLEARR;
        }
        else if (valueToAppend instanceof String) {
            stringLog = new StringLogEntry(log, path);
            thisEntryType = LogType.STRING;
        }
        else if (valueToAppend instanceof String[]) {
            stringArrLog = new StringArrayLogEntry(log, path);
            thisEntryType = LogType.STRINGARR;
        }
        else if (valueToAppend instanceof Integer) {
            integerLog = new IntegerLogEntry(log, path);
            thisEntryType = LogType.INTEGER;
        }
        else if (valueToAppend instanceof long[]) {
            integerArrLog = new IntegerArrayLogEntry(log, path);
            thisEntryType = LogType.INTEGERARR;
        }
        else if (valueToAppend instanceof byte[]) {
            byteArrLog = new RawLogEntry(log, path);
            thisEntryType = LogType.BYTEARR;
        }
        else {
            LogManager.logMessage("Attempted to log an invalid value on " + this);
            return;
        }
        appendToLog(valueToAppend);
    }

    public void appendToLog(Object valueToAppend) {
        // Sets the data in its correct type
        if (valueToAppend instanceof Boolean && thisEntryType == LogType.BOOLEAN) {
            booleanLog.append((boolean) valueToAppend);
        }
        else if (valueToAppend instanceof boolean[] && thisEntryType == LogType.BOOLEANARR) {
            booleanArrLog.append((boolean[]) valueToAppend);
        }
        else if (valueToAppend instanceof Double && thisEntryType == LogType.DOUBLE) {
            doubleLog.append((double) valueToAppend);
        }
        else if (valueToAppend instanceof double[] && thisEntryType == LogType.DOUBLEARR) {
            doubleArrLog.append((double[]) valueToAppend);
        }
        else if (valueToAppend instanceof String && thisEntryType == LogType.STRING) {
            stringLog.append((String) valueToAppend);
        }
        else if (valueToAppend instanceof String[] && thisEntryType == LogType.STRINGARR) {
            stringArrLog.append((String[]) valueToAppend);
        }
        else if (valueToAppend instanceof Integer && thisEntryType == LogType.INTEGER) {
            integerLog.append((int) valueToAppend);
        }
        else if (valueToAppend instanceof long[] && thisEntryType == LogType.INTEGERARR) {
            integerArrLog.append((long[]) valueToAppend);
        }
        else if (valueToAppend instanceof byte[] && thisEntryType == LogType.INTEGERARR) {
            byteArrLog.append((byte[]) valueToAppend);
        }
        else {
            LogManager.logMessage("Attempted to log an invalid value on " + this);
            return;
        }
    }
}