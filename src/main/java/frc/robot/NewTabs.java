package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NewTabs {
    private static HashMap<String, ShuffleboardTab> tabs = new HashMap<>();
    private static HashMap<String, GenericEntry> entries = new HashMap<>();
    private static HashMap<String, ShuffleboardLayout> layouts = new HashMap<>();

    public static ShuffleboardTab getTab(String tabTitle){
        if(!tabs.containsKey(tabTitle)){
            tabs.put(tabTitle, Shuffleboard.getTab(tabTitle));
        }
        return tabs.get(tabTitle);
    }
    public static ShuffleboardLayout getLayout(String tabTitle, String layoutTitle){
        String uniqueTitle = getUniqueTitle(tabTitle, layoutTitle);
        if(!entries.containsKey(uniqueTitle)){
            ShuffleboardLayout layout = getTab(tabTitle).getLayout(layoutTitle);
            layouts.put(uniqueTitle, layout);
            return layout;
        }
        return layouts.get(uniqueTitle);
    }

    public static void putDouble(String tabTitle, String entry, double value){
        String uniqueTitle = getUniqueTitle(tabTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getTab(tabTitle).add(entry, 0).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setDouble(value);
    }
    public static void putDouble(String tabTitle, String entry, double value, int x, int y){
        String uniqueTitle = getUniqueTitle(tabTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getTab(tabTitle).add(entry, 0).withPosition(x, y).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setDouble(value);
    }
    public static void putDouble(String tabTitle, String entry, double value, int x, int y, int w, int h){
        String uniqueTitle = getUniqueTitle(tabTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getTab(tabTitle).add(entry, 0).withPosition(x, y).withSize(w, h).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setDouble(value);
    }
    public static void putDouble(String tabTitle, String layoutTitle, String entry, double value){
        String uniqueTitle = getUniqueTitle(tabTitle, layoutTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getLayout(tabTitle, layoutTitle).add(entry, 0).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setDouble(value);
    }
    public static void putDouble(String tabTitle, String layoutTitle, String entry, double value, int x, int y){
        String uniqueTitle = getUniqueTitle(tabTitle, layoutTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getLayout(tabTitle, layoutTitle).add(entry, 0).withPosition(x, y).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setDouble(value);
    }

    public static void putBoolean(String tabTitle, String entry, boolean value){
        String uniqueTitle = getUniqueTitle(tabTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getTab(tabTitle).add(entry, 0).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setBoolean(value);
    }

    public static void putCommand(String tabTitle, Sendable sendable){
        getTab(tabTitle).add(sendable);
    }

    public static double getDouble(String tabTitle, String entry, double defaultValue){
        String uniqueTitle = getUniqueTitle(tabTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            putDouble(tabTitle, entry, defaultValue);
            return defaultValue;
        } 

        return entries.get(uniqueTitle).getDouble(defaultValue);
    }
    public static double getDouble(String tabTitle, String entry){
        return getDouble(tabTitle, entry, 0);
    }

    private static String getUniqueTitle(String tabTitle, String otherTitle){
        return tabTitle + "_" + otherTitle;
    }
    private static String getUniqueTitle(String tabTitle, String layoutTitle, String entry){
        return tabTitle + "_" + layoutTitle + "_" + entry;
    }
}
