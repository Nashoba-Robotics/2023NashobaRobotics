package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NewTabs {
    private static HashMap<String, ShuffleboardTab> tabs = new HashMap<>();
    private static HashMap<String, GenericEntry> entries = new HashMap<>();

    public static ShuffleboardTab getTab(String tabTitle){
        if(!tabs.containsKey(tabTitle)){
            tabs.put(tabTitle, Shuffleboard.getTab(tabTitle));
        }
        return tabs.get(tabTitle);
    }

    public static void putDouble(String tabTitle, String entry, double value){
        String uniqueTitle = getUniqueTitle(tabTitle, entry);
        if(!entries.containsKey(uniqueTitle)){
            GenericEntry e = getTab(tabTitle).add(entry, 0).getEntry();
            entries.put(uniqueTitle, e);
        }

        entries.get(uniqueTitle).setDouble(value);
    }

    public static String getUniqueTitle(String tabTitle, String entry){
        return tabTitle + "_" + entry;
    }
    
}
