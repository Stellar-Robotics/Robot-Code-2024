package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;

public class Lights extends SubsystemBase{
    // Define the blinken module and the presetlist
    private final Spark lightController = new Spark(MiscConstants.lightControllerPort);
    final Dictionary<String, Double> lightList = new Hashtable<>();

    public Lights() {
        this.setPrimary();

        // Add presets to the dictionary
        lightList.put("solid_yellow", 0.69);
        lightList.put("solid_blue", 0.87);
        lightList.put("solid-red", 0.61);
        lightList.put("solid-green", 0.77);
        lightList.put("solid-pink", 0.57);
    }

    public void setManualy(double PWMVal) {
        lightController.set(PWMVal);
    }

    public void storedPreset(String presetName) {
        lightController.set(lightList.get(presetName));
    }

    public void setPrimary() {
        lightController.set(0.39);
    }

    public void setSecondary() {
        lightController.set(0.37);
    }

    


}
