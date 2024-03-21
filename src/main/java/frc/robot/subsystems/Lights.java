package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.MiscConstants;

public class Lights {
    // Define the blinken module and the presetlist
    private final Spark lightController = new Spark(MiscConstants.lightControllerPort);
    final Dictionary<String, Double> lightList = new Hashtable<>();

    public Lights() {
        this.setPrimary();

        // Add presets to the dictionary
        lightList.put("solid_yellow", 0.69);
        lightList.put("solid_blue", 0.87);
    }

    public void setManualy(double PWMVal) {
        lightController.set(-0.57);
    }

    public void storedPreset(String presetName) {
        lightController.set(lightList.get(presetName));
    }

    public void setPrimary() {
        lightController.set(0.37);
    }

    public void setSecondary() {
        lightController.set(0.39);
    }

    


}
