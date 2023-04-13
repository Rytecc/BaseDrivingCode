package frc.robot;

import java.lang.ModuleLayer.Controller;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    private AddressableLED ledInstance;
    private AddressableLEDBuffer bufferInstance;
    private XboxController controlInstance;
    private int currentEffect;

    public Lights(XboxController controlInstance, int lightPort, int lightCount) {
        this.controlInstance = controlInstance;
        ledInstance = new AddressableLED(lightPort);
        bufferInstance = new AddressableLEDBuffer(lightCount);

        ledInstance.setLength(lightCount);
        ledInstance.start();
        currentEffect = 1;
    }

    public void lightsPeriodic() {
        if(controlInstance.getAButton()) {
            currentEffect = 1;
        } else if(controlInstance.getXButton()) {
            currentEffect = 2;
        }

        if(currentEffect == 1) { off(); }
        else if(currentEffect == 2) { rainbow(); }

        ledInstance.setData(bufferInstance);
    }

    void off() {
        for(int i = 0; i < bufferInstance.getLength(); i++) {
            bufferInstance.setLED(i, Color.kBlack);
        }
    }

    private int rainbowFirstPixelHue = 0;
    void rainbow() {
        for (var i = 0; i < bufferInstance.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / bufferInstance.getLength())) % 180;
            bufferInstance.setHSV(i, hue, 255, 128);
        }
        
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }
}
