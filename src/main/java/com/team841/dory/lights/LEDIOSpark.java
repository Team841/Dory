package com.team841.dory.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOSpark implements LEDIO{
    private final AddressableLED LED = new AddressableLED(0);
    private final AddressableLEDBuffer Buffer = new AddressableLEDBuffer(61);

    public LEDIOSpark() {
        LED.setLength(Buffer.getLength());
        LED.setData(Buffer);
        LED.start();
        setColor("yellow");
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
    //   inputs.sparkOutput = LED.get();
    }

    @Override
    public void setColor(String color){
        if(color == "red"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 255, 0, 0);
            LED.setData(Buffer);
        }
        if(color == "green"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 0, 255, 0);
            LED.setData(Buffer);
        }
        if(color == "blue"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 0, 0, 255);
            LED.setData(Buffer);
        }
        if(color == "yellow"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 255, 200, 0);
            LED.setData(Buffer);
        }
        if(color == "off"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 0, 0, 0);
            LED.setData(Buffer);
        }
        if(color == "white"){
            for(var i = 0; i < Buffer.getLength(); i++)
            Buffer.setRGB(i, 255, 255, 255);
            LED.setData(Buffer);
        }
    
    }

    @Override
    public void set(double value) {
    //    LED.set(value);
    }
}
