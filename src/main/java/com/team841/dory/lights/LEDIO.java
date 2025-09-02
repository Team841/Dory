package com.team841.dory.lights;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public class LedIOInputs{
        public double sparkOutput;
    }

    public default void updateInputs(LedIOInputs inputs) { }

    public default void setColor(String color) { }

    public default void set(double value) { }
}
