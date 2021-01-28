package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CTREEncoder {

    WPI_TalonFX talon;
    boolean invert;

    public CTREEncoder(WPI_TalonFX _talon, boolean invert) {
        talon = _talon;
        talon.getSensorCollection();
        this.invert = invert;
    }

    public int get() {
        return (int) (invert ? -talon.getSelectedSensorPosition() : talon.getSelectedSensorPosition()); // improted the project and now it wants (int) part of this line
    }

    public int getVelocity() {
        return (int) (invert ? -talon.getSelectedSensorVelocity() : talon.getSelectedSensorVelocity()); // Same with one too
    }

    public void setPosition(int pos) {
        talon.setSelectedSensorPosition(pos);
    }

    public void reset() {
        setPosition(0);
    }
}