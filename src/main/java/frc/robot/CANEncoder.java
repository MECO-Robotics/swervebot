package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class CANEncoder implements IEncoder {

    WPI_CANCoder m_canCoder;


    public CANEncoder(int deviceID, boolean inverted) {
        m_canCoder = new WPI_CANCoder(deviceID);
        m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    @Override
    public double getDistance() {
        return m_canCoder.getPosition();
    }

    @Override
    public double getRate() {
        return m_canCoder.getVelocity();
    }
    
}
