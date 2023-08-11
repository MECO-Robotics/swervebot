package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

public class QuadEncoder implements IEncoder {

    Encoder m_encoder;

    public QuadEncoder(int chanA, int chanB, boolean inverted){
        m_encoder = new Encoder(chanA, chanB, inverted);
    }

    @Override
    public double getDistance() {
        return m_encoder.getDistance();
    }

    @Override
    public double getRate() {
        return m_encoder.getRate();
    }
    
}
