package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

/**
 * Quadrature encoders are relative encoders, meaning they provide a rate of
 * turn, but are not ideal for knowing the absolute position at any time.
 * Absolute position is determined by simply adding the pulses received.
 */
public class QuadEncoder implements IEncoder {

    Encoder m_encoder;

    public QuadEncoder(int chanA, int chanB, boolean inverted) {
        m_encoder = new Encoder(chanA, chanB, inverted);
        // TODO: Configure for the encoder we get
        
    }

    @Override
    public double getDistanceDegrees() {
        return m_encoder.getDistance();
    }

    @Override
    public double getRateDegreesPerSecond() {
        return m_encoder.getRate();
    }

}
