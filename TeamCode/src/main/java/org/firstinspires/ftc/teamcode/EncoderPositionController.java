package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

public class EncoderPositionController {
    private final ThroughBoreEncoder encoder;
    private final CRServo servo;

    public double kP = 0.005;
    public double maxPower = 0.4;
    public double toleranceDeg = 2.0;

    public double lastError, lastPower;
    private double targetDeg = 0;

    public EncoderPositionController(ThroughBoreEncoder encoder, CRServo servo) {
        this.encoder = encoder;
        this.servo = servo;
    }

    public void setTarget(double degrees) {
        targetDeg = degrees;
    }

    public boolean atTarget() {
        double error = targetDeg - encoder.getContinuousAngleDegrees();
        boolean withinTolerance = Math.abs(error) < toleranceDeg;
        return withinTolerance;
    }

    public void update() {
        double error = targetDeg - encoder.getContinuousAngleDegrees();
        lastError = error;

        if (Math.abs(error) < toleranceDeg) {
            lastPower = 0;
            servo.setPower(0);
            return;
        }

        double power = kP * error;
        power = Range.clip(power, -maxPower, maxPower);
        lastPower = power;
        servo.setPower(power);
    }
}