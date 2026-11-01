package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Through Bore Encoder Test")
public class ThroughBoreEncoderTest extends OpMode {

    // Must match the motor name you give this port in the Robot Configuration.
    private static final String name = "bore_encoder";

    ThroughBoreEncoder encoder;

    @Override
    public void init() {
        encoder = new ThroughBoreEncoder(hardwareMap, name);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            encoder.reset();
        }

        telemetry.addData("Ticks", encoder.getTicks());
        telemetry.addData("Revolutions", encoder.getRevolutions());
        telemetry.addData("Angle (0-360)", encoder.getAngleDegrees());
        telemetry.addData("Continuous angle", encoder.getContinuousAngleDegrees());
        telemetry.addData("Velocity", encoder.getVelocityDegreesPerSec());
    }
}