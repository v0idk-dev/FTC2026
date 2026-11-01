package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThroughBoreEncoder {
    public static final double ticks_for_rev = 8192.0;

    private final DcMotorEx encoder;
    private final double gearRatio;

    public ThroughBoreEncoder(HardwareMap hwMap, String configName) {
        this(hwMap, configName, 1.0, false);
    }

    public ThroughBoreEncoder(HardwareMap hwMap, String configName,
                              double gearRatio, boolean reversed) {
        this.encoder = hwMap.get(DcMotorEx.class, configName);
        this.gearRatio = gearRatio;

        if (reversed) {
            encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            encoder.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        reset();
    }

    public void reset() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getTicks() {
        return encoder.getCurrentPosition();
    }

    public double getRevolutions() {
        return getTicks() / ticks_for_rev / gearRatio;
    }

    public double getContinuousAngleDegrees() {
        return getRevolutions() * 360.0;
    }

    public double getAngleDegrees() {
        double angle = getContinuousAngleDegrees() % 360.0;

        if (angle < 0) {
            return angle + 360.0;
        } else {
            return angle;
        }
    }

    public double getVelocityTicksPerSec() {
        return encoder.getVelocity();
    }

    public double getVelocityDegreesPerSec() {
        return getVelocityTicksPerSec() / ticks_for_rev / gearRatio * 360.0;
    }
}