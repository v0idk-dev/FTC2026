package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretAligner {
    private CRServo servo;
    private AnalogInput encoder;

    private int centerTicks = 0;
    private int min_degrees = 0; // TODO: set mininum tick
    private int max_degrees = 18000; // TODO: set maximum tick

    private double kP = 0.1;
    private double kD = 0.01;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double max_power = 0.6;
    private double power = 0;

    private double error;
    private double pTerm;
    private double dTerm;

    public double getError() {
        return error;
    }

    public double getPTerm() {
        return pTerm;
    }

    public double getDTerm() {
        return dTerm;
    }

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {

        encoder = hwMap.get(AnalogInput.class, "turret_encoder");
        servo = hwMap.get(CRServo.class, "mvR");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setkP(double newkP) {
        kP = newkP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newkD) {
        kD = newkD;
    }

    public double getkD() {
        return kD;
    }

    public double getPower() {
        return power;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(AprilTagDetection curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (curID == null || curID.ftcPose == null) {
            servo.setPower(0);
            lastError = 0;
            return;
        }

        // start PD controller
        error = goalX - curID.ftcPose.bearing;
        pTerm = error * kP;

        dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD; // fancy d stuff
        }

        if (Math.abs(error) < angleTolerance) { // if error is less than tolerate angle, stop (fulfilled)
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -max_power, max_power); // clips the min and max power the servo can go
        }

         // safety control to not go over the min/max tick boundaries
        double degrees = getEncoderDegrees();

        if (!(min_degrees < max_degrees)) {
            power = 0;
        } else if ((degrees >= max_degrees && power > 0) ||
                (degrees <= min_degrees && power < 0)) {
            power = 0;
        }

        servo.setPower(power);
        lastError = error;

    }

    public double getEncoderDegrees() {
        return encoder.getVoltage() * 360.0 / 3.3;
    }

}
