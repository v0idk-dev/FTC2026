package org.firstinspires.ftc.teamcode.utilities;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretAligner {
    private CRServo servo;
    private DcMotor encoder;

    private int centerTicks;
    private int min_ticks; // TODO: set mininum tick
    private int max_ticks; // TODO: set maximum tick

    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double max_power = 0.6;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        encoder = hwMap.get(DcMotor.class, "turret_encoder");
        centerTicks = encoder.getCurrentPosition();
        servo = hwMap.get(CRServo.class, "turret_servo");
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
        double error = goalX - curID.ftcPose.bearing;
        double pTerm = error * kP; // p is proportional to how much error there is

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD; // fancy d stuff

        }

        if (Math.abs(error) < angleTolerance) { // if error is less than tolerate angle, stop (fulfilled)
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -max_power, max_power); // clips the min and max power the servo can go
        }

        // safety control to not go over the min/max tick boundaries
        int ticks = getTurretTicks();
        if ((ticks >= max_ticks && power > 0) || (ticks <= min_ticks && power < 0)) {
            power = 0;
        }

        servo.setPower(power);
        lastError = error;
    }

    public int getTurretTicks() {
        return encoder.getCurrentPosition() - centerTicks;
    }

}
