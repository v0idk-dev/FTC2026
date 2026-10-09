package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class Servor extends LinearOpMode{
    private Servo turretServo;

    @Override
    public void runOpMode() {
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_left) {
                turretServo.setPosition(0.0); // Left limit
            } else if (gamepad1.dpad_right) {
                turretServo.setPosition(1.0); // Right limit
            } else if (gamepad1.dpad_up) {
                turretServo.setPosition(0.5); // Center
            }

        }
    }

}
