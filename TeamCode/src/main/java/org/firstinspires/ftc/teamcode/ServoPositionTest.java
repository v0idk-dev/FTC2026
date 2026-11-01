package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.EncoderPositionController;
import org.firstinspires.ftc.teamcode.ThroughBoreEncoder;

// to control turret servo with encoder
@TeleOp(name = "Servo Position Test")
public class ServoPositionTest extends OpMode {
    ThroughBoreEncoder encoder;
    EncoderPositionController controller;

    @Override
    public void init() {
        encoder = new ThroughBoreEncoder(hardwareMap, "bore_encoder");
        CRServo servo = hardwareMap.get(CRServo.class, "turret_servo");
        controller = new EncoderPositionController(encoder, servo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            controller.setTarget(0);
        }

        if (gamepad1.b) {
            controller.setTarget(90);
        }

        if (gamepad1.x) {
            controller.setTarget(120);
        }

        if (gamepad1.y) {
            controller.setTarget(200);
        }

        controller.update();

        telemetry.addData("Angle", encoder.getContinuousAngleDegrees());
        telemetry.addData("Currently at", controller.atTarget());
        telemetry.update();
    }
}