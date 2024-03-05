package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test: Systems", group = "Test")
public class SystemsTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) lift.setPower(1);
            if (gamepad1.dpad_down) lift.setPower(-1);
            if (!gamepad1.dpad_up && !gamepad1.dpad_down) lift.setPower(0);
        }
    }
}
