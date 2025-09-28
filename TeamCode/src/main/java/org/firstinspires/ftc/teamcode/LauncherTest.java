package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LauncherTest extends LinearOpMode {
    private DcMotor launchMotor;

    @Override
    public void runOpMode() {
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                launchMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                launchMotor.setPower(-1);
            } else {
                launchMotor.setPower(0);
            }
        }
    }
}