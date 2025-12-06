package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tests.BlueTeleOp;

@TeleOp
public class RedTeleOp extends BlueTeleOp {
    protected int getApriltagAimingPipeline() {
        return 1;
    }

}
