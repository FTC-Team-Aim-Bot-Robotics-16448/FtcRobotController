package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aim.components.Menu;

@TeleOp(name = "Menu Test", group = "Test")
public class MenuTestTeleOp extends LinearOpMode {
    private Menu menu;

    @Override
    public void runOpMode() {
        menu = new Menu();

        menu.addMenuItem("Auto/Blue/Left#1");
        menu.addMenuItem("Auto/Blue/Right#2");
        menu.addMenuItem("Auto/Red/Left#3");
        menu.addMenuItem("Auto/Red/Right#4");
        menu.addMenuItem("Settings/Speed/Fast");
        menu.addMenuItem("Settings/Speed/Medium");
        menu.addMenuItem("Settings/Speed/Slow");
        menu.addMenuItem("Settings/Alliance/Blue#ally");
        menu.addMenuItem("Settings/Alliance/Red#ally");
        menu.addMenuItem("Start Default");

        telemetry.addLine("Menu Test Initialized");
        telemetry.addLine("Use gamepad to select options");
        telemetry.update();

        String selectedOption = menu.show(gamepad1, telemetry);

        telemetry.clear();
        telemetry.addData("Selected Option", selectedOption);
        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();

        if (selectedOption != null) {
            telemetry.clear();
            telemetry.addData("Selected Option", selectedOption);
            telemetry.addLine("---");
            telemetry.addLine("Menu selection complete!");
            telemetry.update();

            sleep(3000);
        }
    }
}
