package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.panels.Panels;
//import com.bylazar.panels.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.aim.components.Menu;

@TeleOp(name = "Menu Test", group = "Test")
public class MenuTestTeleOp extends LinearOpMode {
    private Menu menu;

    public static double ticksIncrement = 0.025;

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final LoopTimer timer = new LoopTimer();

    private double ticks = 0.0;
    private double wave = 0.0;
    private double wave2 = 0.0;

    private final double constant = Math.sin(0.0);
    private final double constant2 = Math.sin(0.0) + 5;

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

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

        ticks = 0.0;
        wave = 0.0;
        wave2 = 0.0;

        waitForStart();

        while (opModeIsActive()) {
            timer.start();

            ticks += ticksIncrement;
            wave = Math.sin(ticks);
            wave2 = Math.sin(ticks + Math.PI) * 2;

            panelsTelemetry.debug("wave: " + wave);
            panelsTelemetry.debug("wave2: " + wave2);
            panelsTelemetry.debug("constant: " + constant);
            panelsTelemetry.debug("constant2: " + constant2);

            panelsTelemetry.addData("wave", wave);
            panelsTelemetry.addData("wave2", wave2);
            panelsTelemetry.addData("constant", constant);
            panelsTelemetry.addData("constant2", constant2);

            //panelsTelemetry.debug("LoopTime: " + timer.ms + "ms / " + timer.hz + "Hz");

            panelsTelemetry.update(telemetry);
            timer.end();
        }


    }
}
