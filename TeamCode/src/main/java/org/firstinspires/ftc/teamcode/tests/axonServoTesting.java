package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class axonServoTesting extends OpMode {

    Servo axon;
    AnalogInput axonEncoder;

    @Override
    public void init() {
        axon = hardwareMap.get(Servo.class, "axon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
    }

    @Override
    public void loop() {
        axon.setPosition(1);
        telemetry.addData("Axon voltage", "%.3f V", axonEncoder.getVoltage());
    }
}
