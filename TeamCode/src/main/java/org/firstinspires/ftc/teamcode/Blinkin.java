package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.annotation.SuppressLint;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp(name="LED")


public class Blinkin extends OpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevColorSensorV3 colorSensor;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void init()
    {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sampleColor");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
        colorSensor.setGain(2);
    }
    @SuppressLint("Range")
    @Override
    public void loop() {
        int myColor = colorSensor.getNormalizedColors().toColor();
        float hue;

        hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));

        telemetry.addData("Hue", hue);
        telemetry.update();
        blinkinLedDriver.setPattern(pattern);

        //TWINKLES_FOREST_PALLETTE is not very good, it's mostly blank and has a lot of white mixed in
        // COLOR_WAVES_FOREST_PALLETTE is pretty good, I like it.
        //RAINBOW_FOREST_PALLETTE might look nicer on a larger matrix but not on what we have
        if (hue == 0){
            pattern = RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE;
        } else if (hue < 30 || hue > 330){
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else if (hue < 90){
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        } else if (hue < 170) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        } else if (hue < 300){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        }

    }
}
