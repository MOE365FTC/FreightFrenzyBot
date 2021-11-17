package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class Toggle {
    public static boolean oldA = false;
    public static boolean aToggled = false;
    public static boolean oldY = false;
    public static boolean yToggled = false;
    public static boolean oldB = false;
    public static boolean bToggled = false;
    public static boolean oldX = false;
    public static boolean xToggled = false;
    public static boolean rbToggled = false;
    public static boolean oldRB = false;
    public static boolean lbToggled = false;
    public static boolean oldLB = false;
    public static boolean a2Toggled = false;
    public static boolean olda2 = false;
    public static boolean dpadUpToggled = false;
    public static boolean oldDpadUp = false;
    public static boolean dpadDownToggled = false;
    public static boolean oldDpadDown = false;
    public static boolean dpadLeftToggled = false;
    public static boolean oldDpadLeft = false;
    public static boolean dpadRightToggled = false;
    public static boolean oldDpadRight = false;
    public static boolean dpadLeft2Toggled = false;
    public static boolean oldDpadLeft2 = false;
    public static boolean oldY2 = false;
    public static boolean y2Toggled = false;
    public static void handleToggles() {
        if (gamepad1.a && !oldA) aToggled = !aToggled;
        oldA = gamepad1.a;
        if (gamepad1.y && !oldY) yToggled = !yToggled;
        oldY = gamepad1.y;
        if (gamepad1.b && !oldB) bToggled = !bToggled;
        oldB = gamepad1.b;
        if (gamepad1.right_bumper && !oldRB) rbToggled = !rbToggled;
        oldRB = gamepad1.right_bumper;
        if (gamepad1.left_bumper && !oldLB) lbToggled = !lbToggled;
        oldLB = gamepad1.left_bumper;
        if (gamepad2.a && !olda2) a2Toggled = !a2Toggled;
        olda2 = gamepad2.a;
        if (gamepad1.x && !oldX) xToggled = !xToggled;
        oldX = gamepad1.x;
        if (gamepad1.dpad_up && !oldDpadUp) dpadUpToggled = !dpadUpToggled;
        oldDpadUp = gamepad1.dpad_up;
        if (gamepad1.dpad_down && !oldDpadDown) dpadDownToggled = !dpadDownToggled;
        oldDpadDown = gamepad1.dpad_down;
        if (gamepad1.dpad_left && !oldDpadLeft) dpadLeftToggled = !dpadLeftToggled;
        oldDpadLeft = gamepad1.dpad_left;
        if (gamepad1.dpad_right && !oldDpadRight) dpadRightToggled = !dpadRightToggled;
        oldDpadRight = gamepad1.dpad_right;
        if (gamepad2.dpad_left && !oldDpadLeft2) dpadLeft2Toggled = !dpadLeft2Toggled;
        oldDpadLeft2 = gamepad2.dpad_left;
        if (gamepad2.y && !oldY2) y2Toggled = !y2Toggled;
        oldY2 = gamepad2.y;
    }
}
