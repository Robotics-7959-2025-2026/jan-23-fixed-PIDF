package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsFarRed;

@Autonomous(name = "Hunter Red Far", preselectTeleOp = "TeleOp")

public class HunterFarAutoRed extends FarAuto {
    @Override
    public Paths getPaths() {
        return new PathsFarRed(pedro);
    }
}
