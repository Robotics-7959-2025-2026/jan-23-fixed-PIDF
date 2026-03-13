package org.firstinspires.ftc.teamcode.Autonomous.FarSide_Unused.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.FarSide_Unused.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide_Unused.Paths.FarPathsBlue;

//Aries spears: "this fellas a homosexual"
@Autonomous(name = "Keir Far Blue", preselectTeleOp = "TeleOp")
public class FarAutoBlue extends FarAuto {
    @Override
    public FarPaths getPaths() {
        return new FarPathsBlue(pedro);
    }
}
