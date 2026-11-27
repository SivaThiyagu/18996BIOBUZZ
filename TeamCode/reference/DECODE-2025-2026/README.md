# DECODE 2025–2026 reference code

Team 18996 S.T.A.T.I.C.'s Java sources from [DECODE commit `37d8daf`](https://github.com/artica1/18996DECODE/tree/37d8daf0de7d58adb23572047ca96f6312eb04cd).

In Android Studio, switch the project pane from **Android** to **Project** to find this folder. Gradle compiles the current robot code from `TeamCode/src/main/java`. This folder is outside that source tree, so its OpModes do not appear on the Driver Station.

- [FullTeleop.java](teamcode/teleops/FullTeleop.java) shows how DECODE drove and scored in the same OpMode.
- [IntakeSubsystem.java](teamcode/subsystems/IntakeSubsystem.java) and [ProgressTransferCommand.java](teamcode/automations/commands/ProgressTransferCommand.java) show intake and feeder controls.
- [Drive.java](teamcode/automations/drive/Drive.java) and [Pedro Pathing constants](teamcode/automations/pedroPathing/Constants.java) show the DECODE navigation setup. Verify the current hardware and retune before adapting it.
