Offseason training projects live here.

Team members should work on a branch, but in this package. 

Some good examples of starting tele-op, and drive team controls are 
in the following classes in the util.templates package which could be 
copied here to work on:

* Drivebase - Mecanum drivebase for tele-op
* TemplateDriverControlled - a basic tele-op program for a robot that uses a drive base with
    Mecanum wheels. It uses a DriverControls class with the team's "standard" controls, robot direction
    on the left stick, rotation on the right stick, fast/slow ("party mode") on the left trigger,
    inversion of controls on the right trigger. 
* TemplateDriverControls - gamepad for the driver
* TemplateOperatorControls - gamepad for the operator
* TemplateDriveTeamSignal - LED driver to signal the drive team
