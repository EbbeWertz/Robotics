De arm is een child solid van de turtlebot, dus moet in webots zelg geen robot zijn, dus:
 - delete: field  SFString    controller      "void"               # Is `Robot.controller`.
 - delete: field  MFString    controllerArgs  []                   # Is `Robot.controllerArgs`.
 - delete:   field  SFBool      supervisor      FALSE                # Is `Robot.supervisor`.
 - replace: Robot, met: Solid
 - delete: controller IS controller
 - delete: controllerArgs IS controllerArgs
 - delete: supervisor IS supervisor

 OOK IN ALLEBEI:
 - linux paths naar meshes vervangen met windows paths