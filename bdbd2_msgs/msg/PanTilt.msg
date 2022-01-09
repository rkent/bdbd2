# Message to set the pan and tilt values for a bracket (typically with a camera)

# pan in degrees. Center is 90, positive is counter-clockwise looking down.
float32 pan

# tilt in degrees. Center is 45, < 45 is looking up.
float32 tilt

# (pan, tilt) are raw sensor values (True), or assume a calibration to center (False)
bool raw
