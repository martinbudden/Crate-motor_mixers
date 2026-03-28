# Slew Rate Recomendations

Recommendations for a hobby-grade brushless motors (BLDC) on a quadcopter.

The goal is to balance aggressive flight performance with the physical protection of the ESCs and motor windings.

These values are starting points for a standard 5-inch racing or freestyle quadcopter:

| Value (per second)      | Typical Result                               |
| ----------------------- | -------------------------------------------- |
| rise rate 4.0 to 10.0   | Reaches 100% throttle in 0.1 to 0.25 seconds |
| fall rate 20.0 to 100.0 | Reaches 0% throttle in 0.01 to 0.05 seconds  |

## Rise rate: 4.0 to 10.0 per second

**Why:** Brushless motors draw massive inrush current during rapid acceleration.
If you jump from 0% to 100% instantly, you risk "desyncing" the ESC or blowing a MOSFET due to current spikes.

**Performance:** A rate of 10.0 (0.1s to full power) is very "punchy" and suitable for racing.
A rate of 4.0 (0.25s) feels smoother and is safer for larger, heavier propellers that have more inertia.

## Fall Rate : 20.0 to 100.0 per second

**Why:** In a quadcopter, "Active Braking" (Damped Light) is used to slow down the props quickly for better control.
You generally want the fall rate to be much faster than the rise rate to ensure the drone can drop altitude or stop a rotation immediately in an emergency.

**Hardware Safety:** Falling too fast can occasionally cause voltage spikes back into the battery (regenerative braking),
but most modern Hobbywing or BLHeli_32 ESCs handle this well. Setting this to 100.0 essentially makes it "instant" for 100Hz loops.

## Tuning Strategy

**Check for "Kicking":** If the drone "jerks" or you hear a sharp "crack" sound from the motors on punch-outs, decrease the rise_rate_per_s.

**Check for "Floating":** If the drone feels like it is "sliding" on ice when you drop the throttle, increase the fall_rate_per_s.

**ESC Protection:** If your ESCs are getting hot but your motors are cool, your slew rates might be too aggressive for the PWM frequency you've chosen.
