# FC -> companion telemetry. Mirrors the 40-byte uplink frame defined in
# fc_core/protocol.h.

uint8 state              # opaque FC state byte
float32 roll             # rad
float32 pitch            # rad
float32 yaw              # rad
float32 p                # rad/s, body roll rate
float32 q                # rad/s, body pitch rate
float32 r                # rad/s, body yaw rate
float32 alt_lidar        # m, downward range sensor reading
float32 vbatt_volts      # V, placeholder until battery monitor lands
uint16 flag_word         # mirrors fc_core planner.h flag bitfield
