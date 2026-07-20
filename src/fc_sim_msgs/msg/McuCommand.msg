# Companion (Jetson) -> MCU high-level mission command. Mirrors the mission
# downlink frame in fc_core/protocol.h (fc_proto_mission_t) field-for-field,
# with the byte codec (Q14 / cm quantization, flag packing) applied at the
# boundary in fc_sim_node. All errors are metric/radian SI; the Jetson has
# already converted pixels with altitude/f.

# ControlMode (MISSION_INTERFACE.md section 1); carried in `mode` (no arm bit).
uint8 MODE_HOLD=0
uint8 MODE_FOLLOW_LINE=1
uint8 MODE_ALIGN_MARKER=2
uint8 MODE_SEARCH_LINE=3
uint8 MODE_MOVE_TO_LANDMARK=4
uint8 MODE_LAND_ON_MARKER=5
uint8 MODE_STOP=6
uint8 MODE_EMERGENCY_LAND=7

uint8 mode                  # one of MODE_* (ControlMode), no arm bit
bool arm                    # request motors armed (folded into wire mode bit 0x80)
uint8 mission_state         # MissionState, telemetry/logging
uint8 seq                   # wraps

int8 node_x                 # grid index
int8 node_y                 # grid index
uint8 move_direction        # MoveDirection: 0 X_POS 1 X_NEG 2 Y_POS 3 Y_NEG

float32 target_altitude     # m, 0..10

# Line information as the [dx, dy, flag] triple. line_dx is the nearest
# vertical line's body +y offset; line_dy the nearest horizontal line's body
# +x offset; presence per axis in vertical_line / horizontal_line.
float32 line_dx             # m, body +y (nearest vertical line)
float32 line_dy             # m, body +x (nearest horizontal line)
bool vertical_line          # line_dx valid
bool horizontal_line        # line_dy valid
float32 line_angle_error    # rad, FLU +CCW (followed line vs travel axis)
float32 line_confidence     # 0..1

bool intersection_detected
bool intersection_forward
bool intersection_left
bool intersection_right
bool intersection_backward

bool marker_detected
int8 marker_id              # -1 = none
float32 marker_error_x      # m, body +x
float32 marker_error_y      # m, body +y
float32 marker_yaw_error    # rad
float32 marker_confidence   # 0..1

float32 vx_est              # m/s, body +x
float32 vy_est              # m/s, body +y
bool vel_est_valid

bool emergency

# Cruise speed scale, percent 0..100 (wire decode clamps above 100). Scales the
# MCU cruise in FOLLOW_LINE/SEARCH_LINE/MOVE_TO_LANDMARK; 100 = full cruise. The
# mission always sets this; an unset 0 creeps to a stop, a safe sim default.
uint8 speed_scale
