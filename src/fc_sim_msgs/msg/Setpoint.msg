# Companion -> FC setpoint. Field semantics mirror the 24-byte USART2
# downlink frame defined in fc_core/protocol.h, with the byte codec
# applied at the boundary.

uint8 MODE_DISARMED=0
uint8 MODE_PQRTHR=0
uint8 MODE_ATTITHR=1
uint8 MODE_ATTIVD=2
uint8 MODE_VEL=3
uint8 MODE_POS=4

uint8 mode               # one of the MODE_* constants above
bool arm                 # request motors armed
float32 roll_sp          # rad,  ATTI modes
float32 pitch_sp         # rad,  ATTI modes
float32 yawrate_sp       # rad/s, all modes (yaw is open-loop in firmware)
float32 vz_sp            # m/s,  ATTIVD / VEL only
float32 thrust_norm      # 0..1,  ATTITHR / PQRTHR
