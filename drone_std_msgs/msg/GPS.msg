uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix
int32 lat      # degE7
int32 lon      # degE7
int32 alt      # mm
uint16 eph     # m * 100
uint16 epv     # m * 100
uint16 vel     # cm/s
int16 vn       # cm/s
int16 ve       # cm/s
int16 vd       # cm/s
uint16 cog     # cdeg
uint8 satellites_visible