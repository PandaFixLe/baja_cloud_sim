# 超声波雷达数据消息
std_msgs/Header header

# 前方雷达 (单位: cm, 精度: 2cm, 量程: 0~510cm)
float32 front_left          # 前左探头自发自收 FL_TX_FL_RX
float32 front_left_mid      # 前左中探头自发自收 FLM_TX_FLM_RX
float32 front_right_mid     # 前右中探头自发自收 FRM_TX_FRM_RX
float32 front_right         # 前右探头自发自收 FR_TX_FR_RX
float32 front_left_corner   # 前左泊车探头自发自收 FLS_TX_FLS_RX
float32 front_right_corner  # 前右泊车探头自发自收 FRS_TX_FRS_RX

# 后方雷达 (单位: cm, 精度: 2cm, 量程: 0~510cm)
float32 rear_left           # 后左探头自发自收 RL_TX_RL_RX
float32 rear_left_mid       # 后左中探头自发自收 RLM_TX_RLM_RX
float32 rear_right_mid      # 后右中探头自发自收 RRM_TX_RRM_RX
float32 rear_right          # 后右探头自发自收 RR_TX_RR_RX
float32 rear_left_corner    # 后左泊车探头自发自收 RLS_TX_RLS_RX
float32 rear_right_corner   # 后右泊车探头自发自收 RRS_TX_RRS_RX

# 温度信息
float32 temperature         # 温度 (摄氏度)
bool temperature_ok         # 温度状态是否正常