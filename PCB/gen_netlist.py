import os
os.environ["KICAD8_SYMBOL_DIR"] = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"
from skidl import *
set_default_tool(KICAD8)
lib_search_paths[KICAD8].append("/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols")

# ---- nets ----
vbat = Net('VBAT'); gnd = Net('GND'); v3 = Net('+3V3')
sda = Net('I2C_SDA'); scl = Net('I2C_SCL'); intp = Net('MPU_INT'); vsense = Net('VBAT_SENSE')
m = [Net(f'MOT{i}') for i in range(1,5)]
ledn = {c: Net(f'LED_{c}') for c in ('R','G','B')}

# ---- XIAO ESP32-S3 as a custom part (pads -> GPIO per verified map) ----
# XIAO ESP32-S3: pin NUMBERS = footprint pad numbers (from OPL symbol).
XIAO = Part(name='XIAO_ESP32S3', tool=SKIDL, dest=TEMPLATE, ref_prefix='U',
            footprint='XIAO:XIAO-ESP32S3-SMD')
xpins = {
  1:'D0_GPIO1', 2:'D1_GPIO2', 3:'D2_GPIO3', 4:'D3_GPIO4',
  5:'D4_GPIO5_SDA', 6:'D5_GPIO6_SCL', 7:'D6_GPIO43_TX', 8:'D7_GPIO44_RX',
  9:'D8_GPIO7', 10:'D9_GPIO8', 11:'D10_GPIO9', 12:'V3V3', 13:'GND_1',
  14:'V5V', 15:'BAT', 16:'GND_2', 17:'MTDI', 18:'MTDO', 19:'CHIP_EN',
  20:'GND_3', 21:'MTMS', 22:'MTCK', 23:'USB_DN', 24:'USB_DP', 25:'PAD',
}
for num,name in xpins.items():
    XIAO += Pin(num=str(num), name=name, func=Pin.types.BIDIR)
u = XIAO()
u[12]+=v3                                  # 3V3 out -> logic
u[13]+=gnd; u[16]+=gnd; u[20]+=gnd; u[25]+=gnd
u[15]+=vbat                                # BAT pad = battery input
u[1]+=m[0]; u[2]+=m[1]; u[3]+=m[2]; u[4]+=m[3]
u[5]+=sda; u[6]+=scl; u[9]+=intp; u[10]+=vsense
u[11]+=ledn['R']; u[7]+=ledn['G']; u[8]+=ledn['B']

# ---- IMU ----
imu = Part('Sensor_Motion','MPU-6050', footprint='Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.15x2.15mm')
imu['VDD']+=v3; imu['VLOGIC']+=v3; imu['GND']+=gnd
imu['SDA']+=sda; imu['SCL']+=scl; imu['INT']+=intp; imu['AD0']+=gnd
Rsda=Part('Device','R',value='10k',footprint='Resistor_SMD:R_0402_1005Metric'); Rsda[1]+=v3; Rsda[2]+=sda
Rscl=Part('Device','R',value='10k',footprint='Resistor_SMD:R_0402_1005Metric'); Rscl[1]+=v3; Rscl[2]+=scl
Cimu=Part('Device','C',value='100n',footprint='Capacitor_SMD:C_0402_1005Metric'); Cimu[1]+=v3; Cimu[2]+=gnd

# ---- 4 motor drivers (generic NMOS GSD) ----
for i in range(4):
    q=Part('Transistor_FET','Q_NMOS_GSD',value='SI2302',footprint='Package_TO_SOT_SMD:SOT-23')
    rg=Part('Device','R',value='100',footprint='Resistor_SMD:R_0402_1005Metric')
    rpd=Part('Device','R',value='10k',footprint='Resistor_SMD:R_0402_1005Metric')
    mp=Part('Connector','Conn_01x02_Pin',value=f'Motor{i+1}',footprint='Connector_Wire:SolderWire-0.127sqmm_1x02_P3.7mm_D0.48mm_OD1mm')
    rg[1]+=m[i]; rg[2]+=q['G']; rpd[1]+=q['G']; rpd[2]+=gnd
    q['S']+=gnd; q['D']+=mp[2]; mp[1]+=vbat   # motor + to VBAT, - to drain

# ---- bulk caps on VBAT ----
for _ in range(2):
    cb=Part('Device','C',value='22u',footprint='Capacitor_SMD:C_0805_2012Metric'); cb[1]+=vbat; cb[2]+=gnd

# ---- battery sense divider -> ADC ----
rt=Part('Device','R',value='100k',footprint='Resistor_SMD:R_0402_1005Metric'); rt[1]+=vbat; rt[2]+=vsense
rb=Part('Device','R',value='100k',footprint='Resistor_SMD:R_0402_1005Metric'); rb[1]+=vsense; rb[2]+=gnd

# ---- battery JST ----
jst=Part('Connector','Conn_01x02_Pin',value='JST-PH 1S',footprint='Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal')
jst[1]+=vbat; jst[2]+=gnd

# ---- LEDs: 3 status (GPIO) + 2 white (3V3) ----
for c in ('R','G','B'):
    d=Part('Device','LED',value=c,footprint='LED_SMD:LED_0402_1005Metric')
    r=Part('Device','R',value='150',footprint='Resistor_SMD:R_0402_1005Metric')
    v3 & r & d['A','K'] if False else None
    r[1]+=v3; r[2]+=d['A']; d['K']+=ledn[c]
for _ in range(2):
    d=Part('Device','LED',value='white',footprint='LED_SMD:LED_0603_1608Metric')
    r=Part('Device','R',value='150',footprint='Resistor_SMD:R_0402_1005Metric')
    r[1]+=v3; r[2]+=d['A']; d['K']+=gnd

ERC()
generate_netlist(file_='esp_fly_clone.net')
print("NETLIST GENERATED")
