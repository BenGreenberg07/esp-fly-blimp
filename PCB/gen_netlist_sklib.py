from collections import defaultdict
from skidl import Pin, Part, Alias, SchLib, SKIDL, TEMPLATE

from skidl.pin import pin_types

SKIDL_lib_version = '0.0.1'

gen_netlist = SchLib(tool=SKIDL).add_parts(*[
        Part(**{ 'name':'XIAO_ESP32S3', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'XIAO_ESP32S3'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'XIAO:XIAO-ESP32S3-SMD', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='D0_GPIO1',func=pin_types.BIDIR),
            Pin(num='2',name='D1_GPIO2',func=pin_types.BIDIR),
            Pin(num='3',name='D2_GPIO3',func=pin_types.BIDIR),
            Pin(num='4',name='D3_GPIO4',func=pin_types.BIDIR),
            Pin(num='5',name='D4_GPIO5_SDA',func=pin_types.BIDIR),
            Pin(num='6',name='D5_GPIO6_SCL',func=pin_types.BIDIR),
            Pin(num='7',name='D6_GPIO43_TX',func=pin_types.BIDIR),
            Pin(num='8',name='D7_GPIO44_RX',func=pin_types.BIDIR),
            Pin(num='9',name='D8_GPIO7',func=pin_types.BIDIR),
            Pin(num='10',name='D9_GPIO8',func=pin_types.BIDIR),
            Pin(num='11',name='D10_GPIO9',func=pin_types.BIDIR),
            Pin(num='12',name='V3V3',func=pin_types.BIDIR),
            Pin(num='13',name='GND_1',func=pin_types.BIDIR),
            Pin(num='14',name='V5V',func=pin_types.BIDIR),
            Pin(num='15',name='BAT',func=pin_types.BIDIR),
            Pin(num='16',name='GND_2',func=pin_types.BIDIR),
            Pin(num='17',name='MTDI',func=pin_types.BIDIR),
            Pin(num='18',name='MTDO',func=pin_types.BIDIR),
            Pin(num='19',name='CHIP_EN',func=pin_types.BIDIR),
            Pin(num='20',name='GND_3',func=pin_types.BIDIR),
            Pin(num='21',name='MTMS',func=pin_types.BIDIR),
            Pin(num='22',name='MTCK',func=pin_types.BIDIR),
            Pin(num='23',name='USB_DN',func=pin_types.BIDIR),
            Pin(num='24',name='USB_DP',func=pin_types.BIDIR),
            Pin(num='25',name='PAD',func=pin_types.BIDIR)] }),
        Part(**{ 'name':'MPU-6050', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'MPU-6050'}), 'ref_prefix':'U', 'fplist':['Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm'], 'footprint':'Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.15x2.15mm', 'keywords':'mems', 'description':'InvenSense 6-Axis Motion Sensor, Gyroscope, Accelerometer, I2C', 'datasheet':'https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf', 'pins':[
            Pin(num='24',name='SDA',func=pin_types.BIDIR,unit=1),
            Pin(num='23',name='SCL',func=pin_types.INPUT,unit=1),
            Pin(num='9',name='AD0',func=pin_types.INPUT,unit=1),
            Pin(num='11',name='FSYNC',func=pin_types.INPUT,unit=1),
            Pin(num='1',name='CLKIN',func=pin_types.INPUT,unit=1),
            Pin(num='2',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='3',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='4',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='5',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='14',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='8',name='VLOGIC',func=pin_types.PWRIN,unit=1),
            Pin(num='18',name='GND',func=pin_types.PWRIN,unit=1),
            Pin(num='13',name='VDD',func=pin_types.PWRIN,unit=1),
            Pin(num='15',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='16',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='17',name='NC',func=pin_types.NOCONNECT,unit=1),
            Pin(num='21',name='RESV',func=pin_types.NOCONNECT,unit=1),
            Pin(num='19',name='RESV',func=pin_types.NOCONNECT,unit=1),
            Pin(num='22',name='RESV',func=pin_types.NOCONNECT,unit=1),
            Pin(num='12',name='INT',func=pin_types.OUTPUT,unit=1),
            Pin(num='6',name='AUX_DA',func=pin_types.BIDIR,unit=1),
            Pin(num='7',name='AUX_CL',func=pin_types.OUTPUT,unit=1),
            Pin(num='20',name='CPOUT',func=pin_types.PASSIVE,unit=1),
            Pin(num='10',name='REGOUT',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] }),
        Part(**{ 'name':'R', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'R'}), 'ref_prefix':'R', 'fplist':[''], 'footprint':'Resistor_SMD:R_0402_1005Metric', 'keywords':'R res resistor', 'description':'Resistor', 'datasheet':'~', 'pins':[
            Pin(num='1',name='~',func=pin_types.PASSIVE,unit=1),
            Pin(num='2',name='~',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] }),
        Part(**{ 'name':'C', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'C'}), 'ref_prefix':'C', 'fplist':[''], 'footprint':'Capacitor_SMD:C_0402_1005Metric', 'keywords':'cap capacitor', 'description':'Unpolarized capacitor', 'datasheet':'~', 'pins':[
            Pin(num='1',name='~',func=pin_types.PASSIVE,unit=1),
            Pin(num='2',name='~',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] }),
        Part(**{ 'name':'Q_NMOS_GSD', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'Q_NMOS_GSD'}), 'ref_prefix':'Q', 'fplist':[''], 'footprint':'Package_TO_SOT_SMD:SOT-23', 'keywords':'transistor NMOS N-MOS N-MOSFET', 'description':'N-MOSFET transistor, gate/source/drain', 'datasheet':'~', 'pins':[
            Pin(num='1',name='G',func=pin_types.INPUT,unit=1),
            Pin(num='3',name='D',func=pin_types.PASSIVE,unit=1),
            Pin(num='2',name='S',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] }),
        Part(**{ 'name':'Conn_01x02_Pin', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'Conn_01x02_Pin'}), 'ref_prefix':'J', 'fplist':[''], 'footprint':'Connector_Wire:SolderWire-0.127sqmm_1x02_P3.7mm_D0.48mm_OD1mm', 'keywords':'connector', 'description':'Generic connector, single row, 01x02, script generated', 'datasheet':'~', 'pins':[
            Pin(num='1',name='Pin_1',func=pin_types.PASSIVE,unit=1),
            Pin(num='2',name='Pin_2',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] }),
        Part(**{ 'name':'LED', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'LED'}), 'ref_prefix':'D', 'fplist':[''], 'footprint':'LED_SMD:LED_0402_1005Metric', 'keywords':'LED diode', 'description':'Light emitting diode', 'datasheet':'~', 'pins':[
            Pin(num='1',name='K',func=pin_types.PASSIVE,unit=1),
            Pin(num='2',name='A',func=pin_types.PASSIVE,unit=1)], 'unit_defs':[] })])