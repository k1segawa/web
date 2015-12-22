#coding: utf-8

import webiopi
import datetime
from webiopi.devices.analog.mcp3x0x import MCP3002

import smbus
import time

#webiopi.setDebug()

mcp = MCP3002()

GPIO = webiopi.GPIO

bus_number  = 1 
i2c_address = 0x76

bus = smbus.SMBus(bus_number)

digT = []
digP = []
digH = []

t_fine = 0.0 

G_TEMP = 0.0
G_PRES = 0.0
G_HUMI = 0.0

LIGHT = 17 # GPIO pin using BCM numbering

HOUR_ON  = 8  # Turn Light ON at 08:00
HOUR_OFF = 18 # Turn Light OFF at 18:00

# setup function is automatically called at WebIOPi startup
def setup():
    # set the GPIO used by the light to output
    GPIO.setFunction(LIGHT, GPIO.OUT)

    # retrieve current datetime
    now = datetime.datetime.now()
    webiopi.debug ("-----")
    webiopi.debug (now)
    webiopi.debug ("-----")

    # test if we are between ON time and tun the light ON
    if ((now.hour >= HOUR_ON) and (now.hour < HOUR_OFF)):
        GPIO.digitalWrite(LIGHT, GPIO.HIGH)
    i2c_setup()
    get_calib_param()

# loop function is repeatedly called by WebIOPi 
def loop():
    # retrieve current datetime
    now = datetime.datetime.now()

    #/if((HOUR_ON >= 8) and (HOUR_OFF <=22)):
    #/    webiopi.debug( "ON 8======-->OFF 22" )
    #/else:
    #/    webiopi.debug( "OXXXXXXXXX" )

    # toggle light ON all days at the correct time
    if ((now.hour == HOUR_ON) and (now.minute == 0) and (now.second == 0)):
        if (GPIO.digitalRead(LIGHT) == GPIO.LOW):
            GPIO.digitalWrite(LIGHT, GPIO.HIGH)

    # toggle light OFF
    if ((now.hour == HOUR_OFF) and (now.minute == 0) and (now.second == 0)):
        if (GPIO.digitalRead(LIGHT) == GPIO.HIGH):
            GPIO.digitalWrite(LIGHT, GPIO.LOW)

    # gives CPU some time before looping again
    webiopi.sleep(1)

# destroy function is called at WebIOPi shutdown
def destroy():
    GPIO.digitalWrite(LIGHT, GPIO.LOW)

# http://webiopi.trouch.com/Tutorial_Macros.html (append segawa)
@webiopi.macro
def getLightHours():
    return "%d;%d" % (HOUR_ON, HOUR_OFF)

@webiopi.macro
def setLightHours(on, off):
    global HOUR_ON, HOUR_OFF
    HOUR_ON = int(on)
    HOUR_OFF = int(off)
    return getLightHours()

@webiopi.macro
def mcp3002AnalogRead():
    v = mcp.analogRead(0)
    webiopi.debug( "---------------Ch0 : " + str(v) )
    return str(v)

@webiopi.macro
def mcp3002AnalogRead1():
    v = mcp.analogRead(1)
    webiopi.debug( "---------------Ch1 : " + str(v) )
    return str(v)

def writeReg(reg_address, data):
    bus.write_byte_data(i2c_address,reg_address,data)

def get_calib_param():
    calib = []

    for i in range (0x88,0x88+24):
        calib.append(bus.read_byte_data(i2c_address,i))
    calib.append(bus.read_byte_data(i2c_address,0xA1))
    for i in range (0xE1,0xE1+7):
        calib.append(bus.read_byte_data(i2c_address,i))

    digT.append((calib[1] << 8) | calib[0])
    digT.append((calib[3] << 8) | calib[2])
    digT.append((calib[5] << 8) | calib[4])
    digP.append((calib[7] << 8) | calib[6])
    digP.append((calib[9] << 8) | calib[8])
    digP.append((calib[11]<< 8) | calib[10])
    digP.append((calib[13]<< 8) | calib[12])
    digP.append((calib[15]<< 8) | calib[14])
    digP.append((calib[17]<< 8) | calib[16])
    digP.append((calib[19]<< 8) | calib[18])
    digP.append((calib[21]<< 8) | calib[20])
    digP.append((calib[23]<< 8) | calib[22])
    digH.append( calib[24] )
    digH.append((calib[26]<< 8) | calib[25])
    digH.append( calib[27] )
    digH.append((calib[28]<< 4) | (0x0F & calib[29]))
    digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
    digH.append( calib[31] )

    for i in range(1,2):
        if digT[i] & 0x8000:
            digT[i] = (-digT[i] ^ 0xFFFF) + 1

    for i in range(1,8):
        if digP[i] & 0x8000:
            digP[i] = (-digP[i] ^ 0xFFFF) + 1

    for i in range(0,6):
        if digH[i] & 0x8000:
            digH[i] = (-digH[i] ^ 0xFFFF) + 1
    webiopi.debug( "---------------get_calib_param : called" )

@webiopi.macro
def i2cReadData():
    global G_TEMP, G_PRES, G_HUMI
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8)  |  data[7]

    compensate_T(temp_raw)
    compensate_H(hum_raw)
    compensate_P(pres_raw)
    #print "%-6.2f," % (temperature),
    #print "%7.2f" % (pressure/100)
    #print "%6.2f," % (var_h),
    webiopi.debug( "---------------I2C : " + str("%-6.2f;%7.2f;%6.2f" % (G_TEMP, G_HUMI, G_PRES)) )
    return "%-6.2f;%7.2f;%6.2f" % (G_TEMP, G_HUMI, G_PRES)

def compensate_P(adc_P):
    global  t_fine
    global G_TEMP, G_PRES, G_HUMI
    pressure = 0.0

    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
    v1 = ((32768 + v1) * digP[0]) / 32768

    if v1 == 0:
        return 0
    pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)

    #print "pressure : %7.2f hPa" % (pressure/100)
    #print "%7.2f" % (pressure/100)
    webiopi.debug( "---------------I2C PRES: " + str("%7.2f" % G_PRES) )
    G_PRES = pressure/100

def compensate_T(adc_T):
    global G_TEMP, G_PRES, G_HUMI
    global t_fine
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
    t_fine = v1 + v2
    temperature = t_fine / 5120.0
    #print "temp : %-6.2f ℃" % (temperature) 
    #print "%-6.2f," % (temperature),
    webiopi.debug( "---------------I2C TEMP: " + str("%-6.2f" % G_TEMP) )
    G_TEMP = temperature

def compensate_H(adc_H):
    global G_TEMP, G_PRES, G_HUMI
    global t_fine
    var_h = t_fine - 76800.0
    if var_h != 0:
        var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
    else:
        return 0
    var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
    if var_h > 100.0:
        var_h = 100.0
    elif var_h < 0.0:
        var_h = 0.0
    #print "hum : %6.2f ％" % (var_h)
    #print "%6.2f," % (var_h),
    G_HUMI = var_h


def i2c_setup():
    osrs_t = 1                      #Temperature oversampling x 1
    osrs_p = 1                      #Pressure oversampling x 1
    osrs_h = 1                      #Humidity oversampling x 1
    mode   = 3                      #Normal mode
    t_sb   = 5                      #Tstandby 1000ms
    filter = 0                      #Filter off
    spi3w_en = 0                    #3-wire SPI Disable

    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
    config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
    ctrl_hum_reg  = osrs_h

    writeReg(0xF2,ctrl_hum_reg)
    writeReg(0xF4,ctrl_meas_reg)
    writeReg(0xF5,config_reg)
    webiopi.debug( "---------------i2c_setup : called" )
