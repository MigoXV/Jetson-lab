# Jetson控制GPIO

## 中级实验



### 将红外传感器连接到Jetson Nano上，并读取其信号

```
import RPi.GPIO as GPIO
import time

#定义红外传感器输出引脚的GPIO编号
ir_pin = 18
使用BCM GPIO编号方案设置GPIO引脚为输入模式
GPIO.setmode(GPIO.BCM)
GPIO.setup(ir_pin, GPIO.IN)
#循环读取红外传感器的状态并输出到控制台
while True:
    ir_status = GPIO.input(ir_pin)
    print("Infrared sensor status: {}".format(ir_status))
    time.sleep(0.1) # 暂停100毫秒以避免在循环中过于频繁地读取传感器状态
```

该程序定义一个变量ir_pin，用于指定连接到Jetson Nano的GPIO引脚的编号。然后，使用RPi.GPIO库将GPIO接口设置为输入模式。

接下来，程序进入一个无限循环，并在每次迭代中读取红外传感器的状态并将其输出到控制台。由于Infrared sensor可能会发生变化，因此程序暂停100毫秒以避免重复检查。

***

### 将超声波传感器连接到Jetson Nano上，并读取其信号

```
import RPi.GPIO as GPIO
import time

# 定义超声波传感器发送脉冲引脚的GPIO编号
trig_pin = 18

# 定义超声波传感器接收回波的引脚的GPIO编号
echo_pin = 24

# 使用BCM编号方案设置GPIO引脚为输出模式（发送脉冲）
GPIO.setmode(GPIO.BCM)
GPIO.setup(trig_pin, GPIO.OUT)

# 使用BCM编号方案设置GPIO引脚为输入模式（接收回波）
GPIO.setup(echo_pin, GPIO.IN)

# 发送脉冲函数
def send_pulse():
    GPIO.output(trig_pin, True)
    time.sleep(0.00001) # 等待10微秒
    GPIO.output(trig_pin, False)
    
# 计算距离函数
def get_distance():
    send_pulse()
  
    #等待超声波返回并计算距离
    pulse_start = time.time()
    pulse_end = time.time()
  
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
  
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
  
    pulse_duration = pulse_end - pulse_start
  
    return pulse_duration * 17150 # 距离(cm) = 时间(秒) * 17150

# 循环读取超声波传感器的距离并输出到控制台
while True:
    distance = get_distance()
    print("Distance: {} cm".format(round(distance, 2)))
    time.sleep(0.1) # 暂停100毫秒以避免在循环中过于频繁地读取传感器状态
```

该程序创建了两个变量trig_pin和echo_pin，分别指定连接到Jetson Nano的GPIO引脚的编号。然后，使用RPi.GPIO库将GPIO接口设置为输出模式（用于发送脉冲）和输入模式（用于接收回波）。

接下来，程序定义一个名为send_pulse()的函数，用于向传感器发送脉冲，并定义一个名为get_distance()的函数，用于计算从传感器到回波传感器距离并返回结果。

最后，程序进入一个无限循环，并在每次迭代中调用get_distance()函数以测量超声波传感器到目标的距离，并将其输出到控制台。由于pulse sensor可能会发生变化，因此程序暂停100毫秒以避免重复检查。

***

***

### 将一个触摸传感器连接到Jetson Nano上，并读取其信号。

要将触摸传感器连接到Jetson Nano上，需要先确定传感器的型号和引脚定义，然后使用适当的电路将其连接到Jetson Nano的GPIO引脚上。大多数触摸传感器具有两个引脚或更多，通常一种用于电源和地线，其他则用于控制和读取状态。

```
import RPi.GPIO as GPIO
import time

# 定义触摸传感器输出引脚的_GPIO编号
touch_pin = 18

# 使用BCM GPIO编号方案设置GPIO引脚为输入模式
GPIO.setmode(GPIO.BCM)
GPIO.setup(touch_pin, GPIO.IN)

# 循环检测触摸传感器的状态并输出到控制台
while True:
    touch_status = GPIO.input(touch_pin)
  
    if touch_status == True:
        print("Touch detected")
    else:
        print("Not touched")
        
    time.sleep(0.1) # 暂停100毫秒以避免在循环中过于频繁地读取传感器状态
```

该程序创建一个名为touch_pin的变量，用于指定连接到Jetson Nano的GPIO引脚的编号。然后，使用RPi.GPIO库将GPIO接口设置为输入模式。

接下来，循环检测触摸传感器的状态并将其输出到控制台。如果触摸传感器被接触，则输出Touch detected；否则，输出Not touched。由于传感器可能会发生变化，因此程序暂停100毫秒以避免重复检查。

***

***

### 将一个加速度传感器连接到Jetson Nano上，并读取其信号。

要将加速度传感器连接到Jetson Nano上，需要先确定传感器的型号和引脚定义，然后使用适当的电路将其连接到Jetson Nano的GPIO引脚上。大多数加速度传感器具有三个输出引脚（X，Y和Z），用于测量在每个轴上的加速度并传递数据到控制器。

```import time
import board
import busio
import adafruit_lsm303_accel

# 创建并初始化I2C总线
i2c = busio.I2C(board.SCL, board.SDA)

# 创建并实例化加速度传感器对象
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

# 循环读取加速度传感器的值并输出到控制台
while True:
    acceleration = accel.acceleration
  
    print("Acceleration (m/s^2): X={0:.2f} Y={1:.2f} Z={2:.2f}".format(*acceleration))
  
    time.sleep(0.1) # 暂停100毫秒以避免在循环中过于频繁地读取传感器状态
```

该程序创建一个名为i2c的变量，表示Jetson Nano上的I2C总线。然后，程序使用adafruit_lsm303_accel库创建并实例化了一个名为accel的加速度传感器对象。

接下来，循环读取加速度传感器的值并将其输出到控制台。由于传感器可能会发生变化，因此程序暂停100毫秒以避免重复检查。在每次迭代中，程序调用accel.acceleration方法获取当前X、Y和Z轴上的加速度，并使用print()函数将其输出到控制台。

***

***

### 控制电机，让它按照一定的速度和方向旋转。

要控制电机按照一定的速度和方向旋转，需要使用适当的电路将其连接到Jetson Nano上，并使用适当的代码发送指令以控制电机。以下是一些概念步骤及注意事项：

- 选择适当的电机驱动器并将其连接到Jetson Nano上。例如，可以使用L298N电机驱动板来控制直流电机或步进电机。
- 确定电机的最大速度和方向控制方式。不同类型的电机通常具有不同的速度范围和控制方式，包括PWM信号、正反转信号等。
- 根据手册或规格表，查找电机驱动盘和电机之间的接线方式。确保正确设置驱动器模式，并根据所需的方向和速度发送适当的信号和PWM值。
- 编写控制代码并测试。创建一个Python程序，使用Jetson Nano上可用的GPIO引脚向驱动器发送信号。您可以使用Jetson Nano上的GPIO库（如RPi.GPIO）来实现此操作。

请注意，在执行这些步骤时，请先了解相关的电气知识，并小心操作，以避免可能的电气危险和损坏设备等问题。

下面是一个使用L298N电机驱动板控制直流电机旋转的Python示例程序：

```
import RPi.GPIO as GPIO
import time

# 将L298N驱动板引脚连接到Jetson Nano上的GPIO引脚
in1 = 17
in2 = 18
enable = 27

# 设置GPIO模式和初始状态
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(enable, GPIO.OUT)

# 将电机停止并开始旋转
GPIO.output(in1, False)
GPIO.output(in2, False)
pwm = GPIO.PWM(enable, 1000)
pwm.start(0)

# 按照给定的速度和方向控制电机旋转
speed = 50 # 范围为0-100
direction = True # True表示正向，False表示反向

if direction:
    GPIO.output(in1, True)
    GPIO.output(in2, False)
else:
    GPIO.output(in1, False)
    GPIO.output(in2, True)

pwm.ChangeDutyCycle(speed)

time.sleep(5) # 让电机旋转5秒钟

# 停止电机并清理GPIO状态
pwm.stop()
GPIO.cleanup()
```

该程序首先将L298N驱动板引脚的变量设置为所需的GPIO引脚（如in1，in2和enable）。然后，程序在GPIO模式中将这些引脚设置为输出，并使用GPIO.output()函数将in1和in2信号设置为Fasle以将电机停止。接下来，程序实例化一个PWM对象并设置其初始占空比为0。

然后，程序按照用户指定的速度和方向发送PWM信号和电机方向信号。在本例中，如果direction变量为True，则通过将in1信号设置为True以及将in2信号设置为False来设置电机正向旋转方向。否则，它会将这些值设置为相反的值以实现电机反向旋转。最后使用pwm.ChangeDutyCycle()函数调整PWM的占空比大小以控制电机速度。

最后，程序暂停5秒钟，允许电机旋转，然后调用pwm.stop()函数停止PWM信号，并调用GPIO.cleanup()来清理引脚状态以避免潜在的问题。

***

***

### 在控制电机时使用PID算法（比例-积分-微分）来实现更好的控制。

在控制电机时使用PID算法可以实现更好的控制，使得电机可以按照指定速度和方向平稳地运行。PID算法包括比例、积分和微分三个部分，它们结合起来使用可以产生更准确的电机控制效果。

下面是一个带有PID控制的Python示例程序，来控制一个直流电机：

```
import time
import RPi.GPIO as GPIO

# 将L298N驱动板引脚连接到Jetson Nano上的GPIO引脚
in1 = 17
in2 = 18
enable = 27

# 设置GPIO模式和初始状态
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(enable, GPIO.OUT)

# 创建PWM对象
pwm = GPIO.PWM(enable, 1000)

# 定义PID参数
Kp = 0.5  # 比例系数
Ki = 0.2  # 积分系数
Kd = 0.1  # 微分系数

setpoint = 50  # 目标速度
integral = 0
last_error = 0

def pid_controller(speed):
    global integral, last_error

    # 计算偏差
    error = setpoint - speed

    # 计算积分项
    integral = integral + error

    # 计算微分项
    derivative = error - last_error

    # 计算输出
    output = Kp * error + Ki * integral + Kd * derivative

    # 限制输出值
    if output > 100:
        output = 100
    elif output < -100:
        output = -100

    pwm.ChangeDutyCycle(abs(output))

    # 记录误差
    last_error = error

# 开始旋转电机，使用PID控制器调整速度
GPIO.output(in1, True)
GPIO.output(in2, False)
pwm.start(0)

for i in range(50):  # 运行50次循环
    speed = i  # 模拟实际速度变化
    pid_controller(speed)  # 使用PID算法进行电机速度控制
    time.sleep(0.1)

# 停止电机并清理GPIO状态
pwm.stop()
GPIO.cleanup()
```

该程序首先设置了L298N驱动板引脚，并定义了PID控制器所需的一些变量。然后创建一个名为pid_controller()的函数来执行PID算法并控制电机。在每次迭代中，该函数计算出速度偏差、积分项和微分项，然后在公式中使用比例系数、积分系数和微分系数以计算输出值。最后，根据输出限制其大小并将其传递给PWM对象。

在主程序中，程序开始使电机开始旋转，并使用pid_controller()函数控制其运行方式，对于每次迭代，程序使用随机生成速度值作为模拟实际速度变化，并使用该速度值作为pid_controller()函数的输入值进行PID控制。在完成50次循环后，程序停止电机并清理GPIO引脚状态。

请注意，这是一个简单的示例程序。在实际情况下，需要对PID参数进行多次调整和测试以确定最佳值。同时，需要时刻注意安全和防范电气危险等问题。

***

***

### 将一个温度传感器连接到Jetson Nano上，并读取其信号。

将温度传感器连接到Jetson Nano上，最常见的方式是使用数字引脚读取传感器信号。下面是一个连接DS18B20温度传感器并读取其信号的Python示例程序：

a).连接硬件
首先，将DS18B20连接到Jetson Nano上的GPIO引脚。DS18B20需要三个引脚：VCC（3.3V电源）、GND（地）和DQ（传感器输出）。DQ引脚需要连接到一个数字引脚，并使用“一线式总线”（OneWire）协议来进行通讯。

例如，将DS18B20的VCC引脚连接到Jetson Nano的3.3V引脚、GND引脚连接到Jetson Nano的GND引脚，将DQ引脚连接到Jetson Nano上的引脚17号。

b).启用内核模块
在开始使用DS18B20之前，需要将内核模块加载到Jetson Nano中。在终端窗口中输入以下命令以启用内核模块：

```
sudo modprobe w1-gpio
sudo modprobe w1-therm
```

c).读取温度
在终端窗口中输入以下命令以进入DS18B20传感器目录：

```
cd /sys/bus/w1/devices/
```

然后，运行以下命令来列出数位目录：
	

```
ls
```

您应该会看到一个名为"28-"（DS18B20传感器的默认地址）的目录。进入该目录并查看"w1_slave"文件以读取温度信号：

```
cd 28-*
cat w1_slave
```

您将看到类似以下内容的输出：

```
67 01 4b 46 7f ff 08 10 04: crc=04 YES
67 01 4b 46 7f ff 08 10 04 t=22687
```

其中，"t="后的数值即为温度数据（以标准摄氏度为单位）。在Python程序中，可以使用以下代码来读取并解析这个数据：

```
import os
import time

# 设置温度传感器地址（根据实际情况更改）
device_file = '/sys/bus/w1/devices/28-*/w1_slave'

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
        
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c
        
while True:
    print "Current temperature: ", read_temp(), " C"
    time.sleep(1)
```

程序首先设置了温度传感器的地址，并定义了两个函数以读取传感器数据。read_temp_raw()函数打开"w1_slave"文件并读取它的内容，返回一个包含所有行的列表。read_temp()函数从read_temp_raw()函数中获取传感器数据，并解析该数据以提取温度值。如果传感器数据错误，则等待200毫秒后重试。

在主程序中，程序一直循环运行并使用read_temp()函数来读取当前温度数据。然后将其打印到控制台输出。程序每隔1秒钟读取温度数据一次。

请注意，这仅是一个简单的示例程序，并且在实际情况下，您需要适当地连接DS18B20并根据实际情况更改Python代码。同时，还需要注意安全和防患于未然的物理和电气风险。

***

***

### 将一个湿度传感器连接到Jetson Nano上，并读取其信号。

连接湿度传感器到Jetson Nano上，最常见的方式是使用模拟引脚读取传感器信号。下面是一个连接DHT11湿度传感器并读取其信号的Python示例程序：

a).连接硬件
首先，将DHT11连接到Jetson Nano上的GPIO引脚。DHT11需要三个引脚：VCC（3.3V电源）、GND（地）和DATA（传感器输出）。DATA引脚连接到Jetson Nano上任何一个数字引脚即可。

例如，将DHT11的VCC引脚连接到Jetson Nano的3.3V引脚、GND引脚连接到Jetson Nano的GND引脚，将DATA引脚连接到Jetson Nano上的引脚17号。

b).安装Adafruit_DHT库
在终端窗口中输入以下命令以安装Adafruit_DHT库：

```
sudo pip install Adafruit_DHT
```

c).读取湿度
在Python程序中，可以使用以下代码来读取并解析DHT11传感器的数据：

```
import Adafruit_DHT
import time

# 定义传感器类型和GPIO引脚（根据实际情况更改）
sensor = Adafruit_DHT.DHT11
pin = 17

while True:
    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
    if humidity is not None and temperature is not None:
        print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity)
    else:
        print 'Failed to read data from sensor!'
    time.sleep(2)
```

程序使用Adafruit_DHT库中的函数来读取DHT11传感器的数据。sensor变量指定传感器类型（这里使用DHT11），pin变量指定GPIO引脚。

然后，程序进入一个无限循环，并在每次迭代中使用Adafruit_DHT.read_retry()函数来读取湿度和温度数据。如果读取成功，则将其打印到控制台输出。如果读取失败，则将错误信息打印到控制台输出。

请注意，这仅是一个简单的示例程序，并且在实际情况下，您需要适当地连接DHT11并根据实际情况更改Python代码。同时，还需要注意安全和防患于未然的物理和电气风险。

***

***

### 在控制LED灯时使用NeoPixel库来控制彩色LED。

控制彩色LED，可以使用Adafruit的Python库中的Neopixel库。它可以驱动多种类型的彩色LED灯，包括WS2812/WS2811系列和SK6812/NEOPIXEL系列等。

下面是一个使用Neopixel库控制彩色LED灯的示例程序：

a).连接硬件
将Neopixel LED灯连接到Jetson Nano上的GPIO引脚。通常情况下，LED灯需要三个引脚：VCC（5V电源）、GND（地）和数据引脚。数据引脚需要连接到适当的数字引脚。

例如，如果你将LED灯的VCC引脚连接到Jetson Nano的5V引脚，GND引脚连接到Jetson Nano的GND引脚，那么需要将数据线连接到Jetson Nano的引脚18号。

b).安装库
在终端窗口中输入以下命令以安装neopixel库：

```
sudo pip3 install adafruit-circuitpython-neopixel
```

c).控制LED灯
在Python程序中，可以使用以下代码来控制Neopixel LED灯：

```
import board
import neopixel
import time

# 在引脚18上配置Neopixel LED
pixel_pin = board.D18
# 设置LED的数量和亮度
num_pixels = 8
brightness = 0.5
# 创建NeoPixel对象
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=brightness)

# 颜色定义（RGB格式）
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 150, 0)
PURPLE = (180, 0, 255)
CYAN = (0, 255, 255)
WHITE = (255, 255, 255)
OFF = (0, 0, 0)

# 点亮前4个LED灯为红色
pixels[0] = RED
pixels[1] = RED
pixels[2] = RED
pixels[3] = RED
pixels.show() # 更新灯光

time.sleep(2) #等待两秒

# 设置所有LED灯的颜色为绿色
pixels.fill(GREEN)
pixels.show()

time.sleep(2)

# 将所有LED灯设置为蓝色-另外一种方法
for i in range(num_pixels):
    pixels[i] = BLUE
pixels.show()

time.sleep(2)

# 关闭所有LED灯
pixels.fill(OFF)
pixels.show()
```

首先，程序将引入所需的库（board、neopixel、time）和变量。然后代码创建一个NeoPixel对象pixels，指定其GPIO引脚和LED灯的数量。

接着，代码定义了不同颜色的RGB值，并使用pixels[]来控制LED灯的颜色和亮度。在示例中，代码将前四个LED灯设置为红色，然后设置所有LED灯的颜色为绿色，然后将所有LED灯设置为蓝色，并在最后关闭所有LED灯。

注意，在实际应用中，您需要适当地连接Neopixel LED灯并根据具体情况更改Python代码。此外，请确保安全并注意防止潜在的电气危险。

***

***

### 使用Jetson Nano控制一个机器人，包括驱动电机、控制舵机和读取传感器。

控制机器人需要使用Jetson Nano上的GPIO引脚。根据具体情况，您可能需要连接电机驱动板、PWM树莓派舵机控制板、串口适配器等电子元件。在这里我们提供以下示例程序作为参考：

a).连接硬件
具体的电路连接方式取决于机器人的组成部分。例如：

控制电机，需要单独的电机驱动模块，然后将其与Jetson Nano上的GPIO引脚相连。
控制舵机，需要使用PWM树莓派舵机控制板，并将其与Jetson Nano上的GPIO引脚连接。
读取传感器，需要一个可以与Jetson Nano通讯的传感器模块，并将其与Jetson Nano上的GPIO引脚相连。
b).安装必要的库
在Python程序中，您需要使用不同的库来操作每个设备。例如，在控制电机时，您可能需要使用RPi.GPIO库。在控制舵机时，您可能需要使用Adafruit_PCA9685库。

c).编写控制代码
下面是一个使用RPi.GPIO库和Adafruit_PCA9685库向前/向后移动机器人的示例代码：

```
python
import RPi.GPIO as GPIO
import time
from Adafruit_PCA9685 import PCA9685

# 初始化GPIO（对于电机驱动）
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 设置PWM管脚（对于舵机控制板）
pwm = PCA9685()
pwm.set_pwm_freq(60)

# 定义变量
motor1_in1 = 18    # 连接到驱动电机模块的引脚
motor1_in2 = 17    
pwm_channel = 0    # 连接到舵机控制板的PWM信号线

# 配置GPIO（对于电机驱动）
GPIO.setup(motor1_in1, GPIO.OUT)
GPIO.setup(motor1_in2, GPIO.OUT)

# 控制函数
def forward():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    pwm.set_pwm(pwm_channel, 0, 400)

def stop():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
    pwm.set_pwm(pwm_channel, 0, 0)

def backward():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    pwm.set_pwm(pwm_channel, 0, 400)    

# 调用控制函数

forward()   # 前进
time.sleep(2) 
stop()      # 停止
time.sleep(1)
backward()  # 后退
time.sleep(2)
stop()      #停止

# 关闭GPIO和PWM管脚
GPIO.cleanup()
pwm.exit()
```

在这个示例代码中，使用RPi.GPIO以及PCA9685库。代码定义了驱动电机的两个引脚motor1_in1和motor1_in2，以及与舵机控制板连接的PWM信号线pwm_channel。通过调用向前、停止、向后等控制函数来控制机器人的移动。

注意：

- 您需要根据自己机器人的实际情况修改代码，并确保安全并避免可能的电气危险。
- 用 PySerial 库进行串口通讯读取传感器值。

***



