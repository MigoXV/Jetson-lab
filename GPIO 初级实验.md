## 初级实验（1-10）

这些实验适合那些初学Jetson GPIO控制的人。它们基于最基本的GPIO控制，逐步引入更高级的功能。

1. 通过Jetson Nano控制LED灯，让它闪烁一次。 
   
   ```python
    mport Jetson.GPIO as GPIO
   import time
   GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
   GPIO.setwarnings(False) # 禁用GPIO警告信息
   led_pin = 12 # 设置要控制的GPIO针脚号
   GPIO.setup(led_pin, GPIO.OUT) # 设置GPIO针脚为输出模式
   GPIO.output(led_pin, GPIO.HIGH) # 点亮LED
   time.sleep(1) # 暂停1秒
   GPIO.output(led_pin, GPIO.LOW) # 熄灭LED
   time.sleep(1) # 暂停1秒
   GPIO.cleanup() # 清理GPIO设置
   ```
   
   2. 将一个按钮连接到Jetson Nano上，并在按钮按下时使LED灯闪烁。
      
      ```python
      import Jetson.GPIO as GPIO
      import time
      GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
      GPIO.setwarnings(False) # 禁用GPIO警告信息
      
      button_pin = 16 # 设置按钮连接的GPIO针脚号
      led_pin = 12 # 设置LED连接的GPIO针脚号
      
      GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # 设置按钮针脚为输入模式，启用上拉电阻
      GPIO.setup(led_pin, GPIO.OUT) # 设置LED针脚为输出模式
      
      while True:
      if GPIO.input(button_pin) == GPIO.LOW: # 如果按钮被按下
      GPIO.output(led_pin, GPIO.HIGH) # 点亮LED
      time.sleep(0.5) # 暂停0.5秒
      GPIO.output(led_pin, GPIO.LOW) # 熄灭LED
      time.sleep(0.5) # 暂停0.5秒
      
      GPIO.cleanup() # 清理GPIO设置
      ```
      
      3.制LED灯，让它按照一定的频率闪烁。
   
   ```python
   import Jetson.GPIO as GPIO
   import time
   
    GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
    GPIO.setwarnings(False) # 禁用GPIO警告信息
   
    led_pin = 12 # 设置LED连接的GPIO针脚号
   
    GPIO.setup(led_pin, GPIO.OUT) # 设置LED针脚为输出模式
   
   while True:
    GPIO.output(led_pin, GPIO.HIGH) # 点亮LED
    time.sleep(0.5) # 暂停0.5秒
    GPIO.output(led_pin, GPIO.LOW) # 熄灭LED
    time.sleep(0.5) # 暂停0.5秒
    GPIO.cleanup() # 清理GPIO设置
   ```
   
   4. 控制多个LED灯，让它们一起闪烁。
   
   ```python
   import Jetson.GPIO as GPIO
   import time
   GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
   GPIO.setwarnings(False) # 禁用GPIO警告信息
   
    led_pins = [12, 16, 18] # 设置LED连接的GPIO针脚号
   
    for led_pin in led_pins:
    GPIO.setup(led_pin, GPIO.OUT) # 设置LED针脚为输出模式
   
    while True:
    for led_pin in led_pins:
    PIO.output(led_pin, GPIO.HIGH) # 点亮LED
    time.sleep(0.5) # 暂停0.5秒
    for led_pin in led_pins:
    GPIO.output(led_pin, GPIO.LOW) # 熄灭LED
    time.sleep(0.5) # 暂停0.5秒
   
    GPIO.cleanup() # 清理GPIO设置
   ```
   
   5. 在控制LED灯时使用PWM（脉冲宽度调制）技术，以改变LED的亮度。
   
   ```python
   import Jetson.GPIO as GPIO
   import time
   
   GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
   GPIO.setwarnings(False) # 禁用GPIO警告信息
   
   led_pin = 12 # 设置LED连接的GPIO针脚号
   
   GPIO.setup(led_pin, GPIO.OUT) # 设置LED针脚为输出模式
   
   pwm_led = GPIO.PWM(led_pin, 100) # 设置PWM对象，频率为100Hz
    pwm_led.start(0) # 启动PWM，占空比为0%
   
   while True:
    for duty_cycle in range(0, 101, 5):
        pwm_led.ChangeDutyCycle(duty_cycle) # 设置PWM占空比
        time.sleep(0.1) # 暂停0.1秒
   
   GPIO.cleanup() # 清理GPIO设置
   ```
   
   6. 使用电位器来控制LED的亮度。

```python
   import Jetson.GPIO as GPIO
  GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
   GPIO.setwarnings(False) # 禁用GPIO警告信息
   led_pin = 12 # 设置LED连接的GPIO针脚号
   pot_pin = 40 # 设置电位器连接的GPIO针脚号

   GPIO.setup(led_pin, GPIO.OUT) # 设置LED针脚为输出模式
   GPIO.setup(pot_pin, GPIO.IN) # 设置电位器针脚为输入模式

   pwm_led = GPIO.PWM(led_pin, 100) # 设置PWM对象，频率为100Hz
   pwm_led.start(0) # 启动PWM，占空比为0%

   while True:
   pot_value = GPIO.input(pot_pin) # 获取电位器值
    duty_cycle = pot_value / 10.23 # 将电位器值转换为占空比
   pwm_led.ChangeDutyCycle(duty_cycle) # 设置PWM占空比

   GPIO.cleanup() # 清理GPIO设置
```

7. 控制舵机，让它按照一定的速度和方向旋转。
   
   ```python
   import Jetson.GPIO as GPIO
   import time
   
    GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
   GPIO.setwarnings(False) # 禁用GPIO警告信息
   
   servo_pin = 12 # 设置舵机连接的GPIO针脚号
   
    GPIO.setup(servo_pin, GPIO.OUT) # 设置舵机针脚为输出模式
   
    servo_pwm = GPIO.PWM(servo_pin, 50) # 设置PWM对象，频率为50Hz
    servo_pwm.start(0) # 启动PWM，占空比为0%
   
   def set_servo_angle(angle):
   duty_cycle = 2.5 + 10 * angle / 180 # 计算占空比，角度范围为0到180度
   servo_pwm.ChangeDutyCycle(duty_cycle) # 设置PWM占空比
    time.sleep(0.3) # 等待舵机旋转到指定角度
   
   try:
   while True:
     # 控制舵机按照一定速度和方向旋转
     set_servo_angle(0)
     set_servo_angle(90)
     set_servo_angle(180)
     set_servo_angle(90)
   
   except KeyboardInterrupt:
    servo_pwm.stop() # 停止PWM
   GPIO.cleanup() # 清理GPIO设置
   ```

8. 将一个热敏电阻连接到Jetson Nano上，并读取其电阻值。

```python
import Jetson.GPIO as GPIO
import time

 GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
GPIO.setwarnings(False) # 禁用GPIO警告信息

thermistor_pin = 7 # 设置热敏电阻连接的GPIO针脚号

GPIO.setup(thermistor_pin, GPIO.IN) # 设置热敏电阻针脚为输入模式

try:
 while True:
  # 读取热敏电阻的电阻值
  reading = GPIO.input(thermistor_pin)
  print("Thermistor resistance: %d" % reading)
  time.sleep(0.5)

except KeyboardInterrupt:
 GPIO.cleanup() # 清理GPIO设置
```

9. 将一个光敏电阻连接到Jetson Nano上，并读取其电阻值。

```python
import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
GPIO.setwarnings(False) # 禁用GPIO警告信息

photoresistor_pin = 7 # 设置光敏电阻连接的GPIO针脚号

GPIO.setup(photoresistor_pin, GPIO.IN) # 设置光敏电阻针脚为输入模式

try:
 while True:
     # 读取光敏电阻的电阻值
     reading = GPIO.input(photoresistor_pin)
     print("Photoresistor resistance: %d" % reading)
     time.sleep(0.5)

except KeyboardInterrupt:
 GPIO.cleanup() # 清理GPIO设置

```

10. 使用MQTT协议（消息队列遥测传输）控制LED灯。
    
    ```python
    import Jetson.GPIO as GPIO
    import paho.mqtt.client as mqtt
    
     GPIO.setmode(GPIO.BOARD) # 设置GPIO模式为BOARD
      GPIO.setwarnings(False) # 禁用GPIO警告信息
    
    led_pin = 7 # 设置LED灯连接的GPIO针脚号
    
    GPIO.setup(led_pin, GPIO.OUT) # 设置LED灯针脚为输出模式
    
     # 定义MQTT回调函数
    
     def on_message(client, userdata, message):
    if message.payload.decode() == "on":
        GPIO.output(led_pin, GPIO.HIGH)
        print("LED is on")
       elif message.payload.decode() == "off":
        GPIO.output(led_pin, GPIO.LOW)
        print("LED is off")
    
     broker_address = "broker.mqtt-dashboard.com" # MQTT服务器地址
     client = mqtt.Client("JetsonNano_LED") # 创建MQTT客户端对象
      client.connect(broker_address) # 连接MQTT服务器
      client.subscribe("led_control") # 订阅MQTT主题
    
    client.on_message = on_message # 注册MQTT回调函数
    
       try:
    client.loop_forever() # 开始MQTT客户端循环
    
     except KeyboardInterrupt:
    GPIO.cleanup() # 清理GPIO设置
    ```
