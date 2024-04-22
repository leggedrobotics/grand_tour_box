import pigpio
FAN_GPIO = 18
speed_in_percentage = 40
pi = pigpio.pi()
pi.set_mode(FAN_GPIO, pigpio.OUTPUT)
pi.set_PWM_frequency(FAN_GPIO, 25000)
pi.set_PWM_dutycycle(FAN_GPIO, int(speed_in_percentage/100*255))