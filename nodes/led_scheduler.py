from led_pwm_tlc5947_proxy import LedPwmTLC5947Proxy
import rospy

class LedScheduler(object):

    def __init__(self,params) :

        # Extract parameters 
        self.led_pin = params['pin'] 
        self.led_dev_num = params['led_dev_num']
        self.led_on_value =  params['led_on_value']
        self.led_off_value = params['led_off_value'] 
        self.led_on_duration = params['led_on_duration'] 
        self.led_minimum_off_duration = params['led_minimum_off_duration']

        # State
        self.led_on = False 
        self.last_on_t  = 0.0
        self.activation_count = 0

        self.led_proxy= LedPwmTLC5947Proxy()
        self.turn_off_led()


    def __del__(self):
        self.turn_off_led()
    
    def turn_on_led(self,t):
        if not self.led_on:
            self.led_proxy.set(self.led_dev_num, self.led_pin, self.led_on_value)
            self.activation_count += 1
            self.led_on = True
            self.last_on_t = t

    def turn_off_led(self):
        if self.led_on:
            self.led_proxy.set(self.led_dev_num, self.led_pin, self.led_off_value)
            self.led_on = False

    def update(self, t, fly_on_food): 
        if self.led_on:
            if (t - self.last_on_t) > self.led_on_duration:
                self.turn_off_led()
        else:
            if fly_on_food:
                if (t - self.last_on_t) > (self.led_on_duration + self.led_minimum_off_duration):
                    self.turn_on_led(t)   





