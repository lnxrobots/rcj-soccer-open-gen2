import soccer_robot.constants as con
from soccer_robot.mathf import mathf
from soccer_robot.mathf.vector2 import Vector2

from PIL import Image, ImageDraw, ImageFont
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_ssd1306

import math

class Display:
    """SSD1306 OLED class for... showing data"""

    def __init__(self):
        self._i2c = I2C(con.DISPLAY_I2C_BUS)
        self._oled = adafruit_ssd1306.SSD1306_I2C(
            con.DISPLAY_WIDTH,
            con.DISPLAY_HEIGHT,
            self._i2c,
            addr=con.DISPLAY_I2C_ADDR
        )
        if con.DISPLAY_FONT:
            self.font = ImageFont.load(con.DISPLAY_FONT)
        else:
            self.font = ImageFont.load_default()
        self.wifi_icon = Image.open(con.DISPLAY_WIFI_ICON)
        self.debug_icon = Image.open(con.DISPLAY_DEBUG_ICON)
        self.bluetooth_icon = Image.open(con.DISPLAY_BTH_ICON)

    def clear(self):
        self._oled.fill(0)
        self._oled.show()

    def update(self, ip, wifi_on, debug_mode, goalie, line_threshold, status):
        from soccer_robot.interface.compass_module import CompassModule
        from soccer_robot.interface.camera_module.camera_module import CameraModule
        from soccer_robot.interface.undercarriage_module import UndercarriageModule
        from soccer_robot.interface.bluetooth_module import BluetoothModule
        from soccer_robot.logger.logger_module import LoggerModule

        frame = Image.new("1", (con.DISPLAY_WIDTH, con.DISPLAY_HEIGHT))
        draw = ImageDraw.Draw(frame)

        draw.text((0, 0), ip, font=self.font, fill=255) # ip address
        x = con.DISPLAY_WIDTH
        if wifi_on:
            x -= self.wifi_icon.width
            frame.paste(self.wifi_icon, (x, 0))
        if debug_mode:
            x -= self.debug_icon.width+3
            frame.paste(self.debug_icon, (x, 0))
        if BluetoothModule.is_connected():
            x -= self.bluetooth_icon.width+3
            frame.paste(self.bluetooth_icon, (x, 0))
        if goalie:
            x -= self.font.getsize('G')[0]+3
            draw.text((x, -3), 'G', font=self.font, fill=255)

        ball_position = CameraModule.get_ball_position()
        goal_position = CameraModule.get_goal_position()
        sensor_values = UndercarriageModule.get_color_sensor_values()

        ball_angle = mathf.img_coords_to_angle_dist(ball_position)[0]
        goal_angle = mathf.img_coords_to_angle_dist(goal_position)[0]
        heading = CompassModule.get_heading()

        angle_text = "C{:<+4.0f} B{:<+4.0f} G{:+.0f}".format(heading, ball_angle, goal_angle)
        draw.text((24, 10), angle_text, font=self.font, fill=255) # angles

        st_size = self.font.getsize(status)
        draw.text(
            (con.DISPLAY_WIDTH-st_size[0], con.DISPLAY_HEIGHT-st_size[1]),
            status, font=self.font, fill=255, anchor="mm"
        ) # status

        active_i = []
        for i, v in enumerate(sensor_values):
            if v > line_threshold:
                active_i.append(i)
                x, y = Vector2(
                    a=math.radians(con.SENSOR_ANGLES[i]-90),
                    m=(con.SENSOR_DIST[i]/con.SENSOR_MAX_DIST)**con.DISPLAY_CIRCLE_E*con.DISPLAY_CIRCLE_D/2
                )
                draw.point((
                    round(x+con.DISPLAY_CIRCLE_D/2),
                    round(-y+11+con.DISPLAY_CIRCLE_D/2)
                ), fill=255) # sensor dots

        line = mathf.get_line_pos(active_i)
        if line is not None:
            x, y = Vector2(
                a=math.radians(line[0]-90),
                m=max(line[1]/con.SENSOR_MAX_DIST*(con.DISPLAY_CIRCLE_D/2-4), 2)
            )
            draw.line([
                (con.DISPLAY_CIRCLE_D/2, 11+con.DISPLAY_CIRCLE_D/2),
                (round(x+con.DISPLAY_CIRCLE_D/2), round(-y+11+con.DISPLAY_CIRCLE_D/2))
            ], fill=255) # line vector

        self._oled.image(frame.rotate(180))
        self._oled.show()
