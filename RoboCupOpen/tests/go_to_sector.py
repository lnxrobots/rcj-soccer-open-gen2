# To be pasted in robot.py

robot_sector = self.tracker.heatmap.index(max(self.tracker.heatmap))
robot_pos = Vector2(*con.TRACKER_HEATMAP_POINTS[robot_sector])
new_index = int(time.time()/10) % 9
target_pos = Vector2(*con.TRACKER_HEATMAP_POINTS[7])
move_vector = target_pos - robot_pos
result_angle = math.degrees(move_vector.get_angle()) - heading
result_angle_rounded = round(result_angle / 45) * 45
spin = mu.direction_to_spin(-heading) * 0.5
motor_values = mu.direction_to_motors(result_angle_rounded, spin)
if robot_pos == target_pos:# and self.tracker.heatmap[robot_sector] >= 0.55:
    motor_values = [0, 0, 0, 0]
UIModule.set_status(f'{robot_sector}')

LoggerModule.log_debug(f'<heatmap> {time.time()} {",".join(map(str, self.tracker.heatmap))}')
