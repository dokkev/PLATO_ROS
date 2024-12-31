import pyspacemouse
import time

success = pyspacemouse.open()
if success:
  while 1:
    state = pyspacemouse.read()
    print(state.x, state.y, state.z, state.roll, state.yaw, state.pitch, state.buttons)
    time.sleep(0.01)