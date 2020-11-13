import pyautogui as pyau
import platform
class Gesture:
    def __init__(self):
        #Darwin
        #Windows
        #Linux
        self.platform = platform.system()
        print(self.platform)

    def zoom_in(self):
        if self.platform == "Darwin":
            pyau.keyDown('command')
            pyau.press('+')
            pyau.keyUp('command')
        else:
            pyau.keyDown('ctrl')
            pyau.press('+')
            pyau.keyUp('ctrl')

    def zoom_out(self):
        if self.platform == "Darwin":
            pyau.keyDown('command')
            pyau.press('-')
            pyau.keyUp('command')
        else:
            pyau.keyDown('ctrl')
            pyau.press('-')
            pyau.keyUp('ctrl')

    def slide_move_left(self):
        pyau.press('left')

    def slide_move_right(self):
        pyau.press('right')
