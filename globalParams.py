from enum import Enum

FRAME_HEIGHT = 600*15/20
FRAME_WIDTH = 600

FRAME_MARGIN = 10
CELL_MARGIN = 0

FREE_SPACE = 0
OBSTACLES = -99


COLORS = ['', 'red', 'green', 'blue', 'black', 'magenta', 'cyan', 'yellow','#e325f8', '#9f62f5', '#62aff5', '#45b3ad', '#45b388', '#aedb9e', '#d1db9e', '#dbc19e', '#f97340', '#bff940', '#FF8000', '#660000', '#CCCCFF']

class Actions(object):
    RIGHT = 0
    UP = 1
    LEFT = 2
    DOWN = 3
    WAIT = 4
    action_dict = {RIGHT:'right', LEFT:'left', UP:'up', DOWN: 'down', WAIT:'wait'}

class MissionType(object):
    PICKUP = -1
    IDLE = -2