from enum import Enum

class State(Enum):
    IDLE       = 'IDLE'
    PERCEIVING = 'PERCEIVING'
    PLANNING   = 'PLANNING'
    EXECUTING  = 'EXECUTING'
    DONE       = 'DONE'
    ERROR      = 'ERROR'

class StateMachine:
    def __init__(self, logger):
        self.state = State.IDLE
        self.logger = logger

    def transition(self, new_state):
        self.logger.info(f'상태 변경: {self.state.value} → {new_state.value}')
        self.state = new_state

    def is_idle(self):
        return self.state == State.IDLE
