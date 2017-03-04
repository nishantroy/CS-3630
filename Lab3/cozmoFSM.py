# Madelyn Juby, Nishant Roy

from transitions import Machine


class cozmoFSM(object):


    def __init__(self, name):
        states = ['initial', 'search', 'drive', 'hitball']
        self.name = name
        machine = Machine(model=self, states=states, initial='initial')

        machine.add_transition(trigger='ballDetected', source='*', dest='drive', after="playSound")
        machine.add_transition(trigger='ballLost', source='*', dest='search', after="playSound")
        machine.add_transition(trigger='ballInReach', source='drive', dest='hitball', after="playSound")

    def playSound(self):
        print('\a')


