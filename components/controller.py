from abc import ABC, abstractmethod


class Controller(ABC):

    @abstractmethod
    def compute_next_command(self, timestep, state, debug=False):
        '''
        method will be called with timestep and state parameters, debug can be specified when calling Simulator.run()
        :param timestep: the current timestep as an int
        :param state: the current state of the network as a list of values [onramp 1-n density, cell 1-n density, congestion state 1-n]
        :param debug: whether to display debug info
        :return: control signals within [0.0, 1.0] in an (n x 1) or (o x 1) array where n = number of cells, o = number of onramps
        '''
        pass
