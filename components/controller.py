from abc import ABC, abstractmethod


class Controller(ABC):

    @abstractmethod
    def compute_next_command(self, timestep, state):
        pass
