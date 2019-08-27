# imports
from abc import ABC, abstractmethod


class Road(ABC):
    def __init__(self, starting_density=0):
        self.current_density = starting_density

    # gets
    def get_current_density(self):
        return self.current_density

    # sets
    def set_current_density(self, new_density):
        self.current_density = new_density

    # other methods
    # both cells and onramps will need to implement these
    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def calculate_next_step(self):
        pass
