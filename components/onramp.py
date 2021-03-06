# imports
from components.road import Road

class OnRamp(Road):
    def __init__(self, starting_density, max_flow_rate):
        super().__init__(starting_density)

        self.max_flow_rate = max_flow_rate
        if self.max_flow_rate == 0:
            self.set_current_density(0) # flow rate of 0 is equivalent to no onramp attached

        self.density_next_step = 0

    # gets
    def get_max_flow_rate(self):
        return self.max_flow_rate

    # sets
    def set_max_flow_rate(self, max_flow_rate):
        self.max_flow_rate = max_flow_rate

    # other methods

    # given input # of vehicles, calculate next time step's density and return number of vehicles exiting the onramp
    def calculate_next_step(self, incoming_vehicles, control=None):
        if self.get_max_flow_rate() == 0:
            self.density_next_step = 0
            return 0

        cur_density = self.get_current_density()

        # add incoming vehicles onto ramp
        new_density = cur_density + incoming_vehicles

        # if input control not provided or invalid, default to 1
        if control is None:
            control = 1

        if control > 1 or control < 0:
            print("WARNING: Invalid control signal sent, will default to 1")
            control = 1

        # calculate number of vehicles exiting ramp and calculate next density
        # currently, the onramp forces up to the max flow rate of vehicles onto the attached cell
        # incoming vehicles may not exit onramp until next time step
        exiting_vehicles = min(cur_density, self.get_max_flow_rate() * control)
        new_density = new_density - exiting_vehicles

        self.density_next_step = new_density

        return exiting_vehicles

    def update(self):
        if self.density_next_step is None:
            print("ERROR: next step density for onramp not calculated, run calculate_next_step() first")
            return
        self.set_current_density(self.density_next_step)
        self.density_next_step = None

    #helper functions
    def to_dict(self):
        output_dict = {
            "max_flow_rate": self.max_flow_rate
        }
        return output_dict
