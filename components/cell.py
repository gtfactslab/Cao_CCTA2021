# imports
from components.road import Road
from components.onramp import OnRamp

class Cell(Road):
    def __init__(self, starting_density, jam_density, w, x_jam, v, attached_onramp=None):
        super().__init__(starting_density)

        self.jam_density = jam_density

        self.supply_slope = w
        self.supply_jam = x_jam
        self.supply_b = -1 * self.supply_slope * self.supply_jam

        self.demand_slope = v

        if bool(attached_onramp):
            self.attached_onramp = attached_onramp
        else:
            # if onramp not attached, equivalent to attaching a ramp with a max flow rate of 0
            self.attached_onramp = OnRamp(starting_density=0,
                                          max_flow_rate=0)

        self.upstream = None # if none, no incoming cars, just input from onramp
        self.downstream = None # if none, assumed next cell has infinite supply

        self.density_next_step = None

    # gets
    def get_attached_onramp(self):
        return self.attached_onramp

    # sets
    def attach_onramp(self, onramp_obj):
        self.attached_onramp = onramp_obj

    # other methods
    def calculate_next_step(self, incoming_to_onramp):

        cur_density = self.get_current_density()

        new_from_onramp = self.attached_onramp.calculate_next_step(incoming_to_onramp)

        # TODO: update with proper equation
        upstream_demand = self.upstream.demand() if self.upstream is not None else 0
        downstream_supply = self.downstream.supply() if self.downstream is not None else self.demand()

        if self.is_congested():
            change_in_density = self.supply() - downstream_supply
        else:
            change_in_density = upstream_demand - self.demand()

        self.density_next_step = max(cur_density + change_in_density + new_from_onramp, 0) # ensures never goes below 0
        return self.density_next_step

    def update(self):
        if self.density_next_step is None:
            print("ERROR: next step density for cell not calculated, run calculate_next_step() first")
            return
        self.attached_onramp.update()
        self.set_current_density(self.density_next_step)
        self.density_next_step = None

    # TODO: is input density really just the current density?
    def supply(self):
        return self.supply_slope * self.current_density + self.supply_b

    def demand(self):
        return self.demand_slope * self.current_density

    # TODO: determines whether cell is congested based on current state
    def is_congested(self):
        return self.get_current_density() >= self.jam_density

    #helper functions
    def to_dict(self):
        output_dict = {
            "jam_density": self.jam_density,
            "supply_w": self.supply_slope,
            "supply_jam": self.supply_jam,
            "demand_v": self.demand_slope
        }
        if self.attached_onramp.get_max_flow_rate() > 0:
            output_dict["attached_onramp"] = self.attached_onramp.to_dict()
        return output_dict
