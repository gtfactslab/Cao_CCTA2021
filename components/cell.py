# imports
from components.road import Road
from components.onramp import OnRamp

class Cell(Road):
    def __init__(self, starting_density, x_upper, x_lower, w, x_jam, v, beta, attached_onramp=None):
        super().__init__(starting_density)

        self.x_upper = x_upper
        self.x_lower = x_lower

        self.supply_slope = w
        self.supply_jam = x_jam
        self.supply_b = -1 * self.supply_slope * self.supply_jam

        self.demand_slope = v

        # represents the ratio of cars that progress onto the next cell as opposed to exiting through an offramp
        # TODO: determine which beta to use (i.e. should beta_i represent the link b/t cells [i-1, i] or [i, i+1])?
        self.beta = beta

        if bool(attached_onramp):
            self.attached_onramp = attached_onramp
        else:
            # if onramp not attached, equivalent to attaching a ramp with a max flow rate of 0
            self.attached_onramp = OnRamp(starting_density=0,
                                          max_flow_rate=0)

        self.upstream = None # if none, no incoming cars, just input from onramp
        self.downstream = None # if none, assumed next cell has infinite supply

        self.density_next_step = None

        self.congested = False

        self.inflow = 0
        self.outflow = 0

    # gets
    def get_attached_onramp(self):
        return self.attached_onramp

    # sets
    def attach_onramp(self, onramp_obj):
        self.attached_onramp = onramp_obj

    # other methods
    def calculate_next_step(self, incoming_to_onramp, h):

        cur_density = self.get_current_density()

        new_from_onramp = self.attached_onramp.calculate_next_step(incoming_to_onramp)

        # update self.inflow and self.outflow, then use to calculate change in density
        self.retrieve_inflow_from_upstream()
        self.calculate_outflow()

        change_in_density = self.inflow - self.outflow

        # discretization factor
        change_in_density = h * change_in_density

        # calculate density of next step (TODO: this is where cars incoming from onramp are added as they are "forced")
        self.density_next_step = max(cur_density + change_in_density + new_from_onramp, 0) # ensures never goes below 0
        return self.density_next_step

    def update(self):
        if self.density_next_step is None:
            print("ERROR: next step density for cell not calculated, run calculate_next_step() first")
            return
        self.attached_onramp.update()
        self.set_current_density(self.density_next_step)
        self.density_next_step = None

    def supply(self):
        return self.supply_slope * self.current_density + self.supply_b

    def demand(self):
        return self.demand_slope * self.current_density

    def calculate_outflow(self):
        # each cell calculates its outflow to the next cell
        if self.downstream is not None:
            if self.downstream.is_congested():
                self.outflow = min(self.demand(), self.downstream.supply())
            else:
                self.outflow = self.demand()
        else:
            self.outflow = self.demand()

    def retrieve_inflow_from_upstream(self):
        # cell assumes that if upstream exists, it has been called first and its outflow has been set
        if self.upstream is not None:
            # TODO: see questions about beta above
            self.inflow = self.upstream.beta * self.upstream.outflow
        else:
            # if no upstream then only inflow is from onramp, which is taken care of in calculate_next_step
            self.inflow = 0


    # determines whether cell is congested based on current state
    def is_congested(self):
        # first checks to see if congestion state has changed
        # if above x_upper, now congested regardless of previous state
        if self.get_current_density() >= self.x_upper:
            self.congested = True
        # if below x_lower, now uncongested regardless of previous state
        elif self.get_current_density() <= self.x_lower:
            self.congested = False
        # if neither, congestion state is the same (hysteresis)
        return self.congested

    #helper functions
    def to_dict(self):
        output_dict = {
            "x_upper": self.x_upper,
            "supply_w": self.supply_slope,
            "supply_jam": self.supply_jam,
            "demand_v": self.demand_slope
        }
        if self.attached_onramp.get_max_flow_rate() > 0:
            output_dict["attached_onramp"] = self.attached_onramp.to_dict()
        return output_dict
