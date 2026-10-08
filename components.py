import math
import warnings
from enum import Enum
import numpy as np
from dataclasses import dataclass, field
import simple_pid

@dataclass
class Vehicle:
    """Vehicle longitudinal model and expected operating values for energy management strategy"""

    m: float  # mass in kilogram
    c_d: float  # aerodynamic drag coefficient based on frontal area
    A_f: float  # frontal area in m^2
    c_r: float  # rolling resistance coefficient based on normal force
    eta_drive: float = 0.85  # wheel, driveshaft, transmmission, motor and inverter efficiency
    rho: float = 1.225  # air density in kg/m^3
    lam: float = 1.03  # rotational inertia factor
    g: float = 9.81  # gravitational acceleration in m/s^2

    def drag_aero(self, v:float) -> float:                  
        return 0.5 * self.A_f * self.c_d * self.rho * (v**2)
    
    def drag_roll(self, alpha:float) -> float:
        return self.c_r * self.m * self.g * np.cos(np.radians(alpha))                 
    
    def drag_incline(self, alpha:float) -> float:
        return self.m * self.g * np.sin(np.radians(alpha))                         
    
    def drag_accel(self, a:float) -> float:
        return self.m * self.lam * a
    
    def drag_total(self, v:float, alpha:float, a:float) -> float:
        return self.drag_aero(v) + self.drag_roll(alpha) + self.drag_incline(alpha) + self.drag_accel(a)

    def power_total(self, v:float, alpha:float, a:float) -> float:
        power_wheel = self.drag_total(v, alpha, a) * v
        if power_wheel >= 0:
            return power_wheel / self.eta_drive  # traction
        return power_wheel * self.eta_drive  # recuperation
    
@dataclass
class BatteryPack:
    name: str
    parameters: dict
    energy_nominal_single: float
    packs_parallel: int = 1
    soc_init: float = 0.98
    soc_min: float = 0.05
    _soc: float = 1.0
    _power_chem: float = 0.0

    def __post_init__(self):
        self.energy_total = self.energy_nominal_single * self.packs_parallel
        self._soc = self.soc_init

        # nearest-neighbour lookup tables on the uniform SOC grid of the parameters
        soc_grid = np.asarray(self.parameters['soc'], dtype=float)
        soc_steps = np.diff(soc_grid)
        if not np.allclose(soc_steps, soc_steps[0]):
            raise ValueError(f"BatteryPack '{self.name}' needs a uniform SOC grid for the nearest SOC lookup")
        self._soc_first = float(soc_grid[0])
        self._soc_step = float(soc_steps[0])
        self._ocv_table = [float(ocv) for ocv in self.parameters['ocv']]
        self._ri_table = [float(ri) / self.packs_parallel for ri in self.parameters['ri']]

    @property
    def soc(self):
        return self._soc

    def _soc_index(self):
        index = round((self._soc - self._soc_first) / self._soc_step)
        return min(max(index, 0), len(self._ocv_table) - 1)

    @property
    def ocv(self):
        return self._ocv_table[self._soc_index()]

    @property
    def ri(self):
        return self._ri_table[self._soc_index()]

    def current(self, power_req):
        radicand = self.ocv ** 2 - (4 * self.ri * power_req)

        if radicand < 0:
            warnings.warn(
                f"BatteryPack '{self.name}' is exceeding power limits at SOC {self._soc:.2f} and power request: {power_req:.1f} W.",
                RuntimeWarning,
                stacklevel=2,
            )
            return self.ocv / (2 * self.ri)  # current for max power
        
        return (self.ocv - np.sqrt(radicand)) / (2 * self.ri)
    
    def voltage(self, current):
        return self.ocv - (self.ri * current)

    def loss(self, current):
        return current ** 2 * self.ri

    def power_max(self, dt):
        """Maximum terminal power for the next step, limited by power capability and energy above soc_min."""
        current_max = self.ocv / (2 * self.ri)  # current at maximum terminal power
        if dt > 0:
            energy_available = max(self._soc - self.soc_min, 0.0) * self.energy_total  # Wh
            power_chem_max = max(7200 * energy_available / dt - self._power_chem, 0.0)  # inverse of trapezoidal step in step()
            current_max = min(current_max, power_chem_max / self.ocv)
        return self.voltage(current_max) * current_max

    def step(self, power_req, dt):
        power_chem_prev = self._power_chem
        soc_prev = self._soc

        current = self.current(power_req)
        voltage = self.voltage(current)
        self._power_chem = current * self.ocv
        power_out = current * voltage
        power_loss = self.loss(current)

        energy_chem_step = dt * (self._power_chem + power_chem_prev) / 7200
        self._soc = soc_prev - energy_chem_step / self.energy_total  # positive power is discharge

        return voltage, current, power_out, power_loss, self._soc

    def check_soc_min(self):
        return self._soc <= self.soc_min


class EnergyStrategy(Enum):
    MONOTONOUS = 'monotonous'
    PROPORTIONAL = 'proportional'
    MIXED = 'mixed'
    FULL_POWER = 'full_power'


class EnergyManager:
    """Split the demanded power between the fixed and portable battery packs."""

    def __init__(self, strategy, v_avg: float | None = None, e_d: float | None = None, vehicle=None, trip=None, speed_min: float = 0.5):
        if isinstance(strategy, str):
            strategy = EnergyStrategy(strategy.lower())
        self.strategy = strategy
        self.vehicle = vehicle
        self.v_avg = v_avg
        self.e_d = e_d
        self.speed_min = speed_min

    def step(self, power_dem, fb: BatteryPack, pb: BatteryPack, speed: float | None = None) -> float:
        """Return the power demanded from the portable battery branch."""
        if speed is not None and speed < self.speed_min:
            return 0.0

        share_energy_pb = pb.soc * pb.energy_total / max(fb.soc * fb.energy_total + pb.soc * pb.energy_total, 1e-9)

        if self.strategy == EnergyStrategy.MONOTONOUS:
            power_dem_branch = self.monotonous(share_energy_pb)
        elif self.strategy == EnergyStrategy.PROPORTIONAL:
            power_dem_branch = self.proportional(power_dem, share_energy_pb)
        elif self.strategy == EnergyStrategy.MIXED:
            power_dem_branch = self.mixed(power_dem, share_energy_pb)
        elif self.strategy == EnergyStrategy.FULL_POWER:
            power_dem_branch = math.inf  # converter delivers its maximum until the pb is depleted
        else:
            raise ValueError(f'Unknown energy strategy: {self.strategy!r}')

        return power_dem_branch

    def monotonous(self, share_energy_pb: float) -> float:
        if self.v_avg is None or self.e_d is None:
            raise ValueError('EnergyManager.monotonous requires v_avg and e_d to be set on the manager.')
        return self.v_avg * self.e_d * share_energy_pb

    def proportional(self, power_dem: float, share_energy_pb: float) -> float:
        return max(0, power_dem * share_energy_pb)

    def mixed(self, power_dem: float, share_energy_pb: float) -> float:
        """Equal-weight mix of the monotonous and proportional splits."""
        return 0.5 * self.monotonous(share_energy_pb) + 0.5 * self.proportional(power_dem, share_energy_pb)

@dataclass
class DCDCConverter:
    """DCDC converter whose efficiency is supplied as a 1D lookup curve over current."""

    name: str
    parameters: dict

    def __post_init__(self):
        self.currents_out = np.asarray(self.parameters['current_out'], dtype=float)
        self.efficiencies = np.asarray(self.parameters['efficiency'], dtype=float)

        if self.currents_out.ndim != 1 or self.efficiencies.ndim != 1:
            raise ValueError('Converter lookup arrays must be one-dimensional')
        if self.currents_out.shape != self.efficiencies.shape:
            raise ValueError('Converter current and efficiency arrays must have the same shape')
        if not np.all(np.diff(self.currents_out) > 0):
            raise ValueError('Converter current values must be strictly increasing')

    def get_efficiency(self, current_out_dem):
        """Return interpolated efficiency for output current in amperes."""
        if current_out_dem > self.currents_out[-1]:
            warnings.warn(
                f"Converter is exceeding current limits",
                RuntimeWarning,
                stacklevel=2,
            )
            return self.efficiencies[-1]

        return np.interp(
            current_out_dem,
            self.currents_out,
            self.efficiencies,
            left=self.efficiencies[0],
            right=self.efficiencies[-1],
        )

    def power_in(self, power_dem, voltage_out):
        """Return input power, delivered output power, output current, efficiency and loss.

        The output current is limited to the last tabulated current; demand beyond it is not delivered.
        """
        current_out = min(power_dem / voltage_out, self.currents_out[-1])
        power_out = current_out * voltage_out
        efficiency = self.get_efficiency(current_out)
        power_in = power_out / efficiency
        loss = power_in - power_out
        return power_in, power_out, current_out, efficiency, loss


class PIController:
    def __init__(self, dt, gain_p=0.0001 , gain_i=-0.0001):
        self.controller = simple_pid.PID(k_p=gain_p, k_i=gain_i, sample_time=dt)

    def get_dcdc_voltage(self, soc, Pdem, P_target):
        ri = self.BP.get_ri(soc)
        ocv = self.BP.get_ocv(soc)
        return ((ocv + math.sqrt((ocv ** 2) - 4 * (Pdem - P_target) * ri)) / 2)
    
    def get_dcdc_current(self, Pdem, U_dcdc, battery_current):
        return (Pdem / U_dcdc) - battery_current
    
    def get_pid(self,index ,time, P_target, U_dcdc, I_dcdc):
        if index == 0:
            P_ist = 0
            P_target = 0
        else:
            P_ist = U_dcdc * I_dcdc
        
        def get_time():
            return time
        
        self.pid.setpoint = P_target
        self.pid.time_fn = get_time
        control_signal = self.pid(P_ist)
        return control_signal

