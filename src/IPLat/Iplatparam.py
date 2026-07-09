# ==========================================
# 1. PARAMETERS DEFINITION
# ==========================================
class SystemParameters:
    def __init__(self):
        # Physical Parameters
        self.mass_at_end = 0.24
        self.m_L = 0.15 + self.mass_at_end
        self.m_B = 4.1
        self.m_W = 3.5
        self.h = 0.115
        self.R = 0.2527
        self.g = 9.81
        
        self.I_w = 0.0459142776779452      # Wheel inertia
        self.I_rod = 1/12 * (0.3 ) * (0.314**2) + 2 * self.mass_at_end * 0.157**2   # Rod inertia
        self.I_b = 0.012904182130007      # Body inertia


        ### PD gains
        self.kp_theta = -781.4476
        self.kd_theta = -146.7735
        self.kp_r = 229.6972
        self.kd_r = 41.4447


        ### Coupleness Controller Parameters
        self.delta_t = 5.4
        self.J = self.m_W * self.R**2 + self.m_B * (self.R + self.h)**2 + self.I_w + self.I_b + self.I_rod
        self.G = self.m_W * self.g * self.R + self.m_B * self.g * (self.R + self.h)
        self.m = 2 * self.m_L

        self.k1_ = self.kp_theta + self.R * self.kp_r
        self.k2_ = self.kp_r
        self.k3_ = self.kd_theta + self.R * self.kd_r
        self.k4_ = self.kd_r

        self.K_1 = self.k1_ / (self.R/self.J * self.delta_t)
        self.K_2 = self.k2_ / (1/self.m * self.delta_t)
        self.K_3 = self.k3_ / (self.R/self.J)
        self.K_4 = self.k4_ / (1/self.m)
        
        self.alpha = self.K_1 * self.R**2 * self.delta_t**2 / self.J**2 + self.K_2 * self.delta_t**2 / self.m**2 + self.K_3 * self.R**2 / self.J**2 + self.K_4 / self.m**2

if __name__ == "__main__":
    params = SystemParameters()
    print("System Parameters:")
    for attr, value in params.__dict__.items():
        print(f"{attr}: {value}")

        


        