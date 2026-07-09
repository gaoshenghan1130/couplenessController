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
        