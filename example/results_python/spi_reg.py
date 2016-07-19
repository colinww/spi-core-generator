"""
Automatically generated registers
"""

from collections import namedtuple

register = namedtuple("register", "addr width value rmask wmask")

# Test registers
class TestRegs:
	def __init__(self):
		self.rfdc_state_0 = register(0, 30, 0, 1, 1)
		self.fdelta_cal_0 = register(1, 25, 0, 1, 1)
		self.dlc_qnc = register(11, 33, 0, 1, 1)
		self.rfdc_state_1 = register(5, 30, 0, 1, 1)
		self.dco = register(18, 9, 0, 1, 1)
		self.dlc = register(17, 22, 0, 1, 1)
		self.dlc_f2p = register(10, 20, 0, 1, 1)
		self.dlc_iir_2 = register(14, 22, 0, 1, 1)
		self.rfdc_0 = register(4, 28, 0, 1, 1)
		self.rfdc_1 = register(9, 28, 0, 1, 1)
		self.fdelta_lms_0 = register(2, 37, 0, 1, 1)
		self.fdelta_lms_1 = register(7, 37, 0, 1, 1)
		self.dcycle_cal_0 = register(3, 20, 0, 1, 1)
		self.dcycle_cal_1 = register(8, 20, 0, 1, 1)
		self.fdelta_cal_1 = register(6, 25, 0, 1, 1)
		self.dlc_iir_3 = register(15, 22, 0, 1, 1)
		self.dlc_iir_0 = register(12, 22, 0, 1, 1)
		self.dlc_iir_1 = register(13, 22, 0, 1, 1)
		self.dlc_pi = register(16, 34, 0, 1, 1)
