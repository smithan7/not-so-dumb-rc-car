"""Shared physical constants and unit conversions.

Convention: ``a_to_b`` is the factor you MULTIPLY a value in unit ``a`` by to get unit ``b``.
"""

g = 9.81  # m/s^2

mph_to_mps = 0.44704
mps_to_mph = 1.0 / mph_to_mps

in_to_m = 0.0254
mm_to_in = 0.03937008
in_to_mm = 1.0 / mm_to_in

lbs_to_kg = 0.45359237

ft_lbs_to_Nm = 1.355818
oz_in_to_ft_lbs = 1.0 / 192.0
oz_in_to_Nm = 1.0 / 141.61193227806
