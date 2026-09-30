"""Six 2 mm cables, sharing two connectors. Dimensions are metres."""
COUNT = 6
RADIUS = .001
PITCH = .0025
OFFSETS = [(i-(COUNT-1)/2)*PITCH for i in range(COUNT)]
AREA_FACTOR = (RADIUS/.0015)**2
GRASP_DAMPING = .06*AREA_FACTOR
FORCE_CAP = .5
CLIP_SCALE = 1.0
FINAL_X = .018
