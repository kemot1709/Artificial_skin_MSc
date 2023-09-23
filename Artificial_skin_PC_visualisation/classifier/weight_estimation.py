HYPERBOLE_A = 35108.0
HYPERBOLE_X0 = 0.0
HYPERBOLE_Y = 0.42
DIVIDER_RESISTANCE = 470
WEIGHT_MULTIPLIER = 2.5740


def estimate_weight(raw_calibrated_image, max_value=4095):
    weight = 0

    for row in raw_calibrated_image:
        for tile in row:
            # Have to be rounded because otherwise have calculation errors with small weight
            normalized_pressure = round(tile * (4095 / max_value), 1)
            try:
                if normalized_pressure >= 4095.0:
                    continue
                resistance = (DIVIDER_RESISTANCE * normalized_pressure) / (4095.0 - normalized_pressure)
                weight += (HYPERBOLE_X0 + (HYPERBOLE_A / (resistance - HYPERBOLE_Y)))
            except ZeroDivisionError:
                continue

    if weight <= 0.0:
        return 0.0

    weight = weight * WEIGHT_MULTIPLIER
    return weight
