"""Generate imx462.json: the base IPA tuning plus one profile per exposure choice.

The ISP reads its AGC tuning once, at camera start, keyed by sensor name, so
each metering mode x exposure compensation is its own named profile and the
driver reports the name of the one selected (imx462_set_profile()).

    python gen_profiles.py            # rewrites imx462.json from imx462_base.json

Edit imx462_base.json, never imx462.json.
"""
import collections, copy, json, os

HERE = os.path.dirname(os.path.abspath(__file__))
base = json.load(open(os.path.join(HERE, "imx462_base.json")),
                 object_pairs_hook=collections.OrderedDict)
tuning = base["IMX462"]

# Metering. Weights are the 5x5 AE zones, top row first.
METERS = [
    ("AVG", "light_threshold_priority", [1] * 25, None),
    ("CENTRE", "low_light_priority",
     [1, 1, 1, 1, 1,
      1, 2, 2, 2, 1,
      2, 3, 4, 3, 2,
      2, 3, 4, 3, 2,
      1, 2, 3, 2, 1],
     {"low_threshold": 25, "high_threshold": 120, "weight_offset": 4, "luma_offset": 2}),
    ("SPOT", "light_threshold_priority",
     [0, 0, 0, 0, 0,
      0, 1, 2, 1, 0,
      0, 2, 16, 2, 0,
      0, 1, 2, 1, 0,
      0, 0, 0, 0, 0], None),
    # Bright sky behind the subject: the top two rows barely count, and dark
    # zones pull hard.
    ("BACKLIT", "low_light_priority",
     [0, 0, 0, 0, 0,
      1, 1, 1, 1, 1,
      2, 3, 4, 3, 2,
      3, 4, 5, 4, 3,
      2, 3, 4, 3, 2],
     {"low_threshold": 20, "high_threshold": 140, "weight_offset": 8, "luma_offset": 4}),
]

# Exposure compensation: AGC luma target. The AE statistics are taken before
# gamma, so one stop is roughly a doubling of the target.
EVS = [(-2, 20), (-1, 38), (0, 70), (1, 125), (2, 185)]


def profile(meter, ev_target):
    name, mode, weights, llp = meter
    t = copy.deepcopy(tuning)
    agc = t["agc"]
    la = agc["luma_adjust"]
    la["target"] = ev_target
    la["target_low"] = max(1, ev_target - 6)
    la["target_high"] = min(250, ev_target + 6)
    la["weight"] = weights
    agc["mode"] = mode
    if llp:
        agc["low_light_priority"] = collections.OrderedDict(llp)
    if name == "BACKLIT":
        la["high_regions"] = 20
    return t


out = collections.OrderedDict()
out["version"] = base["version"]
out["IMX462"] = profile(METERS[1], 70)          # default: CENTRE, EV 0
for mi, m in enumerate(METERS):
    for ei, (ev, target) in enumerate(EVS):
        out[f"IMX462_M{mi}_E{ei}"] = profile(m, target)

json.dump(out, open(os.path.join(HERE, "imx462.json"), "w"), indent=4)
print(f"{len(out) - 1} profiles")
