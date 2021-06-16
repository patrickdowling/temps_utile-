#!/usr/bin/env python3
# Generate table of time divisions for scope app

px = 128
freqs = [ (2000, 1), (1000, 1), (500, 1), (200, 1), (100, 1), (50, 1), (20, 1), (10, 2), (5, 4), (2, 8), (1, 16), (0.5, 16), (0.2, 32) ]

def generate_label(timebase):
    secs = 1 / timebase

    if secs >= 1:
        return "{}s".format(int(secs))

    secs *= 1000
    if secs >= 1:
        return "{}m".format(int(secs))

    secs *= 1000
    return "{}u".format(int(secs))

def generate_parameters():
    return [(generate_label(t[0]), int(t[0] * t[1] * px), t[1]) for t in freqs]

if __name__ == "__main__":

    parameters = generate_parameters()

    print("// BEGIN generated via resources/scope_divs.py")
    print("// clang-format off")
    print("enum Timebase {")
    for p in parameters:
        print(f"  TIMEBASE_{p[0]},")
    print("  TIMEBASE_LAST,")
    print("};")

    print("")
    print("static constexpr TimebaseParameters kTimebaseParameters[TIMEBASE_LAST] = {")
    for p in parameters:
        label, adc_freq, decimate = p
        print(f"{{ .label = \"{label:>4}\", .adc_frequency = {adc_freq}, .decimate = {decimate} }},")
    print("};")

    print("// clang-format on")
    print("// END generated")
