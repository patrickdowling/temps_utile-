#!/usr/bin/env python3
# Generate table of time divisions for scope app

px = 128
freqs = [ 2000, 1000, 500, 200, 100, 50, 20, 10 ]

def generate_label(timebase):
    secs = 1 / timebase
    secs *= 1000
    if secs >= 1:
        return "{:3}m".format(int(secs))

    secs *= 1000
    return "{:3}u".format(int(secs))

def generate_parameters():
    return [(generate_label(t), t * px) for t in freqs]

if __name__ == "__main__":

    print("// BEGIN generated via resources/scope_divs.py")
    print("// clang-format off")
    print("enum Timebase {")
    for t in freqs:
        print(f"  TIMEBASE_{t},")
    print("  TIMEBASE_LAST,")
    print("};")

    print("")
    print("static constexpr TimebaseParameters kTimebaseParameters[TIMEBASE_LAST] = {")
    for p in generate_parameters():
        label, adc_freq = p
        print(f"{{ .label = \"{label}\", .adc_frequency = {adc_freq} }},")
    print("};")

    print("// clang-format on")
    print("// END generated")
