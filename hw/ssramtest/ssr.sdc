create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
report_timing -setup -max_paths 5 -max_common_paths 1
