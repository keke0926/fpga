# Dedicated active-low detection key: only pad -> first synchronizer D is asynchronous.
# The sync_meta Q -> sync_button_n D path remains timed.
set_false_path -from [get_ports {key_detect_n_i}] -to [get_pins -regexp {^u_ar0135_capture_link/u_key_detect/sync_meta~FF\|D$}]
