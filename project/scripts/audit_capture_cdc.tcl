# Per-source-bit worst setup paths; zero paths is a failure.
set coverage [open [file join $audit_dir cdc_coverage.tsv] w]
puts $coverage "group	cell	destination	report"
set cdc_index 0
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/gate_opcode(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 2 || [llength $cdc_cells] > 2} { error "CDC group cmd physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "cmd	$cdc_cell	cmos_pclk	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock cmos_pclk -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/gate_tag(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 32 || [llength $cdc_cells] > 32} { error "CDC group tag physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "tag	$cdc_cell	clk_sys	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_sys -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/gate_pixels(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 20 || [llength $cdc_cells] > 20} { error "CDC group pixels physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "pixels	$cdc_cell	clk_sys	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_sys -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/gate_crc(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 32 || [llength $cdc_cells] > 32} { error "CDC group crc physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "crc	$cdc_cell	clk_sys	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_sys -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/gate_bad(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 1 || [llength $cdc_cells] > 1} { error "CDC group geometry physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "geometry	$cdc_cell	clk_sys	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_sys -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/check_crc(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 32 || [llength $cdc_cells] > 32} { error "CDC group check_crc physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "check_crc	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/check_match(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 1 || [llength $cdc_cells] > 1} { error "CDC group check_match physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "check_match	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/event_tag(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 32 || [llength $cdc_cells] > 32} { error "CDC group event_tag physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "event_tag	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/event_crc(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 32 || [llength $cdc_cells] > 32} { error "CDC group event_crc physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "event_crc	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/event_error(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 4 || [llength $cdc_cells] > 32} { error "CDC group event_error physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "event_error	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/event_locked(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 1 || [llength $cdc_cells] > 1} { error "CDC group event_locked physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "event_locked	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/event_restored(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 1 || [llength $cdc_cells] > 1} { error "CDC group event_restored physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "event_restored	$cdc_cell	clk_lvds_1x	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_lvds_1x -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
set cdc_cells [get_cells -regexp {^u_ar0135_capture_link/freezer/check_bad_reg(\[[0-9]+\])?~FF$}]
if {[llength $cdc_cells] < 1 || [llength $cdc_cells] > 1} { error "CDC group check_bad physical bit count mismatch" }
foreach cdc_cell $cdc_cells {
 set cdc_name [format "cdc_%03d.rpt" $cdc_index]
 puts $coverage "check_bad	$cdc_cell	clk_sys	$cdc_name"
 report_timing -setup -from [get_cells $cdc_cell] -to_clock clk_sys -npaths 1000 -nworst 1 -detail full_path -file [file join $report_dir $cdc_name]
 incr cdc_index
}
close $coverage
puts CAPTURE_CDC_REPORTS_WRITTEN
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/link_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/lr\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_link_setup.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/lr\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_link_fanout_setup.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/link_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/lr\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_link_hold.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/lr\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_link_fanout_hold.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/freezer/c_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/freezer/cr\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_cmos_setup.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/freezer/cr\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_cmos_fanout_setup.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/freezer/c_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/freezer/cr\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_cmos_hold.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/freezer/cr\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_cmos_fanout_hold.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/freezer/a_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/freezer/ar\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_axi_setup.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/freezer/ar\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_axi_fanout_setup.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/freezer/a_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/freezer/ar\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_axi_hold.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/freezer/ar\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_axi_fanout_hold.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/freezer/l_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/freezer/lr\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_lcd_setup.rpt]
report_timing -setup -from [get_pins -regexp {^u_ar0135_capture_link/freezer/lr\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_lcd_fanout_setup.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/freezer/l_reset/release_pipe\[0\]\~FF\|Q$}] -to [get_pins -regexp {^u_ar0135_capture_link/freezer/lr\~FF\|D$}] -npaths 1 -detail full_path -file [file join $report_dir reset_lcd_hold.rpt]
report_timing -hold -from [get_pins -regexp {^u_ar0135_capture_link/freezer/lr\~FF\|Q$}] -npaths 10000 -nworst 1 -detail full_path -file [file join $report_dir reset_lcd_fanout_hold.rpt]
puts CAPTURE_RESET_REPORTS_WRITTEN
