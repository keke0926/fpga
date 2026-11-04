# efx_run.py's sta_tclsh wrapper does not forward --timing_model.
set_timing_model I3
puts "AUDIT_TIMING_MODEL=[get_timing_model]"
if {[get_timing_model] ne "I3"} {error "Expected actual I3 timing model"}
set audit_dir [file normalize outflow/median_audit_I3]
file mkdir $audit_dir
set cells [get_cells {u_lvds_sobel_link/*}]
set pins [get_pins $cells]
if {[llength $cells] == 0 || [llength $pins] == 0} {error "Empty median/Sobel selection"}
set f [open [file join $audit_dir selection.rpt] w]
puts $f "cell_count=[llength $cells]"
puts $f "pin_count=[llength $pins]"
close $f
check_timing -verbose -file [file join $audit_dir timing_checks.rpt]
report_clocks -file median_audit_I3/clocks.rpt
report_timing_summary -file median_audit_I3/timing_summary.rpt
report_timing -setup -npaths 100 -nworst 1 -detail full_path -file median_audit_I3/global_setup100.rpt
report_timing -hold -npaths 100 -nworst 1 -detail full_path -file median_audit_I3/global_hold100.rpt
report_timing -setup -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit_I3/median_setup100.rpt
report_timing -hold -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit_I3/median_hold100.rpt
report_timing -setup -from [get_clocks clk_lvds_1x] -to [get_clocks clk_lvds_1x] -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit_I3/median_internal_setup100.rpt
report_timing -hold -from [get_clocks clk_lvds_1x] -to [get_clocks clk_lvds_1x] -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit_I3/median_internal_hold100.rpt
write_sdc [file join $audit_dir effective.sdc]
puts MEDIAN_AUDIT_PASS
