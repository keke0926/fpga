set audit_dir [file normalize outflow/median_audit]
file mkdir $audit_dir
set cells [get_cells {u_lvds_sobel_link/*}]
set pins [get_pins $cells]
if {[llength $cells] == 0 || [llength $pins] == 0} {error "Empty median/Sobel selection"}
set f [open [file join $audit_dir selection.rpt] w]
puts $f "cell_count=[llength $cells]"
puts $f "pin_count=[llength $pins]"
close $f
check_timing -verbose -file [file join $audit_dir timing_checks.rpt]
report_clocks -file median_audit/clocks.rpt
report_timing_summary -file median_audit/timing_summary.rpt
report_timing -setup -npaths 100 -nworst 1 -detail full_path -file median_audit/global_setup100.rpt
report_timing -hold -npaths 100 -nworst 1 -detail full_path -file median_audit/global_hold100.rpt
report_timing -setup -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit/median_setup100.rpt
report_timing -hold -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit/median_hold100.rpt
write_sdc [file join $audit_dir effective.sdc]
puts MEDIAN_AUDIT_PASS
