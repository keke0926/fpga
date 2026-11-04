# Capture-specific reports, not a timing-closure waiver. Existing SDC untouched.
set audit_dir [file normalize outflow/capture_audit]
file mkdir $audit_dir
set report_dir capture_audit
set cells [get_cells {u_ar0135_capture_link/*}]
set pins {}
if {[llength $cells] > 0} { set pins [get_pins $cells] }
set selection [open [file join $audit_dir selection.rpt] w]
puts $selection "cell_count=[llength $cells]"
puts $selection "pin_count=[llength $pins]"
close $selection
if {[llength $cells] == 0 || [llength $pins] == 0} { error "No capture cells/pins selected" }
check_timing -verbose -file [file join $audit_dir timing_checks.rpt]
report_clocks -file [file join $report_dir clocks.rpt]
report_timing_summary -file [file join $report_dir timing_summary.rpt]
report_timing -setup -npaths 100 -nworst 1 -detail full_path -file [file join $report_dir global_setup100.rpt]
report_timing -hold -npaths 100 -nworst 1 -detail full_path -file [file join $report_dir global_hold100.rpt]
report_timing -setup -through $pins -npaths 100 -nworst 1 -detail full_path -file [file join $report_dir capture_setup100.rpt]
report_timing -hold -through $pins -npaths 100 -nworst 1 -detail full_path -file [file join $report_dir capture_hold100.rpt]
report_cdc -file [file join $audit_dir cdc_summary.rpt]
report_cdc -details -file [file join $audit_dir cdc_details.rpt]
write_sdc [file join $audit_dir effective.sdc]
source scripts/audit_capture_cdc.tcl
foreach name {
 selection.rpt timing_checks.rpt clocks.rpt timing_summary.rpt
 global_setup100.rpt global_hold100.rpt capture_setup100.rpt capture_hold100.rpt
 cdc_summary.rpt cdc_details.rpt effective.sdc cdc_coverage.tsv
} {
 set path [file join $audit_dir $name]
 if {![file exists $path] || [file size $path] == 0} { error "Missing audit artifact: $path" }
}
puts CAPTURE_AUDIT_PASS
