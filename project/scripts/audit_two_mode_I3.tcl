# New additive scopes; preserve all baseline timing and object checks.
source scripts/audit_keys_osd_I3.tcl
if {[get_timing_model] ne "I3"} {error "Application audit is not actual I3"}
if {![info exists ::env(TWO_MODE_AUDIT_SCOPES)] || $::env(TWO_MODE_AUDIT_SCOPES) eq ""} {
    error "No declared application scopes"
}
set app_dest [file normalize outflow/$::env(BOARD_FRONT_AUDIT_NAME)]
set app_selection [open [file join $app_dest app_selection.rpt] w]
foreach prefix [split $::env(TWO_MODE_AUDIT_SCOPES) ,] {
    if {![regexp {^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$} $prefix]} {
        error "Unsafe application scope"
    }
    set scope_cells [get_cells -regexp "^${prefix}/.*\$"]
    set scope_pins [get_pins $scope_cells]
    if {[llength $scope_cells]==0 || [llength $scope_pins]==0} {error "Empty application scope $prefix"}
    set label [string map {/ _} $prefix]
    puts $app_selection "$prefix cells=[llength $scope_cells] pins=[llength $scope_pins]"
    puts $app_selection $scope_cells
    foreach mode {setup hold} {
        report_timing -$mode -through $scope_pins -npaths 100 -nworst 1 -detail full_path -file [file join $::env(BOARD_FRONT_AUDIT_NAME) app_${label}_${mode}100.rpt]
    }
}
close $app_selection
puts TWO_MODE_ACTUAL_I3_AUDIT_PASS
