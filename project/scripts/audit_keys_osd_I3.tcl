# Retains all seed3 audit scopes, including restored flag4 guard policy reports.
# No timing exceptions are added by this audit script.
# efx_run.py's sta_tclsh wrapper does not forward --timing_model.
set_timing_model I3
puts "AUDIT_TIMING_MODEL=[get_timing_model]"
if {[get_timing_model] ne "I3"} {error "Expected actual I3 timing model"}
set audit_dir [file normalize outflow/$::env(BOARD_FRONT_AUDIT_NAME)]
file mkdir $audit_dir
set cells [get_cells {u_lvds_sobel_link/*}]
set pins [get_pins $cells]
if {[llength $cells] == 0 || [llength $pins] == 0} {error "Empty median/Sobel selection"}
set f [open [file join $audit_dir selection.rpt] w]
puts $f "cell_count=[llength $cells]"
puts $f "pin_count=[llength $pins]"
close $f
check_timing -verbose -file [file join $audit_dir timing_checks.rpt]
report_clocks -file $::env(BOARD_FRONT_AUDIT_NAME)/clocks.rpt
report_timing_summary -file $::env(BOARD_FRONT_AUDIT_NAME)/timing_summary.rpt
report_timing -setup -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/global_setup100.rpt
report_timing -hold -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/global_hold100.rpt
report_timing -setup -through $pins -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/median_setup100.rpt
report_timing -hold -through $pins -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/median_hold100.rpt
report_timing -setup -from_clock clk_lvds_1x -to_clock clk_lvds_1x -through $pins -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/median_internal_setup100.rpt
report_timing -hold -from_clock clk_lvds_1x -to_clock clk_lvds_1x -through $pins -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/median_internal_hold100.rpt
write_sdc [file join $audit_dir effective.sdc]
foreach name {selection.rpt clocks.rpt timing_summary.rpt timing_checks.rpt effective.sdc global_setup100.rpt global_hold100.rpt median_setup100.rpt median_hold100.rpt median_internal_setup100.rpt median_internal_hold100.rpt} {
    if {![file exists [file join $audit_dir $name]] || [file size [file join $audit_dir $name]] == 0} {error "Missing/empty audit file $name"}
}
set sf [open [file join $audit_dir reset_cdc_selection.rpt] w]
set selected [get_pins -regexp {^u_axi4_ctrl/u_rframe_reset/release_pipe\[0\]~FF\|SR$}]
if {[llength $selected] != 1} {error "Audit selection reader_first changed"}
puts $sf "reader_first=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/rframe_reset_n~FF\|SR$}]
if {[llength $selected] != 1} {error "Audit selection reader_second changed"}
puts $sf "reader_second=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/u_rfifo_axi_reset/release_pipe\[0\]~FF\|SR$}]
if {[llength $selected] != 1} {error "Audit selection axi_first changed"}
puts $sf "axi_first=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/r_rfifo_rst~FF\|SR$}]
if {[llength $selected] != 1} {error "Audit selection axi_second changed"}
puts $sf "axi_second=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/genblk2\.wr_rst\[[01]\]~FF\|SR$}]
if {[llength $selected] != 2} {error "Audit selection fifo_wr changed"}
puts $sf "fifo_wr=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/genblk2\.rd_rst\[[01]\]~FF\|SR$}]
if {[llength $selected] != 2} {error "Audit selection fifo_rd changed"}
puts $sf "fifo_rd=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/xefx_fifo_ctl/genblk7\.wr2rd_addr_sync/dff_5/i[0-9]+_2\|D$}]
if {[llength $selected] != 10} {error "Audit selection gray_sys_to_reader changed"}
puts $sf "gray_sys_to_reader=[llength $selected] $selected"
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/xefx_fifo_ctl/genblk7\.xrd2wr_addr_sync/dff_5/i[0-9]+_2\|D$}]
if {[llength $selected] != 10} {error "Audit selection gray_reader_to_sys changed"}
puts $sf "gray_reader_to_sys=[llength $selected] $selected"
close $sf
set selected [get_pins -regexp {^u_axi4_ctrl/rfifo_cnt\[[0-4]\]~FF\|SR$}]
if {[llength $selected] != 5} {error "Functional/stage selection reader_functional_counter changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/reader_functional_counter_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/reader_functional_counter_hold.rpt
set selected [get_pins {u_axi4_ctrl/rframe_vsync_dly~FF|SR u_axi4_ctrl/rfifo_rst~FF|SR}]
if {[llength $selected] != 2} {error "Functional/stage selection reader_functional_other changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/reader_functional_other_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/reader_functional_other_hold.rpt
set selected [get_pins {u_axi4_ctrl/rframe_reset_n~FF|D u_axi4_ctrl/r_rfifo_rst~FF|D}]
if {[llength $selected] != 2} {error "Functional/stage selection local_release_stages changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/local_release_stages_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/local_release_stages_hold.rpt
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8.*genblk2\..*_rst\[1\]~FF\|D$}]
if {[llength $selected] != 2} {error "Functional/stage selection fifo_release_stages changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/fifo_release_stages_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/fifo_release_stages_hold.rpt
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/xefx_fifo_ctl/genblk7\.wr2rd_addr_sync/dff_5/i[0-9]+_2\|D$}]
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/gray_sys_to_reader_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/gray_sys_to_reader_hold.rpt
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/xefx_fifo_ctl/genblk7\.xrd2wr_addr_sync/dff_5/i[0-9]+_2\|D$}]
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/gray_reader_to_sys_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/gray_reader_to_sys_hold.rpt

set rf [open [file join $audit_dir runtime_selection.rpt] w]
set cap_cells [get_cells {u_ar0135_capture_link/*}]
set run_cells [get_cells -regexp {^u_ar0135_capture_link/runtime_(control|demux|response)/.*$}]
if {[llength $cap_cells] < 2446 || [llength $run_cells] == 0} {error "Runtime logic absent"}
puts $rf "capture_cells=[llength $cap_cells]"
puts $rf "runtime_cells=[llength $run_cells]"
puts $rf $run_cells
close $rf
set run_pins [get_pins $run_cells]
report_timing -setup -through $run_pins -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/runtime_setup100.rpt
report_timing -hold -through $run_pins -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/runtime_hold100.rpt


set df [open [file join $audit_dir dynamic_selection.rpt] w]
foreach {label pattern} {
 dynamic {^u_lvds_sobel_link/dynamic_inst/.*$}
 histogram {^u_lvds_sobel_link/dynamic_inst/.*histogram.*$}
 table {^u_lvds_sobel_link/dynamic_inst/frame_table/.*$}
 gradient {^u_lvds_sobel_link/gradient_inst/.*$}
} {
 set dc [get_cells -regexp $pattern]
 if {[llength $dc] == 0} {error "Empty dynamic selection $label"}
 puts $df "$label=[llength $dc]"
 puts $df $dc
 set dp [get_pins $dc]
 report_timing -setup -through $dp -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/${label}_setup100.rpt
 report_timing -hold -through $dp -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/${label}_hold100.rpt
}
close $df
set rs [get_pins -regexp {^u_lvds_sobel_link/.*reset_pipe.*~FF\|D$}]
if {[llength $rs] == 0} {error "Missing dynamic pipeline reset release stages"}
report_timing -setup -to $rs -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/video_reset_setup100.rpt
report_timing -hold -to $rs -npaths 100 -nworst 1 -detail full_path -file $::env(BOARD_FRONT_AUDIT_NAME)/video_reset_hold100.rpt
puts DYNAMIC_VIDEO_AUDIT_PASS

puts RUNTIME_CONTROL_AUDIT_PASS
puts DOMAIN_RESET_AUDIT_PASS

# Appended to the passed baseline audit by prepare.py. No timing exceptions added.
if {[get_timing_model] ne "I3"} {error "Frontend audit is not actual I3"}
set front_dest [file normalize outflow/$::env(BOARD_FRONT_AUDIT_NAME)]
file mkdir $front_dest
set front_relative $::env(BOARD_FRONT_AUDIT_NAME)
set inclusive_selection [open [file join $front_dest inclusive_selection.rpt] w]
foreach {label pattern} {
 video {(^|.*/)u_lvds_sobel_link/.*$}
 dynamic {(^|.*/)u_lvds_sobel_link/dynamic_inst/.*$}
 histogram {(^|.*/)u_lvds_sobel_link/dynamic_inst/.*histogram.*$}
 table {(^|.*/)u_lvds_sobel_link/dynamic_inst/frame_table/.*$}
 gradient {(^|.*/)u_lvds_sobel_link/gradient_inst/.*$}
} {
    set inclusive_cells [get_cells -regexp $pattern]
    set inclusive_pins [get_pins $inclusive_cells]
    if {[llength $inclusive_cells] == 0 || [llength $inclusive_pins] == 0} {error "Empty inclusive scope $label"}
    puts $inclusive_selection "$label=[llength $inclusive_cells] pins=[llength $inclusive_pins]"
    puts $inclusive_selection $inclusive_cells
    report_timing -setup -through $inclusive_pins -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative inclusive_${label}_setup100.rpt]
    report_timing -hold -through $inclusive_pins -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative inclusive_${label}_hold100.rpt]
}
close $inclusive_selection
set front_selection [open [file join $front_dest frontend_selection.rpt] w]
foreach {label pattern} {
 frontend {(^|.*/)u_lvds_sobel_link/front_inst/.*$}
 nms {(^|.*/)u_lvds_sobel_link/front_inst/nms_window_inst/.*$}
 weak {(^|.*/)u_lvds_sobel_link/front_inst/support_window_inst/.*$}
} {
    set front_cells [get_cells -regexp $pattern]
    set front_pins [get_pins $front_cells]
    if {[llength $front_cells] == 0 || [llength $front_pins] == 0} {error "Empty frontend scope $label"}
    puts $front_selection "$label=[llength $front_cells] pins=[llength $front_pins]"
    puts $front_selection $front_cells
    report_timing -setup -through $front_pins -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative ${label}_setup100.rpt]
    report_timing -hold -through $front_pins -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative ${label}_hold100.rpt]
}
close $front_selection
set auto_d [get_pins {u_ar0135_capture_link/runtime_control/committed_flags[3]~FF|D}]
set auto_q [get_pins {u_ar0135_capture_link/runtime_control/committed_flags[3]~FF|Q}]
set guard4_q [get_pins {u_ar0135_capture_link/runtime_control/committed_flags[4]~FF|Q}]
set policy_d [get_pins {u_lvds_sobel_link/locked_control[13]~FF|D}]
if {[llength $auto_d]!=1 || [llength $auto_q]!=1 || [llength $guard4_q]!=1 || [llength $policy_d]!=1} {error "Guarded mode policy selectors changed"}
foreach mode {setup hold} {
    report_timing -$mode -to $auto_d -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative runtime_auto_${mode}.rpt]
    report_timing -$mode -from $auto_q -to $policy_d -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative runtime_auto_policy_${mode}.rpt]
    report_timing -$mode -from $guard4_q -to $policy_d -npaths 100 -nworst 1 -detail full_path -file [file join $front_relative runtime_guard4_policy_${mode}.rpt]
}
check_timing -verbose -file [file join $front_dest frontend_timing_checks.rpt]
report_clocks -file [file join $front_relative frontend_clocks.rpt]
report_timing_summary -file [file join $front_relative frontend_timing_summary.rpt]
write_sdc [file join $front_dest frontend_effective.sdc]
puts BOARD_FRONT_ACTUAL_I3_AUDIT_PASS


if {[get_timing_model] ne "I3"} {error "Keys/OSD audit is not actual I3"}
set key_dest [file normalize outflow/$::env(BOARD_FRONT_AUDIT_NAME)]
set key_relative $::env(BOARD_FRONT_AUDIT_NAME)
set key_select [open [file join $key_dest keys_selection.rpt] w]
foreach {label port} {app key_app_n_i auto key_auto_n_i view key_view_n_i plus key_plus_n_i minus key_minus_n_i detect key_detect_n_i} {
    set first [get_cells -regexp "^u_ar0135_capture_link/u_key_${label}/sync_meta~FF\$"]
    set second [get_cells -regexp "^u_ar0135_capture_link/u_key_${label}/sync_button_n~FF\$"]
    set pad [get_ports $port]
    if {[llength $first]!=1 || [llength $second]!=1 || [llength $pad]!=1} {error "Non-unique key synchronizer $label"}
    puts $key_select "$label pad=$pad first=$first second=$second"
    set first_q [get_pins -regexp "^u_ar0135_capture_link/u_key_${label}/sync_meta~FF\\|Q\$"]
    set second_d [get_pins -regexp "^u_ar0135_capture_link/u_key_${label}/sync_button_n~FF\\|D\$"]
    if {[llength $first_q]!=1 || [llength $second_d]!=1} {error "Missing exact key stage Q/D $label"}
    foreach mode {setup hold} {
        report_timing -$mode -from $first_q -to $second_d -npaths 100 -nworst 1 -detail full_path -file [file join $key_relative keys_${label}_stage12_${mode}.rpt]
    }
}
close $key_select
foreach {label pattern} {
 keys {^u_ar0135_capture_link/u_key_(app|auto|view|plus|minus|detect)/.*$}
 keys_control {^u_ar0135_capture_link/runtime_control/.*$}
 status_osd {^u_status_osd/.*$}
} {
    set scope_cells [get_cells -regexp $pattern]
    set scope_pins [get_pins $scope_cells]
    if {[llength $scope_cells]==0 || [llength $scope_pins]==0} {error "Empty keys/OSD audit scope $label"}
    foreach mode {setup hold} {
        report_timing -$mode -through $scope_pins -npaths 100 -nworst 1 -detail full_path -file [file join $key_relative ${label}_${mode}100.rpt]
    }
}
check_timing -verbose -file [file join $key_dest keys_osd_timing_checks.rpt]
write_sdc [file join $key_dest keys_osd_effective.sdc]
puts KEYS_OSD_ACTUAL_I3_AUDIT_PASS
