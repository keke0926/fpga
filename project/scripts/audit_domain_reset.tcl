# efx_run.py's sta_tclsh wrapper does not forward --timing_model.
set_timing_model I3
puts "AUDIT_TIMING_MODEL=[get_timing_model]"
if {[get_timing_model] ne "I3"} {error "Expected actual I3 timing model"}
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
report_timing -setup -from_clock clk_lvds_1x -to_clock clk_lvds_1x -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit/median_internal_setup100.rpt
report_timing -hold -from_clock clk_lvds_1x -to_clock clk_lvds_1x -through $pins -npaths 100 -nworst 1 -detail full_path -file median_audit/median_internal_hold100.rpt
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
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/reader_functional_counter_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/reader_functional_counter_hold.rpt
set selected [get_pins {u_axi4_ctrl/rframe_vsync_dly~FF|SR u_axi4_ctrl/rfifo_rst~FF|SR}]
if {[llength $selected] != 2} {error "Functional/stage selection reader_functional_other changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/reader_functional_other_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/reader_functional_other_hold.rpt
set selected [get_pins {u_axi4_ctrl/rframe_reset_n~FF|D u_axi4_ctrl/r_rfifo_rst~FF|D}]
if {[llength $selected] != 2} {error "Functional/stage selection local_release_stages changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/local_release_stages_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/local_release_stages_hold.rpt
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8.*genblk2\..*_rst\[1\]~FF\|D$}]
if {[llength $selected] != 2} {error "Functional/stage selection fifo_release_stages changed"}
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/fifo_release_stages_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/fifo_release_stages_hold.rpt
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/xefx_fifo_ctl/genblk7\.wr2rd_addr_sync/dff_5/i[0-9]+_2\|D$}]
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/gray_sys_to_reader_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/gray_sys_to_reader_hold.rpt
set selected [get_pins -regexp {^u_axi4_ctrl/Gen_RFIFO_8\.u_R0_FIFO_8/u_efx_fifo_top/xefx_fifo_ctl/genblk7\.xrd2wr_addr_sync/dff_5/i[0-9]+_2\|D$}]
report_timing -setup -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/gray_reader_to_sys_setup.rpt
report_timing -hold -to $selected -npaths 100 -nworst 1 -detail full_path -file median_audit/gray_reader_to_sys_hold.rpt

set rf [open [file join $audit_dir runtime_selection.rpt] w]
set cap_cells [get_cells {u_ar0135_capture_link/*}]
set run_cells [get_cells -regexp {^u_ar0135_capture_link/runtime_(control|demux|response)/.*$}]
if {[llength $cap_cells] < 2446 || [llength $run_cells] == 0} {error "Runtime logic absent"}
puts $rf "capture_cells=[llength $cap_cells]"
puts $rf "runtime_cells=[llength $run_cells]"
puts $rf $run_cells
close $rf
set run_pins [get_pins $run_cells]
report_timing -setup -through $run_pins -npaths 100 -nworst 1 -detail full_path -file median_audit/runtime_setup100.rpt
report_timing -hold -through $run_pins -npaths 100 -nworst 1 -detail full_path -file median_audit/runtime_hold100.rpt


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
 report_timing -setup -through $dp -npaths 100 -nworst 1 -detail full_path -file median_audit/${label}_setup100.rpt
 report_timing -hold -through $dp -npaths 100 -nworst 1 -detail full_path -file median_audit/${label}_hold100.rpt
}
close $df
set rs [get_pins -regexp {^u_lvds_sobel_link/.*reset_pipe.*~FF\|D$}]
if {[llength $rs] == 0} {error "Missing dynamic pipeline reset release stages"}
report_timing -setup -to $rs -npaths 100 -nworst 1 -detail full_path -file median_audit/video_reset_setup100.rpt
report_timing -hold -to $rs -npaths 100 -nworst 1 -detail full_path -file median_audit/video_reset_hold100.rpt
puts DYNAMIC_VIDEO_AUDIT_PASS

puts RUNTIME_CONTROL_AUDIT_PASS
puts DOMAIN_RESET_AUDIT_PASS
