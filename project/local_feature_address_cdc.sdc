# C2-only retained address bits; existing local_ddr_cdc.sdc is unchanged.
# req_addr_hold is written only with a new source request and remains stable
# until the returned response is consumed. d_state exposes the request only
# after req_sync[1] crosses the two destination synchronizer stages.
# Match the existing bundle contract: 8 ns < one 96 MHz destination cycle;
# no payload false path, no change to previous endpoints or timing budgets.
set feature_address_registers [get_cells {analysis_req_addr[0]~FF analysis_req_addr[1]~FF analysis_req_addr[24]~FF analysis_req_addr[25]~FF analysis_req_addr[26]~FF analysis_req_addr[27]~FF analysis_req_addr[28]~FF analysis_req_addr[29]~FF analysis_req_addr[2]~FF analysis_req_addr[30]~FF analysis_req_addr[31]~FF analysis_req_addr[3]~FF}]
if {[llength $feature_address_registers] != 12} {error "Feature address bundle changed"}
set_max_delay -from $feature_address_registers -to [get_clocks clk_sys] 8
set_min_delay -from $feature_address_registers -to [get_clocks clk_sys] 0
