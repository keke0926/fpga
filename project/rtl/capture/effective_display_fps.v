`timescale 1ns/1ps
// Complete geometry follows from bounded line count and exact per-line width.
// Epoch width defaults to24 bits; public capture identities remain32 bits.
// Passive source-generation observer. A scan of an old buffer is not an update.
// The AXI mailbox changes only on the existing once-per-raster read commitment;
// its bundle settles before the synchronized toggle is consumed in LCD domain.
module effective_display_fps #(
 parameter WIDTH=1024,HEIGHT=600,LCD_CLOCK_HZ=48000000,GENERATION_BITS=24)(
 input wire clk_cmos_i,rst_cmos_n_i,src_vs_i,src_de_i,
 input wire clk_axi_i,rst_axi_n_i,ddr_ready_i,
 input wire[6:0] axi_observation_i,
 input wire clk_lcd_i,rst_lcd_n_i,lcd_vs_i,lcd_de_i,lcd_underflow_i,
 input wire hold_begin_i,hold_end_i,
 output reg[11:0] fps_bcd_o);
 function integer bits_for;input integer n;integer v;begin
  bits_for=1;v=n-1;while(v>1)begin bits_for=bits_for+1;v=v>>1;end
 end endfunction
 localparam XW=bits_for(WIDTH+1),
   YW=bits_for(HEIGHT+1),TW=bits_for(LCD_CLOCK_HZ);
 // Burst byte count can round a short source frame up. Check the real gated
 // camera raster as well; its EOF result stays stable until the next EOF.
 reg camera_vs_d,camera_de_d,camera_started,camera_bad,camera_good,camera_toggle;
 reg[XW-1:0] camera_row_pixels;reg[YW-1:0] camera_rows;
 always @(posedge clk_cmos_i)begin
  if(!rst_cmos_n_i)begin
   camera_vs_d<=0;camera_de_d<=0;camera_started<=0;camera_bad<=0;camera_good<=0;camera_toggle<=0;
   camera_row_pixels<=0;camera_rows<=0;
  end else begin
   camera_vs_d<=src_vs_i;camera_de_d<=src_de_i;
   if(src_vs_i&&!camera_vs_d)begin
    camera_started<=1;camera_bad<=0;
    camera_row_pixels<=src_de_i?1:0;camera_rows<=0;
   end else if(camera_started)begin
    if(src_de_i)begin
     if(!src_vs_i)camera_bad<=1;
     if(camera_row_pixels<WIDTH)camera_row_pixels<=camera_row_pixels+1'b1;else camera_bad<=1;
    end
    if(camera_de_d&&!src_de_i)begin
     if(camera_row_pixels!=WIDTH)camera_bad<=1;
     camera_row_pixels<=0;if(camera_rows<HEIGHT)camera_rows<=camera_rows+1'b1;else camera_bad<=1;
    end
    if(camera_vs_d&&!src_vs_i)begin
     camera_good<=!camera_bad&&!src_de_i&&
      (camera_rows+(camera_de_d&&!src_de_i))==HEIGHT&&
      (camera_de_d?camera_row_pixels==WIDTH:camera_row_pixels==0);
     camera_toggle<=~camera_toggle;camera_started<=0;
    end
   end
  end
 end
 (* async_reg="true" *)reg[1:0] camera_sync;
 reg camera_seen,pending_write,pending_full,pending_read;
 reg[1:0] pending_bank,pending_read_bank;
 wire write_done=axi_observation_i[5],write_full=axi_observation_i[6];
 wire[1:0] write_bank=axi_observation_i[4:3],read_bank=axi_observation_i[1:0];
 wire read_commit=axi_observation_i[2];
 reg[GENERATION_BITS-1:0] generation,bank0,bank1,bank2,bank3;
 reg[3:0] bank_valid;
 reg[GENERATION_BITS:0] mailbox;reg publish;
 wire camera_fresh=camera_sync[1]!=camera_seen;
 wire collision=pending_write&&write_done;
 wire resolve_write=(pending_write||write_done)&&camera_fresh&&!collision;
 wire[1:0] resolved_bank=pending_write?pending_bank:write_bank;
 wire resolved_valid=(pending_write?pending_full:write_full)&&camera_good;
 wire defer_read=!resolve_write&&((pending_write&&pending_bank==read_bank)||(write_done&&write_bank==read_bank));
 reg[GENERATION_BITS-1:0] selected_generation;reg selected_valid;
 always @*begin
  selected_generation=bank0;selected_valid=bank_valid[0];
  case(read_bank)
   1:begin selected_generation=bank1;selected_valid=bank_valid[1];end
   2:begin selected_generation=bank2;selected_valid=bank_valid[2];end
   3:begin selected_generation=bank3;selected_valid=bank_valid[3];end
  endcase
  // The scheduler's simultaneous case reads the just-completed write bank.
  if(resolve_write&&resolved_bank==read_bank)begin
   selected_generation=generation+1'b1;selected_valid=resolved_valid;
  end else if(write_bank==read_bank||(pending_write&&pending_bank==read_bank))begin
   selected_valid=0;
  end
 end
 always @(posedge clk_axi_i)begin
  if(!rst_axi_n_i)begin
   generation<=0;bank0<=0;bank1<=0;bank2<=0;bank3<=0;bank_valid<=0;
   mailbox<=0;publish<=0;
   camera_sync<=0;camera_seen<=0;pending_write<=0;pending_full<=0;pending_bank<=0;
   pending_read<=0;pending_read_bank<=0;
  end else begin
   camera_sync<={camera_sync[0],camera_toggle};
   if(!ddr_ready_i)begin
    bank_valid<=0;pending_write<=0;pending_read<=0;camera_seen<=camera_sync[1];
   end else if(collision)begin
    // A second completion before the first EOF result is a malformed stream.
    // Invalidate instead of associating a late tag with the wrong bank.
    bank_valid<=0;pending_write<=0;pending_read<=0;camera_seen<=camera_sync[1];
   end
   else begin
    // A bank being reused is no longer a valid old image. In particular the
    // legacy scheduler's saved last bank can equal its current write bank.
    bank_valid[write_bank]<=0;
    if(write_done)begin
     if(!camera_fresh)begin pending_write<=1;pending_full<=write_full;pending_bank<=write_bank;end
    end
    if(resolve_write)begin
     camera_seen<=camera_sync[1];pending_write<=0;bank_valid[resolved_bank]<=resolved_valid;
     if(resolved_valid)begin
      generation<=generation+1'b1;
      case(resolved_bank)
       0:bank0<=generation+1'b1;1:bank1<=generation+1'b1;
       2:bank2<=generation+1'b1;3:bank3<=generation+1'b1;
      endcase
     end
    end
    if(read_commit)begin
     if(defer_read)begin pending_read<=1;pending_read_bank<=read_bank;end
     else begin mailbox<={selected_valid,selected_generation};publish<=~publish;pending_read<=0;end
    end else if(pending_read&&resolve_write&&pending_read_bank==resolved_bank)begin
     mailbox<={resolved_valid,generation+1'b1};publish<=~publish;pending_read<=0;
    end
   end
  end
 end
 (* async_reg="true" *)reg[1:0] publish_sync;
 reg publish_seen;reg[GENERATION_BITS:0] source;
 always @(posedge clk_lcd_i)begin
  if(!rst_lcd_n_i)begin publish_sync<=0;publish_seen<=0;source<=0;end
  else begin
   publish_sync<={publish_sync[0],publish};
   if(publish_sync[1]!=publish_seen)begin publish_seen<=publish_sync[1];source<=mailbox;end
  end
 end
 reg vs_d,de_d,frame_started,frame_bad;
 reg frame_has_pixels;reg[XW-1:0] row_pixels;reg[YW-1:0] rows;
 reg[GENERATION_BITS:0] frame_source;
 reg[GENERATION_BITS-1:0] latest_presented;reg have_latest;
 wire frame_boundary=vs_d&&!lcd_vs_i;
 wire[GENERATION_BITS-1:0] age=frame_source[GENERATION_BITS-1:0]-latest_presented;
 wire complete_frame=frame_started&&frame_has_pixels&&!frame_bad&&frame_source[GENERATION_BITS]&&
   row_pixels==0&&rows==HEIGHT;
 wire new_frame=frame_boundary&&complete_frame&&(!have_latest||(age!=0&&!age[GENERATION_BITS-1]));
 always @(posedge clk_lcd_i)begin
  if(!rst_lcd_n_i)begin
   vs_d<=1;de_d<=0;frame_started<=0;frame_bad<=0;frame_has_pixels<=0;row_pixels<=0;rows<=0;
   frame_source<=0;latest_presented<=0;have_latest<=0;
  end else begin
   vs_d<=lcd_vs_i;de_d<=lcd_de_i;
   if(frame_boundary)begin
    frame_started<=1;frame_bad<=0;frame_has_pixels<=0;row_pixels<=0;rows<=0;frame_source<=0;
    // Track freshness during hold too, so resuming an old held image is excluded.
    if(new_frame)begin latest_presented<=frame_source[GENERATION_BITS-1:0];have_latest<=1;end
   end else if(frame_started)begin
    // FIFO request precedes displayed DE by one clock in lcd_driver. Observe
    // every requested empty read, including the first request before DE rises.
    if(lcd_underflow_i)frame_bad<=1;
    if(lcd_de_i)begin
     if(!frame_has_pixels)begin frame_source<=source;frame_has_pixels<=1;end
     else if(frame_source!=source)frame_bad<=1;
     if(row_pixels<WIDTH)row_pixels<=row_pixels+1'b1;else frame_bad<=1;
    end
    if(de_d&&!lcd_de_i)begin
     if(row_pixels!=WIDTH)frame_bad<=1;
     row_pixels<=0;if(rows<HEIGHT)rows<=rows+1'b1;else frame_bad<=1;
    end
   end
  end
 end
 function[11:0] next_bcd;input[11:0] v;begin
  next_bcd=v;
  if(v!=12'h999)begin
   if(v[3:0]!=9)next_bcd[3:0]=v[3:0]+1'b1;
   else begin next_bcd[3:0]=0;
    if(v[7:4]!=9)next_bcd[7:4]=v[7:4]+1'b1;
    else begin next_bcd[7:4]=0;next_bcd[11:8]=v[11:8]+1'b1;end
   end
  end
 end endfunction
 reg held;reg[TW-1:0] timer;reg[11:0] count;
 wire[11:0] counted=new_frame?next_bcd(count):count;
 always @(posedge clk_lcd_i)begin
  if(!rst_lcd_n_i)begin held<=0;timer<=0;count<=0;fps_bcd_o<=0;end
  else begin
   if(hold_begin_i)held<=1;else if(hold_end_i)held<=0;
   if(hold_begin_i||held)begin timer<=0;count<=0;fps_bcd_o<=0;end
   else if(timer==LCD_CLOCK_HZ-1)begin
    timer<=0;count<=0;fps_bcd_o<=counted;
   end else begin timer<=timer+1'b1;count<=counted;end
  end
 end
endmodule
