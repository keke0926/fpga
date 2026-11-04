// Integer, bounded geometry classifier. The caller supplies the exact selected
// component mask on a one-clock synchronous read port and keeps it stable.
// No digital closing: 4-neighbor exterior flood operates at original resolution.
module shape_geometry_engine #(
    parameter TEMPLATE_FILE = "geometry_templates.mem"
)(
    input wire clk, input wire rst_n, input wire abort_i, input wire start_i,
    input wire [8:0] width_i, input wire [7:0] height_i,
    output reg [8:0] mask_x_o, output reg [7:0] mask_y_o,
    input wire mask_data_i,
    output reg done_o, output reg busy_o, output reg accepted_o,
    output reg [4:0] label_o, output reg [13:0] score_o,
    output reg [3:0] reason_o, output reg [15:0] hole_pixels_o,
    output reg [31:0] cycles_o
);
    localparam IDLE=0,CLEAR=1,SEED_REQ=2,SEED_WAIT=3,POP_REQ=4,
      POP_WAIT=5,NEIGH_REQ=6,NEIGH_WAIT=7,NEIGH_NEXT=8,SEED_NEXT=9,
      SCAN_REQ=10,SCAN_WAIT=11,NORM_INIT=12,NORM_REQ=13,NORM_WAIT=14,
      SELECT=15,SELECT_TRI=16,DECIDE=17,DIVIDE=18,FINISH=19;
    reg [4:0] state;
    reg [8:0] width, clear_x, seed_x, scan_x;
    reg [7:0] height, clear_y, seed_y, scan_y;
    reg [1:0] seed_phase,neighbor;
    reg [15:0] center;
    reg [9:0] qhead,qtail;
    reg [10:0] qcount;
    reg visited[0:38399];
    reg [15:0] queue[0:1023];
    reg [15:0] shape_rom[0:1023];
    reg visited_q;
    reg [15:0] queue_q,rom_q;
    wire [15:0] read_addr={mask_y_o,mask_x_o[7:0]};
    reg neighbor_ok;
    reg [8:0] nx;
    reg [7:0] ny;
    reg [9:0] norm_index;
    wire [14:0] norm_x_product=({1'b0,norm_index[4:0],1'b1})*width;
    wire [13:0] norm_y_product=({1'b0,norm_index[9:5],1'b1})*height;
    wire normalized_bit=mask_data_i | ~visited_q;
    wire normalized_valid=(state==NORM_WAIT);
    reg [10:0] intersections[0:15];
    reg [10:0] unions[0:15];
    // Digital Euler number: 8-connected object / 4-connected background.
    // First row reads zero; every later row was written by the preceding scan.
    // Flood queue is idle during SCAN: reuse its bit zero as the previous row.
    wire previous_scan_bit=scan_y==0?1'b0:queue_q[0];
    reg prev_top,prev_current;
    reg row_seen,row_gap_bad;
    reg [1:0] row_gap;
    wire scan_border=(scan_x==width || scan_y==height);
    wire scan_bit=scan_border ? 1'b0 : mask_data_i;
    wire [3:0] quad={prev_top,previous_scan_bit,prev_current,scan_bit};
    reg signed [17:0] euler_sum;
    reg signed [17:0] hole_components;
    reg [8:0] hole_x0,hole_x1;
    reg [7:0] hole_y0,hole_y1;
    reg [15:0] filled_pixels;
    // Small enclosed background regions can be produced by local thresholding.
    // Class 10 describes the circular outer contour, not a claim of solidity.
    wire small_hole_ok=(10*hole_pixels_o<=filled_pixels);
    wire [8:0] hole_w=hole_x1-hole_x0+1'b1;
    wire [7:0] hole_h=hole_y1-hole_y0+1'b1;
    wire [16:0] hole_bbox_area=hole_w*hole_h;
    wire signed [10:0] hole_cx=$signed({1'b0,hole_x0})+$signed({1'b0,hole_x1})+1-$signed({1'b0,width});
    wire signed [9:0] hole_cy=$signed({1'b0,hole_y0})+$signed({1'b0,hole_y1})+1-$signed({1'b0,height});
    wire [10:0] hole_dx=hole_cx[10] ? -hole_cx : hole_cx;
    wire [9:0] hole_dy=hole_cy[9] ? -hole_cy : hole_cy;
    wire hole_ring_ok=(hole_components==1 && 100*hole_w>=80*hole_h && 100*hole_w<=125*hole_h &&
      100*hole_dx<=16*width && 100*hole_dy<=16*height &&
      100*hole_pixels_o>=68*hole_bbox_area && 100*hole_pixels_o<=90*hole_bbox_area);
    reg [3:0] template_index,best_triangle;
    wire [21:0] candidate_cross=intersections[template_index]*unions[best_triangle];
    wire [21:0] incumbent_cross=intersections[best_triangle]*unions[template_index];
    reg [10:0] numerator_count,denominator_count;
    reg [23:0] dividend,quotient;
    reg [24:0] remainder;
    reg [4:0] divide_count;
    wire [24:0] division_shift={remainder[23:0],dividend[23]};
    wire division_bit=(division_shift>={14'b0,denominator_count});
    integer i;
    initial $readmemh(TEMPLATE_FILE,shape_rom);
    always @* begin
        nx={1'b0,center[7:0]};ny=center[15:8];neighbor_ok=1;
        case(neighbor)
          0:if(center[7:0]==0)neighbor_ok=0;else nx={1'b0,center[7:0]}-1'b1;
          1:if({1'b0,center[7:0]}+1'b1>=width)neighbor_ok=0;else nx={1'b0,center[7:0]}+1'b1;
          2:if(center[15:8]==0)neighbor_ok=0;else ny=center[15:8]-1'b1;
          3:if({1'b0,center[15:8]}+1'b1>=height)neighbor_ok=0;else ny=center[15:8]+1'b1;
        endcase
        mask_x_o=0;mask_y_o=0;
        case(state)
          SEED_REQ,SEED_WAIT:begin mask_x_o=seed_x;mask_y_o=seed_y;end
          NEIGH_REQ,NEIGH_WAIT:if(neighbor_ok)begin mask_x_o=nx;mask_y_o=ny;end
          SCAN_REQ,SCAN_WAIT:if(!scan_border)begin mask_x_o=scan_x;mask_y_o=scan_y;end
          NORM_REQ,NORM_WAIT:begin mask_x_o=norm_x_product>>6;mask_y_o=norm_y_product>>6;end
        endcase
    end
    always @(posedge clk) begin
        visited_q<=visited[read_addr];
        queue_q<=queue[(state==SCAN_REQ||state==SCAN_WAIT)?{1'b0,scan_x}:qhead];
        rom_q<=shape_rom[norm_index];
        // One physical RAM writer, shared by mutually exclusive phases.
        if(rst_n && !abort_i && (state==SCAN_WAIT ||
           ((state==SEED_WAIT||state==NEIGH_WAIT)&&!mask_data_i&&!visited_q&&qcount<1024)))
            queue[state==SCAN_WAIT?{1'b0,scan_x}:qtail]<=state==SCAN_WAIT?{15'd0,scan_bit}:read_addr;
        if(rst_n && !abort_i)begin
            if(state==CLEAR)visited[{clear_y,clear_x[7:0]}]<=0;
            else if((state==SEED_WAIT || state==NEIGH_WAIT) && !mask_data_i && !visited_q && qcount<1024)begin
                visited[read_addr]<=1;

            end
        end
    end
    always @(posedge clk) begin
      if(!rst_n || abort_i)begin
        state<=IDLE;busy_o<=0;done_o<=0;accepted_o<=0;label_o<=14;score_o<=0;
        reason_o<=0;hole_pixels_o<=0;cycles_o<=0;width<=0;height<=0;
        clear_x<=0;clear_y<=0;seed_x<=0;seed_y<=0;seed_phase<=0;
        qhead<=0;qtail<=0;qcount<=0;neighbor<=0;center<=0;
        scan_x<=0;scan_y<=0;prev_top<=0;prev_current<=0;
        row_seen<=0;row_gap<=0;row_gap_bad<=0;
        euler_sum<=0;hole_components<=0;hole_x0<=511;hole_x1<=0;hole_y0<=255;hole_y1<=0;
        filled_pixels<=0;
        norm_index<=0;template_index<=0;best_triangle<=1;numerator_count<=0;denominator_count<=1;
        dividend<=0;quotient<=0;remainder<=0;divide_count<=0;
        for(i=0;i<16;i=i+1)begin intersections[i]<=0;unions[i]<=0;end
      end else begin
        done_o<=0;
        if(busy_o)cycles_o<=cycles_o+1'b1;
        case(state)
          IDLE:if(start_i)begin
            busy_o<=1;accepted_o<=0;label_o<=14;score_o<=0;reason_o<=0;cycles_o<=0;
            hole_pixels_o<=0;width<=width_i;height<=height_i;
            clear_x<=0;clear_y<=0;seed_x<=0;seed_y<=0;seed_phase<=0;
            qhead<=0;qtail<=0;qcount<=0;
            prev_top<=0;prev_current<=0;euler_sum<=0;hole_components<=0;
            row_seen<=0;row_gap<=0;row_gap_bad<=0;
            hole_x0<=511;hole_x1<=0;hole_y0<=255;hole_y1<=0;norm_index<=0;
            filled_pixels<=0;
            for(i=0;i<16;i=i+1)begin intersections[i]<=0;unions[i]<=0;end
            if(width_i==0 || width_i>256 || height_i==0 || height_i>150)begin reason_o<=8;state<=FINISH;end
            else state<=CLEAR;
          end
          CLEAR:begin
            if(clear_x+1'b1<width)clear_x<=clear_x+1'b1;
            else if(clear_y+1'b1<height)begin clear_x<=0;clear_y<=clear_y+1'b1;end
            else state<=SEED_REQ;
          end
          SEED_REQ:state<=SEED_WAIT;
          SEED_WAIT:begin
            if(!mask_data_i && !visited_q)begin
              if(qcount==1024)begin reason_o<=9;state<=FINISH;end
              else begin qtail<=qtail+1'b1;qcount<=qcount+1'b1;state<=POP_REQ;end
            end else state<=SEED_NEXT;
          end
          POP_REQ:state<=POP_WAIT;
          POP_WAIT:begin center<=queue_q;qhead<=qhead+1'b1;qcount<=qcount-1'b1;neighbor<=0;state<=NEIGH_REQ;end
          NEIGH_REQ:if(neighbor_ok)state<=NEIGH_WAIT;else state<=NEIGH_NEXT;
          NEIGH_WAIT:begin
            if(!mask_data_i && !visited_q)begin
              if(qcount==1024)begin reason_o<=9;state<=FINISH;end
              else begin qtail<=qtail+1'b1;qcount<=qcount+1'b1;state<=NEIGH_NEXT;end
            end else state<=NEIGH_NEXT;
          end
          NEIGH_NEXT:begin
            if(neighbor!=3)begin neighbor<=neighbor+1'b1;state<=NEIGH_REQ;end
            else if(qcount!=0)state<=POP_REQ;
            else state<=SEED_NEXT;
          end
          SEED_NEXT:begin
            state<=SEED_REQ;
            case(seed_phase)
              0:if(seed_x+1'b1<width)seed_x<=seed_x+1'b1;
                else if(height>1)begin seed_x<=0;seed_y<=height-1'b1;seed_phase<=1;end
                else begin scan_x<=0;scan_y<=0;state<=SCAN_REQ;end
              1:if(seed_x+1'b1<width)seed_x<=seed_x+1'b1;
                else if(height>2)begin seed_x<=0;seed_y<=1;seed_phase<=2;end
                else begin scan_x<=0;scan_y<=0;state<=SCAN_REQ;end
              2:if(seed_y+2<height)seed_y<=seed_y+1'b1;
                else if(width>1)begin seed_x<=width-1'b1;seed_y<=1;seed_phase<=3;end
                else begin scan_x<=0;scan_y<=0;state<=SCAN_REQ;end
              3:if(seed_y+2<height)seed_y<=seed_y+1'b1;
                else begin scan_x<=0;scan_y<=0;state<=SCAN_REQ;end
            endcase
          end
          SCAN_REQ:state<=SCAN_WAIT;
          SCAN_WAIT:begin
            prev_top<=previous_scan_bit;prev_current<=scan_bit;
            case(quad)
              4'b0001,4'b0010,4'b0100,4'b1000:euler_sum<=euler_sum+1;
              4'b1110,4'b1101,4'b1011,4'b0111:euler_sum<=euler_sum-1;
              4'b1001,4'b0110:euler_sum<=euler_sum-2;
            endcase
            if(!scan_border && !mask_data_i && !visited_q)begin
              hole_pixels_o<=hole_pixels_o+1'b1;
              if(scan_x<hole_x0)hole_x0<=scan_x;if(scan_x>hole_x1)hole_x1<=scan_x;
              if(scan_y<hole_y0)hole_y0<=scan_y;if(scan_y>hole_y1)hole_y1<=scan_y;
            end
            if(!scan_border)begin
              if(mask_data_i || !visited_q)begin
                filled_pixels<=filled_pixels+1'b1;
                if(row_seen && row_gap==3)row_gap_bad<=1;
                row_seen<=1;row_gap<=0;
              end else if(row_seen && row_gap!=3)row_gap<=row_gap+1'b1;
            end
            if(scan_x<width)begin scan_x<=scan_x+1'b1;state<=SCAN_REQ;end
            else if(scan_y<height)begin scan_x<=0;scan_y<=scan_y+1'b1;prev_top<=0;prev_current<=0;row_seen<=0;row_gap<=0;state<=SCAN_REQ;end
            else state<=NORM_INIT;
          end
          NORM_INIT:begin hole_components<=1-(euler_sum>>>2);norm_index<=0;state<=NORM_REQ;end
          NORM_REQ:state<=NORM_WAIT;
          NORM_WAIT:begin
            for(i=0;i<16;i=i+1)begin
              if(normalized_bit && rom_q[i])intersections[i]<=intersections[i]+1'b1;
              if(normalized_bit || rom_q[i])unions[i]<=unions[i]+1'b1;
            end
            if(norm_index==1023)state<=SELECT;
            else begin norm_index<=norm_index+1'b1;state<=NORM_REQ;end
          end
          SELECT:begin best_triangle<=1;template_index<=2;state<=SELECT_TRI;end
          SELECT_TRI:begin
            if(candidate_cross>incumbent_cross)best_triangle<=template_index;
            if(template_index==14)state<=DECIDE;else template_index<=template_index+1'b1;
          end
          DECIDE:begin
            reason_o<=6;label_o<=14;accepted_o<=0;state<=FINISH;
            if(hole_components>=0 && hole_components<=1 && !row_gap_bad)begin
              if(intersections[15]*10000>=unions[15]*9400)begin
                accepted_o<=1;reason_o<=0;
                if((width>=height && 100*(width-height)<=5*width) || (width<height && 100*(height-width)<=5*height))label_o<=13;else label_o<=12;
                numerator_count<=intersections[15];denominator_count<=unions[15];
                dividend<=intersections[15]*10000;quotient<=0;remainder<=0;divide_count<=0;state<=DIVIDE;
              end else if(100*width>=85*height && 100*width<=115*height && intersections[0]*10000>=unions[0]*9000 &&
                (small_hole_ok || hole_ring_ok))begin
                accepted_o<=1;reason_o<=0;label_o<=small_hole_ok ? 10 : 17;
                numerator_count<=intersections[0];denominator_count<=unions[0];
                dividend<=intersections[0]*10000;quotient<=0;remainder<=0;divide_count<=0;state<=DIVIDE;
              end else if(intersections[best_triangle]*10000>=unions[best_triangle]*8800)begin
                accepted_o<=1;reason_o<=0;label_o<=11;
                numerator_count<=intersections[best_triangle];denominator_count<=unions[best_triangle];
                dividend<=intersections[best_triangle]*10000;quotient<=0;remainder<=0;divide_count<=0;state<=DIVIDE;
              end
            end
          end
          DIVIDE:begin
            dividend<={dividend[22:0],1'b0};quotient<={quotient[22:0],division_bit};
            remainder<=division_bit ? division_shift-denominator_count : division_shift;
            if(divide_count==23)begin score_o<={quotient[12:0],division_bit};state<=FINISH;end
            else divide_count<=divide_count+1'b1;
          end
          FINISH:begin done_o<=1;busy_o<=0;state<=IDLE;end
          default:begin reason_o<=8;accepted_o<=0;state<=FINISH;end
        endcase
      end
    end
endmodule
