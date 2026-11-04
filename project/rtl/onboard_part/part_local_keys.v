`timescale 1ns/1ps
// One debounced detection key. Context: preview=0, confirm=1, held=2,
// unavailable=3. A short release acts only in the context where it began.
// Long preview press requests teaching; its release never confirms teaching.
module part_local_keys #(parameter integer HOLD_CYCLES=96000000)(
 input wire clk,rst_n,pressed_i,cancel_i,input wire[1:0]context_i,
 output reg teach_o,action_o
);
 reg[26:0]count;reg[1:0]press_context;
 reg down,blocked,fired,armed;
 always@(posedge clk)begin
  if(!rst_n)begin
   count<=0;press_context<=3;down<=0;blocked<=0;fired<=0;
   armed<=0;teach_o<=0;action_o<=0;
  end
  else begin
   teach_o<=0;action_o<=0;
   if(!pressed_i)begin
    if(down&&armed&&!blocked&&!fired&&!cancel_i&&
       context_i!=3&&context_i==press_context)action_o<=1;
    armed<=1;down<=0;blocked<=0;fired<=0;count<=0;
   end else if(!down)begin
    down<=1;press_context<=context_i;count<=0;fired<=0;
    blocked<=!armed||cancel_i||context_i==3;
   end else if(cancel_i||context_i!=press_context||context_i==3)begin
    blocked<=1;
   end else if(!blocked&&!fired)begin
    if(count==HOLD_CYCLES-1)begin
     fired<=1;
     if(context_i==0)teach_o<=1;
    end else count<=count+1'b1;
   end
  end
 end
endmodule
