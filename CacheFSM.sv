module CacheFSM(
  input  logic hit, miss, CLK, RST,
  output logic update, pc_stall
);
  typedef enum logic [0:0] { ST_READ_CACHE, ST_READ_MEM } state_type;
  state_type PS, NS;

  always_ff @(posedge CLK or posedge RST) begin
    if (RST) PS <= ST_READ_CACHE;  // start reading cache
    else     PS <= NS;
  end

  always_comb begin
    // Defaults
    update   = 1'b0;     // only assert in ST_READ_MEM
    pc_stall = 1'b0;
    NS       = PS;

    unique case (PS)
      ST_READ_CACHE: begin
        if (hit) begin
          // normal flow
          NS = ST_READ_CACHE;
        end else begin
          // miss detected
          pc_stall = 1'b1;   // freeze PC this cycle
          NS       = ST_READ_MEM;
        end
      end

      ST_READ_MEM: begin
        // while we "fetch" the 8-word block (combinational imem here),
        // hold PC, and assert update ONCE to write the block into cache
        pc_stall = 1'b1;
        update   = 1'b1;     // one cycle cache line fill
        NS       = ST_READ_CACHE;
      end
    endcase
  end
endmodule
