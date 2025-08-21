module Cache(
  input  logic [31:0] PC,
  input  logic        CLK,
  input  logic        update,
  input  logic [31:0] w0,w1,w2,w3,w4,w5,w6,w7,
  output logic [31:0] rd,
  output logic        hit, miss
);
  localparam NUM_BLOCKS        = 16;
  localparam BLOCK_SIZE        = 8;
  localparam INDEX_SIZE        = 4;
  localparam WORD_OFFSET_SIZE  = 3;
  localparam BYTE_OFFSET       = 2; // <- bytes in a word
  localparam TAG_SIZE          = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET; // 32-4-3-2=23

  logic [31:0]              data [NUM_BLOCKS-1:0][BLOCK_SIZE-1:0];
  logic [TAG_SIZE-1:0]      tags [NUM_BLOCKS-1:0];
  logic                     valid_bits [NUM_BLOCKS-1:0];

  // Address breakdown
  logic [INDEX_SIZE-1:0]    index;      // PC[8:5]
  logic [WORD_OFFSET_SIZE-1:0] pc_offset; // PC[4:2]
  logic [TAG_SIZE-1:0]      pc_tag, cache_tag;
  logic                     validity;

  // Latch the fill index/tag so they don't change if PC wiggles (we stall, but be safe)
  logic [INDEX_SIZE-1:0]    fill_index;
  logic [TAG_SIZE-1:0]      fill_tag;

  initial begin
    int i,j;
    for (i=0;i<NUM_BLOCKS;i++) begin
      valid_bits[i] = 1'b0;
      tags[i]       = '0;
      for (j=0;j<BLOCK_SIZE;j++) data[i][j] = '0;
    end
  end

  assign index     = PC[8:5];
  assign pc_offset = PC[4:2];
  assign pc_tag    = PC[31:9];
  assign validity  = valid_bits[index];
  assign cache_tag = tags[index];

  assign hit  = (validity && (cache_tag == pc_tag));
  assign miss = !hit;

  // On a miss, output a NOP. On a hit, the selected word.
  always_comb begin
    rd = 32'h00000013;        // NOP
    if (hit) rd = data[index][pc_offset];
  end

  // Capture fill address on the *edge of miss* (when update will follow in next cycle)
  // Because the FSM goes READ_CACHE (miss) -> READ_MEM (update=1),
  // we can latch during READ_CACHE->READ_MEM boundary by looking at miss.
  always_ff @(posedge CLK) begin
    if (miss) begin
      fill_index <= index;
      fill_tag   <= pc_tag;
    end
  end

  // Write the whole block and tag on update
  always_ff @(posedge CLK) begin
    if (update) begin
      data[fill_index][0] <= w0;
      data[fill_index][1] <= w1;
      data[fill_index][2] <= w2;
      data[fill_index][3] <= w3;
      data[fill_index][4] <= w4;
      data[fill_index][5] <= w5;
      data[fill_index][6] <= w6;
      data[fill_index][7] <= w7;
      tags[fill_index]    <= fill_tag;
      valid_bits[fill_index] <= 1'b1;
    end
  end
endmodule
