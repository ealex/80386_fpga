/*
 * Simple SDR SDRAM controller for 4Mx16x4 devices, e.g. ISSI IS45S16160G
 * (32MB).
 *
 * This controller performs simple 16 bit accesses with one-hot byte enables.
 * There is no burst support and all accesses are performed as a single
 * access with auto precharge so this is definitely not the most optimal
 * scheme.
 */
module sdram_controller(input wire clk,
			/* Host interface. */
			input wire [30:0] h_addr,
			input wire [15:0] h_wdata,
			output reg [15:0] h_rdata,
			input wire h_wr_en,
			input wire [1:0] h_bytesel,
			output reg h_compl,
			output reg h_config_done,
			/* SDRAM signals. */
			output wire s_ras_n,
			output wire s_cas_n,
			output wire s_wr_en,
			output wire [1:0] s_bytesel,
			output reg [12:0] s_addr,
			output wire s_cs_n,
			output reg s_clken,
			inout [15:0] s_data,
			output reg [1:0] s_banksel);

parameter clkf			= 50000000;

localparam ns_per_clk		= (1000000000 / clkf);
localparam tReset		= 100000 / ns_per_clk;
localparam tRC			= 8;
localparam tRP			= 2;
localparam tMRD			= 2;
localparam tRCD			= 2;
localparam cas			= 2;
/*
 * From idle, what is the longest path to get back to idle (excluding
 * autorefresh)?  We need to know this to make sure that we issue the
 * autorefresh command often enough.
 */
localparam max_cmd_period	= tRCD + tRP + 1;
/*
 * tRef of 64ms for normal temperatures (< 85C).
 *
 * Need to refresh 8192 times every tRef.
 */
localparam tRef			= ((64 * 1000000) / ns_per_clk) / 8192;

/* Command truth table: CS  RAS  CAS  WE. */
localparam CMD_NOP		= 4'b0111;
localparam CMD_BST		= 4'b0110;
localparam CMD_READ		= 4'b0101;
localparam CMD_WRITE		= 4'b0100;
localparam CMD_ACT		= 4'b0011;
localparam CMD_PRE		= 4'b0010;
localparam CMD_REF		= 4'b0001;
localparam CMD_MRS		= 4'b0000;

localparam STATE_RESET		= 4'b0000;
localparam STATE_RESET_PCH	= 4'b0001;
localparam STATE_RESET_REF1	= 4'b0011;
localparam STATE_RESET_REF2	= 4'b0010;
localparam STATE_MRS		= 4'b0110;
localparam STATE_IDLE		= 4'b0111;
localparam STATE_ACT		= 4'b0101;
localparam STATE_READ		= 4'b1101;
localparam STATE_WRITE		= 4'b1001;
localparam STATE_PCH		= 4'b1000;
localparam STATE_AUTOREF	= 4'b1010;

reg [3:0] state			= STATE_RESET;
reg [3:0] next_state		= STATE_RESET;

reg [3:0] cmd			= CMD_NOP;
reg [15:0] wdata		= 16'b0;
reg [1:0] nbytesel		= 2'b0;
assign s_cs_n			= cmd[3];
assign s_ras_n			= cmd[2];
assign s_cas_n			= cmd[1];
assign s_wr_en			= cmd[0];
assign s_data			= h_wr_en ? wdata : {16{1'bz}};
assign s_bytesel		= nbytesel;

reg [31:0] addr			= 32'b0;

/*
 * We support 4 banks of 8MB each, rather than interleaving one bank follows
 * the next.  We ignore the LSB of the address - unaligned accesses are not
 * supported and are undefined.
 */
wire [1:0] h_banksel		= addr[24:23];
wire [12:0] h_rowsel		= addr[22:10];
wire [8:0] h_colsel		= addr[9:1];

initial begin
	s_clken			= 1'b1;
	s_addr			= 13'b0;
	s_banksel		= 2'b00;
	h_rdata			= 16'b0;
	h_compl			= 1'b0;
	h_config_done		= 1'b0;
end

/*
 * State machine counter.  Counts every cycle, resets on change of state
 * - once we reach one of the timing parameters we can transition again.  On
 * count 0 we emit the command, after that it's NOP's all the way.
 */
localparam timec_width = $clog2(tReset);
wire [timec_width - 1:0] timec;

counter		#(.count_width(timec_width),
		  .count_max(tReset))
		timec_counter(.clk(clk),
			      .count(timec),
			      .reset(state != next_state));

/*
 * Make sure that we refresh the correct number of times per refresh period
 * and have sufficient time to complete any transaction in progress.
 */
localparam refresh_counter_width = $clog2(tRef);
wire [refresh_counter_width - 1:0] refresh_count;
reg autorefresh_counter_clr = 1'b0;
counter		#(.count_width(refresh_counter_width),
		  .count_max(tRef - max_cmd_period))
		refresh_counter(.clk(clk),
				.count(refresh_count),
				.reset(autorefresh_counter_clr));
wire autorefresh_pending = refresh_count == tRef[refresh_counter_width - 1:0] - max_cmd_period;

always @(*) begin
	next_state = state;
	case (state)
	STATE_RESET: begin
		if (timec == tReset[timec_width - 1:0] - 1)
			next_state = STATE_RESET_PCH;
	end
	STATE_RESET_PCH: begin
		if (timec == tRP - 1)
			next_state = STATE_RESET_REF1;
	end
	STATE_RESET_REF1: begin
		if (timec == tRC - 1)
			next_state = STATE_RESET_REF2;
	end
	STATE_RESET_REF2: begin
		if (timec == tRC - 1)
			next_state = STATE_MRS;
	end
	STATE_MRS: begin
		if (timec == tMRD - 1)
			next_state = STATE_IDLE;
	end
	STATE_IDLE: begin
		/*
		 * If we have a refresh pending then make sure we handle that
		 * first!
		 *
		 * Rather than an enable we can just wait for a non-zero byte
		 * select - reading/writing 0 bytes doesn't make any sense!
		 */
		if (!h_compl && autorefresh_pending)
			next_state = STATE_AUTOREF;
		else if (!h_compl && h_bytesel != 2'b00)
			next_state = STATE_ACT;
	end
	STATE_ACT: begin
		if (timec == tRCD - 1)
			next_state = h_wr_en ? STATE_WRITE : STATE_READ;
	end
	STATE_WRITE: begin
		if (timec == tRP)
			next_state = STATE_IDLE;
	end
	STATE_READ: begin
		if (timec == cas)
			next_state = STATE_IDLE;
	end
	STATE_AUTOREF: begin
		if (timec == tRC - 1)
			next_state = STATE_IDLE;
	end
	default: begin
		next_state = STATE_IDLE;
	end
	endcase
end

always @(*) begin
	cmd = CMD_NOP;
	s_addr = 13'b0;
	s_banksel = 2'b00;
	autorefresh_counter_clr = 1'b0;

	case (state)
	STATE_RESET_PCH: begin
		if (timec == 0) begin
			cmd = CMD_PRE;
			s_addr = 13'b10000000000;
			s_banksel = 2'b11;
		end
	end
	STATE_RESET_REF1: begin
		if (timec == 0)
			cmd = CMD_REF;
	end
	STATE_RESET_REF2: begin
		if (timec == 0)
			cmd = CMD_REF;
	end
	STATE_MRS: begin
		if (timec == 0) begin
			cmd = CMD_MRS;
			s_banksel = 2'b00;
			s_addr = 13'b1000100000;
		end
	end
	STATE_ACT: begin
		if (timec == 0) begin
			cmd = CMD_ACT;
			s_banksel = h_banksel;
			s_addr = h_rowsel;
		end
	end
	STATE_WRITE: begin
		if (timec == 0) begin
			cmd = CMD_WRITE;
			/* Write with autoprecharge. */
			s_addr = {2'b00, 1'b1, 1'b0, h_colsel};
			s_banksel = h_banksel;
		end
	end
	STATE_READ: begin
		if (timec == 0) begin
			cmd = CMD_READ;
			/* Read with autoprecharge. */
			s_addr = {2'b00, 1'b1, 1'b0, h_colsel};
			s_banksel = h_banksel;
		end
	end
	STATE_AUTOREF: begin
		if (timec == 0)
			cmd = CMD_REF;
		if (timec == tRC - 1)
			autorefresh_counter_clr = 1'b1;
	end
	default: begin
	end
	endcase
end

always @(posedge clk) begin
	if (state == STATE_IDLE) begin
		h_config_done <= 1'b1;
		addr <= {h_addr, 1'b0};
	end if (state == STATE_READ && timec == cas) begin
		/* Register the read data after CAS cycles. */
		h_rdata <= s_data;
		h_compl <= 1'b1;
	end else if (state == STATE_WRITE && timec == tRP) begin
		h_compl <= 1'b1;
	end else if (state == STATE_MRS && timec == tMRD - 1) begin
		h_compl <= 1'b1;
	end else begin
		h_compl <= 1'b0;
	end

	wdata <= h_wdata;
	nbytesel <= ~h_bytesel;
end

always @(posedge clk)
	state <= next_state;

endmodule
