module slotmachine_fpga (SW, KEY, CLOCK_50, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [9:0] SW;
	input [3:0] KEY;
	input CLOCK_50;
	output [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;

	wire [2:0] firstcol, secondcol, thirdcol;
	wire [4:0] score;
	wire [3:0] timerMSD, timerLSD;

	slotmachine S0(
		CLOCK_50,
		KEY[0],
		SW[0],
		KEY[3:1],
		firstcol,
secondcol,
thirdcol,
		score,
		timerMSD,
		timerLSD
	);

	hexDecoder COL1 (HEX2, firstcol);
	hexDecoder COL2 (HEX1, secondcol);
hexDecoder COL3 (HEX0, thirdcol);

hexDecoder TIME_MSD (HEX4, timerMSD);
hexDecoder TIME_LSD (HEX3, timerLSD);

hexDecoder SCORE (HEX5, score);

endmodule



module slotmachine (
	input clk,
	input resetn,
	input switch,
	input [2:0] key,

	output [2:0] firstcol, secondcol, thirdcol,
	output [4:0] score,
	output [3:0] timerMSD, timerLSD
	);

// lots of wires to connect our datapath and control
	wire timerover;
	wire en_spinfirst, en_spinsecond, en_spinthird;
	wire en_storefirst, en_storesecond, en_storethird;
	wire en_compare, en_clearprev, en_timer;

control C0(
		clk,
		resetn,
		switch,
		key [2:0],
timerover,
		en_spinfirst,
en_spinsecond,
en_spinthird,
		en_storefirst,
en_storesecond,
en_storethird,
		en_compare,
en_clearprev,
en_timer
	);

	datapath D0(
		clk,
		resetn,
		switch,
en_spinfirst,
en_spinsecond,
en_spinthird,
en_storefirst,
en_storesecond,
en_storethird,
en_compare,
en_clearprev,
en_timer,
		firstcol,
secondcol,
thirdcol,
score,
timerMSD,
timerLSD,
		timerover
	);
endmodule



module control(
	input clk,
	input resetn,
	input switch,
	input [2:0] key,
	input timerover,

	output reg en_spinfirst, en_spinsecond, en_spinthird,
	output reg en_storefirst, en_storesecond, en_storethird,
	output reg en_compare, en_clearprev, en_timer
	);
	reg [2:0] current_state, next_state;

	localparam  	SHOW_PREV_SCREEN = 3'd0,
           			CLEAR_SCREEN = 3'd1,
            		START_SPIN = 3'd2,
            		STOP_FIRST_COL = 3'd3,
            		STOP_SECOND_COL = 3'd4,
            		STOP_THIRD_COL = 3'd5;

	// Next state logic aka our state table
	always@(*)
	begin: state_table
        	case (current_state)
            	SHOW_PREV_SCREEN: if (switch) next_state = CLEAR_SCREEN;
					       else next_state = SHOW_PREV_SCREEN;

            	CLEAR_SCREEN: if (!switch) next_state = START_SPIN;
				       else next_state = CLEAR_SCREEN;

            	START_SPIN: if (!key[2]) next_state = STOP_FIRST_COL;
				else if (timerover) next_state = STOP_FIRST_COL;
				else next_state = START_SPIN;

            	STOP_FIRST_COL: if (!key[1]) next_state = STOP_SECOND_COL;
				          else if (timerover) next_state = STOP_SECOND_COL;
				          else next_state = STOP_FIRST_COL;

            	STOP_SECOND_COL: if (!key[0]) next_state = STOP_THIRD_COL;
					   else if (timerover) next_state = STOP_THIRD_COL;
					   else next_state = STOP_SECOND_COL;

            	STOP_THIRD_COL: next_state = SHOW_PREV_SCREEN;
        	default: 	next_state = SHOW_PREV_SCREEN;
    	endcase
	end // state_table

	// Output logic aka all of our datapath control signals
	always @(*)
	begin: enable_signals
    	// By default make all our signals 0 to avoid latches.
    	// This is a different style from using a default statement.
    	// It makes the code easier to read.  If you add other out
    	// signals be sure to assign a default value for them here.

	// Enables RateDiv’s, spins the columns
	en_spinfirst = 1’b0;
	en_spinsecond = 1’b0;
	en_spinthird = 1’b0;

	// Stores the columns in a register for comparison
	en_storefirst = 1’b0;
	en_storesecond = 1’b0;
	en_storethird = 1’b0;

	// Enables comparison between the three columns, adds/subtracts score
	en_compare = 1’b0;

	// Clears previous columns
	en_clearprev = 1’b0;

	// Enables timer (99 seconds)
	en_timer = 1’b0;

    	case (current_state)
        	SHOW_PREV_SCREEN: begin
            	end
        	CLEAR_SCREEN: begin
		en_clearprev = 1’b1;
            	end
        	START_SPIN: begin
		en_timer = 1’b1;
		en_spinfirst = 1’b1;
		en_spinsecond = 1’b1;
		en_spinthird = 1’b1;
            	end
        	STOP_FIRST_COL: begin
		en_timer = 1’b1;
		en_storefirst = 1’b1;
		en_spinsecond = 1’b1;
		en_spinthird = 1’b1;
            	end
        	STOP_SECOND_COL: begin
		en_timer = 1’b1;
		en_storesecond = 1’b1;
		en_spinthird = 1’b1;
        		end
        	STOP_THIRD_COL: begin
		en_timer = 1’b1;
		en_storethird = 1’b1;
		en_compare = 1’b1;
        	end
    	// default:	// don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
    	endcase
	end // enable_signals

	// current_state registers
	always@(posedge clk)
	begin: state_FFs
    	if(!resetn)
        	current_state <= SHOW_PREV_SCREEN;
    	else
        	current_state <= next_state;
	end // state_FFS
endmodule



module datapath(
	input clk,
	input resetn,
input switch,
input en_spinfirst, en_spinsecond, en_spinthird,
	input en_storefirst, en_storesecond, en_storethird,
	input en_compare, en_clearprev, en_timer,
	output reg [2:0] firstcol, secondcol, thirdcol,
	output reg [4:0] score, // Currently capped at 15, for testing only
	output reg [3:0] timerMSD, timerLSD;
	output reg timerover
	);

	// Registers for RateDiv always block
	reg [26:0] speedfirst, speedsecond, speedthird;
	reg pulsefirst, pulsesecond, pulsethird;

	// Register for timer always block
	reg [26:0] timerseconds;

	// Third column’s RateDiv - 2Hz Speed - 25mil clock cycles - fast
always @(posedge clk) begin
if (resetn == 1'b0) begin
speedthird <= 27’d0;
pulsethird <= 1’b0;
end

else if (speedthird == 27’d24999999) begin
speedthird <= 27’d0;
pulsethird <= 1’b1;
end

else if (en_spinthird == 1'b1 && switch == 1’b0) begin
speedthird <= speedthird + 1;
pulsethird <= 1’b0;
end

else if (en_storethird == 1’b1) begin
	speedthird <= 27’d0;
	pulsethird <= 1’b0;
end
end

	// Second column’s RateDiv - 1Hz Speed - 50mil clock cycles - medium
always @(posedge clk) begin
if (resetn == 1'b0) begin
speedsecond <= 27’d0;
pulsesecond <= 1’b0;
end

else if (speedsecond == 27’d49999999) begin
speedsecond <= 27’d0;
pulsesecond <= 1’b1;
end

else if (en_spinsecond == 1'b1 && switch == 1’b0) begin
speedsecond <= speedsecond + 1;
pulsesecond <= 1’b0;
end

else if (en_storesecond == 1’b1) begin
	speedsecond <= 27’d0;
	pulsesecond <= 1’b0;
end
end

	// First column’s RateDiv - 0.5Hz Speed - 100mil clock cycles - slow
always @(posedge clk) begin
if (resetn == 1'b0) begin
speedfirst <= 27’d0;
pulsefirst <= 1’b0;
end

else if (speedfirst == 27’d99999999) begin
speedfirst <= 27’d0;
pulsefirst <= 1’b1;
end

else if (en_spinfirst == 1'b1 && switch == 1’b0) begin
speedfirst <= speedfirst + 1;
pulsefirst <= 1’b0;
end

else if (en_storefirst == 1’b1) begin
	speedfirst <= 27’d0;
	pulsefirst <= 1’b0;
end
end

// Three 3-bit counters that represent the three columns
always @(posedge clk) begin
if (resetn == 1'b0) begin
firstcol <= 3’d0;
secondcol <= 3’d0;
thirdcol <= 3’d0;
end

if (pulsefirst == 1’b1) begin
firstcol <= firstcol + 1;
end

if (pulsesecond == 1’b1) begin
secondcol <= secondcol + 1;
end

if (pulsethird == 1’b1) begin
thirdcol <= thirdcol + 1;
end

if (en_clearprev == 1’b1) begin
firstcol <= 3’d0;
secondcol <= 3’d0;
thirdcol <= 3’d0;
end
end

// Compares the three columns - keeps track of score
	always @(posedge clk) begin
if (resetn == 1'b0) begin
score <= 4’d5;
end

else if (en_compare) begin
if (firstcol == secondcol && firstcol == thirdcol) score <= score + 1;
else score <= score - 1;
end

else if (score == 4’d0 || score == 4’d15) begin
	// Need to change behaviour after testing
	// Display LOSE on hex display or show lose screen on VGA
	score <= 4’d5;
end
end

// Timer - user has 99 seconds to stop all three columns
	always @(posedge clk) begin
if (resetn == 1'b0) begin
	timerseconds <= 27’d0;
timerMSD <= 4’d9;
timerLSD <= 4’d9;
timerover <= 1’b0;
end

if (en_timer == 1’b1) begin
	if (timerseconds != 27’d49999999) begin
		timerseconds <= timerseconds + 1;
	end

	else if (timerseconds == 27’d49999999) begin
		timerseconds <= 27’d0;

		if (timerLSD != 4’d0) begin
			timerLSD <= timerLSD - 1;
		end

		else if (timerLSD == 4’d0) begin
			timerMSD <= timerMSD - 1;
timerLSD <= 4’d9;
		end
	end
end

if (en_clearprev == 1’b1) begin
	timerseconds <= 27’d0;
timerMSD <= 4’d9;
timerLSD <= 4’d9;
timerover <= 1’b0;
end

if (timerMSD == 4’d0) begin
	timerMSD <= 4’d0;
timerLSD <= 4’d0;
timerover <= 1’b1;
end
end
endmodule



module hexDecoder (s, x);
 input [3:0] x;
 output [6:0] s;

 // Turns the top-middle segment on or off
 assign s[0] = (~x[3] & ~x[2] & ~x[1] & x[0]) |
    	(~x[3] & x[2] & ~x[1] & ~x[0]) |
    	(x[3] & ~x[2] & x[1] & x[0]) |
    	(x[3] & x[2] & ~x[1] & x[0]);
 // Turns the top-right segment on or off
 assign s[1] = (~x[3] & x[2] & ~x[1] & x[0]) |
    	(~x[3] & x[2] & x[1] & ~x[0]) |
    	(x[3] & ~x[2] & x[1] & x[0]) |
    	(x[3] & x[2] & ~x[1] & ~x[0]) |
    	(x[3] & x[2] & x[1] & ~x[0]) |
    	(x[3] & x[2] & x[1] & x[0]);
 // Turns the bottom-right segment on or off
 assign s[2] = (~x[3] & ~x[2] & x[1] & ~x[0]) |
    	(x[3] & x[2] & ~x[1] & ~x[0]) |
    	(x[3] & x[2] & x[1] & ~x[0]) |
    	(x[3] & x[2] & x[1] & x[0]);
 // Turns the bottom-middle segment on or off
 assign s[3] = (~x[3] & ~x[2] & ~x[1] & x[0]) |
    	(~x[3] & x[2] & ~x[1] & ~x[0]) |
    	(~x[3] & x[2] & x[1] & x[0]) |
    	(x[3] & ~x[2] & ~x[1] & x[0]) |
    	(x[3] & ~x[2] & x[1] & ~x[0]) |
    	(x[3] & x[2] & x[1] & x[0]);
 // Turns the bottom-left segment on or off
 assign s[4] = (~x[3] & ~x[2] & ~x[1] & x[0]) |
    	(~x[3] & ~x[2] & x[1] & x[0]) |
    	(~x[3] & x[2] & ~x[1] & ~x[0]) |
    	(~x[3] & x[2] & ~x[1] & x[0]) |
    	(~x[3] & x[2] & x[1] & x[0]) |
    	(x[3] & ~x[2] & ~x[1] & x[0]);
 // Turns the top-left segment on or off
 assign s[5] = (~x[3] & ~x[2] & ~x[1] & x[0]) |
    	(~x[3] & ~x[2] & x[1] & ~x[0]) |
    	(~x[3] & ~x[2] & x[1] & x[0]) |
    	(~x[3] & x[2] & x[1] & x[0]) |
    	(x[3] & x[2] & ~x[1] & x[0]);
 // Turns the middle segment on or off
 assign s[6] = (~x[3] & ~x[2] & ~x[1] & ~x[0]) |
    	(~x[3] & ~x[2] & ~x[1] & x[0]) |
    	(~x[3] & x[2] & x[1] & x[0]) |
    	(x[3] & x[2] & ~x[1] & ~x[0]);
endmodule









module vga_adapter(
			resetn,
			clock,
			colour,
			x, y, plot,
			/* Signals for the DAC to drive the monitor. */
			VGA_R,
			VGA_G,
			VGA_B,
			VGA_HS,
			VGA_VS,
			VGA_BLANK,
			VGA_SYNC,
			VGA_CLK);
		parameter BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.COLOUR_CHANNEL_DEPTH = 1;
		defparam VGA.BACKGROUND_IMAGE = "slotmachine.mif";









VGA - SPRITE ANIMATION:


160x120.v

module vga160x120(
    input wire i_clk,           // base clock
    input wire i_pix_stb,       // pixel clock strobe
    input wire i_rst,           // reset: restarts frame
    output wire o_hs,           // horizontal sync
    output wire o_vs,           // vertical sync
    output wire o_blanking,     // high during blanking interval
    output wire o_active,       // high during active pixel drawing
    output wire o_screenend,    // high for one tick at the end of screen
    output wire o_animate,      // high for one tick at end of active drawing
    output wire [9:0] o_x,      // current pixel x position
    output wire [8:0] o_y       // current pixel y position
    );

    // VGA timings
    localparam HS_STA = front porch              // horizontal sync start
    localparam HS_END = front porch + sync pulse;         // horizontal sync end
    localparam HA_STA = front porch, sync pulse & back porch;    // horizontal active pixel start
    localparam VS_STA = 480 + 10;        // vertical sync start
    localparam VS_END = active video+front porch;    // vertical sync end
    localparam VA_STA = 60;              // vertical active pixel start
    localparam VA_END = 420;             // vertical active pixel end
    localparam LINE   = 800;             // complete line (pixels)
    localparam SCREEN = 525;             // complete screen (lines)

   // generate sync signals (active low for 640x480)
    assign o_hs = ~((h_count >= HS_STA) & (h_count < HS_END));
    assign o_vs = ~((v_count >= VS_STA) & (v_count < VS_END));

    // keep x and y bound within the active pixels
    assign o_x = ((h_count < HA_STA) ? 0 : (h_count - HA_STA)) >> 1;
    assign o_y = ((v_count >= VA_END) ?
                    (VA_END - VA_STA - 1) : (v_count - VA_STA)) >> 1;

    // blanking: high within the blanking period
    assign o_blanking = ((h_count < HA_STA) | (v_count > VA_END - 1));

    // active: high during active pixel drawing
    assign o_active = ~((h_count < HA_STA) |
                        (v_count > VA_END - 1) |
                        (v_count < VA_STA));

    // screenend: high for one tick at the end of the screen
    assign o_screenend = ((v_count == SCREEN - 1) & (h_count == LINE));

    // animate: high for one tick at the end of the final active pixel line
    assign o_animate = ((v_count == VA_END - 1) & (h_count == LINE));

    always @ (posedge i_clk)
    begin
        if (i_rst)  // reset to start of frame
        begin
            h_count <= 0;
            v_count <= 0;
        end
        if (i_pix_stb)  // once per pixel
        begin
            if (h_count == LINE)  // end of line
            begin
                h_count <= 0;
                v_count <= v_count + 1;
            end
            else
                h_count <= h_count + 1;

            if (v_count == SCREEN)  // end of screen
                v_count <= 0;
        end
    end
endmodule





SRAM.v

module sram #(parameter ADDR_WIDTH=8, DATA_WIDTH=8, DEPTH=256, MEMFILE="") (
    input wire i_clk,
    input wire [ADDR_WIDTH-1:0] i_addr,
    input wire i_write,
    input wire [DATA_WIDTH-1:0] i_data,
    output reg [DATA_WIDTH-1:0] o_data
    );

    reg [DATA_WIDTH-1:0] memory_array [0:DEPTH-1];

    initial begin
        if (MEMFILE > 0)
        begin
            $display("Loading memory init file '" + MEMFILE + "' into array.");
            $readmemh(MEMFILE, memory_array);
        end
    end

    always @ (posedge i_clk)
    begin
        if(i_write) begin
            memory_array[i_addr] <= i_data;
        end
        else begin
            o_data <= memory_array[i_addr];
        end
    end
endmodule


Sample for drawing 32x32 sprites….every pixel drawn per frame


module top(
    input wire CLK,             // board clock: 100 MHz on Arty/Basys3/Nexys
    input wire RST_BTN,         // reset button
    input wire [3:0] sw,        // four switches
    output wire VGA_HS_O,       // horizontal sync output
    output wire VGA_VS_O,       // vertical sync output
    output reg [3:0] VGA_R,     // 4-bit VGA red output
    output reg [3:0] VGA_G,     // 4-bit VGA green output
    output reg [3:0] VGA_B      // 4-bit VGA blue output
    );

    wire rst = ~RST_BTN;    // reset is active low on Arty & Nexys Video
    // wire rst = RST_BTN;  // reset is active high on Basys3 (BTNC)

    // generate a 25 MHz pixel strobe
    reg [15:0] cnt;
    reg pix_stb;
    always @(posedge CLK)
        {pix_stb, cnt} <= cnt + 16'h4000;  // divide by 4: (2^16)/4 = 0x4000

    wire [9:0] x;       // current pixel x position: 10-bit value: 0-1023
    wire [8:0] y;       // current pixel y position:  9-bit value: 0-511
    wire blanking;      // high within the blanking period
    wire active;        // high during active pixel drawing
    wire screenend;     // high for one tick at the end of screen
    wire animate;       // high for one tick at end of active drawing

    vga320x180 display (
        .i_clk(CLK),
        .i_pix_stb(pix_stb),
        .i_rst(rst),
        .o_hs(VGA_HS_O),
        .o_vs(VGA_VS_O),
        .o_x(x),
        .o_y(y),
        .o_blanking(blanking),
        .o_active(active),
        .o_screenend(screenend),
        .o_animate(animate)
    );

    // VRAM frame buffers (read-write)
    localparam SCREEN_WIDTH = 320;
    localparam SCREEN_HEIGHT = 180;
    localparam VRAM_DEPTH = SCREEN_WIDTH * SCREEN_HEIGHT;
    localparam VRAM_A_WIDTH = 16;  // 2^16 > 320 x 180
    localparam VRAM_D_WIDTH = 8;   // colour bits per pixel

    reg [VRAM_A_WIDTH-1:0] address_a, address_b;
    reg [VRAM_D_WIDTH-1:0] datain_a, datain_b;
    wire [VRAM_D_WIDTH-1:0] dataout_a, dataout_b;
    reg we_a = 0, we_b = 1;  // write enable bit

    // frame buffer A VRAM
    sram #(
        .ADDR_WIDTH(VRAM_A_WIDTH),
        .DATA_WIDTH(VRAM_D_WIDTH),
        .DEPTH(VRAM_DEPTH),
        .MEMFILE(""))
        vram_a (
        .i_addr(address_a),
        .i_clk(CLK),
        .i_write(we_a),
        .i_data(datain_a),
        .o_data(dataout_a)
    );

    // frame buffer B VRAM
    sram #(
        .ADDR_WIDTH(VRAM_A_WIDTH),
        .DATA_WIDTH(VRAM_D_WIDTH),
        .DEPTH(VRAM_DEPTH),
        .MEMFILE(""))
        vram_b (
        .i_addr(address_b),
        .i_clk(CLK),
        .i_write(we_b),
        .i_data(datain_b),
        .o_data(dataout_b)
    );

    // sprite buffer (read-only)
    localparam SPRITE_SIZE = 32;  // dimensions of square sprites in pixels
    localparam SPRITE_COUNT = 8;  // number of sprites in buffer
    localparam SPRITEBUF_D_WIDTH = 8;  // colour bits per pixel
    localparam SPRITEBUF_DEPTH = SPRITE_SIZE * SPRITE_SIZE * SPRITE_COUNT;
    localparam SPRITEBUF_A_WIDTH = 13;  // 2^13 == 8,096 == 32 x 256

    reg [SPRITEBUF_A_WIDTH-1:0] address_s;
    wire [SPRITEBUF_D_WIDTH-1:0] dataout_s;

    // sprite buffer memory
    sram #(
        .ADDR_WIDTH(SPRITEBUF_A_WIDTH),
        .DATA_WIDTH(SPRITEBUF_D_WIDTH),
        .DEPTH(SPRITEBUF_DEPTH),
        .MEMFILE("sprites.mem"))
        spritebuf (
        .i_addr(address_s),
        .i_clk(CLK),
        .i_write(0),  // read only
        .i_data(0),
        .o_data(dataout_s)
    );

    reg [11:0] palette [0:255];  // 256 x 12-bit colour palette entries
    reg [11:0] colour;
    initial begin
        $display("Loading palette.");
        $readmemh("sprites_palette.mem", palette);
    end

    // sprites to load and position of player sprite in frame
    localparam SPRITE_BG_INDEX = 7;  // background sprite
    localparam SPRITE_PL_INDEX = 0;  // player sprite
    localparam SPRITE_BG_OFFSET = SPRITE_BG_INDEX * SPRITE_SIZE * SPRITE_SIZE;
    localparam SPRITE_PL_OFFSET = SPRITE_PL_INDEX * SPRITE_SIZE * SPRITE_SIZE;
    localparam SPRITE_PL_X = SCREEN_WIDTH - SPRITE_SIZE >> 1; // centre
    localparam SPRITE_PL_Y = SCREEN_HEIGHT - SPRITE_SIZE;     // bottom

    reg [9:0] draw_x;
    reg [8:0] draw_y;
    reg [9:0] pl_x = SPRITE_PL_X;
    reg [9:0] pl_y = SPRITE_PL_Y;
    reg [9:0] pl_pix_x;
    reg [8:0] pl_pix_y;

    // pipeline registers for for address calculation
    reg [VRAM_A_WIDTH-1:0] address_fb1;
    reg [VRAM_A_WIDTH-1:0] address_fb2;

    always @ (posedge CLK)
    begin
        // reset drawing
        if (rst)
        begin
            draw_x <= 0;
            draw_y <= 0;
            pl_x <= SPRITE_PL_X;
            pl_y <= SPRITE_PL_Y;
            pl_pix_x <= 0;
            pl_pix_y <= 0;
        end

        // draw background
        if (address_fb1 < VRAM_DEPTH)
        begin
            if (draw_x < SCREEN_WIDTH)
                draw_x <= draw_x + 1;
            else
            begin
                draw_x <= 0;
                draw_y <= draw_y + 1;
            end

            // calculate address of sprite and frame buffer (with pipeline)
            address_s <= SPRITE_BG_OFFSET +
                        (SPRITE_SIZE * draw_y[4:0]) + draw_x[4:0];
            address_fb1 <= (SCREEN_WIDTH * draw_y) + draw_x;
            address_fb2 <= address_fb1;

            if (we_a)
            begin
                address_a <= address_fb2;
                datain_a <= dataout_s;
            end
            else
            begin
                address_b <= address_fb2;
                datain_b <= dataout_s;
            end
        end

        if (pix_stb)  // once per pixel
        begin
            if (we_a)  // when drawing to A, output from B
            begin
                address_b <= y * SCREEN_WIDTH + x;
                colour <= active ? palette[dataout_b] : 0;
            end
            else  // otherwise output from A
            begin
                address_a <= y * SCREEN_WIDTH + x;
                colour <= active ? palette[dataout_a] : 0;
            end

            if (screenend)  // switch active buffer once per frame
            begin
                we_a <= ~we_a;
                we_b <= ~we_b;
                // reset background position at start of frame
                draw_x <= 0;
                draw_y <= 0;
                // reset player position
                pl_pix_x <= 0;
                pl_pix_y <= 0;
                // reset frame address
                address_fb1 <= 0;
            end
        end

        VGA_R <= colour[11:8];
        VGA_G <= colour[7:4];
        VGA_B <= colour[3:0];
    end
endmodule 
