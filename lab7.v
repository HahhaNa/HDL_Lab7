`define hc  32'd524 // q   
`define hd  32'd588 // w  
`define he  32'd660 // e  
`define hf  32'd698 // r  
`define hg  32'd784 // t   
`define ha  32'd880 // y
`define hb  32'd988 // u
`define c  32'd262  // a   
`define d  32'd294  // s  
`define e  32'd330  // d  
`define f  32'd349  // f  
`define g  32'd392  // g   
`define a  32'd440  // h
`define b  32'd494  // j
`define lc  32'd131 // z   
`define ld  32'd147 // x 
`define le  32'd165 // c  
`define lf  32'd174 // v  
`define lg  32'd196 // b   
`define la  32'd220 // n
`define lb  32'd247 // m

`define sil   32'd50000000 // slience


module lab7(
    input clk,
    input rst,        // BTNC: active high reset
    input _play,      // SW0: Play/Pause
    input _start,     // SW1: Start/Exit
    input _mute,      // SW14: Mute
    input _mode,      // SW15: Mode
    input _volUP,     // BTNU: Vol up
    input _volDOWN,   // BTND: Vol down
    inout PS2_DATA,   // Keyboard I/O
    inout PS2_CLK,    // Keyboard I/O
    output reg [15:0] _led,       // LED: [15:9] key & [4:0] volume
    output audio_mclk, // master clock
    output audio_lrck, // left-right clock
    output audio_sck,  // serial clock
    output audio_sdin, // serial audio data input
    output [6:0] DISPLAY,    
    output [3:0] DIGIT
    );    

    // For key
    wire [65:0] key_down;
	wire [8:0] last_change;
    reg [8:0] last_key;
    reg [4:0] key_num;
    wire shift_down;
	wire been_ready;   

    KeyboardDecoder key_de (
		.key_down(key_down),
		.last_change(last_change),
		.key_valid(been_ready),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);   

    // KEY_CODES
    parameter [8:0] KEY_CODES [0:20] = {
        9'b0_0001_0101, // Q => 15
        9'b0_0001_1101, // W => 1D
        9'b0_0010_0100, // E => 24
        9'b0_0010_1101, // R => 2D
        9'b0_0010_1100, // T => 2C
        9'b0_0011_0101, // Y => 35
        9'b0_0011_1100, // U => 3C

        9'b0_0001_1100, // A => 1C
        9'b0_0001_1011, // S => 1B
        9'b0_0010_0011, // D => 23
        9'b0_0010_1011, // F => 2B
        9'b0_0011_0100, // G => 34
        9'b0_0011_0011, // H => 33
        9'b0_0011_1011, // J => 3B

        9'b0_0001_1010, // Z => 1A
        9'b0_0010_0010, // X => 22
        9'b0_0010_0001, // C => 21
        9'b0_0010_1010, // V => 2A         
        9'b0_0011_0010, // B => 32
        9'b0_0011_0001, // N => 31
        9'b0_0011_1010 // M => 3A
    }; 

    always @ (*) begin
		case (last_change)
			KEY_CODES[00] : key_num = 5'b00000;
			KEY_CODES[01] : key_num = 5'b00001;
			KEY_CODES[02] : key_num = 5'b00010;
			KEY_CODES[03] : key_num = 5'b00011;
			KEY_CODES[04] : key_num = 5'b00100;
			KEY_CODES[05] : key_num = 5'b00101;
			KEY_CODES[06] : key_num = 5'b00110;
			KEY_CODES[07] : key_num = 5'b00111;
			KEY_CODES[08] : key_num = 5'b01000;
			KEY_CODES[09] : key_num = 5'b01001;
			KEY_CODES[10] : key_num = 5'b01010;
			KEY_CODES[11] : key_num = 5'b01011;
			KEY_CODES[12] : key_num = 5'b01100;
			KEY_CODES[13] : key_num = 5'b01101;
			KEY_CODES[14] : key_num = 5'b01110;
			KEY_CODES[15] : key_num = 5'b01111;
            KEY_CODES[16] : key_num = 5'b10000;
			KEY_CODES[17] : key_num = 5'b10001;
			KEY_CODES[18] : key_num = 5'b10010;
			KEY_CODES[19] : key_num = 5'b10011;
			KEY_CODES[20] : key_num = 5'b10100;
            default: key_num = 5'b11111;
		endcase
	end


    // Internal Signal
    wire [15:0] audio_in_left, audio_in_right;

    wire [11:0] ibeatNum;               // Beat counter
    wire [31:0] freqL, freqR;           // Raw frequency, produced by music module
    wire [21:0] freq_outL, freq_outR;    // Processed frequency, adapted to the clock rate of Basys3

    // clkDiv22
    wire clkDiv22;
    clock_divider #(.n(22)) clock_22(.clk(clk), .clk_div(clkDiv22));    // for audio

    // debounce, one_pulse
    wire up_db, down_db;
    wire up_1p, down_1p;
    debounce d1(.pb_debounced(up_db), .pb(_volUP), .clk(clk));
    onepulse o1(.signal(up_db), .op(up_1p), .clk(clk));
    debounce d2(.pb_debounced(down_db), .pb(_volDOWN), .clk(clk));
    onepulse o2(.signal(down_db), .op(down_1p), .clk(clk));

    // volume control
    reg [2:0] volume;
    always @(posedge clk, posedge rst) begin
        if(rst) begin
            volume <= 3;
        end else begin
            if(up_1p && volume != 5) 
                volume <= volume + 1; 
            else if(down_1p && volume != 1)
                volume <= volume - 1;
            else volume <= volume;
        end
    end

    // led control 
    always @(*) begin
        if(volume == 0)
            _led <= 16'd0;
        else if(volume == 1) 
            _led <= 16'b0000_0000_0000_0001;
        else if(volume == 2) 
            _led <= 16'b0000_0000_0000_0011;
        else if(volume == 3) 
            _led <= 16'b0000_0000_0000_0111;
        else if(volume == 4) 
            _led <= 16'b0000_0000_0000_1111;
        else if(volume == 5) 
            _led <= 16'b0000_0000_0001_1111;
        else _led <= 16'b0000_0000_0000_0111;
    end

    // 7 segment Control
    reg[16:0] nums;
	SevenSegment seven_seg (
		.display(DISPLAY),
		.digit(DIGIT),
		.nums(nums),
		.rst(rst),
		.clk(clk)
	);
    always @(*) begin
        if(_mode || (!_mode && !_start)) begin // DEMONSTRATE mode
            if(_play || (!_mode && !_start)) begin
                case (freqR)
                    `hc: nums = {4'd10, 4'd10, 5'd11, 4'd5};
                    `hd: nums = {4'd10, 4'd10, 5'd12, 4'd5};
                    `he: nums = {4'd10, 4'd10, 5'd13, 4'd5};
                    `hf: nums = {4'd10, 4'd10, 5'd14, 4'd5};
                    `hg: nums = {4'd10, 4'd10, 5'd15, 4'd5};
                    `ha: nums = {4'd10, 4'd10, 5'd16, 4'd5};
                    `hb: nums = {4'd10, 4'd10, 5'd17, 4'd5};

                    `c: nums = {4'd10, 4'd10, 5'd11, 4'd4};
                    `d: nums = {4'd10, 4'd10, 5'd12, 4'd4};
                    `e: nums = {4'd10, 4'd10, 5'd13, 4'd4};
                    `f: nums = {4'd10, 4'd10, 5'd14, 4'd4};
                    `g: nums = {4'd10, 4'd10, 5'd15, 4'd4};
                    `a: nums = {4'd10, 4'd10, 5'd16, 4'd4};
                    `b: nums = {4'd10, 4'd10, 5'd17, 4'd4};

                    `lc: nums = {4'd10, 4'd10, 5'd11, 4'd3};
                    `ld: nums = {4'd10, 4'd10, 5'd12, 4'd3};
                    `le: nums = {4'd10, 4'd10, 5'd13, 4'd3};
                    `lf: nums = {4'd10, 4'd10, 5'd14, 4'd3};
                    `lg: nums = {4'd10, 4'd10, 5'd15, 4'd3};
                    `la: nums = {4'd10, 4'd10, 5'd16, 4'd3};
                    `lb: nums = {4'd10, 4'd10, 5'd17, 4'd3};
                    
                    default: nums = {4'd10, 4'd10, 5'd10, 4'd10};
                endcase
            end else nums = {4'd10, 4'd10, 5'd10, 4'd10};
        end else begin  // PLAY mode
            if(_play) begin
                case (freqR)
                    `hc: nums = {4'd10, 4'd10, 5'd11, 4'd5};
                    `hd: nums = {4'd10, 4'd10, 5'd12, 4'd5};
                    `he: nums = {4'd10, 4'd10, 5'd13, 4'd5};
                    `hf: nums = {4'd10, 4'd10, 5'd14, 4'd5};
                    `hg: nums = {4'd10, 4'd10, 5'd15, 4'd5};
                    `ha: nums = {4'd10, 4'd10, 5'd16, 4'd5};
                    `hb: nums = {4'd10, 4'd10, 5'd17, 4'd5};

                    `c: nums = {4'd10, 4'd10, 5'd11, 4'd4};
                    `d: nums = {4'd10, 4'd10, 5'd12, 4'd4};
                    `e: nums = {4'd10, 4'd10, 5'd13, 4'd4};
                    `f: nums = {4'd10, 4'd10, 5'd14, 4'd4};
                    `g: nums = {4'd10, 4'd10, 5'd15, 4'd4};
                    `a: nums = {4'd10, 4'd10, 5'd16, 4'd4};
                    `b: nums = {4'd10, 4'd10, 5'd17, 4'd4};

                    `lc: nums = {4'd10, 4'd10, 5'd11, 4'd3};
                    `ld: nums = {4'd10, 4'd10, 5'd12, 4'd3};
                    `le: nums = {4'd10, 4'd10, 5'd13, 4'd3};
                    `lf: nums = {4'd10, 4'd10, 5'd14, 4'd3};
                    `lg: nums = {4'd10, 4'd10, 5'd15, 4'd3};
                    `la: nums = {4'd10, 4'd10, 5'd16, 4'd3};
                    `lb: nums = {4'd10, 4'd10, 5'd17, 4'd3};
                    
                    default: nums = {4'd10, 4'd10, 5'd10, 4'd10};
                endcase
            end else nums = {4'd10, 4'd10, 5'd10, 4'd10};
        end
    end


    // Player Control
    // [in]  reset, clock, _play, _slow, _music, and _mode
    // [out] beat number
    player_control #(.LEN(128)) playerCtrl_00 ( 
        .clk(clkDiv22),
        .reset(rst),
        ._play(_play), 
        ._mode(_mode),
        .ibeat(ibeatNum)
    );

    // Music module
    // [in]  beat number and en
    // [out] left & right raw frequency
    wire [4:0] key;
    assign key = (key_down[last_change] && been_ready)? key_num : 5'b11111;
    music_example music_00 (
        .ibeatNum(ibeatNum),
        .key(key),
        .en(_play),
        .demonstrate(_mode),
        .been_ready(been_ready),
        .last_change(last_change),
        .start(_start),
        .toneL(freqL),
        .toneR(freqR)
    );

    // freq_outL, freq_outR
    // Note gen makes no sound, if freq_out = 50000000 / `silence = 1
    assign freq_outL = 50000000 / freqL;
    assign freq_outR = 50000000 / freqR;

    // Note generation
    // [in]  processed frequency
    // [out] audio wave signal (using square wave here)
    note_gen noteGen_00(
        .clk(clk), 
        .rst(rst), 
        .volume(volume),
        .mute(_mute),
        .note_div_left(freq_outL), 
        .note_div_right(freq_outR), 
        .audio_left(audio_in_left),     // left sound audio
        .audio_right(audio_in_right)    // right sound audio
    );

    // Speaker controller
    speaker_control sc(
        .clk(clk), 
        .rst(rst), 
        .audio_in_left(audio_in_left),      // left channel audio data input
        .audio_in_right(audio_in_right),    // right channel audio data input
        .audio_mclk(audio_mclk),            // master clock
        .audio_lrck(audio_lrck),            // left-right clock
        .audio_sck(audio_sck),              // serial clock
        .audio_sdin(audio_sdin)             // serial audio data input
    );

endmodule


module player_control (
	input clk, 
	input reset, 
	input _play, 
	input _slow, 
    input _start,
	input _mode, 
	output reg [11:0] ibeat
);
	parameter LEN = 4095;
    reg [11:0] next_ibeat;

	always @(posedge clk, posedge reset) begin
		if (reset) begin
			ibeat <= 0;
		end else if(_play) begin
            ibeat <= next_ibeat;
		end else begin
            ibeat <= ibeat;
		end
	end

    always @* begin
        // Demomstrate
        if(_mode)
            next_ibeat = (ibeat + 1 < LEN) ? (ibeat + 1) : 0;
        // Play
        else if(!_mode && _start)
            next_ibeat = (ibeat + 1 < LEN) ? (ibeat + 1) : ibeat;
        else next_ibeat = ibeat;
    end

endmodule


    

module SevenSegment(
	output reg [6:0] display,
	output reg [3:0] digit,
	input wire [16:0] nums,
	input wire rst,
	input wire clk
    );
    
    reg [15:0] clk_divider;
    reg [4:0] display_num;
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		clk_divider <= 15'b0;
    	end else begin
    		clk_divider <= clk_divider + 15'b1;
    	end
    end
    
    always @ (posedge clk_divider[15], posedge rst) begin
    	if (rst) begin
    		display_num <= 4'b0000;
    		digit <= 4'b1111;
    	end else begin
    		case (digit)
    			4'b1110 : begin //nums[3:0]
    					display_num <= nums[8:4];
    					digit <= 4'b1101;
    				end
    			4'b1101 : begin //nums[7:4]
						display_num <= nums[12:9];
						digit <= 4'b1011;
					end
    			4'b1011 : begin //nums[11:8]
						display_num <= nums[16:13];
						digit <= 4'b0111;
					end
    			4'b0111 : begin //nums[15:12]
						display_num <= nums[3:0];
						digit <= 4'b1110;
					end
    			default : begin
						display_num <= nums[3:0];
						digit <= 4'b1110;
					end				
    		endcase
    	end
    end
    
    always @ (*) begin
        case (display_num)
            0 : display = 7'b1000000;   //0000
            1 : display = 7'b1111001;   //0001
            2 : display = 7'b0100100;   //0010
            3 : display = 7'b0110000;   //0011
            4 : display = 7'b0011001;   //0100
            5 : display = 7'b0010010;   //0101
            6 : display = 7'b0000010;   //0110
            7 : display = 7'b1111000;   //0111
            8 : display = 7'b0000000;   //1000
            9 : display = 7'b0010000;   //1001

            10: display = 7'b0111111; //'-'
            11: display = 7'b1000110; //c
            12: display = 7'b0100001; //d
            13: display = 7'b0000110; //e
            14: display = 7'b0001110; //f
            15: display = 7'b0010000; //g
            16: display = 7'b0001000; //a
            17: display = 7'b0000011; //b
            default : display = 7'b1111111;
        endcase
    end
    
endmodule





// c(Do) , d(Re) , e(Mi) , f(Fa) , g(Sol) , a(La) , b(Si)

module music_example (
	input [11:0] ibeatNum,
	input en,
    input [4:0] key,
    input demonstrate,
    input start,
    input been_ready, 
    input [8:0] last_change,
	output reg [31:0] toneL,
    output reg [31:0] toneR
);

    reg [31:0] key_tone;

    // PITCH_TABLE
    parameter [31:0] PITCH_TABLE [0:20] = {
        32'd524, 32'd588, 32'd660, 32'd698, 32'd784, 32'd880, 32'd988, // High
        32'd262, 32'd294, 32'd330, 32'd349, 32'd392, 32'd440, 32'd494, 
        32'd131, 32'd147, 32'd165, 32'd174, 32'd196, 32'd220, 32'd247   // Low
    };

    always @* begin
        if(demonstrate || (!demonstrate && start)) begin
            if(en == 1 || (!demonstrate && start)) begin
                case(ibeatNum)
                    // --- Measure 1 ---
                    12'd0: toneR = `hg;      12'd1: toneR = `hg; // HG (half-beat)
                    12'd2: toneR = `hg;      12'd3: toneR = `hg;
                    12'd4: toneR = `hg;      12'd5: toneR = `hg;
                    12'd6: toneR = `hg;      12'd7: toneR = `hg;
                    12'd8: toneR = `he;      12'd9: toneR = `he; // HE (half-beat)
                    12'd10: toneR = `he;     12'd11: toneR = `he;
                    12'd12: toneR = `he;     12'd13: toneR = `he;
                    12'd14: toneR = `he;     12'd15: toneR = `sil; // (Short break for repetitive notes: high E)

                    12'd16: toneR = `he;     12'd17: toneR = `he; // HE (one-beat)
                    12'd18: toneR = `he;     12'd19: toneR = `he;
                    12'd20: toneR = `he;     12'd21: toneR = `he;
                    12'd22: toneR = `he;     12'd23: toneR = `he;
                    12'd24: toneR = `he;     12'd25: toneR = `he;
                    12'd26: toneR = `he;     12'd27: toneR = `he;
                    12'd28: toneR = `he;     12'd29: toneR = `he;
                    12'd30: toneR = `he;     12'd31: toneR = `he;

                    12'd32: toneR = `hf;     12'd33: toneR = `hf; // HF (half-beat)
                    12'd34: toneR = `hf;     12'd35: toneR = `hf;
                    12'd36: toneR = `hf;     12'd37: toneR = `hf;
                    12'd38: toneR = `hf;     12'd39: toneR = `hf;
                    12'd40: toneR = `hd;     12'd41: toneR = `hd; // HD (half-beat)
                    12'd42: toneR = `hd;     12'd43: toneR = `hd;
                    12'd44: toneR = `hd;     12'd45: toneR = `hd;
                    12'd46: toneR = `hd;     12'd47: toneR = `sil; // (Short break for repetitive notes: high D)

                    12'd48: toneR = `hd;     12'd49: toneR = `hd; // HD (one-beat)
                    12'd50: toneR = `hd;     12'd51: toneR = `hd;
                    12'd52: toneR = `hd;     12'd53: toneR = `hd;
                    12'd54: toneR = `hd;     12'd55: toneR = `hd;
                    12'd56: toneR = `hd;     12'd57: toneR = `hd;
                    12'd58: toneR = `hd;     12'd59: toneR = `hd;
                    12'd60: toneR = `hd;     12'd61: toneR = `hd;
                    12'd62: toneR = `hd;     12'd63: toneR = `hd;

                    // --- Measure 2 ---
                    12'd64: toneR = `hc;     12'd65: toneR = `hc; // HC (half-beat)
                    12'd66: toneR = `hc;     12'd67: toneR = `hc;
                    12'd68: toneR = `hc;     12'd69: toneR = `hc;
                    12'd70: toneR = `hc;     12'd71: toneR = `hc;
                    12'd72: toneR = `hd;     12'd73: toneR = `hd; // HD (half-beat)
                    12'd74: toneR = `hd;     12'd75: toneR = `hd;
                    12'd76: toneR = `hd;     12'd77: toneR = `hd;
                    12'd78: toneR = `hd;     12'd79: toneR = `hd;

                    12'd80: toneR = `he;     12'd81: toneR = `he; // HE (half-beat)
                    12'd82: toneR = `he;     12'd83: toneR = `he;
                    12'd84: toneR = `he;     12'd85: toneR = `he;
                    12'd86: toneR = `he;     12'd87: toneR = `he;
                    12'd88: toneR = `hf;     12'd89: toneR = `hf; // HF (half-beat)
                    12'd90: toneR = `hf;     12'd91: toneR = `hf;
                    12'd92: toneR = `hf;     12'd93: toneR = `hf;
                    12'd94: toneR = `hf;     12'd95: toneR = `hf;

                    12'd96: toneR = `hg;     12'd97: toneR = `hg; // HG (half-beat)
                    12'd98: toneR = `hg;     12'd99: toneR = `hg;
                    12'd100: toneR = `hg;    12'd101: toneR = `hg;
                    12'd102: toneR = `hg;    12'd103: toneR = `sil; // (Short break for repetitive notes: high D)
                    12'd104: toneR = `hg;    12'd105: toneR = `hg; // HG (half-beat)
                    12'd106: toneR = `hg;    12'd107: toneR = `hg;
                    12'd108: toneR = `hg;    12'd109: toneR = `hg;
                    12'd110: toneR = `hg;    12'd111: toneR = `sil; // (Short break for repetitive notes: high D)

                    12'd112: toneR = `hg;    12'd113: toneR = `hg; // HG (one-beat)
                    12'd114: toneR = `hg;    12'd115: toneR = `hg;
                    12'd116: toneR = `hg;    12'd117: toneR = `hg;
                    12'd118: toneR = `hg;    12'd119: toneR = `hg;
                    12'd120: toneR = `hg;    12'd121: toneR = `hg;
                    12'd122: toneR = `hg;    12'd123: toneR = `hg;
                    12'd124: toneR = `hg;    12'd125: toneR = `hg;
                    12'd126: toneR = `hg;    12'd127: toneR = `hg;

                    default: toneR = `sil;
                endcase
                if(!demonstrate && start) begin              
                    case (key)
                        5'd0:  key_tone = `hc;
                        5'd1:  key_tone = `hd;
                        5'd2:  key_tone = `he;
                        5'd3:  key_tone = `hf;
                        5'd4:  key_tone = `hg;
                        5'd5:  key_tone = `ha;
                        5'd6:  key_tone = `hb;
                        5'd7:  key_tone = `c;
                        5'd8:  key_tone = `d;
                        5'd9:  key_tone = `e;
                        5'd10: key_tone = `f;
                        5'd11: key_tone = `g;
                        5'd12: key_tone = `a;
                        5'd13: key_tone = `b;
                        5'd14: key_tone = `lc;
                        5'd15: key_tone = `ld;
                        5'd16: key_tone = `le;
                        5'd17: key_tone = `lf;
                        5'd18: key_tone = `lg;
                        5'd19: key_tone = `la;
                        5'd20: key_tone = `lb;
                        default: key_tone = `sil;
                    endcase
                    if(PITCH_TABLE[key_tone] == toneR) toneR = toneR;
                    else toneR = `sil;
                end else toneR = toneR;               
            end else toneR = `sil;     
        end else if(!demonstrate && !start) begin // PLAY mode
                case (key)
                    5'd0:  toneR = `hc;
                    5'd1:  toneR = `hd;
                    5'd2:  toneR = `he;
                    5'd3:  toneR = `hf;
                    5'd4:  toneR = `hg;
                    5'd5:  toneR = `ha;
                    5'd6:  toneR = `hb;
                    5'd7:  toneR = `c;
                    5'd8:  toneR = `d;
                    5'd9:  toneR = `e;
                    5'd10: toneR = `f;
                    5'd11: toneR = `g;
                    5'd12: toneR = `a;
                    5'd13: toneR = `b;
                    5'd14: toneR = `lc;
                    5'd15: toneR = `ld;
                    5'd16: toneR = `le;
                    5'd17: toneR = `lf;
                    5'd18: toneR = `lg;
                    5'd19: toneR = `la;
                    5'd20: toneR = `lb;
                    default: toneR = `sil;
                endcase 
        end else begin
            toneR = `sil;
        end
    end

    
    always @* begin
        if(demonstrate || (!demonstrate && start)) begin
            if(en == 1 || (!demonstrate && start)) begin
                case(ibeatNum)
                    12'd0: toneL = `hc;  	12'd1: toneL = `hc; // HC (two-beat)
                    12'd2: toneL = `hc;  	12'd3: toneL = `hc;
                    12'd4: toneL = `hc;	    12'd5: toneL = `hc;
                    12'd6: toneL = `hc;  	12'd7: toneL = `hc;
                    12'd8: toneL = `hc;	    12'd9: toneL = `hc;
                    12'd10: toneL = `hc;	12'd11: toneL = `hc;
                    12'd12: toneL = `hc;	12'd13: toneL = `hc;
                    12'd14: toneL = `hc;	12'd15: toneL = `hc;

                    12'd16: toneL = `hc;	12'd17: toneL = `hc;
                    12'd18: toneL = `hc;	12'd19: toneL = `hc;
                    12'd20: toneL = `hc;	12'd21: toneL = `hc;
                    12'd22: toneL = `hc;	12'd23: toneL = `hc;
                    12'd24: toneL = `hc;	12'd25: toneL = `hc;
                    12'd26: toneL = `hc;	12'd27: toneL = `hc;
                    12'd28: toneL = `hc;	12'd29: toneL = `hc;
                    12'd30: toneL = `hc;	12'd31: toneL = `hc;

                    12'd32: toneL = `g;	    12'd33: toneL = `g; // G (one-beat)
                    12'd34: toneL = `g;	    12'd35: toneL = `g;
                    12'd36: toneL = `g;	    12'd37: toneL = `g;
                    12'd38: toneL = `g;	    12'd39: toneL = `g;
                    12'd40: toneL = `g;	    12'd41: toneL = `g;
                    12'd42: toneL = `g;	    12'd43: toneL = `g;
                    12'd44: toneL = `g;	    12'd45: toneL = `g;
                    12'd46: toneL = `g;	    12'd47: toneL = `g;

                    12'd48: toneL = `b;	    12'd49: toneL = `b; // B (one-beat)
                    12'd50: toneL = `b;	    12'd51: toneL = `b;
                    12'd52: toneL = `b;	    12'd53: toneL = `b;
                    12'd54: toneL = `b;	    12'd55: toneL = `b;
                    12'd56: toneL = `b;	    12'd57: toneL = `b;
                    12'd58: toneL = `b;	    12'd59: toneL = `b;
                    12'd60: toneL = `b;	    12'd61: toneL = `b;
                    12'd62: toneL = `b;	    12'd63: toneL = `b;

                    12'd64: toneL = `hc;	12'd65: toneL = `hc; // HC (two-beat)
                    12'd66: toneL = `hc;    12'd67: toneL = `hc;
                    12'd68: toneL = `hc;	12'd69: toneL = `hc;
                    12'd70: toneL = `hc;	12'd71: toneL = `hc;
                    12'd72: toneL = `hc;	12'd73: toneL = `hc;
                    12'd74: toneL = `hc;	12'd75: toneL = `hc;
                    12'd76: toneL = `hc;	12'd77: toneL = `hc;
                    12'd78: toneL = `hc;	12'd79: toneL = `hc;

                    12'd80: toneL = `hc;	12'd81: toneL = `hc;
                    12'd82: toneL = `hc;    12'd83: toneL = `hc;
                    12'd84: toneL = `hc;    12'd85: toneL = `hc;
                    12'd86: toneL = `hc;    12'd87: toneL = `hc;
                    12'd88: toneL = `hc;    12'd89: toneL = `hc;
                    12'd90: toneL = `hc;    12'd91: toneL = `hc;
                    12'd92: toneL = `hc;    12'd93: toneL = `hc;
                    12'd94: toneL = `hc;    12'd95: toneL = `hc;

                    12'd96: toneL = `g;	    12'd97: toneL = `g; // G (one-beat)
                    12'd98: toneL = `g; 	12'd99: toneL = `g;
                    12'd100: toneL = `g;	12'd101: toneL = `g;
                    12'd102: toneL = `g;	12'd103: toneL = `g;
                    12'd104: toneL = `g;	12'd105: toneL = `g;
                    12'd106: toneL = `g;	12'd107: toneL = `g;
                    12'd108: toneL = `g;	12'd109: toneL = `g;
                    12'd110: toneL = `g;	12'd111: toneL = `g;

                    12'd112: toneL = `b;	12'd113: toneL = `b; // B (one-beat)
                    12'd114: toneL = `b;	12'd115: toneL = `b;
                    12'd116: toneL = `b;	12'd117: toneL = `b;
                    12'd118: toneL = `b;	12'd119: toneL = `b;
                    12'd120: toneL = `b;	12'd121: toneL = `b;
                    12'd122: toneL = `b;	12'd123: toneL = `b;
                    12'd124: toneL = `b;	12'd125: toneL = `b;
                    12'd126: toneL = `b;	12'd127: toneL = `b;

                    default : toneL = `sil;
                endcase
                if(!demonstrate && start) begin              
                    case (key)
                        5'd0:  key_tone = `hc;
                        5'd1:  key_tone = `hd;
                        5'd2:  key_tone = `he;
                        5'd3:  key_tone = `hf;
                        5'd4:  key_tone = `hg;
                        5'd5:  key_tone = `ha;
                        5'd6:  key_tone = `hb;
                        5'd7:  key_tone = `c;
                        5'd8:  key_tone = `d;
                        5'd9:  key_tone = `e;
                        5'd10: key_tone = `f;
                        5'd11: key_tone = `g;
                        5'd12: key_tone = `a;
                        5'd13: key_tone = `b;
                        5'd14: key_tone = `lc;
                        5'd15: key_tone = `ld;
                        5'd16: key_tone = `le;
                        5'd17: key_tone = `lf;
                        5'd18: key_tone = `lg;
                        5'd19: key_tone = `la;
                        5'd20: key_tone = `lb;
                        default: key_tone = `sil;
                    endcase
                    if(PITCH_TABLE[key_tone] == toneR) toneL = toneR;
                    else toneL = `sil;
                end else toneL = toneL;               
            end else toneL = `sil;     
        end else if(!demonstrate && !start) begin // PLAY mode
                case (key)
                    5'd0:  toneL = `hc;
                    5'd1:  toneL = `hd;
                    5'd2:  toneL = `he;
                    5'd3:  toneL = `hf;
                    5'd4:  toneL = `hg;
                    5'd5:  toneL = `ha;
                    5'd6:  toneL = `hb;
                    5'd7:  toneL = `c;
                    5'd8:  toneL = `d;
                    5'd9:  toneL = `e;
                    5'd10: toneL = `f;
                    5'd11: toneL = `g;
                    5'd12: toneL = `a;
                    5'd13: toneL = `b;
                    5'd14: toneL = `lc;
                    5'd15: toneL = `ld;
                    5'd16: toneL = `le;
                    5'd17: toneL = `lf;
                    5'd18: toneL = `lg;
                    5'd19: toneL = `la;
                    5'd20: toneL = `lb;
                    default: toneL = `sil;
                endcase 
        end else begin
            toneL = `sil;
        end
    end

endmodule

module note_gen(
    input clk, // clock from crystal
    input rst, // active high reset
    input mute,
    input [2:0] volume, 
    input [21:0] note_div_left, // div for note generation
    input [21:0] note_div_right,
    output [15:0] audio_left,
    output [15:0] audio_right
    );

    // Declare internal signals
    reg [21:0] clk_cnt_next, clk_cnt;
    reg [21:0] clk_cnt_next_2, clk_cnt_2;
    reg b_clk, b_clk_next;
    reg c_clk, c_clk_next;

    // Note frequency generation
    // clk_cnt, clk_cnt_2, b_clk, c_clk
    always @(posedge clk or posedge rst)
        if (rst == 1'b1)
            begin
                clk_cnt <= 22'd0;
                clk_cnt_2 <= 22'd0;
                b_clk <= 1'b0;
                c_clk <= 1'b0;
            end
        else
            begin
                clk_cnt <= clk_cnt_next;
                clk_cnt_2 <= clk_cnt_next_2;
                b_clk <= b_clk_next;
                c_clk <= c_clk_next;
            end
    
    // clk_cnt_next, b_clk_next
    always @*
        if (clk_cnt == note_div_left)
            begin
                clk_cnt_next = 22'd0;
                b_clk_next = ~b_clk;
            end
        else
            begin
                clk_cnt_next = clk_cnt + 1'b1;
                b_clk_next = b_clk;
            end

    // clk_cnt_next_2, c_clk_next
    always @*
        if (clk_cnt_2 == note_div_right)
            begin
                clk_cnt_next_2 = 22'd0;
                c_clk_next = ~c_clk;
            end
        else
            begin
                clk_cnt_next_2 = clk_cnt_2 + 1'b1;
                c_clk_next = c_clk;
            end

    // volume wave
    reg [15:0] v_amp;
    always @(posedge clk) begin
        if(mute) 
            v_amp <= 16'd0;
        else if(volume == 1)
            v_amp <= 16'h0400; 
        else if(volume == 2)
            v_amp <= 16'h0800;
        else if(volume == 3)
            v_amp <= 16'h1000;
        else if(volume == 4)
            v_amp <= 16'h2000;
        else if(volume == 5)
            v_amp <= 16'h4000;
        else v_amp <= 16'd0;
    end

    // Assign the amplitude of the note
    // Volume is controlled here
    assign audio_left = (note_div_left == 22'd1) ? 16'h0000 : 
                                (b_clk == 1'b0) ? -v_amp : v_amp;
    assign audio_right = (note_div_right == 22'd1) ? 16'h0000 : 
                                (c_clk == 1'b0) ? -v_amp : v_amp;
endmodule