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
    input _play,      // SW0: Play/sil
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
    reg [4:0] key_num, last_key_num;
    wire shift_down;
	wire been_ready; 

    always @(posedge clk) begin
        if(!key_down) begin
            last_key <= 0;
            last_key_num <= 0;
        end else if (key_down[last_change] == 1'b1) begin
            if(!key_down[last_key] && key_num != 5'b11111) begin
                last_key <= last_change;
                last_key_num <= key_num;
            end else begin
                last_key <= last_key;
                key_num <= key_num;
            end
        end else begin
            last_key <= last_key;
            key_num <= key_num;
        end
    end  

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

    wire [11:0] ibeatNum, hbeatNum;               // Beat counter (demonstrate, player)
    wire [31:0] freqL, freqR;           // Raw frequency, produced by music module
    wire [21:0] freq_outL, freq_outR;    // Processed frequency, adapted to the clock rate of Basys3

    // clkDiv22
    wire clkDiv22, clkDiv24;
    clock_divider #(.n(22)) clock_22(.clk(clk), .clk_div(clkDiv22));    // for audio
    clock_divider #(.n(24)) clock_24(.clk(clk), .clk_div(clkDiv24));    // for score

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
            _led = 16'd0;
        else if(volume == 1) 
            _led = 16'b0000_0000_0000_0001;
        else if(volume == 2) 
            _led = 16'b0000_0000_0000_0011;
        else if(volume == 3) 
            _led = 16'b0000_0000_0000_0111;
        else if(volume == 4) 
            _led = 16'b0000_0000_0000_1111;
        else if(volume == 5) 
            _led = 16'b0000_0000_0001_1111;
        else _led = 16'b0000_0000_0000_0111;

        if(!_mode && _start) begin    // Play Helper mode
            case(freqR) 
                `hc, `c, `lc : _led[15] = 1;
                `hd, `d, `ld : _led[14] = 1;
                `he, `e, `le : _led[13] = 1;
                `hf, `f, `lf : _led[12] = 1;
                `hg, `g, `lg : _led[11] = 1;
                `ha, `a, `la : _led[10] = 1;
                `hb, `b, `lb : _led[9] = 1;
            endcase
        end
        // Helper Finish
        if(finish) _led[15:9] = 7'b1111111;
    end

    // key_tone (PITCH for key)
    reg [31:0] key_toneR, key_toneL;
    wire finish;

    // PITCH_TABLE
    parameter [31:0] PITCH_TABLE [0:20] = {
        32'd524, 32'd588, 32'd660, 32'd698, 32'd784, 32'd880, 32'd988, // High
        32'd262, 32'd294, 32'd330, 32'd349, 32'd392, 32'd440, 32'd494, 
        32'd131, 32'd147, 32'd165, 32'd174, 32'd196, 32'd220, 32'd247   // Low
    };

    always @(*) begin
        if(!_mode && !finish) begin
            case (key)
                5'd0:  key_toneR = `hc;
                5'd1:  key_toneR = `hd;
                5'd2:  key_toneR = `he;
                5'd3:  key_toneR = `hf;
                5'd4:  key_toneR = `hg;
                5'd5:  key_toneR = `ha;
                5'd6:  key_toneR = `hb;
                5'd7:  key_toneR = `c;
                5'd8:  key_toneR = `d;
                5'd9:  key_toneR = `e;
                5'd10: key_toneR = `f;
                5'd11: key_toneR = `g;
                5'd12: key_toneR = `a;
                5'd13: key_toneR = `b;
                5'd14: key_toneR = `lc;
                5'd15: key_toneR = `ld;
                5'd16: key_toneR = `le;
                5'd17: key_toneR = `lf;
                5'd18: key_toneR = `lg;
                5'd19: key_toneR = `la;
                5'd20: key_toneR = `lb;
                default: key_toneR = `sil;
            endcase
        end else if (finish) begin
            key_toneR = `sil;
        end  else begin
            key_toneR = freqR;
        end  
    end

    always @(*) begin
        if(!_mode && !finish) begin
            case (key)
                5'd0:  key_toneL = `hc;
                5'd1:  key_toneL = `hd;
                5'd2:  key_toneL = `he;
                5'd3:  key_toneL = `hf;
                5'd4:  key_toneL = `hg;
                5'd5:  key_toneL = `ha;
                5'd6:  key_toneL = `hb;
                5'd7:  key_toneL = `c;
                5'd8:  key_toneL = `d;
                5'd9:  key_toneL = `e;
                5'd10: key_toneL = `f;
                5'd11: key_toneL = `g;
                5'd12: key_toneL = `a;
                5'd13: key_toneL = `b;
                5'd14: key_toneL = `lc;
                5'd15: key_toneL = `ld;
                5'd16: key_toneL = `le;
                5'd17: key_toneL = `lf;
                5'd18: key_toneL = `lg;
                5'd19: key_toneL = `la;
                5'd20: key_toneL = `lb;
                default: key_toneL = `sil;
            endcase
        end else if (finish) begin
            key_toneL = `sil;
        end  else begin
            key_toneL = freqL;
        end  
    end
    


    // 7 segment Control
    reg [16:0] nums;
    reg [6:0] score;
    wire [3:0] ten, unit;
    assign ten = score / 10;
    assign unit = score % 10;
    always @(posedge clkDiv24, posedge rst) begin
        if(rst) begin
            score <= 0;
        end else if(!_mode && _start) begin
            if(!finish && freqR != `sil && (key_toneR == freqR) && score < 7'd99) begin
                score <= score + 7'd1;
            end else begin
                score <= score;
            end
        end else begin
            score <= 0;
        end
    end
	SevenSegment seven_seg (
		.display(DISPLAY),
		.digit(DIGIT),
		.nums(nums),
		.rst(rst),
		.clk(clk)
	);
    always @(*) begin
        if(_mode || (!_mode && !_start)) begin // DEMONSTRATE mode || PIANO mode
            if(_play || (!_mode && !_start)) begin
                case (key_toneR)
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
                    
                    `sil: nums = {4'd10, 4'd10, 5'd10, 4'd10};
                    default: nums = {4'd10, 4'd10, 5'd10, 4'd10};
                endcase
            end else nums = {4'd10, 4'd10, 5'd10, 4'd10};
        end else begin  // HELPER mode
            case (freqR)
                `hc: nums = {ten, unit, 5'd11, 4'd5};
                `hd: nums = {ten, unit, 5'd12, 4'd5};
                `he: nums = {ten, unit, 5'd13, 4'd5};
                `hf: nums = {ten, unit, 5'd14, 4'd5};
                `hg: nums = {ten, unit, 5'd15, 4'd5};
                `ha: nums = {ten, unit, 5'd16, 4'd5};
                `hb: nums = {ten, unit, 5'd17, 4'd5};

                `c: nums = {ten, unit, 5'd11, 4'd4};
                `d: nums = {ten, unit, 5'd12, 4'd4};
                `e: nums = {ten, unit, 5'd13, 4'd4};
                `f: nums = {ten, unit, 5'd14, 4'd4};
                `g: nums = {ten, unit, 5'd15, 4'd4};
                `a: nums = {ten, unit, 5'd16, 4'd4};
                `b: nums = {ten, unit, 5'd17, 4'd4};

                `lc: nums = {ten, unit, 5'd11, 4'd3};
                `ld: nums = {ten, unit, 5'd12, 4'd3};
                `le: nums = {ten, unit, 5'd13, 4'd3};
                `lf: nums = {ten, unit, 5'd14, 4'd3};
                `lg: nums = {ten, unit, 5'd15, 4'd3};
                `la: nums = {ten, unit, 5'd16, 4'd3};
                `lb: nums = {ten, unit, 5'd17, 4'd3};

                default: nums = {ten, unit, 5'd10, 4'd10};
            endcase
        end
    end


    // Player Control
    // [in]  reset, clock, _play, _slow, _music, and _mode
    // [out] beat number
    player_control #(.LEN(512)) playerCtrl_00 ( 
        .clk(clkDiv22),
        .reset(rst),
        ._play(_play), 
        ._mode(_mode),
        ._start(_start),
        .ibeat(ibeatNum),
        .hbeat(hbeatNum),
        .finish(finish)
    );

    // Music module
    // [in]  beat number and en
    // [out] left & right raw frequency
    wire [4:0] key;
    wire correct;
    assign key = (key_down[last_key])? last_key_num : 5'b11111;
    music_example music_00 (
        .ibeatNum(ibeatNum),
        .hbeatNum(hbeatNum),
        .en(_play),
        .demonstrate(_mode),
        .start(_start),
        .toneL(freqL),
        .toneR(freqR)
    );

    // freq_outL, freq_outR
    // Note gen makes no sound, if freq_out = 50000000 / `silence = 1
    assign freq_outL = 50000000 / key_toneL;
    assign freq_outR = 50000000 / key_toneR;

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
	output reg [11:0] ibeat,
    output reg [11:0] hbeat,
    output reg finish
);
	parameter LEN = 4095;
    reg [11:0] next_ibeat, next_hbeat;
    

	always @(posedge clk, posedge reset) begin
		if (reset) begin
			ibeat <= 0;
		end else if(_play) begin
            ibeat <= next_ibeat;
		end else begin
            ibeat <= ibeat;
		end
	end

    always @(posedge clk, posedge reset) begin
		if (reset) begin
			hbeat <= 0;
		end else begin
            hbeat <= next_hbeat;
		end
	end

    always @(*) begin
        finish = 0; 
        // Play
        if(!_mode && _start) begin
            next_hbeat = (hbeat + 1 < LEN) ? (hbeat + 1) : hbeat;
            if(next_hbeat==hbeat) finish = 1;
        end else next_hbeat = 0;
    end

    always @* begin
        // Demomstrate    
        if(_mode) begin
            next_ibeat = (ibeat + 1 < LEN) ? (ibeat + 1) : 0;
        end else next_ibeat = ibeat;
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
    input [11:0] hbeatNum,
	input en,
    input demonstrate,
    input start,
	output reg [31:0] toneL,
    output reg [31:0] toneR
);

    
    always @* begin
        if(en == 1 && demonstrate) begin
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

                12'd128: toneR = `hg;      12'd129: toneR = `hg; // HG (half-beat)
                12'd130: toneR = `hg;      12'd131: toneR = `hg;
                12'd132: toneR = `hg;      12'd133: toneR = `hg;
                12'd134: toneR = `hg;      12'd135: toneR = `hg;
                12'd136: toneR = `he;      12'd137: toneR = `he; // HE (half-beat)
                12'd138: toneR = `he;      12'd139: toneR = `he;
                12'd140: toneR = `he;      12'd141: toneR = `he;
                12'd142: toneR = `he;      12'd143: toneR = `sil; // (Short break for repetitive notes: high E)

                12'd144: toneR = `he;     12'd145: toneR = `he; // HE (one-beat)
                12'd146: toneR = `he;     12'd147: toneR = `he;
                12'd148: toneR = `he;     12'd149: toneR = `he;
                12'd150: toneR = `he;     12'd151: toneR = `he;
                12'd152: toneR = `he;     12'd153: toneR = `he;
                12'd154: toneR = `he;     12'd156: toneR = `he;
                12'd157: toneR = `he;     12'd158: toneR = `he;
                12'd159: toneR = `he;     12'd160: toneR = `he;

                12'd161: toneR = `hf;     12'd162: toneR = `hf; // HF (half-beat)
                12'd163: toneR = `hf;     12'd164: toneR = `hf;
                12'd165: toneR = `hf;     12'd166: toneR = `hf;
                12'd167: toneR = `hf;     12'd168: toneR = `hf;
                12'd169: toneR = `hd;     12'd170: toneR = `hd; // HD (half-beat)
                12'd171: toneR = `hd;     12'd172: toneR = `hd;
                12'd173: toneR = `hd;     12'd174: toneR = `hd;
                12'd175: toneR = `hd;     12'd176: toneR = `sil; // (Short break for repetitive notes: high D)

                12'd177: toneR = `hd;     12'd178: toneR = `hd; // HD (one-beat)
                12'd179: toneR = `hd;     12'd180: toneR = `hd;
                12'd181: toneR = `hd;     12'd182: toneR = `hd;
                12'd183: toneR = `hd;     12'd184: toneR = `hd;
                12'd185: toneR = `hd;     12'd186: toneR = `hd;
                12'd187: toneR = `hd;     12'd188: toneR = `hd;
                12'd189: toneR = `hd;     12'd190: toneR = `hd;
                12'd191: toneR = `hd;     12'd192: toneR = `hd;

                12'd193: toneR = `hc;     12'd194: toneR = `hc; // HC (half-beat)
                12'd195: toneR = `hc;     12'd196: toneR = `hc;
                12'd197: toneR = `hc;     12'd198: toneR = `hc;
                12'd199: toneR = `hc;     12'd200: toneR = `hc;
                12'd201: toneR = `he;     12'd202: toneR = `he; // HE (half-beat)
                12'd203: toneR = `he;     12'd204: toneR = `he;
                12'd205: toneR = `he;     12'd206: toneR = `he;
                12'd207: toneR = `he;     12'd208: toneR = `he;

                12'd209: toneR = `hg;     12'd210: toneR = `hg; // HG (half-beat)
                12'd211: toneR = `hg;     12'd212: toneR = `hg;
                12'd213: toneR = `hg;    12'd214: toneR = `hg;
                12'd215: toneR = `hg;    12'd216: toneR = `sil; // (Short break for repetitive notes: high D)
                12'd217: toneR = `hg;    12'd218: toneR = `hg; // HG (half-beat)
                12'd219: toneR = `hg;    12'd220: toneR = `hg;
                12'd221: toneR = `hg;    12'd222: toneR = `hg;
                12'd223: toneR = `hg;    12'd224: toneR = `hg; // (Short break for repetitive notes: high D)

                12'd225: toneR = `he;    12'd226: toneR = `he; // HE (half-beat)
                12'd227: toneR = `he;    12'd228: toneR = `he;
                12'd229: toneR = `he;    12'd230: toneR = `he;
                12'd231: toneR = `he;    12'd232: toneR = `sil;
                12'd233: toneR = `he;    12'd234: toneR = `he; // HE (half-beat)
                12'd235: toneR = `he;    12'd236: toneR = `he;
                12'd237: toneR = `he;    12'd238: toneR = `he;
                12'd239: toneR = `he;    12'd240: toneR = `sil;

                12'd241: toneR = `he;     12'd242: toneR = `he; // HE (one-beat)
                12'd243: toneR = `he;     12'd244: toneR = `he;
                12'd245: toneR = `he;     12'd246: toneR = `he;
                12'd247: toneR = `he;     12'd248: toneR = `he;
                12'd249: toneR = `he;     12'd250: toneR = `he;
                12'd251: toneR = `he;     12'd252: toneR = `he;
                12'd253: toneR = `he;     12'd254: toneR = `he;
                12'd255: toneR = `he;     12'd256: toneR = `he;

                12'd257: toneR = `hd;     12'd258: toneR = `hd; // HD (half-beat)
                12'd259: toneR = `hd;     12'd260: toneR = `hd;
                12'd261: toneR = `hd;     12'd262: toneR = `hd;
                12'd263: toneR = `hd;     12'd264: toneR = `sil;
                12'd265: toneR = `hd;     12'd266: toneR = `hd; // HD (half-beat)
                12'd267: toneR = `hd;     12'd268: toneR = `hd;
                12'd269: toneR = `hd;     12'd270: toneR = `hd;
                12'd271: toneR = `hd;     12'd272: toneR = `sil;

                12'd273: toneR = `hd;     12'd274: toneR = `hd; // HD (half-beat)
                12'd275: toneR = `hd;     12'd276: toneR = `hd;
                12'd277: toneR = `hd;     12'd278: toneR = `hd;
                12'd279: toneR = `hd;     12'd280: toneR = `sil;
                12'd281: toneR = `hd;     12'd282: toneR = `hd; // HD (half-beat)
                12'd283: toneR = `hd;     12'd284: toneR = `hd;
                12'd285: toneR = `hd;     12'd286: toneR = `hd;
                12'd287: toneR = `hd;     12'd288: toneR = `sil;

                12'd289: toneR = `hd;     12'd290: toneR = `hd; // HD (half-beat)
                12'd291: toneR = `hd;     12'd292: toneR = `hd;
                12'd293: toneR = `hd;    12'd294: toneR = `hd;
                12'd295: toneR = `hd;    12'd296: toneR = `hd; // (Short break for repetitive notes: high D)
                12'd297: toneR = `he;    12'd298: toneR = `he; // HE (half-beat)
                12'd299: toneR = `he;    12'd300: toneR = `he;
                12'd301: toneR = `he;    12'd302: toneR = `he;
                12'd303: toneR = `he;    12'd304: toneR = `he; // (Short break for repetitive notes: high D)

                12'd305: toneR = `hf;    12'd306: toneR = `hf; // HF (one-beat)
                12'd307: toneR = `hf;    12'd308: toneR = `hf;
                12'd309: toneR = `hf;    12'd310: toneR = `hf;
                12'd311: toneR = `hf;    12'd312: toneR = `hf;
                12'd313: toneR = `hf;    12'd314: toneR = `hf;
                12'd315: toneR = `hf;    12'd316: toneR = `hf;
                12'd317: toneR = `hf;    12'd318: toneR = `hf;
                12'd319: toneR = `hf;    12'd320: toneR = `hf;

                12'd321: toneR = `he;     12'd322: toneR = `he; // HE (half-beat)
                12'd323: toneR = `he;     12'd324: toneR = `he;
                12'd325: toneR = `he;     12'd326: toneR = `he;
                12'd327: toneR = `he;     12'd328: toneR = `sil;
                12'd329: toneR = `he;     12'd330: toneR = `he; // HE (half-beat)
                12'd331: toneR = `he;     12'd332: toneR = `he;
                12'd333: toneR = `he;     12'd334: toneR = `he;
                12'd335: toneR = `he;     12'd336: toneR = `sil;

                12'd337: toneR = `he;     12'd338: toneR = `he; // He (half-beat)
                12'd339: toneR = `he;     12'd340: toneR = `he;
                12'd341: toneR = `he;     12'd342: toneR = `he;
                12'd343: toneR = `he;     12'd344: toneR = `sil;
                12'd345: toneR = `he;     12'd346: toneR = `he; // He (half-beat)
                12'd347: toneR = `he;     12'd348: toneR = `he;
                12'd349: toneR = `he;     12'd350: toneR = `he;
                12'd351: toneR = `he;     12'd352: toneR = `sil;

                12'd353: toneR = `he;     12'd354: toneR = `he; // He (half-beat)
                12'd355: toneR = `he;     12'd356: toneR = `he;
                12'd357: toneR = `he;     12'd358: toneR = `he;
                12'd359: toneR = `he;     12'd360: toneR = `he;
                12'd361: toneR = `hf;     12'd362: toneR = `hf; // Hf (half-beat)
                12'd363: toneR = `hf;     12'd364: toneR = `hf;
                12'd365: toneR = `hf;     12'd366: toneR = `hf;
                12'd367: toneR = `hf;     12'd368: toneR = `hf;

                12'd369: toneR = `hg;     12'd370: toneR = `hg; // HG (one-beat)
                12'd371: toneR = `hg;     12'd372: toneR = `hg;
                12'd373: toneR = `hg;    12'd374: toneR = `hg;
                12'd375: toneR = `hg;    12'd376: toneR = `hg; 
                12'd377: toneR = `hg;    12'd378: toneR = `hg; 
                12'd379: toneR = `hg;    12'd380: toneR = `hg;
                12'd381: toneR = `hg;    12'd382: toneR = `hg;
                12'd383: toneR = `hg;    12'd384: toneR = `sil; 

                12'd385: toneR = `hg;    12'd386: toneR = `hg; // Hg (half-beat)
                12'd387: toneR = `hg;    12'd388: toneR = `hg;
                12'd389: toneR = `hg;    12'd390: toneR = `hg;
                12'd391: toneR = `hg;    12'd392: toneR = `hg;
                12'd393: toneR = `he;    12'd394: toneR = `he; // HE (half-beat)
                12'd395: toneR = `he;    12'd396: toneR = `he;
                12'd397: toneR = `he;    12'd398: toneR = `he;
                12'd399: toneR = `he;    12'd400: toneR = `sil;

                12'd401: toneR = `he;     12'd402: toneR = `he; // HE (one-beat)
                12'd403: toneR = `he;     12'd404: toneR = `he;
                12'd405: toneR = `he;     12'd406: toneR = `he;
                12'd407: toneR = `he;     12'd408: toneR = `he;
                12'd409: toneR = `he;     12'd410: toneR = `he;
                12'd411: toneR = `he;     12'd412: toneR = `he;
                12'd413: toneR = `he;     12'd414: toneR = `he;
                12'd415: toneR = `he;     12'd416: toneR = `he;

                12'd417: toneR = `hf;     12'd418: toneR = `hf; // Hf (half-beat)
                12'd419: toneR = `hf;     12'd420: toneR = `hf;
                12'd421: toneR = `hf;     12'd422: toneR = `hf;
                12'd423: toneR = `hf;     12'd424: toneR = `hf;
                12'd425: toneR = `hd;     12'd426: toneR = `hd; // HD (half-beat)
                12'd427: toneR = `hd;     12'd428: toneR = `hd;
                12'd429: toneR = `hd;     12'd430: toneR = `hd;
                12'd431: toneR = `hd;     12'd432: toneR = `sil;

                12'd433: toneR = `hd;     12'd434: toneR = `hd; // HD (half-beat)
                12'd435: toneR = `hd;     12'd436: toneR = `hd;
                12'd437: toneR = `hd;     12'd438: toneR = `hd;
                12'd439: toneR = `hd;     12'd440: toneR = `hd;
                12'd441: toneR = `hd;     12'd442: toneR = `hd; // HD (half-beat)
                12'd443: toneR = `hd;     12'd444: toneR = `hd;
                12'd445: toneR = `hd;     12'd446: toneR = `hd;
                12'd447: toneR = `hd;     12'd448: toneR = `hd;

                12'd449: toneR = `hc;     12'd450: toneR = `hc; // Hc (half-beat)
                12'd451: toneR = `hc;     12'd452: toneR = `hc;
                12'd453: toneR = `hc;    12'd454: toneR = `hc;
                12'd455: toneR = `hc;    12'd456: toneR = `hc; 
                12'd457: toneR = `he;    12'd458: toneR = `he; // HE (half-beat)
                12'd459: toneR = `he;    12'd460: toneR = `he;
                12'd461: toneR = `he;    12'd462: toneR = `he;
                12'd463: toneR = `he;    12'd464: toneR = `he; 

                12'd465: toneR = `hg;    12'd466: toneR = `hg; // HG (half-beat)
                12'd467: toneR = `hg;    12'd468: toneR = `hg;
                12'd469: toneR = `hg;    12'd470: toneR = `hg;
                12'd471: toneR = `hg;    12'd472: toneR = `sil;
                12'd473: toneR = `hg;    12'd474: toneR = `hg; // HG (half-beat)
                12'd475: toneR = `hg;    12'd476: toneR = `hg;
                12'd477: toneR = `hg;    12'd478: toneR = `hg;
                12'd479: toneR = `hg;    12'd480: toneR = `hg;

                12'd481: toneR = `hc;     12'd482: toneR = `hc; // Hc (two-beat)
                12'd483: toneR = `hc;     12'd484: toneR = `hc;
                12'd485: toneR = `hc;     12'd486: toneR = `hc;
                12'd487: toneR = `hc;     12'd488: toneR = `hc;
                12'd489: toneR = `hc;     12'd490: toneR = `hc; 
                12'd491: toneR = `hc;     12'd492: toneR = `hc;
                12'd493: toneR = `hc;     12'd494: toneR = `hc;
                12'd495: toneR = `hc;     12'd496: toneR = `hc;

                12'd497: toneR = `hc;     12'd498: toneR = `hc; 
                12'd499: toneR = `hc;     12'd500: toneR = `hc;
                12'd501: toneR = `hc;     12'd502: toneR = `hc;
                12'd503: toneR = `hc;     12'd504: toneR = `hc;
                12'd505: toneR = `hc;     12'd506: toneR = `hc; 
                12'd507: toneR = `hc;     12'd508: toneR = `hc;
                12'd509: toneR = `hc;     12'd510: toneR = `hc;
                12'd511: toneR = `hc;     12'd512: toneR = `hc;

                default: toneR = `sil;
            endcase  
        end else if(en == 0 && demonstrate) begin
            toneR = `sil;
        end else if(!demonstrate && start)begin
            case(hbeatNum)
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
                12'd126: toneR = `hg;    12'd127: toneR = `sil;

                12'd128: toneR = `hg;      12'd129: toneR = `hg; // HG (half-beat)
                12'd130: toneR = `hg;      12'd131: toneR = `hg;
                12'd132: toneR = `hg;      12'd133: toneR = `hg;
                12'd134: toneR = `hg;      12'd135: toneR = `hg;
                12'd136: toneR = `he;      12'd137: toneR = `he; // HE (half-beat)
                12'd138: toneR = `he;      12'd139: toneR = `he;
                12'd140: toneR = `he;      12'd141: toneR = `he;
                12'd142: toneR = `he;      12'd143: toneR = `sil; // (Short break for repetitive notes: high E)

                12'd144: toneR = `he;     12'd145: toneR = `he; // HE (one-beat)
                12'd146: toneR = `he;     12'd147: toneR = `he;
                12'd148: toneR = `he;     12'd149: toneR = `he;
                12'd150: toneR = `he;     12'd151: toneR = `he;
                12'd152: toneR = `he;     12'd153: toneR = `he;
                12'd154: toneR = `he;     12'd156: toneR = `he;
                12'd157: toneR = `he;     12'd158: toneR = `he;
                12'd159: toneR = `he;     12'd160: toneR = `he;

                12'd161: toneR = `hf;     12'd162: toneR = `hf; // HF (half-beat)
                12'd163: toneR = `hf;     12'd164: toneR = `hf;
                12'd165: toneR = `hf;     12'd166: toneR = `hf;
                12'd167: toneR = `hf;     12'd168: toneR = `hf;
                12'd169: toneR = `hd;     12'd170: toneR = `hd; // HD (half-beat)
                12'd171: toneR = `hd;     12'd172: toneR = `hd;
                12'd173: toneR = `hd;     12'd174: toneR = `hd;
                12'd175: toneR = `hd;     12'd176: toneR = `sil; // (Short break for repetitive notes: high D)

                12'd177: toneR = `hd;     12'd178: toneR = `hd; // HD (one-beat)
                12'd179: toneR = `hd;     12'd180: toneR = `hd;
                12'd181: toneR = `hd;     12'd182: toneR = `hd;
                12'd183: toneR = `hd;     12'd184: toneR = `hd;
                12'd185: toneR = `hd;     12'd186: toneR = `hd;
                12'd187: toneR = `hd;     12'd188: toneR = `hd;
                12'd189: toneR = `hd;     12'd190: toneR = `hd;
                12'd191: toneR = `hd;     12'd192: toneR = `hd;

                12'd193: toneR = `hc;     12'd194: toneR = `hc; // HC (half-beat)
                12'd195: toneR = `hc;     12'd196: toneR = `hc;
                12'd197: toneR = `hc;     12'd198: toneR = `hc;
                12'd199: toneR = `hc;     12'd200: toneR = `hc;
                12'd201: toneR = `he;     12'd202: toneR = `he; // HE (half-beat)
                12'd203: toneR = `he;     12'd204: toneR = `he;
                12'd205: toneR = `he;     12'd206: toneR = `he;
                12'd207: toneR = `he;     12'd208: toneR = `he;

                12'd209: toneR = `hg;     12'd210: toneR = `hg; // HG (half-beat)
                12'd211: toneR = `hg;     12'd212: toneR = `hg;
                12'd213: toneR = `hg;    12'd214: toneR = `hg;
                12'd215: toneR = `hg;    12'd216: toneR = `sil; // (Short break for repetitive notes: high D)
                12'd217: toneR = `hg;    12'd218: toneR = `hg; // HG (half-beat)
                12'd219: toneR = `hg;    12'd220: toneR = `hg;
                12'd221: toneR = `hg;    12'd222: toneR = `hg;
                12'd223: toneR = `hg;    12'd224: toneR = `hg; // (Short break for repetitive notes: high D)

                12'd225: toneR = `he;    12'd226: toneR = `he; // HE (half-beat)
                12'd227: toneR = `he;    12'd228: toneR = `he;
                12'd229: toneR = `he;    12'd230: toneR = `he;
                12'd231: toneR = `he;    12'd232: toneR = `sil;
                12'd233: toneR = `he;    12'd234: toneR = `he; // HE (half-beat)
                12'd235: toneR = `he;    12'd236: toneR = `he;
                12'd237: toneR = `he;    12'd238: toneR = `he;
                12'd239: toneR = `he;    12'd240: toneR = `sil;

                12'd241: toneR = `he;     12'd242: toneR = `he; // HE (one-beat)
                12'd243: toneR = `he;     12'd244: toneR = `he;
                12'd245: toneR = `he;     12'd246: toneR = `he;
                12'd247: toneR = `he;     12'd248: toneR = `he;
                12'd249: toneR = `he;     12'd250: toneR = `he;
                12'd251: toneR = `he;     12'd252: toneR = `he;
                12'd253: toneR = `he;     12'd254: toneR = `he;
                12'd255: toneR = `he;     12'd256: toneR = `he;

                12'd257: toneR = `hd;     12'd258: toneR = `hd; // HD (half-beat)
                12'd259: toneR = `hd;     12'd260: toneR = `hd;
                12'd261: toneR = `hd;     12'd262: toneR = `hd;
                12'd263: toneR = `hd;     12'd264: toneR = `sil;
                12'd265: toneR = `hd;     12'd266: toneR = `hd; // HD (half-beat)
                12'd267: toneR = `hd;     12'd268: toneR = `hd;
                12'd269: toneR = `hd;     12'd270: toneR = `hd;
                12'd271: toneR = `hd;     12'd272: toneR = `sil;

                12'd273: toneR = `hd;     12'd274: toneR = `hd; // HD (half-beat)
                12'd275: toneR = `hd;     12'd276: toneR = `hd;
                12'd277: toneR = `hd;     12'd278: toneR = `hd;
                12'd279: toneR = `hd;     12'd280: toneR = `sil;
                12'd281: toneR = `hd;     12'd282: toneR = `hd; // HD (half-beat)
                12'd283: toneR = `hd;     12'd284: toneR = `hd;
                12'd285: toneR = `hd;     12'd286: toneR = `hd;
                12'd287: toneR = `hd;     12'd288: toneR = `sil;

                12'd289: toneR = `hd;     12'd290: toneR = `hd; // HD (half-beat)
                12'd291: toneR = `hd;     12'd292: toneR = `hd;
                12'd293: toneR = `hd;    12'd294: toneR = `hd;
                12'd295: toneR = `hd;    12'd296: toneR = `hd; // (Short break for repetitive notes: high D)
                12'd297: toneR = `he;    12'd298: toneR = `he; // HE (half-beat)
                12'd299: toneR = `he;    12'd300: toneR = `he;
                12'd301: toneR = `he;    12'd302: toneR = `he;
                12'd303: toneR = `he;    12'd304: toneR = `he; // (Short break for repetitive notes: high D)

                12'd305: toneR = `hf;    12'd306: toneR = `hf; // HF (one-beat)
                12'd307: toneR = `hf;    12'd308: toneR = `hf;
                12'd309: toneR = `hf;    12'd310: toneR = `hf;
                12'd311: toneR = `hf;    12'd312: toneR = `hf;
                12'd313: toneR = `hf;    12'd314: toneR = `hf;
                12'd315: toneR = `hf;    12'd316: toneR = `hf;
                12'd317: toneR = `hf;    12'd318: toneR = `hf;
                12'd319: toneR = `hf;    12'd320: toneR = `hf;

                12'd321: toneR = `he;     12'd322: toneR = `he; // HE (half-beat)
                12'd323: toneR = `he;     12'd324: toneR = `he;
                12'd325: toneR = `he;     12'd326: toneR = `he;
                12'd327: toneR = `he;     12'd328: toneR = `sil;
                12'd329: toneR = `he;     12'd330: toneR = `he; // HE (half-beat)
                12'd331: toneR = `he;     12'd332: toneR = `he;
                12'd333: toneR = `he;     12'd334: toneR = `he;
                12'd335: toneR = `he;     12'd336: toneR = `sil;

                12'd337: toneR = `he;     12'd338: toneR = `he; // He (half-beat)
                12'd339: toneR = `he;     12'd340: toneR = `he;
                12'd341: toneR = `he;     12'd342: toneR = `he;
                12'd343: toneR = `he;     12'd344: toneR = `sil;
                12'd345: toneR = `he;     12'd346: toneR = `he; // He (half-beat)
                12'd347: toneR = `he;     12'd348: toneR = `he;
                12'd349: toneR = `he;     12'd350: toneR = `he;
                12'd351: toneR = `he;     12'd352: toneR = `sil;

                12'd353: toneR = `he;     12'd354: toneR = `he; // He (half-beat)
                12'd355: toneR = `he;     12'd356: toneR = `he;
                12'd357: toneR = `he;     12'd358: toneR = `he;
                12'd359: toneR = `he;     12'd360: toneR = `he;
                12'd361: toneR = `hf;     12'd362: toneR = `hf; // Hf (half-beat)
                12'd363: toneR = `hf;     12'd364: toneR = `hf;
                12'd365: toneR = `hf;     12'd366: toneR = `hf;
                12'd367: toneR = `hf;     12'd368: toneR = `hf;

                12'd369: toneR = `hg;     12'd370: toneR = `hg; // HG (one-beat)
                12'd371: toneR = `hg;     12'd372: toneR = `hg;
                12'd373: toneR = `hg;    12'd374: toneR = `hg;
                12'd375: toneR = `hg;    12'd376: toneR = `hg; 
                12'd377: toneR = `hg;    12'd378: toneR = `hg; 
                12'd379: toneR = `hg;    12'd380: toneR = `hg;
                12'd381: toneR = `hg;    12'd382: toneR = `hg;
                12'd383: toneR = `hg;    12'd384: toneR = `sil; 

                12'd385: toneR = `hg;    12'd386: toneR = `hg; // Hg (half-beat)
                12'd387: toneR = `hg;    12'd388: toneR = `hg;
                12'd389: toneR = `hg;    12'd390: toneR = `hg;
                12'd391: toneR = `hg;    12'd392: toneR = `hg;
                12'd393: toneR = `he;    12'd394: toneR = `he; // HE (half-beat)
                12'd395: toneR = `he;    12'd396: toneR = `he;
                12'd397: toneR = `he;    12'd398: toneR = `he;
                12'd399: toneR = `he;    12'd400: toneR = `sil;

                12'd401: toneR = `he;     12'd402: toneR = `he; // HE (one-beat)
                12'd403: toneR = `he;     12'd404: toneR = `he;
                12'd405: toneR = `he;     12'd406: toneR = `he;
                12'd407: toneR = `he;     12'd408: toneR = `he;
                12'd409: toneR = `he;     12'd410: toneR = `he;
                12'd411: toneR = `he;     12'd412: toneR = `he;
                12'd413: toneR = `he;     12'd414: toneR = `he;
                12'd415: toneR = `he;     12'd416: toneR = `he;

                12'd417: toneR = `hf;     12'd418: toneR = `hf; // Hf (half-beat)
                12'd419: toneR = `hf;     12'd420: toneR = `hf;
                12'd421: toneR = `hf;     12'd422: toneR = `hf;
                12'd423: toneR = `hf;     12'd424: toneR = `hf;
                12'd425: toneR = `hd;     12'd426: toneR = `hd; // HD (half-beat)
                12'd427: toneR = `hd;     12'd428: toneR = `hd;
                12'd429: toneR = `hd;     12'd430: toneR = `hd;
                12'd431: toneR = `hd;     12'd432: toneR = `sil;

                12'd433: toneR = `hd;     12'd434: toneR = `hd; // HD (half-beat)
                12'd435: toneR = `hd;     12'd436: toneR = `hd;
                12'd437: toneR = `hd;     12'd438: toneR = `hd;
                12'd439: toneR = `hd;     12'd440: toneR = `hd;
                12'd441: toneR = `hd;     12'd442: toneR = `hd; // HD (half-beat)
                12'd443: toneR = `hd;     12'd444: toneR = `hd;
                12'd445: toneR = `hd;     12'd446: toneR = `hd;
                12'd447: toneR = `hd;     12'd448: toneR = `hd;

                12'd449: toneR = `hc;     12'd450: toneR = `hc; // Hc (half-beat)
                12'd451: toneR = `hc;     12'd452: toneR = `hc;
                12'd453: toneR = `hc;    12'd454: toneR = `hc;
                12'd455: toneR = `hc;    12'd456: toneR = `hc; 
                12'd457: toneR = `he;    12'd458: toneR = `he; // HE (half-beat)
                12'd459: toneR = `he;    12'd460: toneR = `he;
                12'd461: toneR = `he;    12'd462: toneR = `he;
                12'd463: toneR = `he;    12'd464: toneR = `he; 

                12'd465: toneR = `hf;    12'd466: toneR = `hf; // HF (one-beat)
                12'd467: toneR = `hf;    12'd468: toneR = `hf;
                12'd469: toneR = `hf;    12'd470: toneR = `hf;
                12'd471: toneR = `hf;    12'd472: toneR = `sil;
                12'd473: toneR = `hf;    12'd474: toneR = `hf;
                12'd475: toneR = `hf;    12'd476: toneR = `hf;
                12'd477: toneR = `hf;    12'd478: toneR = `hf;
                12'd479: toneR = `hf;    12'd480: toneR = `hf;

                12'd481: toneR = `hc;     12'd482: toneR = `hc; // Hc (two-beat)
                12'd483: toneR = `hc;     12'd484: toneR = `hc;
                12'd485: toneR = `hc;     12'd486: toneR = `hc;
                12'd487: toneR = `hc;     12'd488: toneR = `hc;
                12'd489: toneR = `hc;     12'd490: toneR = `hc; 
                12'd491: toneR = `hc;     12'd492: toneR = `hc;
                12'd493: toneR = `hc;     12'd494: toneR = `hc;
                12'd495: toneR = `hc;     12'd496: toneR = `hc;

                12'd497: toneR = `hc;     12'd498: toneR = `hc; 
                12'd499: toneR = `hc;     12'd500: toneR = `hc;
                12'd501: toneR = `hc;     12'd502: toneR = `hc;
                12'd503: toneR = `hc;     12'd504: toneR = `hc;
                12'd505: toneR = `hc;     12'd506: toneR = `hc; 
                12'd507: toneR = `hc;     12'd508: toneR = `hc;
                12'd509: toneR = `hc;     12'd510: toneR = `hc;
                12'd511: toneR = `hc;     12'd512: toneR = `hc;

                default: toneR = `sil;
            endcase
        end else begin
            toneR = `sil;
        end
    end

    
    always @* begin
        if(en == 1 && demonstrate) begin
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

                12'd128: toneL = `hc;      12'd129: toneL = `hc; // HC (two-beat)
                12'd130: toneL = `hc;      12'd131: toneL = `hc;
                12'd132: toneL = `hc;      12'd133: toneL = `hc;
                12'd134: toneL = `hc;      12'd135: toneL = `hc;
                12'd136: toneL = `hc;      12'd137: toneL = `hc; 
                12'd138: toneL = `hc;      12'd139: toneL = `hc;
                12'd140: toneL = `hc;      12'd141: toneL = `hc;
                12'd142: toneL = `hc;      12'd143: toneL = `hc;

                12'd144: toneL = `hc;     12'd145: toneL = `hc; 
                12'd146: toneL = `hc;     12'd147: toneL = `hc;
                12'd148: toneL = `hc;     12'd149: toneL = `hc;
                12'd150: toneL = `hc;     12'd151: toneL = `hc;
                12'd152: toneL = `hc;     12'd153: toneL = `hc;
                12'd154: toneL = `hc;     12'd156: toneL = `hc;
                12'd157: toneL = `hc;     12'd158: toneL = `hc;
                12'd159: toneL = `hc;     12'd160: toneL = `hc;

                12'd161: toneL = `g;     12'd162: toneL = `g; // G (one-beat)
                12'd163: toneL = `g;     12'd164: toneL = `g;
                12'd165: toneL = `g;     12'd166: toneL = `g;
                12'd167: toneL = `g;     12'd168: toneL = `g;
                12'd169: toneL = `g;     12'd170: toneL = `g; 
                12'd171: toneL = `g;     12'd172: toneL = `g;
                12'd173: toneL = `g;     12'd174: toneL = `g;
                12'd175: toneL = `g;     12'd176: toneL = `g; 

                12'd177: toneL = `b;     12'd178: toneL = `b; // B (one-beat)
                12'd179: toneL = `b;     12'd180: toneL = `b;
                12'd181: toneL = `b;     12'd182: toneL = `b;
                12'd183: toneL = `b;     12'd184: toneL = `b;
                12'd185: toneL = `b;     12'd186: toneL = `b;
                12'd187: toneL = `b;     12'd188: toneL = `b;
                12'd189: toneL = `b;     12'd190: toneL = `b;
                12'd191: toneL = `b;     12'd192: toneL = `b;

                12'd193: toneL = `hc;     12'd194: toneL = `hc; // HC (one-beat)
                12'd195: toneL = `hc;     12'd196: toneL = `hc;
                12'd197: toneL = `hc;     12'd198: toneL = `hc;
                12'd199: toneL = `hc;     12'd200: toneL = `hc;
                12'd201: toneL = `hc;     12'd202: toneL = `hc;
                12'd203: toneL = `hc;     12'd204: toneL = `hc;
                12'd205: toneL = `hc;     12'd206: toneL = `hc;
                12'd207: toneL = `hc;     12'd208: toneL = `hc;

                12'd209: toneL = `g;     12'd210: toneL = `g; // g (one-beat)
                12'd211: toneL = `g;     12'd212: toneL = `g;
                12'd213: toneL = `g;     12'd214: toneL = `g;
                12'd215: toneL = `g;     12'd216: toneL = `g; 
                12'd217: toneL = `g;     12'd218: toneL = `g; 
                12'd219: toneL = `g;     12'd220: toneL = `g;
                12'd221: toneL = `g;     12'd222: toneL = `g;
                12'd223: toneL = `g;     12'd224: toneL = `g; 

                12'd225: toneL = `e;    12'd226: toneL = `e; // E (one-beat)
                12'd227: toneL = `e;    12'd228: toneL = `e;
                12'd229: toneL = `e;    12'd230: toneL = `e;
                12'd231: toneL = `e;    12'd232: toneL = `e;
                12'd233: toneL = `e;    12'd234: toneL = `e;
                12'd235: toneL = `e;    12'd236: toneL = `e;
                12'd237: toneL = `e;    12'd238: toneL = `e;
                12'd239: toneL = `e;    12'd240: toneL = `e;

                12'd241: toneL = `c;     12'd242: toneL = `c; // C (one-beat)
                12'd243: toneL = `c;     12'd244: toneL = `c;
                12'd245: toneL = `c;     12'd246: toneL = `c;
                12'd247: toneL = `c;     12'd248: toneL = `c;
                12'd249: toneL = `c;     12'd250: toneL = `c;
                12'd251: toneL = `c;     12'd252: toneL = `c;
                12'd253: toneL = `c;     12'd254: toneL = `c;
                12'd255: toneL = `c;     12'd256: toneL = `c;

                12'd257: toneL = `g;     12'd258: toneL = `g; // G (two-beat)
                12'd259: toneL = `g;     12'd260: toneL = `g;
                12'd261: toneL = `g;     12'd262: toneL = `g;
                12'd263: toneL = `g;     12'd264: toneL = `g;
                12'd265: toneL = `g;     12'd266: toneL = `g; 
                12'd267: toneL = `g;     12'd268: toneL = `g;
                12'd269: toneL = `g;     12'd270: toneL = `g;
                12'd271: toneL = `g;     12'd272: toneL = `g;

                12'd273: toneL = `g;     12'd274: toneL = `g;
                12'd275: toneL = `g;     12'd276: toneL = `g;
                12'd277: toneL = `g;     12'd278: toneL = `g;
                12'd279: toneL = `g;     12'd280: toneL = `g;
                12'd281: toneL = `g;     12'd282: toneL = `g; 
                12'd283: toneL = `g;     12'd284: toneL = `g;
                12'd285: toneL = `g;     12'd286: toneL = `g;
                12'd287: toneL = `g;     12'd288: toneL = `g;

                12'd289: toneL = `f;     12'd290: toneL = `f; // F (half-beat)
                12'd291: toneL = `f;     12'd292: toneL = `f;
                12'd293: toneL = `f;    12'd294: toneL = `f;
                12'd295: toneL = `f;    12'd296: toneL = `f; 
                12'd297: toneL = `f;    12'd298: toneL = `f;
                12'd299: toneL = `f;    12'd300: toneL = `f;
                12'd301: toneL = `f;    12'd302: toneL = `f;
                12'd303: toneL = `f;    12'd304: toneL = `f; 

                12'd305: toneL = `d;    12'd306: toneL = `d; // D (one-beat)
                12'd307: toneL = `d;    12'd308: toneL = `d;
                12'd309: toneL = `d;    12'd310: toneL = `d;
                12'd311: toneL = `d;    12'd312: toneL = `d;
                12'd313: toneL = `d;    12'd314: toneL = `d;
                12'd315: toneL = `d;    12'd316: toneL = `d;
                12'd317: toneL = `d;    12'd318: toneL = `d;
                12'd319: toneL = `d;    12'd320: toneL = `d;

                12'd321: toneL = `e;     12'd322: toneL = `e; // E (two-beat)
                12'd323: toneL = `e;     12'd324: toneL = `e;
                12'd325: toneL = `e;     12'd326: toneL = `e;
                12'd327: toneL = `e;     12'd328: toneL = `e;
                12'd329: toneL = `e;     12'd330: toneL = `e; 
                12'd331: toneL = `e;     12'd332: toneL = `e;
                12'd333: toneL = `e;     12'd334: toneL = `e;
                12'd335: toneL = `e;     12'd336: toneL = `e;

                12'd337: toneL = `e;     12'd338: toneL = `e; 
                12'd339: toneL = `e;     12'd340: toneL = `e;
                12'd341: toneL = `e;     12'd342: toneL = `e;
                12'd343: toneL = `e;     12'd344: toneL = `e;
                12'd345: toneL = `e;     12'd346: toneL = `e; 
                12'd347: toneL = `e;     12'd348: toneL = `e;
                12'd349: toneL = `e;     12'd350: toneL = `e;
                12'd351: toneL = `e;     12'd352: toneL = `e;

                12'd353: toneL = `g;     12'd354: toneL = `g; // G (one-beat)
                12'd355: toneL = `g;     12'd356: toneL = `g;
                12'd357: toneL = `g;     12'd358: toneL = `g;
                12'd359: toneL = `g;     12'd360: toneL = `g;
                12'd361: toneL = `g;     12'd362: toneL = `g; 
                12'd363: toneL = `g;     12'd364: toneL = `g;
                12'd365: toneL = `g;     12'd366: toneL = `g;
                12'd367: toneL = `g;     12'd368: toneL = `g;

                12'd369: toneL = `b;     12'd370: toneL = `b; // B (one-beat)
                12'd371: toneL = `b;     12'd372: toneL = `b;
                12'd373: toneL = `b;    12'd374: toneL = `b;
                12'd375: toneL = `b;    12'd376: toneL = `b; 
                12'd377: toneL = `b;    12'd378: toneL = `b; 
                12'd379: toneL = `b;    12'd380: toneL = `b;
                12'd381: toneL = `b;    12'd382: toneL = `b;
                12'd383: toneL = `b;    12'd384: toneL = `b; 

                12'd385: toneL = `hc;    12'd386: toneL = `hc; // HC (two-beat)
                12'd387: toneL = `hc;    12'd388: toneL = `hc;
                12'd389: toneL = `hc;    12'd390: toneL = `hc;
                12'd391: toneL = `hc;    12'd392: toneL = `hc;
                12'd393: toneL = `hc;    12'd394: toneL = `hc; 
                12'd395: toneL = `hc;    12'd396: toneL = `hc;
                12'd397: toneL = `hc;    12'd398: toneL = `hc;
                12'd399: toneL = `hc;    12'd400: toneL = `hc;

                12'd401: toneL = `hc;     12'd402: toneL = `hc;
                12'd403: toneL = `hc;     12'd404: toneL = `hc;
                12'd405: toneL = `hc;     12'd406: toneL = `hc;
                12'd407: toneL = `hc;     12'd408: toneL = `hc;
                12'd409: toneL = `hc;     12'd410: toneL = `hc;
                12'd411: toneL = `hc;     12'd412: toneL = `hc;
                12'd413: toneL = `hc;     12'd414: toneL = `hc;
                12'd415: toneL = `hc;     12'd416: toneL = `hc;

                12'd417: toneL = `g;     12'd418: toneL = `g; // G (one-beat)
                12'd419: toneL = `g;     12'd420: toneL = `g;
                12'd421: toneL = `g;     12'd422: toneL = `g;
                12'd423: toneL = `g;     12'd424: toneL = `g;
                12'd425: toneL = `g;     12'd426: toneL = `g; 
                12'd427: toneL = `g;     12'd428: toneL = `g;
                12'd429: toneL = `g;     12'd430: toneL = `g;
                12'd431: toneL = `g;     12'd432: toneL = `g;

                12'd433: toneL = `b;     12'd434: toneL = `b; // b (one-beat)
                12'd435: toneL = `b;     12'd436: toneL = `b;
                12'd437: toneL = `b;     12'd438: toneL = `b;
                12'd439: toneL = `b;     12'd440: toneL = `b;
                12'd441: toneL = `b;     12'd442: toneL = `b; 
                12'd443: toneL = `b;     12'd444: toneL = `b;
                12'd445: toneL = `b;     12'd446: toneL = `b;
                12'd447: toneL = `b;     12'd448: toneL = `b;

                12'd449: toneL = `hc;     12'd450: toneL = `hc; // Hc (one-beat)
                12'd451: toneL = `hc;     12'd452: toneL = `hc;
                12'd453: toneL = `hc;    12'd454: toneL = `hc;
                12'd455: toneL = `hc;    12'd456: toneL = `hc; 
                12'd457: toneL = `hc;    12'd458: toneL = `hc; 
                12'd459: toneL = `hc;    12'd460: toneL = `hc;
                12'd461: toneL = `hc;    12'd462: toneL = `hc;
                12'd463: toneL = `hc;    12'd464: toneL = `hc; 

                12'd465: toneL = `g;    12'd466: toneL = `g; // g (one-beat)
                12'd467: toneL = `g;    12'd468: toneL = `g;
                12'd469: toneL = `g;    12'd470: toneL = `g;
                12'd471: toneL = `g;    12'd472: toneL = `g;
                12'd473: toneL = `g;    12'd474: toneL = `g;
                12'd475: toneL = `g;    12'd476: toneL = `g;
                12'd477: toneL = `g;    12'd478: toneL = `g;
                12'd479: toneL = `g;    12'd480: toneL = `g;

                12'd481: toneL = `c;     12'd482: toneL = `c; // C (two-beat)
                12'd483: toneL = `c;     12'd484: toneL = `c;
                12'd485: toneL = `c;     12'd486: toneL = `c;
                12'd487: toneL = `c;     12'd488: toneL = `c;
                12'd489: toneL = `c;     12'd490: toneL = `c; 
                12'd491: toneL = `c;     12'd492: toneL = `c;
                12'd493: toneL = `c;     12'd494: toneL = `c;
                12'd495: toneL = `c;     12'd496: toneL = `c;

                12'd497: toneL = `c;     12'd498: toneL = `c; 
                12'd499: toneL = `c;     12'd500: toneL = `c;
                12'd501: toneL = `c;     12'd502: toneL = `c;
                12'd503: toneL = `c;     12'd504: toneL = `c;
                12'd505: toneL = `c;     12'd506: toneL = `c; 
                12'd507: toneL = `c;     12'd508: toneL = `c;
                12'd509: toneL = `c;     12'd510: toneL = `c;
                12'd511: toneL = `c;     12'd512: toneL = `c;



                
                default : toneL = `sil;
            endcase        
        end else if(demonstrate) begin
            toneL = `sil;
        end else if(!demonstrate && start) begin
            case(hbeatNum)
                // --- Measure 1 ---
                12'd0: toneL = `hg;      12'd1: toneL = `hg; // HG (half-beat)
                12'd2: toneL = `hg;      12'd3: toneL = `hg;
                12'd4: toneL = `hg;      12'd5: toneL = `hg;
                12'd6: toneL = `hg;      12'd7: toneL = `hg;
                12'd8: toneL = `he;      12'd9: toneL = `he; // HE (half-beat)
                12'd10: toneL = `he;     12'd11: toneL = `he;
                12'd12: toneL = `he;     12'd13: toneL = `he;
                12'd14: toneL = `he;     12'd15: toneL = `sil; // (Short break for repetitive notes: high E)

                12'd16: toneL = `he;     12'd17: toneL = `he; // HE (one-beat)
                12'd18: toneL = `he;     12'd19: toneL = `he;
                12'd20: toneL = `he;     12'd21: toneL = `he;
                12'd22: toneL = `he;     12'd23: toneL = `he;
                12'd24: toneL = `he;     12'd25: toneL = `he;
                12'd26: toneL = `he;     12'd27: toneL = `he;
                12'd28: toneL = `he;     12'd29: toneL = `he;
                12'd30: toneL = `he;     12'd31: toneL = `he;

                12'd32: toneL = `hf;     12'd33: toneL = `hf; // HF (half-beat)
                12'd34: toneL = `hf;     12'd35: toneL = `hf;
                12'd36: toneL = `hf;     12'd37: toneL = `hf;
                12'd38: toneL = `hf;     12'd39: toneL = `hf;
                12'd40: toneL = `hd;     12'd41: toneL = `hd; // HD (half-beat)
                12'd42: toneL = `hd;     12'd43: toneL = `hd;
                12'd44: toneL = `hd;     12'd45: toneL = `hd;
                12'd46: toneL = `hd;     12'd47: toneL = `sil; // (Short break for repetitive notes: high D)

                12'd48: toneL = `hd;     12'd49: toneL = `hd; // HD (one-beat)
                12'd50: toneL = `hd;     12'd51: toneL = `hd;
                12'd52: toneL = `hd;     12'd53: toneL = `hd;
                12'd54: toneL = `hd;     12'd55: toneL = `hd;
                12'd56: toneL = `hd;     12'd57: toneL = `hd;
                12'd58: toneL = `hd;     12'd59: toneL = `hd;
                12'd60: toneL = `hd;     12'd61: toneL = `hd;
                12'd62: toneL = `hd;     12'd63: toneL = `hd;

                // --- Measure 2 ---
                12'd64: toneL = `hc;     12'd65: toneL = `hc; // HC (half-beat)
                12'd66: toneL = `hc;     12'd67: toneL = `hc;
                12'd68: toneL = `hc;     12'd69: toneL = `hc;
                12'd70: toneL = `hc;     12'd71: toneL = `hc;
                12'd72: toneL = `hd;     12'd73: toneL = `hd; // HD (half-beat)
                12'd74: toneL = `hd;     12'd75: toneL = `hd;
                12'd76: toneL = `hd;     12'd77: toneL = `hd;
                12'd78: toneL = `hd;     12'd79: toneL = `hd;

                12'd80: toneL = `he;     12'd81: toneL = `he; // HE (half-beat)
                12'd82: toneL = `he;     12'd83: toneL = `he;
                12'd84: toneL = `he;     12'd85: toneL = `he;
                12'd86: toneL = `he;     12'd87: toneL = `he;
                12'd88: toneL = `hf;     12'd89: toneL = `hf; // HF (half-beat)
                12'd90: toneL = `hf;     12'd91: toneL = `hf;
                12'd92: toneL = `hf;     12'd93: toneL = `hf;
                12'd94: toneL = `hf;     12'd95: toneL = `hf;

                12'd96: toneL = `hg;     12'd97: toneL = `hg; // HG (half-beat)
                12'd98: toneL = `hg;     12'd99: toneL = `hg;
                12'd100: toneL = `hg;    12'd101: toneL = `hg;
                12'd102: toneL = `hg;    12'd103: toneL = `sil; // (Short break for repetitive notes: high D)
                12'd104: toneL = `hg;    12'd105: toneL = `hg; // HG (half-beat)
                12'd106: toneL = `hg;    12'd107: toneL = `hg;
                12'd108: toneL = `hg;    12'd109: toneL = `hg;
                12'd110: toneL = `hg;    12'd111: toneL = `sil; // (Short break for repetitive notes: high D)

                12'd112: toneL = `hg;    12'd113: toneL = `hg; // HG (one-beat)
                12'd114: toneL = `hg;    12'd115: toneL = `hg;
                12'd116: toneL = `hg;    12'd117: toneL = `hg;
                12'd118: toneL = `hg;    12'd119: toneL = `hg;
                12'd120: toneL = `hg;    12'd121: toneL = `hg;
                12'd122: toneL = `hg;    12'd123: toneL = `hg;
                12'd124: toneL = `hg;    12'd125: toneL = `hg;
                12'd126: toneL = `hg;    12'd127: toneL = `sil;

                12'd128: toneL = `hg;      12'd129: toneL = `hg; // HG (half-beat)
                12'd130: toneL = `hg;      12'd131: toneL = `hg;
                12'd132: toneL = `hg;      12'd133: toneL = `hg;
                12'd134: toneL = `hg;      12'd135: toneL = `hg;
                12'd136: toneL = `he;      12'd137: toneL = `he; // HE (half-beat)
                12'd138: toneL = `he;      12'd139: toneL = `he;
                12'd140: toneL = `he;      12'd141: toneL = `he;
                12'd142: toneL = `he;      12'd143: toneL = `sil; // (Short break for repetitive notes: high E)

                12'd144: toneL = `he;     12'd145: toneL = `he; // HE (one-beat)
                12'd146: toneL = `he;     12'd147: toneL = `he;
                12'd148: toneL = `he;     12'd149: toneL = `he;
                12'd150: toneL = `he;     12'd151: toneL = `he;
                12'd152: toneL = `he;     12'd153: toneL = `he;
                12'd154: toneL = `he;     12'd156: toneL = `he;
                12'd157: toneL = `he;     12'd158: toneL = `he;
                12'd159: toneL = `he;     12'd160: toneL = `he;

                12'd161: toneL = `hf;     12'd162: toneL = `hf; // HF (half-beat)
                12'd163: toneL = `hf;     12'd164: toneL = `hf;
                12'd165: toneL = `hf;     12'd166: toneL = `hf;
                12'd167: toneL = `hf;     12'd168: toneL = `hf;
                12'd169: toneL = `hd;     12'd170: toneL = `hd; // HD (half-beat)
                12'd171: toneL = `hd;     12'd172: toneL = `hd;
                12'd173: toneL = `hd;     12'd174: toneL = `hd;
                12'd175: toneL = `hd;     12'd176: toneL = `sil; // (Short break for repetitive notes: high D)

                12'd177: toneL = `hd;     12'd178: toneL = `hd; // HD (one-beat)
                12'd179: toneL = `hd;     12'd180: toneL = `hd;
                12'd181: toneL = `hd;     12'd182: toneL = `hd;
                12'd183: toneL = `hd;     12'd184: toneL = `hd;
                12'd185: toneL = `hd;     12'd186: toneL = `hd;
                12'd187: toneL = `hd;     12'd188: toneL = `hd;
                12'd189: toneL = `hd;     12'd190: toneL = `hd;
                12'd191: toneL = `hd;     12'd192: toneL = `hd;

                12'd193: toneL = `hc;     12'd194: toneL = `hc; // HC (half-beat)
                12'd195: toneL = `hc;     12'd196: toneL = `hc;
                12'd197: toneL = `hc;     12'd198: toneL = `hc;
                12'd199: toneL = `hc;     12'd200: toneL = `hc;
                12'd201: toneL = `he;     12'd202: toneL = `he; // HE (half-beat)
                12'd203: toneL = `he;     12'd204: toneL = `he;
                12'd205: toneL = `he;     12'd206: toneL = `he;
                12'd207: toneL = `he;     12'd208: toneL = `he;

                12'd209: toneL = `hg;     12'd210: toneL = `hg; // HG (half-beat)
                12'd211: toneL = `hg;     12'd212: toneL = `hg;
                12'd213: toneL = `hg;    12'd214: toneL = `hg;
                12'd215: toneL = `hg;    12'd216: toneL = `sil; // (Short break for repetitive notes: high D)
                12'd217: toneL = `hg;    12'd218: toneL = `hg; // HG (half-beat)
                12'd219: toneL = `hg;    12'd220: toneL = `hg;
                12'd221: toneL = `hg;    12'd222: toneL = `hg;
                12'd223: toneL = `hg;    12'd224: toneL = `hg; // (Short break for repetitive notes: high D)

                12'd225: toneL = `he;    12'd226: toneL = `he; // HE (half-beat)
                12'd227: toneL = `he;    12'd228: toneL = `he;
                12'd229: toneL = `he;    12'd230: toneL = `he;
                12'd231: toneL = `he;    12'd232: toneL = `sil;
                12'd233: toneL = `he;    12'd234: toneL = `he; // HE (half-beat)
                12'd235: toneL = `he;    12'd236: toneL = `he;
                12'd237: toneL = `he;    12'd238: toneL = `he;
                12'd239: toneL = `he;    12'd240: toneL = `sil;

                12'd241: toneL = `he;     12'd242: toneL = `he; // HE (one-beat)
                12'd243: toneL = `he;     12'd244: toneL = `he;
                12'd245: toneL = `he;     12'd246: toneL = `he;
                12'd247: toneL = `he;     12'd248: toneL = `he;
                12'd249: toneL = `he;     12'd250: toneL = `he;
                12'd251: toneL = `he;     12'd252: toneL = `he;
                12'd253: toneL = `he;     12'd254: toneL = `he;
                12'd255: toneL = `he;     12'd256: toneL = `he;

                12'd257: toneL = `hd;     12'd258: toneL = `hd; // HD (half-beat)
                12'd259: toneL = `hd;     12'd260: toneL = `hd;
                12'd261: toneL = `hd;     12'd262: toneL = `hd;
                12'd263: toneL = `hd;     12'd264: toneL = `sil;
                12'd265: toneL = `hd;     12'd266: toneL = `hd; // HD (half-beat)
                12'd267: toneL = `hd;     12'd268: toneL = `hd;
                12'd269: toneL = `hd;     12'd270: toneL = `hd;
                12'd271: toneL = `hd;     12'd272: toneL = `sil;

                12'd273: toneL = `hd;     12'd274: toneL = `hd; // HD (half-beat)
                12'd275: toneL = `hd;     12'd276: toneL = `hd;
                12'd277: toneL = `hd;     12'd278: toneL = `hd;
                12'd279: toneL = `hd;     12'd280: toneL = `sil;
                12'd281: toneL = `hd;     12'd282: toneL = `hd; // HD (half-beat)
                12'd283: toneL = `hd;     12'd284: toneL = `hd;
                12'd285: toneL = `hd;     12'd286: toneL = `hd;
                12'd287: toneL = `hd;     12'd288: toneL = `sil;

                12'd289: toneL = `hd;     12'd290: toneL = `hd; // HD (half-beat)
                12'd291: toneL = `hd;     12'd292: toneL = `hd;
                12'd293: toneL = `hd;    12'd294: toneL = `hd;
                12'd295: toneL = `hd;    12'd296: toneL = `hd; // (Short break for repetitive notes: high D)
                12'd297: toneL = `he;    12'd298: toneL = `he; // HE (half-beat)
                12'd299: toneL = `he;    12'd300: toneL = `he;
                12'd301: toneL = `he;    12'd302: toneL = `he;
                12'd303: toneL = `he;    12'd304: toneL = `he; // (Short break for repetitive notes: high D)

                12'd305: toneL = `hf;    12'd306: toneL = `hf; // HF (one-beat)
                12'd307: toneL = `hf;    12'd308: toneL = `hf;
                12'd309: toneL = `hf;    12'd310: toneL = `hf;
                12'd311: toneL = `hf;    12'd312: toneL = `hf;
                12'd313: toneL = `hf;    12'd314: toneL = `hf;
                12'd315: toneL = `hf;    12'd316: toneL = `hf;
                12'd317: toneL = `hf;    12'd318: toneL = `hf;
                12'd319: toneL = `hf;    12'd320: toneL = `hf;

                12'd321: toneL = `he;     12'd322: toneL = `he; // HE (half-beat)
                12'd323: toneL = `he;     12'd324: toneL = `he;
                12'd325: toneL = `he;     12'd326: toneL = `he;
                12'd327: toneL = `he;     12'd328: toneL = `sil;
                12'd329: toneL = `he;     12'd330: toneL = `he; // HE (half-beat)
                12'd331: toneL = `he;     12'd332: toneL = `he;
                12'd333: toneL = `he;     12'd334: toneL = `he;
                12'd335: toneL = `he;     12'd336: toneL = `sil;

                12'd337: toneL = `he;     12'd338: toneL = `he; // He (half-beat)
                12'd339: toneL = `he;     12'd340: toneL = `he;
                12'd341: toneL = `he;     12'd342: toneL = `he;
                12'd343: toneL = `he;     12'd344: toneL = `sil;
                12'd345: toneL = `he;     12'd346: toneL = `he; // He (half-beat)
                12'd347: toneL = `he;     12'd348: toneL = `he;
                12'd349: toneL = `he;     12'd350: toneL = `he;
                12'd351: toneL = `he;     12'd352: toneL = `sil;

                12'd353: toneL = `he;     12'd354: toneL = `he; // He (half-beat)
                12'd355: toneL = `he;     12'd356: toneL = `he;
                12'd357: toneL = `he;     12'd358: toneL = `he;
                12'd359: toneL = `he;     12'd360: toneL = `he;
                12'd361: toneL = `hf;     12'd362: toneL = `hf; // Hf (half-beat)
                12'd363: toneL = `hf;     12'd364: toneL = `hf;
                12'd365: toneL = `hf;     12'd366: toneL = `hf;
                12'd367: toneL = `hf;     12'd368: toneL = `hf;

                12'd369: toneL = `hg;     12'd370: toneL = `hg; // HG (one-beat)
                12'd371: toneL = `hg;     12'd372: toneL = `hg;
                12'd373: toneL = `hg;    12'd374: toneL = `hg;
                12'd375: toneL = `hg;    12'd376: toneL = `hg; 
                12'd377: toneL = `hg;    12'd378: toneL = `hg; 
                12'd379: toneL = `hg;    12'd380: toneL = `hg;
                12'd381: toneL = `hg;    12'd382: toneL = `hg;
                12'd383: toneL = `hg;    12'd384: toneL = `sil; 

                12'd385: toneL = `hg;    12'd386: toneL = `hg; // Hg (half-beat)
                12'd387: toneL = `hg;    12'd388: toneL = `hg;
                12'd389: toneL = `hg;    12'd390: toneL = `hg;
                12'd391: toneL = `hg;    12'd392: toneL = `hg;
                12'd393: toneL = `he;    12'd394: toneL = `he; // HE (half-beat)
                12'd395: toneL = `he;    12'd396: toneL = `he;
                12'd397: toneL = `he;    12'd398: toneL = `he;
                12'd399: toneL = `he;    12'd400: toneL = `sil;

                12'd401: toneL = `he;     12'd402: toneL = `he; // HE (one-beat)
                12'd403: toneL = `he;     12'd404: toneL = `he;
                12'd405: toneL = `he;     12'd406: toneL = `he;
                12'd407: toneL = `he;     12'd408: toneL = `he;
                12'd409: toneL = `he;     12'd410: toneL = `he;
                12'd411: toneL = `he;     12'd412: toneL = `he;
                12'd413: toneL = `he;     12'd414: toneL = `he;
                12'd415: toneL = `he;     12'd416: toneL = `he;

                12'd417: toneL = `hf;     12'd418: toneL = `hf; // Hf (half-beat)
                12'd419: toneL = `hf;     12'd420: toneL = `hf;
                12'd421: toneL = `hf;     12'd422: toneL = `hf;
                12'd423: toneL = `hf;     12'd424: toneL = `hf;
                12'd425: toneL = `hd;     12'd426: toneL = `hd; // HD (half-beat)
                12'd427: toneL = `hd;     12'd428: toneL = `hd;
                12'd429: toneL = `hd;     12'd430: toneL = `hd;
                12'd431: toneL = `hd;     12'd432: toneL = `sil;

                12'd433: toneL = `hd;     12'd434: toneL = `hd; // HD (half-beat)
                12'd435: toneL = `hd;     12'd436: toneL = `hd;
                12'd437: toneL = `hd;     12'd438: toneL = `hd;
                12'd439: toneL = `hd;     12'd440: toneL = `hd;
                12'd441: toneL = `hd;     12'd442: toneL = `hd; // HD (half-beat)
                12'd443: toneL = `hd;     12'd444: toneL = `hd;
                12'd445: toneL = `hd;     12'd446: toneL = `hd;
                12'd447: toneL = `hd;     12'd448: toneL = `hd;

                12'd449: toneL = `hc;     12'd450: toneL = `hc; // Hc (half-beat)
                12'd451: toneL = `hc;     12'd452: toneL = `hc;
                12'd453: toneL = `hc;    12'd454: toneL = `hc;
                12'd455: toneL = `hc;    12'd456: toneL = `hc; 
                12'd457: toneL = `he;    12'd458: toneL = `he; // HE (half-beat)
                12'd459: toneL = `he;    12'd460: toneL = `he;
                12'd461: toneL = `he;    12'd462: toneL = `he;
                12'd463: toneL = `he;    12'd464: toneL = `he; 

                12'd465: toneL = `hg;    12'd466: toneL = `hg; // HG (half-beat)
                12'd467: toneL = `hg;    12'd468: toneL = `hg;
                12'd469: toneL = `hg;    12'd470: toneL = `hg;
                12'd471: toneL = `hg;    12'd472: toneL = `sil;
                12'd473: toneL = `hg;    12'd474: toneL = `hg; // HG (half-beat)
                12'd475: toneL = `hg;    12'd476: toneL = `hg;
                12'd477: toneL = `hg;    12'd478: toneL = `hg;
                12'd479: toneL = `hg;    12'd480: toneL = `hg;

                12'd481: toneL = `hc;     12'd482: toneL = `hc; // Hc (two-beat)
                12'd483: toneL = `hc;     12'd484: toneL = `hc;
                12'd485: toneL = `hc;     12'd486: toneL = `hc;
                12'd487: toneL = `hc;     12'd488: toneL = `hc;
                12'd489: toneL = `hc;     12'd490: toneL = `hc; 
                12'd491: toneL = `hc;     12'd492: toneL = `hc;
                12'd493: toneL = `hc;     12'd494: toneL = `hc;
                12'd495: toneL = `hc;     12'd496: toneL = `hc;

                12'd497: toneL = `hc;     12'd498: toneL = `hc; 
                12'd499: toneL = `hc;     12'd500: toneL = `hc;
                12'd501: toneL = `hc;     12'd502: toneL = `hc;
                12'd503: toneL = `hc;     12'd504: toneL = `hc;
                12'd505: toneL = `hc;     12'd506: toneL = `hc; 
                12'd507: toneL = `hc;     12'd508: toneL = `hc;
                12'd509: toneL = `hc;     12'd510: toneL = `hc;
                12'd511: toneL = `hc;     12'd512: toneL = `hc;



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

module KeyboardDecoder(
	input wire rst,
	input wire clk,
	inout wire PS2_DATA,
	inout wire PS2_CLK,
	output reg [65:0] key_down,
	output wire [8:0] last_change,
	output reg key_valid
    );
    
    parameter [1:0] INIT			= 2'b00;
    parameter [1:0] WAIT_FOR_SIGNAL = 2'b01;
    parameter [1:0] GET_SIGNAL_DOWN = 2'b10;
    parameter [1:0] WAIT_RELEASE    = 2'b11;
    
	parameter [7:0] IS_INIT			= 8'hAA;
    parameter [7:0] IS_EXTEND		= 8'hE0;
    parameter [7:0] IS_BREAK		= 8'hF0;
    
    reg [9:0] key;		// key = {been_extend, been_break, key_in}
    reg [1:0] state;
    reg been_ready, been_extend, been_break;
    
    wire [7:0] key_in;
    wire is_extend;
    wire is_break;
    wire valid;
    wire err;
    
    wire [511:0] key_decode = 1 << last_change;
    assign last_change = {key[9], key[7:0]};
    
    KeyboardCtrl_0 inst (
		.key_in(key_in),
		.is_extend(is_extend),
		.is_break(is_break),
		.valid(valid),
		.err(err),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);
	
	onepulse op (
		.signal(been_ready),
		.clk(clk),
		.op(pulse_been_ready)
	);
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		state <= INIT;
    		been_ready  <= 1'b0;
    		been_extend <= 1'b0;
    		been_break  <= 1'b0;
    		key <= 10'b0_0_0000_0000;
    	end else begin
    		state <= state;
			been_ready  <= been_ready;
			been_extend <= (is_extend) ? 1'b1 : been_extend;
			been_break  <= (is_break ) ? 1'b1 : been_break;
			key <= key;
    		case (state)
    			INIT : begin
    					if (key_in == IS_INIT) begin
    						state <= WAIT_FOR_SIGNAL;
    						been_ready  <= 1'b0;
							been_extend <= 1'b0;
							been_break  <= 1'b0;
							key <= 10'b0_0_0000_0000;
    					end else begin
    						state <= INIT;
    					end
    				end
    			WAIT_FOR_SIGNAL : begin
    					if (valid == 0) begin
    						state <= WAIT_FOR_SIGNAL;
    						been_ready <= 1'b0;
    					end else begin
    						state <= GET_SIGNAL_DOWN;
    					end
    				end
    			GET_SIGNAL_DOWN : begin
						state <= WAIT_RELEASE;
						key <= {been_extend, been_break, key_in};
						been_ready  <= 1'b1;
    				end
    			WAIT_RELEASE : begin
    					if (valid == 1) begin
    						state <= WAIT_RELEASE;
    					end else begin
    						state <= WAIT_FOR_SIGNAL;
    						been_extend <= 1'b0;
    						been_break  <= 1'b0;
    					end
    				end
    			default : begin
    					state <= INIT;
						been_ready  <= 1'b0;
						been_extend <= 1'b0;
						been_break  <= 1'b0;
						key <= 10'b0_0_0000_0000;
    				end
    		endcase
    	end
    end
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		key_valid <= 1'b0;
    		key_down <= 511'b0;
    	end else if (key_decode[last_change] && pulse_been_ready) begin
    		key_valid <= 1'b1;
    		if (key[8] == 0) begin
    			key_down <= key_down | key_decode;
    		end else begin
    			key_down <= key_down & (~key_decode);
    		end
    	end else begin
    		key_valid <= 1'b0;
			key_down <= key_down;
    	end
    end

endmodule