module tt_um_alu_bns (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    localparam SEL_A_LOW  = 3'b000;
    localparam SEL_A_HIGH = 3'b001;
    localparam SEL_B_LOW  = 3'b010;
    localparam SEL_B_HIGH = 3'b011;
    localparam SEL_CTRL   = 3'b100;
    localparam SEL_START  = 3'b101;

    wire [7:0] data_in  = ui_in;
    wire [2:0] reg_sel  = uio_in[7:5];
    wire       ld       = uio_in[4];
    wire [1:0] read_sel = uio_in[1:0];

    reg [15:0] A_reg, B_reg;
    reg        Cin_reg;
    reg [2:0]  opcode_reg;
    reg [1:0]  sub_op_reg;

    reg [31:0] Result_reg;
    reg        Cout_reg;
    reg        Valid_reg;

    wire [31:0] core_result;
    wire        core_cout;
    wire        core_valid;

    alu_core u_alu (
        .A      (A_reg),
        .B      (B_reg),
        .Cin    (Cin_reg),
        .opcode (opcode_reg),
        .sub_op (sub_op_reg),
        .Result (core_result),
        .Cout   (core_cout),
        .Valid  (core_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            A_reg      <= 16'b0;
            B_reg      <= 16'b0;
            Cin_reg    <= 1'b0;
            opcode_reg <= 3'b0;
            sub_op_reg <= 2'b0;
            Result_reg <= 32'b0;
            Cout_reg   <= 1'b0;
            Valid_reg  <= 1'b0;
        end else if (ena && ld) begin
            case (reg_sel)
                SEL_A_LOW:  A_reg[7:0]  <= data_in;
                SEL_A_HIGH: A_reg[15:8] <= data_in;
                SEL_B_LOW:  B_reg[7:0]  <= data_in;
                SEL_B_HIGH: B_reg[15:8] <= data_in;
                SEL_CTRL: begin
                    opcode_reg <= data_in[2:0];
                    sub_op_reg <= data_in[4:3];
                    Cin_reg    <= data_in[5];
                end
                SEL_START: begin
                    Result_reg <= core_result;
                    Cout_reg   <= core_cout;
                    Valid_reg  <= core_valid;
                end
                default: ; // reserved, no-op
            endcase
        end
    end

    reg [7:0] result_byte;
    always @(*) begin
        case (read_sel)
            2'b00:   result_byte = Result_reg[7:0];
            2'b01:   result_byte = Result_reg[15:8];
            2'b10:   result_byte = Result_reg[23:16];
            default: result_byte = Result_reg[31:24];
        endcase
    end

    assign uo_out  = result_byte;
    assign uio_out = {4'b0, Valid_reg, Cout_reg, 2'b0};
    assign uio_oe  = 8'b0000_1100; // bits [3:2] driven out (Valid, Cout); rest are inputs

endmodule

// the design of alu core

module alu_core (
    input  wire [15:0] A,
    input  wire [15:0] B,
    input  wire        Cin,
    input  wire [2:0]  opcode,
    input  wire [1:0]  sub_op,
    output reg  [31:0] Result,
    output reg          Cout,
    output reg          Valid
);

    wire [15:0] cla_sum;
    wire        cla_cout;
    wire [31:0] wallace_mult_prod;
    wire [15:0] logic_result;
    wire [15:0] shift_pop_result;
    wire [15:0] comp_result;

    cla_adder_16bit u_cla (
        .A(A), .B(B), .Cin(Cin), .sum(cla_sum), .Cout(cla_cout)
    );

    wallace_tree_mult_16bit u_wallace (
        .A(A), .B(B), .product(wallace_mult_prod)
    );

    logic_unit #(.M(16)) u_logic (
        .A(A), .B(B), .op_sel(sub_op), .result(logic_result)
    );

    wire [1:0] sp_op = (opcode == 3'b111) ? 2'b10 : sub_op;
    shift_popcount_unit u_shift_pop (
        .opcode(sp_op), .A(A), .B(B), .result(shift_pop_result)
    );

    comparator_unit #(.N(16)) u_comp (
        .A(A), .B(B), .comp_out(comp_result)
    );

    always @(*) begin
        Result = 32'b0;
        Cout   = 1'b0;
        Valid  = 1'b1;
        case (opcode)
            3'b000: begin Result = {16'b0, cla_sum};   Cout = cla_cout; end
            3'b001: begin Result = {16'b0, cla_sum};   Cout = cla_cout; end
            3'b001: begin Result = {16'b0, cla_sum};   Cout = cla_cout; end
            3'b010: begin Result = wallace_mult_prod;                   end
            3'b011: begin Result = wallace_mult_prod;                   end
            3'b100: begin Result = {16'b0, logic_result};               end
            3'b101: begin Result = {16'b0, shift_pop_result};           end
            3'b110: begin Result = {16'b0, comp_result};                end
            3'b111: begin Result = {16'b0, shift_pop_result};           end
            default: Valid = 1'b0;
        endcase
    end

endmodule

// the design of carry adder

module cla_adder_16bit ( A, B, Cin, sum, Cout );

    parameter m = 16 ;

    input  [m-1:0] A;
    input  [m-1:0] B;
    input  Cin;
    output [m-1:0] sum;
    output  Cout;

    wire C4, C8, C12;

    cla_4bit u0 (
        .A    (A[3:0]),
        .B    (B[3:0]),
        .Cin  (Cin),
        .sum  (sum[3:0]),
        .Cout (C4)
    );

    cla_4bit u1 (
        .A    (A[7:4]),
        .B    (B[7:4]),
        .Cin  (C4),
        .sum  (sum[7:4]),
        .Cout (C8)
    );

    cla_4bit u2 (
        .A    (A[11:8]),
        .B    (B[11:8]),
        .Cin  (C8),
        .sum  (sum[11:8]),
        .Cout (C12)
    );

    cla_4bit u3 (
        .A    (A[15:12]),
        .B    (B[15:12]),
        .Cin  (C12),
        .sum  (sum[15:12]),
        .Cout (Cout)
    );

endmodule

// the design of cla_4bit

module cla_4bit ( A, B, Cin, sum, Cout );

    parameter n = 4 ;

    input [n-1:0] A , B;
    input Cin;
    output [n-1:0] sum;
    output Cout;

    wire g0, g1, g2, g3;
    wire p0, p1, p2, p3;

    assign g0 = A[0] & B[0];
    assign g1 = A[1] & B[1];
    assign g2 = A[2] & B[2];
    assign g3 = A[3] & B[3];

    assign p0 = A[0] ^ B[0];
    assign p1 = A[1] ^ B[1];
    assign p2 = A[2] ^ B[2];
    assign p3 = A[3] ^ B[3];

    wire C1, C2, C3;

    assign C1 = g0 | (p0 & Cin);

    assign C2 = g1 | (p1 & g0) | (p1 & p0 & Cin);

    assign C3 = g2 | (p2 & g1) | (p2 & p1 & g0) | (p2 & p1 & p0 & Cin);

    assign Cout = g3
                | (p3 & g2)
                | (p3 & p2 & g1)
                | (p3 & p2 & p1 & g0)
                | (p3 & p2 & p1 & p0 & Cin);

    assign sum[0] = p0 ^ Cin;
    assign sum[1] = p1 ^ C1;
    assign sum[2] = p2 ^ C2;
    assign sum[3] = p3 ^ C3;

endmodule

// the design of Wallace_tree_mult

module wallace_tree_mult_16bit ( A, B, product );

    parameter n = 16 ;
    parameter m = 32 ;

    input  [n-1:0] A;
    input  [n-1:0] B;
    output [m-1:0] product;

    reg [n-1:0] pp [0:n-1];
    reg [m-1:0] pp_ext [0:n-1];

    integer i, j;

    always @(*) begin
        for (i = 0; i < 16; i = i + 1) begin
            for (j = 0; j < 16; j = j + 1) begin
                pp[i][j] = A[j] & B[i];
            end
        end
    end

    always @(*) begin
        for (i = 0; i < 16; i = i + 1) begin
            pp_ext[i] = {{(16){1'b0}}, pp[i]} << i;
        end
    end

    assign product = pp_ext[0]  + pp_ext[1]  + pp_ext[2]  + pp_ext[3]
                   + pp_ext[4]  + pp_ext[5]  + pp_ext[6]  + pp_ext[7]
                   + pp_ext[8]  + pp_ext[9]  + pp_ext[10] + pp_ext[11]
                   + pp_ext[12] + pp_ext[13] + pp_ext[14] + pp_ext[15];

endmodule


// the design of Comparator

module comparator_unit #(parameter N = 16) (
    input  wire [N-1:0] A,
    input  wire [N-1:0] B,
    output reg  [N-1:0] comp_out
);
    always @(*) begin
        if (A == B)
            comp_out = {N{1'b1}};
        else if (A > B)
            comp_out = {{(N-1){1'b0}}, 1'b1};
        else
            comp_out = {N{1'b0}};
    end
endmodule


// the design of logic unit

module logic_unit #(parameter M = 16) (
    input  wire [M-1:0] A,
    input  wire [M-1:0] B,
    input  wire [1:0]   op_sel,
    output reg  [M-1:0] result
);
    always @(*) begin
        case (op_sel)
            2'b00:   result = A & B;
            2'b01:   result = A | B;
            2'b10:   result = A ^ B;
            default: result = {M{1'b0}};
        endcase
    end
endmodule


// the design of shift popcount

module shift_popcount_unit(
    input  wire [1:0]  opcode,
    input  wire [15:0] A,
    input  wire [15:0] B,
    output reg  [15:0] result
);

    wire [4:0]  popcount;
    wire [15:0] popcount_ext;

    assign popcount = A[0]+A[1]+A[2]+A[3]+A[4]+A[5]+A[6]+A[7]
                     + A[8]+A[9]+A[10]+A[11]+A[12]+A[13]+A[14]+A[15];
    assign popcount_ext = {11'b0, popcount};

    wire [15:0] shift_left;
    wire [15:0] shift_right;

    assign shift_left  = A << B[3:0];
    assign shift_right = A >> B[3:0];

    always @(*) begin
        case (opcode)
            2'b00:   result = shift_left;
            2'b01:   result = shift_right;
            2'b10:   result = popcount_ext;
            default: result = 16'b0;
        endcase
    end

endmodule


// the design of half adder , full adder , fall adder 1_bit

module half_adder (
    input  A, B,
    output sum, Cout
);
    assign sum  = A ^ B;
    assign Cout = A & B;
endmodule

module full_adder (
    input  A, B, Cin,
    output sum, Cout
);
    assign sum  = A ^ B ^ Cin;
    assign Cout = (A & B) | (B & Cin) | (A & Cin);
endmodule

module fa_1bit (
    input  A,
    input  B,
    input  Cin,
    output sum,
    output Cout
);
    assign sum  = A ^ B ^ Cin;
    assign Cout = (A & B) | (B & Cin) | (A & Cin);
endmodule
