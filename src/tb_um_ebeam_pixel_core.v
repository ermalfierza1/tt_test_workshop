`timescale 1ns/1ps

module tb_ebeam_pixel_core;
    reg clk = 0;
    always #10 clk = ~clk; // 50 MHz

    reg rst_n = 0;
    reg ena   = 1;

    reg  [7:0] ui_in;
    wire [7:0] uo_out;

    reg  [7:0] uio_in;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // DUT
    tt_um_ebeam_pixel_core dut(
        .clk(clk), .rst_n(rst_n), .ena(ena),
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe)
    );

    // Aliases
    wire cfg_miso = uio_out[5];
    reg  pix_valid, frame_start, csn, sclk, mosi;
    reg  [7:0] rd;
    localparam [7:0] TB_THRESH = 8'd128;
    integer pass_cnt;
    integer fail_cnt;
    reg [7:0] tb_prev_pix;
    reg       tb_prev_valid;
    reg [7:0] tb_exp_uo;

    localparam [7:0] SB_DEF_THRESH       = 8'd128;
    localparam [7:0] SB_DEF_CONTRAST_THR = 8'd12;
    localparam [7:0] SB_DEF_EDGE_THR     = 8'd12;
    localparam [2:0] SB_DEF_ALPHA_SHIFT  = 3'd3;

    reg  [7:0] sb_reg_thresh;
    reg  [7:0] sb_reg_contrast_thr;
    reg  [7:0] sb_reg_edge_thr;
    reg  [2:0] sb_reg_alpha_shift;
    reg  [7:0] sb_reg_mode;

    reg  [7:0] sb_pixel_q;
    reg  [7:0] sb_prev_pixel;
    reg  [7:0] sb_mean_q;

    reg  [7:0] sb_exp_uo;
    reg        sb_check_valid;
    reg  [7:0] sb_dbg_pixel_in;
    reg        sb_dbg_frame_start;

    // Map uio_in
    always @* begin
        uio_in = 8'h00;
        uio_in[0] = pix_valid;
        uio_in[1] = frame_start;
        uio_in[2] = csn;
        uio_in[3] = sclk;
        uio_in[4] = mosi;
    end

    // SPI helpers (CPOL=0,CPHA=0)
    task spi_byte_inout(input [7:0] tx, output [7:0] rx);
        integer i;
        reg [7:0] t, r;
        begin
            t = tx; r = 8'h00;
            for (i=7; i>=0; i=i-1) begin
                mosi = t[i];
                #5; sclk = 1; #5; // rising edge: sample MOSI, shift out MISO
                r[i] = cfg_miso;
                sclk = 0; #10;
            end
            rx = r;
        end
    endtask

    task spi_write(input [2:0] addr, input [7:0] data);
        reg [7:0] r;
        begin
            csn = 0; #20;
            spi_byte_inout({1'b1, addr, 4'b0000}, r);
            spi_byte_inout(data, r);
            csn = 1; #100;

            case (addr)
                3'h0: sb_reg_thresh       = data;
                3'h1: sb_reg_contrast_thr = data;
                3'h2: sb_reg_edge_thr     = data;
                3'h3: sb_reg_alpha_shift  = data[2:0];
                3'h4: sb_reg_mode         = data;
                default: ;
            endcase
        end
    endtask

    task spi_read(input [2:0] addr, output [7:0] data);
        reg [7:0] r;
        begin
            csn = 0; #20;
            spi_byte_inout({1'b0, addr, 4'b0000}, r);
            spi_byte_inout(8'h00, data);
            csn = 1; #100;
        end
    endtask

    task check8(input [7:0] got, input [7:0] exp);
        begin
            if (got !== exp) begin
                $display("CHECK_FAIL t=%0t got=%b exp=%b", $time, got, exp);
                fail_cnt = fail_cnt + 1;
            end else begin
                pass_cnt = pass_cnt + 1;
            end
        end
    endtask

    task thresh_step(input [7:0] pix, input do_check_prev);
        begin
            ui_in = pix;
            @(posedge clk);
            #1;
        end
    endtask

    task passthrough_step(input [7:0] pix);
        begin
            ui_in = pix;
            @(posedge clk);
            #1;
        end
    endtask

    // Reference-model scoreboard (pixel datapath + output muxing)
    always @(posedge clk or negedge rst_n) begin
        reg signed [8:0] diff_pm;
        reg signed [8:0] diff_edge;
        reg [7:0] abs_diff;
        reg [7:0] edge_mag;
        reg bright_defect;
        reg dark_defect;
        reg edge_strong;
        reg defect_any;
        reg [7:0] mag_max;
        reg [3:0] mag_nib;
        reg thresh_bit;
        reg signed [8:0] mean_delta;
        reg [7:0] mean_next;
        reg [7:0] exp;

        if (!rst_n) begin
            sb_reg_thresh       <= SB_DEF_THRESH;
            sb_reg_contrast_thr <= SB_DEF_CONTRAST_THR;
            sb_reg_edge_thr     <= SB_DEF_EDGE_THR;
            sb_reg_alpha_shift  <= SB_DEF_ALPHA_SHIFT;
            sb_reg_mode         <= 8'h00;

            sb_pixel_q     <= 8'd0;
            sb_prev_pixel  <= 8'd0;
            sb_mean_q      <= 8'd0;

            sb_exp_uo      <= 8'd0;
            sb_check_valid <= 1'b0;
            sb_dbg_pixel_in <= 8'd0;
            sb_dbg_frame_start <= 1'b0;
        end else begin
            sb_check_valid <= 1'b0;

            if (ena && pix_valid) begin
                // Compute using *previous* registered state (matches DUT NBA semantics)
                diff_pm = {1'b0, sb_pixel_q} - {1'b0, sb_mean_q};
                abs_diff = diff_pm[8] ? (~diff_pm[7:0] + 8'd1) : diff_pm[7:0];

                diff_edge = {1'b0, sb_pixel_q} - {1'b0, sb_prev_pixel};
                edge_mag = diff_edge[8] ? (~diff_edge[7:0] + 8'd1) : diff_edge[7:0];

                bright_defect = (diff_pm[8] == 1'b0) && (abs_diff >= sb_reg_contrast_thr);
                dark_defect   = (diff_pm[8] == 1'b1) && (abs_diff >= sb_reg_contrast_thr);
                edge_strong   = (edge_mag >= sb_reg_edge_thr);
                defect_any    = bright_defect | dark_defect | edge_strong;

                mag_max = (abs_diff >= edge_mag) ? abs_diff : edge_mag;
                mag_nib = mag_max[3:0];

                thresh_bit = (sb_pixel_q >= sb_reg_thresh);

                if (sb_reg_mode[0]) begin
                    exp = ui_in;
                end else if (sb_reg_mode[1]) begin
                    exp = {thresh_bit, 7'd0};
                end else begin
                    exp = {defect_any, bright_defect, dark_defect, edge_strong, mag_nib};
                end

                sb_exp_uo <= exp;
                sb_check_valid <= 1'b1;
                sb_dbg_pixel_in <= ui_in;
                sb_dbg_frame_start <= frame_start;

                // Update reference model state (matches DUT)
                mean_delta = diff_pm >>> sb_reg_alpha_shift;
                mean_next = frame_start ? ui_in : (sb_mean_q + mean_delta[7:0]);
                sb_mean_q <= mean_next;

                if (frame_start)
                    sb_prev_pixel <= ui_in;
                else
                    sb_prev_pixel <= sb_pixel_q;

                sb_pixel_q <= ui_in;
            end
        end
    end

    always @(posedge clk) begin
        #1;
        if (sb_check_valid) begin
            if (uo_out !== sb_exp_uo) begin
                $display("SB_MISM t=%0t mode=%b pix_in=%0d fs=%0d pix_q=%0d mean=%0d prev=%0d got=%b exp=%b",
                    $time, sb_reg_mode, sb_dbg_pixel_in, sb_dbg_frame_start,
                    sb_pixel_q, sb_mean_q, sb_prev_pixel, uo_out, sb_exp_uo);
            end
            check8(uo_out, sb_exp_uo);
        end
    end

    // Stimulus
    integer k;
    initial begin
        // Init
        ui_in = 8'd0; pix_valid=0; frame_start=0;
        csn=1; sclk=0; mosi=0;
        pass_cnt = 0;
        fail_cnt = 0;

        // Reset
        #100; rst_n = 1;

        // Configure thresholds: contrast=20, edge=20, alpha_shift=3
        spi_write(3'h1, 8'd20);
        spi_write(3'h2, 8'd20);
        spi_write(3'h3, 8'd3);
        // mode=0 (normal flags)
        spi_write(3'h4, 8'h00);
        repeat (6) @(posedge clk);

        // Read back mean, initially ~0
        spi_read(3'h6, rd);
        $display("Mean initial = %0d", rd);
        check8(rd, 8'd0);

        // Send a line: background 80, abrupt step to 160 at sample 40, a bright blip 230 at 60
        frame_start = 1; pix_valid=1; ui_in=8'd80; #20; frame_start=0;
        for (k=1; k<100; k=k+1) begin
            if (k==40) ui_in = 8'd160;
            else if (k==60) ui_in = 8'd230;
            else if (k==61) ui_in = 8'd160;
            pix_valid=1; #20;
            $display("t=%0t pix=%0d out=%b flags={any:%0d bright:%0d dark:%0d edge:%0d} mag=%0d",
                $time, ui_in, uo_out,
                uo_out[7], uo_out[6], uo_out[5], uo_out[4], uo_out[3:0]);
            if (uo_out[7] !== (uo_out[6] | uo_out[5] | uo_out[4])) begin
                $display("INVAR_FAIL t=%0t out=%b (any != bright|dark|edge)", $time, uo_out);
                fail_cnt = fail_cnt + 1;
            end else begin
                pass_cnt = pass_cnt + 1;
            end
        end
        pix_valid=0; #200;

        // Switch to threshold mode: thresh=128
        spi_write(3'h0, 8'd128);
        spi_write(3'h4, 8'b0000_0010);
        repeat (6) @(posedge clk);

        spi_read(3'h0, rd);
        check8(rd, 8'd128);
        spi_read(3'h4, rd);
        check8(rd, 8'b0000_0010);

        // Push a few pixels to see threshold bit
        repeat (4) @(posedge clk);

        pix_valid = 1'b1;
        frame_start = 1'b1;
        thresh_step(8'd0, 1'b0);
        frame_start = 1'b0;
        thresh_step(8'd1, 1'b1);
        thresh_step(8'd127, 1'b1);
        thresh_step(8'd128, 1'b1);
        thresh_step(8'd129, 1'b1);
        thresh_step(8'd255, 1'b1);
        thresh_step(8'd128, 1'b1);
        thresh_step(8'd127, 1'b1);
        thresh_step(8'd200, 1'b1);
        thresh_step(8'd0, 1'b1);

        pix_valid = 1'b0;
        repeat (2) @(posedge clk);

        // Debug passthrough mode
        spi_write(3'h4, 8'b0000_0001);
        repeat (6) @(posedge clk);
        spi_read(3'h4, rd);
        check8(rd, 8'b0000_0001);
        repeat (4) @(posedge clk);

        pix_valid = 1'b1;
        frame_start = 1'b1;
        passthrough_step(8'd0);
        frame_start = 1'b0;
        passthrough_step(8'd5);
        passthrough_step(8'd80);
        passthrough_step(8'd128);
        passthrough_step(8'd200);
        passthrough_step(8'd255);
        pix_valid = 1'b0; #100;

        $display("CHECK_SUMMARY pass=%0d fail=%0d", pass_cnt, fail_cnt);
        if (fail_cnt != 0) $fatal;

        $finish;
    end
endmodule