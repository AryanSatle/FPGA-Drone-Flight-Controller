module drone_pid_sim (
    input  logic clk,
    input  logic rst_n,
    input  logic signed [31:0] setpoint,
    input  logic signed [31:0] feedback,
    input  logic signed [31:0] Kp,
    input  logic signed [31:0] Ki,
    input  logic signed [31:0] Kd,
    output logic signed [31:0] control_out,
    output logic signed [31:0] current_error
);

    logic signed [31:0] error;
    logic signed [31:0] error_prev;
    logic signed [31:0] integral;
    logic signed [31:0] derivative;

    assign current_error = error;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_prev  <= 0;
            integral    <= 0;
            control_out <= 0;
            error       <= 0;
        end else begin
            error = setpoint - feedback;
            integral <= integral + error;
            derivative = error - error_prev;
            control_out <= (Kp * error) + (Ki * integral) + (Kd * derivative);
            error_prev <= error;
        end
    end
endmodule
