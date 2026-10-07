module drone_pid_sim_tb;

    logic clk;
    logic rst_n;
    logic signed [31:0] setpoint;
    logic signed [31:0] feedback;
    logic signed [31:0] Kp, Ki, Kd;
    logic signed [31:0] control_out;
    logic signed [31:0] current_error;

    logic signed [31:0] velocity;
    logic signed [31:0] acceleration;
    logic signed [31:0] gravity = 98;
    logic signed [31:0] wind_noise;

    int file_handle;
    int time_step;

    drone_pid_sim uut (
        .clk(clk), .rst_n(rst_n),
        .setpoint(setpoint), .feedback(feedback),
        .Kp(Kp), .Ki(Ki), .Kd(Kd),
        .control_out(control_out), .current_error(current_error)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end

    initial begin
        file_handle = $fopen("drone_pid_output.csv", "w");
        $fdisplay(file_handle, "Time,Setpoint,Altitude,Error,ControlSignal");

        rst_n = 0;
        setpoint = 1000;
        feedback = 0;
        velocity = 0;
        wind_noise = 0;
        time_step = 0;
        
        Kp = 20; 
        Ki = 1; 
        Kd = 50; 

        #20 rst_n = 1;

        repeat(150) begin
            @(posedge clk);
            
            acceleration = (control_out / 10) - gravity + wind_noise;
            velocity = velocity + acceleration;
            feedback = feedback + (velocity / 10);
            
            if (feedback < 0) begin
                feedback = 0;
                velocity = 0;
            end

            $fdisplay(file_handle, "%0d,%0d,%0d,%0d,%0d", 
                      time_step, setpoint, feedback, current_error, control_out);
            time_step++;

            if (time_step == 75) wind_noise = -150;
            if (time_step == 85) wind_noise = 0;
        end

        $fclose(file_handle);
        $display("Simulation complete. Data written to drone_pid_output.csv");
        $finish;
    end
endmodule
