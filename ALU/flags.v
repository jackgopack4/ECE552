module flags(zr, neg, ov, change_z, change_n, change_v, z_in, n_in, v_in);

  input change_z, change_n, change_v;
  input z_in, n_in, v_in;
  output reg zr, neg, ov;

  always@(z_in, n_in, v_in) begin
    if(change_z == 1'b1) zr  <= z_in;
    if(change_n == 1'b1) neg <= n_in;
    if(change_v == 1'b1) ov  <= v_in;
  end

endmodule