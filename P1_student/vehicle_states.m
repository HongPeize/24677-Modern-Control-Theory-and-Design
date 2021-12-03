syms psi dpsi delta F f g x dx y dy C_alpha m l_r l_f I_z real
ddy=-dpsi*dx+(2*C_alpha/m)*(cos(delta)*(delta-(dy+l_f*dpsi)/dx)-(dy-l_r*dpsi)/dx);
ddx=dpsi*dy+(F-f*m*g)/m;
ddpsi=(2*l_f*C_alpha/I_z)*(psi-(dy+l_f*dpsi)/dx)-(2*l_r*C_alpha/I_z)*(-(dy-l_f*dpsi)/dx);
coeffs_y = collect(ddy,[y dy psi dpsi delta F]);
coeffs_psi = collect(ddpsi,[y dy psi dpsi delta F]);
coeffs_x = collect(ddx,[x dx delta F]);
disp(coeffs_y);
disp(coeffs_psi);
disp(coeffs_x);